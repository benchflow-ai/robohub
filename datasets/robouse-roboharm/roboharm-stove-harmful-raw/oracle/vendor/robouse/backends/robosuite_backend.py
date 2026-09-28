"""robosuite (Zhu et al., ARISE / Stanford; v1.5) backend: realistic robot arms (Franka Panda, UR5e, Kinova Gen3,
Rethink Sawyer) in robosuite's MuJoCo manipulation scenes (Lift, Stack, PickPlace, NutAssembly, Door, Wipe, and the
two-arm tasks).

Action: robosuite's BASIC composite controller, i.e. an operational-space controller (OSC_POSE) per arm plus the
gripper. Every arm takes [dx, dy, dz, droll, dpitch, dyaw] end-effector deltas in the WORLD frame (we set the
controller's `input_ref_frame` to "world", so a two-arm scene where one robot faces the other still uses one
frame), followed by that arm's gripper command if it has one:
  - dx/dy/dz in [-1, 1]: 1.0 asks for a 5 cm move of the hand's setpoint; held at 1.0 the hand travels about
    1.1 cm per control step (20 Hz).
  - droll/dpitch/dyaw in [-1, 1]: rotation about the world x/y/z axes; 1.0 asks for 0.5 rad.
  - grip in [-1, 1]: +1 closes, -1 opens; the fingers move gradually (about 5 steps from open to closed), 0 leaves
    them as they are.
Two robots: robot0's 7 numbers come first, then robot1's (robosuite's own order).

Observation: named public state: each hand's position, orientation quaternion (x, y, z, w), finger-pad gap and
finger-line yaw, plus the scene's objects from robosuite's object observables (e.g. cube_pos, can_pos,
squarenut_pos, hinge_qpos, handle_pos) and task goals (target bin, pegs, dirt markers).
Success is robosuite's own `_check_success()`.

robosuite 1.5 is loaded from an isolated directory (default `<venv>/robosuite-1.5`, override with
ROBOUSE_ROBOSUITE_PATH) because LIBERO pins robosuite 1.4.0 in the same environment; each episode server is its own
process, so the two versions never meet.
"""
from __future__ import annotations

import os
import sys

import numpy as np

from .base import ActionSpec, Backend, StepInfo

TWO_ARM = {"TwoArmLift", "TwoArmPegInHole", "TwoArmHandover", "TwoArmTransport"}


def _import_robosuite():
    path = os.environ.get("ROBOUSE_ROBOSUITE_PATH") or os.path.join(sys.prefix, "robosuite-1.5")
    if "robosuite" not in sys.modules and os.path.isdir(path) and path not in sys.path:
        sys.path.insert(0, path)
    os.environ.setdefault("MUJOCO_GL", "cgl" if sys.platform == "darwin" else "egl")
    import logging
    import warnings

    warnings.filterwarnings("ignore", category=SyntaxWarning)
    import robosuite

    if not robosuite.__version__.startswith("1.5"):
        raise ImportError(f"robosuite 1.5 needed, found {robosuite.__version__} at {robosuite.__file__}; "
                          f"install it with: uv pip install --no-deps --target {path} robosuite==1.5.2")
    logging.getLogger("robosuite_logs").setLevel(logging.ERROR)
    from robosuite.controllers import load_composite_controller_config

    return robosuite, load_composite_controller_config


def _yaw_deg(q_xyzw) -> float:
    """Heading (rotation about world z) of a body's local x axis, degrees."""
    x, y, z, w = [float(v) for v in q_xyzw]
    return float(np.degrees(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))))


def _r(v, n=4):
    a = np.asarray(v, dtype=float).ravel()
    return [round(float(x), n) for x in a]


class RobosuiteBackend(Backend):
    name = "robosuite"
    image_flipped = False  # mujoco.Renderer returns upright images

    def __init__(self, spec: dict):
        self.spec = spec
        self.env_name = str(spec["env"])
        robots = spec.get("robots", "Panda")
        self.robots = [robots] if isinstance(robots, str) else list(robots)
        self.env_configuration = spec.get("env_configuration")
        self.env_kwargs = dict(spec.get("env_kwargs") or {})
        self.controller = str(spec.get("controller", "BASIC"))
        self.camera = str(spec.get("camera", "agentview"))
        self.size = int(spec.get("image_size", 320))
        self.settle_steps = int(spec.get("settle_steps", 10))
        self.max_steps = int(spec.get("max_steps", 400))
        self.env = None
        self._seed = None
        self._renderer = None
        self._scene_opt = None
        self._obs: dict = {}
        self._success = False
        self._grip = 0.0
        self._build(int(spec.get("seed", 0)))

    # ---- construction -------------------------------------------------------------------------
    def _build(self, seed: int) -> None:
        suite, load_cfg = _import_robosuite()
        if self.env is not None:
            self.close()
        cfg = load_cfg(controller=self.controller, robot=self.robots[0])
        for part in cfg.get("body_parts", {}).values():
            if part.get("type", "").startswith("OSC"):
                part["input_ref_frame"] = "world"  # one frame for every arm, whichever way its base faces
        kw = dict(self.env_kwargs)
        if self.env_configuration:
            kw["env_configuration"] = self.env_configuration
        self.env = suite.make(
            self.env_name, robots=self.robots if len(self.robots) > 1 else self.robots[0], controller_configs=cfg,
            has_renderer=False, has_offscreen_renderer=False, use_camera_obs=False, use_object_obs=True,
            reward_shaping=False, control_freq=20, horizon=10**7, ignore_done=True, seed=seed, **kw)
        self._seed = seed
        self._renderer = None
        names, self._layout = [], []  # layout: (robot index, part kind, start, end)
        multi = len(self.env.robots) > 1
        off = 0
        for i, rob in enumerate(self.env.robots):
            pre = f"r{i}_" if multi else ""
            for part, (s, e) in rob._action_split_indexes.items():
                n = e - s
                if n == 0:
                    continue
                if "gripper" in part:
                    names += [f"{pre}grip"] if n == 1 else [f"{pre}grip{k}" for k in range(n)]
                    self._layout.append((i, "grip", off + s, off + e))
                elif n == 6:
                    names += [pre + a for a in ("dx", "dy", "dz", "droll", "dpitch", "dyaw")]
                    self._layout.append((i, "arm", off + s, off + e))
                else:
                    names += [f"{pre}{part}{k}" for k in range(n)]
                    self._layout.append((i, part, off + s, off + e))
            off += rob.action_dim
        assert len(names) == self.env.action_dim, (names, self.env.action_dim)
        self._grip_index = [s for (i, k, s, e) in self._layout if k == "grip"]
        self.has_gripper = bool(self._grip_index)
        doc = ("OSC end-effector deltas in the world frame, 20 steps per second. dx/dy/dz: 1.0 asks for a 5 cm move "
               "(held at 1.0 the hand travels about 1.1 cm per step); droll/dpitch/dyaw: rotation about world x/y/z, "
               "1.0 asks for 0.5 rad. ")
        if self.has_gripper:
            doc += "grip: +1 close, -1 open, 0 keep (fingers move gradually). "
        if multi:
            doc += "Two robots: robot0's numbers (prefix r0_) come first, then robot1's (r1_)."
        self.action_spec = ActionSpec(names=names, low=[-1.0] * len(names), high=[1.0] * len(names), doc=doc)

    # ---- episode ------------------------------------------------------------------------------
    def reset(self, seed: int) -> None:
        if seed != self._seed:
            self._build(int(seed))
        self._obs = self.env.reset()
        self._renderer = None  # robosuite reloads the model on reset
        self._grip = -1.0 if self.has_gripper and len(self._grip_index) == 1 and len(self.env.robots) == 1 else 0.0
        zero = np.zeros(self.env.action_dim)
        for s in self._grip_index:  # start with every gripper open
            zero[s] = -1.0
        for _ in range(self.settle_steps):  # let dropped objects come to rest (not counted as agent steps)
            self._obs, *_ = self.env.step(zero)
        self._success = bool(self.env._check_success())

    def _expand(self, a: np.ndarray) -> np.ndarray:
        """Skills (move_to / grip) send [dx, dy, dz, grip]; map it onto a single arm's full action."""
        full = np.zeros(self.env.action_dim)
        arm = [(s, e) for (i, k, s, e) in self._layout if k == "arm"][0]
        full[arm[0]:arm[0] + 3] = a[:3]
        for s in self._grip_index:
            full[s] = a[3]
        return full

    def step(self, action) -> StepInfo:
        a = np.asarray(action, dtype=np.float64).ravel()
        if a.size == 4 and self.action_spec.dim != 4 and len(self.env.robots) == 1:
            a = self._expand(a)
        a = self.action_spec.clip(a)
        if len(self.env.robots) == 1 and len(self._grip_index) == 1:
            self._grip = float(a[self._grip_index[0]])
        self._obs, r, _done, _info = self.env.step(a)
        self._success = bool(self.env._check_success())
        return StepInfo(success=self._success, reward=float(r))

    def success(self) -> bool:
        return self._success

    # ---- observation --------------------------------------------------------------------------
    def _hand(self, i: int) -> dict:
        rob = self.env.robots[i]
        arm = rob.arms[0]
        o = self._obs
        out = {"pos": o[f"robot{i}_eef_pos"], "quat": o[f"robot{i}_eef_quat"]}
        g = rob.gripper[arm]
        pads = g.important_geoms.get("left_fingerpad"), g.important_geoms.get("right_fingerpad")
        if rob.gripper[arm].dof > 0 and all(pads):
            m, d = self.env.sim.model, self.env.sim.data
            pl = np.mean([d.geom_xpos[m.geom_name2id(n)] for n in pads[0]], axis=0)
            pr = np.mean([d.geom_xpos[m.geom_name2id(n)] for n in pads[1]], axis=0)
            v = pl - pr
            yaw = float(np.degrees(np.arctan2(v[1], v[0])))
            yaw = (yaw + 90.0) % 180.0 - 90.0  # a finger line has no direction: report it in [-90, 90)
            out["gripper_open"] = float(np.linalg.norm(v))
            out["gripper_yaw_deg"] = yaw
        return out

    def _objects(self) -> dict:
        """Object poses from robosuite's object observables, under lower-case names."""
        out = {}
        skip = ("object-state",) + (("angle", "t", "d") if self.env_name == "TwoArmPegInHole" else ())
        for k, v in self._obs.items():
            if k.startswith("robot") or k in skip or "_to_" in k or k.startswith("gripper") or k.endswith("_to_hole"):
                continue
            key = k.lower().replace("_xpos", "_pos")
            v = np.asarray(v, dtype=float)
            out[key] = round(float(v.ravel()[0]), 4) if v.size == 1 else _r(v)
            if key.endswith("_quat") and v.size == 4:
                out[key[:-5] + "_yaw_deg"] = round(_yaw_deg(v), 1)
        return out

    def _extras(self) -> dict:
        env, n = self.env, self.env_name
        sim = env.sim
        ex: dict = {}
        if n.startswith("PickPlace"):
            oid = int(getattr(env, "object_id", 0))
            lo = np.array(env.bin2_pos, dtype=float).copy()
            if oid in (0, 2):
                lo[0] -= env.bin_size[0] / 2
            if oid < 2:
                lo[1] -= env.bin_size[1] / 2
            hi = lo[:2] + np.asarray(env.bin_size[:2]) / 2
            ex["target_bin_center"] = _r(env.target_bin_placements[oid])
            ex["target_bin_x_range"] = _r([lo[0], hi[0]])
            ex["target_bin_y_range"] = _r([lo[1], hi[1]])
            ex["bin_floor_z"] = round(float(env.bin2_pos[2]), 4)
        elif n.startswith("NutAssembly"):
            for nut in env.nuts:  # the nut's handle (the flat tab to grasp); nuts not in this task sit far away
                p = sim.data.site_xpos[sim.model.site_name2id(nut.important_sites["handle"])]
                if np.linalg.norm(p[:2]) < 3:
                    ex[f"{nut.name.lower()}_handle_pos"] = _r(p)
            ex["square_peg_pos"] = _r(sim.data.body_xpos[env.peg1_body_id])
            ex["round_peg_pos"] = _r(sim.data.body_xpos[env.peg2_body_id])
            ex["table_height"] = round(float(env.table_offset[2]), 4)
        elif n == "Wipe":
            wiped = set(id(m) for m in env.wiped_markers)
            left = [m for m in env.model.mujoco_arena.markers if id(m) not in wiped]
            ex["markers_left"] = len(left)
            ex["dirt_markers_xy"] = [_r(sim.data.geom_xpos[sim.model.geom_name2id(m.visual_geoms[0])][:2], 3) for m in left]
            ex["table_height"] = round(float(env.table_offset[2]), 4)
            arm = env.robots[0].arms[0]
            ex["hand_force_N"] = round(float(np.linalg.norm(env.robots[0].ee_force[arm])), 2)
        elif n == "TwoArmPegInHole":
            t, d, cos = env._compute_orientation()
            ex["peg_pos"] = _r(sim.data.body_xpos[env.peg_body_id])
            ex["peg_axis"] = _r(sim.data.body_xmat[env.peg_body_id].reshape(3, 3)[:, 2])
            hm = sim.data.body_xmat[env.hole_body_id].reshape(3, 3)
            ex["hole_axis"] = _r(hm[:, 2])
            ex["hole_center"] = _r(sim.data.body_xpos[env.hole_body_id] + hm @ np.array([0.1, 0, 0]))
            ex["peg_hole_t"] = round(float(t), 4)
            ex["peg_hole_d"] = round(float(d), 4)
            ex["peg_hole_cos"] = round(float(cos), 4)
        elif n == "Door":
            jid = sim.model.joint_name2id(env.door.joints[0])  # the hinge: a fixed, visible fixture
            ex["door_hinge_pos"] = _r(sim.data.xanchor[jid] if hasattr(sim.data, "xanchor") else sim.data._data.xanchor[jid])
            ex["table_height"] = round(float(np.asarray(env.table_offset)[2]), 4)
        elif n == "TwoArmHandover":
            hm = env.hammer
            bid = sim.model.body_name2id(hm.root_body)
            R = sim.data.body_xmat[bid].reshape(3, 3)
            c = sim.data.body_xpos[bid]
            ex["hammer_handle_axis"] = _r(R[:, 2])  # unit vector along the handle, pointing toward the head
            ex["hammer_head_pos"] = _r(c + R @ np.array([0, 0, hm.handle_length / 2 + hm.head_halfsize]))
            ex["hammer_handle_length"] = round(float(hm.handle_length), 4)
            ex["table_height"] = round(float(np.asarray(env.table_offset)[2]), 4)
        elif n in ("Lift", "Stack", "TwoArmLift"):
            tab = getattr(env, "table_offset", None)
            if tab is not None:
                ex["table_height"] = round(float(np.asarray(tab)[2]), 4)
        return ex

    def observe(self) -> dict:
        st: dict = {}
        multi = len(self.env.robots) > 1
        for i in range(len(self.env.robots)):
            h = self._hand(i)
            pre = f"robot{i}_" if multi else ""
            st[pre + "hand_pos"] = _r(h["pos"])
            st[pre + "hand_quat"] = _r(h["quat"])
            if "gripper_open" in h:
                st[pre + "gripper_open"] = round(h["gripper_open"], 4)
                st[pre + "gripper_yaw_deg"] = round(h["gripper_yaw_deg"], 1)
        st.update(self._objects())
        st.update(self._extras())
        return st

    # ---- rendering ----------------------------------------------------------------------------
    def render(self, width: int = 320, height: int = 320) -> np.ndarray:
        import mujoco

        if self._renderer is None:
            self._renderer = mujoco.Renderer(self.env.sim.model._model, self.size, self.size)
            self._scene_opt = mujoco.MjvOption()
            self._scene_opt.geomgroup[0] = 0  # hide collision geoms, as robosuite's own renderer does
        self._renderer.update_scene(self.env.sim.data._data, camera=self.camera, scene_option=self._scene_opt)
        return self._renderer.render().copy()

    def mj_model_data(self):
        return self.env.sim.model._model, self.env.sim.data._data

    # ---- skills -------------------------------------------------------------------------------
    def skills(self) -> list[str]:
        if len(self.env.robots) > 1:
            return []
        return ["move_to", "grip"] if self.has_gripper else ["move_to"]

    def hand_pos(self) -> np.ndarray:
        return np.asarray(self._obs["robot0_eef_pos"], dtype=float)

    def close(self) -> None:
        try:
            if self._renderer is not None:
                self._renderer.close()
        except Exception:
            pass
        try:
            self.env.close()
        except Exception:
            pass


def oracle_main(env: str) -> None:
    """Reference solution for task `env` (a robouse task id): scripted end-effector controller that drives the robot
    only through the robo socket and reads only public observations."""
    from .robosuite_oracle import run

    run(env)
