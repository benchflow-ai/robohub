"""dm_control simulation worker for the Robo Use `dmcontrol` suite. Runs inside its own virtualenv (Python 3.12,
`dm-control==1.0.47`, which pins its own MuJoCo, `mujoco==3.14.0`), because the main Robo Use environment has MuJoCo 3.3.0.

The Robo Use `dmcontrol` backend (dmcontrol.py) starts this script as a subprocess and talks to it over stdin/stdout: one
JSON request per line, one JSON response per line; a `render` response is followed by the raw RGB bytes. Only numpy,
mujoco and dm_control are imported here (never the robouse package).

Two kinds of scene:
- `suite`: a DeepMind Control Suite task (`dm_control.suite.load(domain, task, task_kwargs={"random": seed})`), stepped
  with its own actuator commands (one step = the task's control timestep).
- `manipulation`: a dm_control.manipulation task (Kinova Jaco arm with the three-finger Jaco hand), stepped with an
  end-effector command [DX, DY, DZ, GRIP] that this worker turns into the arm's joint-velocity commands (damped
  least-squares on the pinch site's Jacobian, keeping the hand pointing down) and a finger velocity command.

The success check runs here, every control step, from the physical state: `in_target` is the task's own "in target"
condition (e.g. dm_control's sparse reward equal to 1), and `hold_steps` counts consecutive control steps with it true.
"""

from __future__ import annotations

import math
import os
import sys

os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("OMP_NUM_THREADS", "1")
if sys.platform.startswith("linux"):
    os.environ.setdefault("MUJOCO_GL", "osmesa")
    os.environ.setdefault("PYOPENGL_PLATFORM", os.environ["MUJOCO_GL"])

import warnings

warnings.filterwarnings("ignore")

import numpy as np


def _r(v, n: int = 4):
    a = np.asarray(v, dtype=float)
    if a.ndim == 0:
        return round(float(a), n)
    return [round(float(x), n) for x in a.ravel()]


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


# ---------------------------------------------------------------------------------------------------------------------
# Control Suite: per domain, the published extras and the "in target" rule
# ---------------------------------------------------------------------------------------------------------------------


def _suite_extras(domain: str, task: str, p) -> dict:
    n, d = p.named.model, p.named.data
    if domain == "cartpole":
        ang = _wrap(float(d.qpos["hinge_1"][0]))  # 0 = upright
        return {
            "cart_x": _r(d.qpos["slider"][0]),
            "cart_vel": _r(d.qvel["slider"][0]),
            "pole_angle": _r(ang),
            "pole_angvel": _r(d.qvel["hinge_1"][0]),
        }
    if domain == "pendulum":
        return {"pole_angle": _r(_wrap(float(d.qpos["hinge"][0]))), "pole_angvel": _r(d.qvel["hinge"][0])}
    if domain == "acrobot":
        return {
            "shoulder_angle": _r(_wrap(float(d.qpos["shoulder"][0]))),
            "elbow_angle": _r(_wrap(float(d.qpos["elbow"][0]))),
            "shoulder_vel": _r(d.qvel["shoulder"][0]),
            "elbow_vel": _r(d.qvel["elbow"][0]),
            "tip_pos": _r(d.site_xpos["tip"][[0, 2]]),
            "target_pos": _r(d.site_xpos["target"][[0, 2]]),
            "target_radius": _r(n.site_size["target"][0]),
            "tip_to_target": _r(p.to_target()),
        }
    if domain == "reacher":
        return {
            "shoulder_angle": _r(_wrap(float(d.qpos["shoulder"][0]))),
            "wrist_angle": _r(d.qpos["wrist"][0]),
            "shoulder_vel": _r(d.qvel["shoulder"][0]),
            "wrist_vel": _r(d.qvel["wrist"][0]),
            "fingertip_pos": _r(d.geom_xpos["finger"][:2]),
            "target_pos": _r(d.geom_xpos["target"][:2]),
            "target_radius": _r(n.geom_size["target"][0] + n.geom_size["finger"][0]),
            "fingertip_to_target": _r(p.finger_to_target_dist()),
        }
    if domain == "point_mass":
        return {
            "mass_pos": _r(d.geom_xpos["pointmass"][:2]),
            "mass_vel": _r(d.qvel[["root_x", "root_y"]]),
            "target_pos": _r(d.geom_xpos["target"][:2]),
            "target_radius": _r(n.geom_size["target"][0]),
            "mass_to_target": _r(p.mass_to_target_dist()),
        }
    if domain == "finger":
        sd = d.sensordata
        return {
            "proximal_angle": _r(sd["proximal"][0]),
            "distal_angle": _r(sd["distal"][0]),
            "proximal_vel": _r(sd["proximal_velocity"][0]),
            "distal_vel": _r(sd["distal_velocity"][0]),
            "fingertip_pos": _r(d.geom_xpos["fingertip"][[0, 2]]),  # centre of the fingertip capsule
            "fingertip_end_pos": _r(
                (d.xpos["distal"] + d.xmat["distal"].reshape(3, 3) @ np.array([0, 0, -0.161]))[[0, 2]]
            ),
            "spinner_center": _r(d.xpos["spinner"][[0, 2]]),
            "spinner_angle": _r(_wrap(float(d.qpos["hinge"][0]))),
            "spinner_vel": _r(d.qvel["hinge"][0]),
            "spinner_tip_pos": _r(sd["tip"][[0, 2]]),
            "target_pos": _r(sd["target"][[0, 2]]),
            "target_radius": _r(n.site_size["target"][0]),
            "tip_to_target": _r(np.linalg.norm(p.to_target())),
        }
    if domain == "ball_in_cup":
        return {
            "cup_pos": _r(d.xpos["cup"][[0, 2]]),
            "cup_vel": _r(d.qvel[["cup_x", "cup_z"]]),
            "ball_pos": _r(d.xpos["ball"][[0, 2]]),
            "ball_vel": _r(d.qvel[["ball_x", "ball_z"]]),
            "cup_target_center": _r(d.site_xpos["target"][[0, 2]]),
            "string_length": 0.3,
        }
    if domain == "manipulator":
        obj = "peg" if "peg" in task else "ball"
        joints = ["arm_root", "arm_shoulder", "arm_elbow", "arm_wrist", "finger", "fingertip", "thumb", "thumbtip"]
        out = {
            "joint_angles": {j: _r(_wrap(float(d.qpos[j][0]))) for j in joints},
            "joint_vels": {j: _r(d.qvel[j][0]) for j in joints},
            "grasp_site_pos": _r(d.site_xpos["grasp"][[0, 2]]),
            "pinch_site_pos": _r(d.site_xpos["pinch"][[0, 2]]),
            f"{obj}_site_pos": _r(d.site_xpos[obj][[0, 2]]),
            f"target_{obj}_site_pos": _r(d.site_xpos["target_" + obj][[0, 2]]),
            f"{obj}_angle": _r(_wrap(float(d.qpos[obj + "_y"][0]))),
            f"{obj}_to_target": _r(p.site_distance(obj, "target_" + obj)),
        }
        if obj == "peg":
            out.update(
                {
                    "peg_tip_pos": _r(d.site_xpos["peg_tip"][[0, 2]]),
                    "target_peg_tip_pos": _r(d.site_xpos["target_peg_tip"][[0, 2]]),
                    "peg_grasp_site_pos": _r(d.site_xpos["peg_grasp"][[0, 2]]),
                    "peg_tip_to_target": _r(p.site_distance("peg_tip", "target_peg_tip")),
                }
            )
        return out
    return {}


def _suite_in_target(domain: str, task: str, p, env) -> bool:
    n, d = p.named.model, p.named.data
    if domain == "cartpole":  # balance_sparse / swingup_sparse: cart within 0.25 m of the centre, pole cosine >= 0.995
        return abs(float(d.qpos["slider"][0])) <= 0.25 and float(p.pole_angle_cosine()[0]) >= 0.995
    if domain == "pendulum":  # the task's reward: pole within 30 degrees of upright
        return float(p.pole_vertical()) >= math.cos(math.radians(30))
    if domain == "acrobot":  # swingup_sparse: tip inside the target sphere
        return float(p.to_target()) <= float(n.site_size["target"][0])
    if domain == "reacher":  # the task's reward: fingertip within target radius + finger radius
        return float(p.finger_to_target_dist()) <= float(n.geom_size["target"][0] + n.geom_size["finger"][0])
    if domain == "point_mass":  # the near-target factor of the reward: mass centre within the target radius
        return float(p.mass_to_target_dist()) <= float(n.geom_size["target"][0])
    if domain == "finger":  # the task's reward: spinner tip inside the target circle
        return float(p.dist_to_target()) <= 0
    if domain == "ball_in_cup":  # the task's reward: ball entirely inside the cup
        return bool(p.in_target())
    if domain == "manipulator":  # the task's reward at 1: object (and peg tip) within 1 cm of its target
        if "peg" in task:
            return p.site_distance("peg", "target_peg") <= 0.01 and p.site_distance("peg_tip", "target_peg_tip") <= 0.01
        return p.site_distance("ball", "target_ball") <= 0.01
    raise KeyError(domain)


def mjcf_frame(entity):
    """The free body that carries a prop attached to the arena (its attachment frame)."""
    from dm_control import mjcf

    return mjcf.get_attachment_frame(entity.mjcf_model)


# ---------------------------------------------------------------------------------------------------------------------
# Manipulation (Jaco)
# ---------------------------------------------------------------------------------------------------------------------

EE_STEP = 0.01  # m of commanded pinch-site motion per unit of DX/DY/DZ
EE_LIM = 2.0  # |DX|, |DY|, |DZ| <= 2 units per step (2 cm per 40 ms step = 0.5 m/s)
YAW_STEP = 0.1  # rad of commanded hand yaw per unit of DYAW
YAW_LIM = 2.0
WS_LOW, WS_HIGH = [-0.35, -0.35, -0.02], [0.35, 0.35, 0.6]  # the commanded pinch point stays in this box (m)
FINGER_VEL = 3.0  # rad/s finger velocity at GRIP = +-1


class Worker:
    def __init__(self):
        self.env = None

    # ---- episode -----------------------------------------------------------------------------------------------
    def make(self, kind: str, domain: str, task: str, seed: int, hold: int, **kw) -> dict:
        self.kind, self.domain, self.task, self.hold = kind, domain, task, int(hold)
        self.cfg = kw
        if kind == "suite":
            from dm_control import suite

            self.env = suite.load(domain, task, task_kwargs={"random": int(seed), "time_limit": float("inf")})
            self.physics = self.env.physics
        else:
            from dm_control import manipulation

            self.env = manipulation.load(task, seed=int(seed))
            self.env._time_limit = float("inf")
            self.physics = self.env.physics
        self.ts = self.env.reset()
        self.physics = self.env.physics
        self.steps, self.hold_steps, self.succeeded, self.reward = 0, 0, False, 0.0
        if kind == "manipulation":
            self._manip_setup()
        self.in_target = self._in_target()
        m = self.physics.model
        import mujoco

        cams = [mujoco.mj_id2name(m.ptr, mujoco.mjtObj.mjOBJ_CAMERA, i) for i in range(m.ncam)]
        spec = self.env.action_spec()
        return {
            "cameras": cams,
            "control_dt": float(self.env.control_timestep()),
            "action_low": _r(spec.minimum),
            "action_high": _r(spec.maximum),
            "in_target": self.in_target,
        }

    def _in_target(self) -> bool:
        if self.kind == "suite":
            return bool(_suite_in_target(self.domain, self.task, self.physics, self.env))
        return bool(self._manip_in_target())

    def step(self, action) -> dict:
        a = np.asarray(action, dtype=float)
        if not np.all(np.isfinite(a)):
            raise ValueError("non-finite action")
        if self.kind == "suite":
            spec = self.env.action_spec()
            self.ts = self.env.step(np.clip(a, spec.minimum, spec.maximum))
        else:
            self.ts = self.env.step(self._manip_action(a))
        self.reward = float(self.ts.reward or 0.0)
        self.steps += 1
        self.in_target = self._in_target()
        self.hold_steps = self.hold_steps + 1 if self.in_target else 0
        if self.hold_steps >= self.hold:
            self.succeeded = True
        return {"success": self.succeeded, "in_target": self.in_target, "hold_steps": self.hold_steps}

    def observe(self) -> dict:
        out = {}
        if self.kind == "suite":
            for k, v in self.ts.observation.items():
                out[k] = _r(v)
            out.update(_suite_extras(self.domain, self.task, self.physics))
        else:
            out.update(self._manip_observe())
        out["reward"] = _r(self.reward)
        out["in_target"] = bool(self.in_target)
        out["hold_steps"] = int(self.hold_steps)
        out["hold_required"] = int(self.hold)
        out["time_s"] = _r(self.physics.data.time, 3)
        return out

    def success(self) -> dict:
        return {"success": bool(self.succeeded), "in_target": bool(self.in_target), "hold_steps": int(self.hold_steps)}

    # ---- manipulation ------------------------------------------------------------------------------------------
    def _manip_setup(self):
        import mujoco

        p = self.physics
        m = p.model.ptr
        self._pinch = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "jaco_arm/jaco_hand/pinchsite")
        self._arm_dofs = [
            p.model.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, f"jaco_arm/joint_{i}")]
            for i in range(1, 7)
        ]
        self._arm_q = [
            p.model.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, f"jaco_arm/joint_{i}")]
            for i in range(1, 7)
        ]
        self._finger_q = [
            p.model.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, f"jaco_arm/jaco_hand/finger_{i}")]
            for i in range(1, 4)
        ]
        self.cmd_pos = p.data.site_xpos[self._pinch].copy()
        self.cmd_yaw = self._hand_yaw()
        self.grip = 0.0
        self._props = self._find_props()

    def _find_props(self) -> dict:
        """The task's free props (brick, bricks) by name."""
        t = self.env.task
        out = {}
        if getattr(t, "_prop", None) is not None:
            out["brick"] = t._prop
        for i, b in enumerate(getattr(t, "_bricks", None) or []):
            out[f"brick_{i}"] = b
        return out

    def _hand_yaw(self) -> float:
        R = self.physics.data.site_xmat[self._pinch].reshape(3, 3)
        return math.atan2(R[1, 0], R[0, 0])

    def _manip_action(self, a) -> np.ndarray:
        """[DX, DY, DZ, DYAW, GRIP] -> joint velocities (damped least squares on the pinch site's 6-D Jacobian) and
        finger velocities. The commanded pose is a point plus a yaw about the vertical, with the hand pointing down."""
        import mujoco

        p = self.physics
        m, d = p.model.ptr, p.data.ptr
        self.cmd_pos = self.cmd_pos + np.clip(a[:3], -EE_LIM, EE_LIM) * EE_STEP
        self.cmd_yaw = _wrap(self.cmd_yaw + float(np.clip(a[3], -YAW_LIM, YAW_LIM)) * YAW_STEP)
        self.grip = float(np.clip(a[4], -1, 1))
        cur = p.data.site_xpos[self._pinch].copy()
        lo, hi = np.array(self.cfg.get("ws_low", WS_LOW)), np.array(self.cfg.get("ws_high", WS_HIGH))
        self.cmd_pos = np.clip(self.cmd_pos, lo, hi)
        self.cmd_pos = cur + np.clip(self.cmd_pos - cur, -0.08, 0.08)  # the command stays within 8 cm of the hand
        jacp = np.zeros((3, m.nv))
        jacr = np.zeros((3, m.nv))
        mujoco.mj_jacSite(m, d, jacp, jacr, self._pinch)
        J = np.vstack([jacp[:, self._arm_dofs], jacr[:, self._arm_dofs]])
        dt = float(self.env.control_timestep())
        v = (self.cmd_pos - cur) / dt * 0.5
        R = p.data.site_xmat[self._pinch].reshape(3, 3)
        cy, sy = math.cos(self.cmd_yaw), math.sin(self.cmd_yaw)
        R_des = np.array([[cy, sy, 0.0], [sy, -cy, 0.0], [0.0, 0.0, -1.0]])  # x axis at the yaw, z axis down
        E = R_des @ R.T
        ang = np.array([E[2, 1] - E[1, 2], E[0, 2] - E[2, 0], E[1, 0] - E[0, 1]]) * 0.5  # small-angle rotation error
        w = ang / dt * 0.3
        qd = J.T @ np.linalg.solve(J @ J.T + 1e-2 * np.eye(6), np.concatenate([v, w]))
        spec = self.env.action_spec()
        act = np.zeros(spec.shape)
        act[:6] = qd
        act[6:9] = self.grip * FINGER_VEL
        return np.clip(act, spec.minimum, spec.maximum)

    def _manip_observe(self) -> dict:
        p = self.physics
        out = {
            "hand_pos": _r(p.data.site_xpos[self._pinch]),
            "hand_target_pos": _r(self.cmd_pos),
            "hand_yaw_deg": _r(math.degrees(self._hand_yaw()), 1),
            "hand_target_yaw_deg": _r(math.degrees(self.cmd_yaw), 1),
            "hand_z_axis": _r(p.data.site_xmat[self._pinch].reshape(3, 3)[:, 2], 3),
            "arm_joints": _r(p.data.qpos[self._arm_q]),
            "finger_joints": _r(p.data.qpos[self._finger_q]),
            "grip_command": _r(self.grip, 2),
        }
        for name, e in self._props.items():
            out[f"{name}_pos"] = _r(self._entity_pos(e))
            out[f"{name}_quat_wxyz"] = _r(self._entity_quat(e))
            out[f"{name}_yaw_deg"] = _r(math.degrees(self._entity_yaw(e)), 1)
        out.update(self._manip_task_extras())
        return out

    def _entity_pos(self, e):
        return np.asarray(self.physics.bind(mjcf_frame(e)).xpos)

    def _entity_quat(self, e):
        return np.asarray(self.physics.bind(mjcf_frame(e)).xquat)

    def _entity_yaw(self, e) -> float:
        R = np.asarray(self.physics.bind(mjcf_frame(e)).xmat).reshape(3, 3)
        return math.atan2(R[1, 0], R[0, 0])

    def _manip_task_extras(self) -> dict:
        t, p = self.env.task, self.physics
        name = self.task
        out = {}
        if name.startswith("reach_site"):
            out["target_pos"] = _r(p.bind(t._target).xpos)
            out["target_radius"] = 0.05
            out["hand_to_target"] = _r(np.linalg.norm(p.data.site_xpos[self._pinch] - p.bind(t._target).xpos))
        elif name.startswith("lift"):
            out["brick_lowest_point_z"] = _r(min(p.bind(t._prop.vertices).xpos[:, 2]))
            out["target_height"] = _r(t._target_height)
        elif name.startswith("place"):
            out["target_pos"] = _r(p.bind(t._pedestal.target_site).xpos)
            out["target_radius"] = 0.05
            out["brick_to_target"] = _r(
                np.linalg.norm(p.bind(t._prop_frame).xpos - p.bind(t._pedestal.target_site).xpos)
            )
            out["hand_to_target"] = _r(
                np.linalg.norm(p.data.site_xpos[self._pinch] - p.bind(t._pedestal.target_site).xpos)
            )
        elif name.startswith("stack"):
            out["desired_order_bottom_to_top"] = [int(i) for i in t._desired_order]
        return out

    def _manip_in_target(self) -> bool:
        return float(self.env.task.get_reward(self.physics)) >= 1.0 - 1e-6

    # ---- rendering -----------------------------------------------------------------------------------------------
    def render(self, camera: str, width: int, height: int) -> np.ndarray:
        g = self.physics.model.vis.global_  # the offscreen buffer must hold the frame (visual only; recordings)
        if width > g.offwidth or height > g.offheight:
            if getattr(self.physics, "_contexts", None):
                self.physics._free_rendering_contexts()
            g.offwidth, g.offheight = max(g.offwidth, width), max(g.offheight, height)
        img = self.physics.render(height=height, width=width, camera_id=camera)
        return np.ascontiguousarray(img, dtype=np.uint8)
