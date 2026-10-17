"""DexJoCo simulation worker. Runs inside the separate DexJoCo virtualenv (Python 3.11, MuJoCo 3.4.0, numpy 1.26, the
`dexjoco` package from github.com/brave-eai/dexjoco), because those pins conflict with the main robouse environment.

The robouse `dexjoco` backend (dexjoco.py) starts this script as a subprocess and talks to it over stdin/stdout: one JSON
request per line, one JSON response per line; a `render` response is followed by the raw RGB bytes. Only numpy, scipy,
mujoco and dexjoco are imported here (never the robouse package).

Control (per arm): the worker keeps a commanded hand pose (position + orientation, DexJoCo's mocap target, tracked by
DexJoCo's own operational-space controller) and 16 commanded Allegro joint angles. An action moves these commands by
deltas: DX, DY, DZ (2 cm per unit along world x, y, z), RX, RY, RZ (0.2 rad per unit about the world axes) and one delta per
hand joint (0.3 rad per unit), clipped to +-8, +-3 and +-6 units per step and to the joints' ranges. A 4-number action [DX, DY, DZ, G] (the
move_to / grip skills) drives the active arm and sets the hand to a generic power-grasp closure G (0 keeps the hand, G > 0
closes to fraction G of the preset, G < 0 opens fully).
"""

from __future__ import annotations

import os
import time
import types

os.environ.setdefault(
    "OPENBLAS_NUM_THREADS", "1"
)  # DexJoCo's controller inverts small matrices every substep; threaded
os.environ.setdefault("OMP_NUM_THREADS", "1")  # BLAS makes that ~70x slower on macOS

import numpy as np

POS_STEP = 0.02
ROT_STEP = 0.2
HAND_STEP = 0.3
POS_LIM, ROT_LIM, HAND_LIM = 8.0, 3.0, 6.0  # per-step limits in action units (16 cm, 0.6 rad, 1.8 rad)
FINGERS = ("ff", "mf", "rf", "th")

# Per task: which scene quantities `observe` reports. kind: body (current pose of a body), jbody (body carrying a free
# joint), geom, site, sensor (value), joint (qpos of a 1-dof joint), attr (an attribute of the DexJoCo env).
FIELDS: dict[str, list[tuple[str, str, str]]] = {
    "water_plant": [
        ("sprayer", "jbody", "spray_root"),
        ("sprayer_trigger_angle", "sensor", "spray_joint_0_pos"),
        ("nozzle_ref_point", "site", "ref_point"),
        ("plant", "body", "plant"),
    ],
    "hammer_nail": [
        ("hammer", "jbody", "hammer_joint"),
        ("nail_head", "attr", "_nail_head"),
        ("nail_depth", "attr", "_nail_depth"),
        ("nail_success_depth", "attr", "_success_depth"),
    ],
    "click_mouse": [
        ("mouse", "jbody", "mouse_root"),
        ("mouse_left_button", "sensor", "mouse_joint0_pos"),
        ("mousepad", "geom", "@mousepad"),
        ("mousepad_radius", "attr", "_mousepad_radius"),
        ("monitor", "jbody", "display_root"),
        ("display_blue", "attr", "_display_blue"),
    ],
    "pick_bucket": [
        ("bucket", "jbody", "bucket_root"),
        ("bucket_handle_angle", "joint", "bucket_joint_0"),
        ("food_box", "jbody", "boxed_food_0_freejoint"),
    ],
    "pinch_tongs": [
        ("tongs", "jbody", "tongs_root"),
        ("tongs_opening", "sensor", "tongs_joint_0_pos"),
        ("pinch_count", "attr", "_pinch_count"),
        ("lift_height_z", "attr", "_lift_z"),
    ],
    "fold_glasses": [
        ("glasses", "jbody", "glass_root"),
        ("left_temple_angle", "joint", "glass_joint_0"),
        ("right_temple_angle", "joint", "glass_joint_1"),
        ("open_box", "body", "open_box"),
    ],
    "bimanual_assembly": [
        ("peg", "jbody", "industreal_round_peg_8mm_joint"),
        ("socket", "jbody", "industreal_tray_insert_round_peg_8mm_joint"),
    ],
    "bimanual_microwave_cook": [
        ("hot_dog", "jbody", "hot_dog_free"),
        ("microwave", "body", "microwave_object"),
        ("microwave_door_angle", "joint", "microjoint"),
        ("start_button", "geom", "start_button"),
    ],
    "bimanual_photograph": [
        ("camera", "jbody", "camera_root"),
        ("shutter_button", "geom", "@shutter"),
        ("camera_view_point", "site", "@view"),
        ("target_region_center", "geom", "@target_region"),
    ],
    "bimanual_unlock_ipad": [
        ("ipad", "jbody", "ipad_freejoint"),
        ("stand", "body", "ipad_stand"),
        ("digits_entered", "attr", "_unlock_index"),
        ("unlocked", "attr", "_screen_unlocked"),
    ]
    + [(f"button_{d}", "geom", f"btn{d}_cyl") for d in range(10)],
}


def _q2R(q_wxyz):
    from scipy.spatial.transform import Rotation as R

    w, x, y, z = q_wxyz
    return R.from_quat([x, y, z, w])


def _R2q(r):
    x, y, z, w = r.as_quat()
    q = np.array([w, x, y, z])
    return q if q[0] >= 0 else -q


def _pose_fields(pos, q_wxyz):
    r = _q2R(q_wxyz)
    zaxis = r.as_matrix()[:, 2]
    tilt = float(np.degrees(np.arccos(np.clip(zaxis[2], -1, 1))))
    yaw = float(np.degrees(np.arctan2(r.as_matrix()[1, 0], r.as_matrix()[0, 0])))
    return {
        "pos": [round(float(v), 4) for v in pos],
        "quat_wxyz": [round(float(v), 4) for v in q_wxyz],
        "tilt_deg": round(tilt, 1),
        "yaw_deg": round(yaw, 1),
    }


class Worker:
    def __init__(self):
        self.env = None

    # ---- setup ---------------------------------------------------------------------------------------------------
    def make(self, task: str, init_state: list | None, seed: int = 0) -> dict:
        import importlib
        import pkgutil

        import dexjoco.sim.envs as envs_pkg
        import mujoco
        from dexjoco.tasks.mappings import CONFIG_MAPPING
        from dexjoco.tasks.state_restorers import restore_initial_state

        # the envs sleep to hold 30 Hz for human teleoperation; the episode server keeps its own clock
        for m in pkgutil.iter_modules(envs_pkg.__path__):
            mod = importlib.import_module(f"dexjoco.sim.envs.{m.name}")
            if hasattr(mod, "time"):
                mod.time = types.SimpleNamespace(time=time.time, sleep=lambda s: None)
        self.task = task
        cfg = CONFIG_MAPPING[task]()
        self.env = cfg.get_environment(
            policy_mode=True, render_mode="none", randomize=False, randomize_dynamics=False, seed=int(seed)
        )
        self.env.reset()
        if init_state is not None:
            restore_initial_state(self.env, task, cfg, np.asarray(init_state, dtype=np.float64))
        self.raw = self.env.unwrapped
        self.M, self.D = self.raw._model, self.raw._data
        self.bimanual = task.startswith("bimanual_")
        self.arms = ["right", "left"] if self.bimanual else ["right"]
        if self.bimanual:
            self.mocap = {
                "right": int(getattr(self.raw, "_mocap_right_id", 0)),
                "left": int(getattr(self.raw, "_mocap_left_id", 1)),
            }
        else:
            self.mocap = {"right": int(getattr(self.raw, "_panda_mocap_id", 0))}
        ids = np.asarray(self.raw._allegro_ctrl_ids, dtype=int)
        self.hand_ids = {"right": ids[:16], "left": ids[16:32]} if self.bimanual else {"right": ids[:16]}
        self.cmd_pos = {a: self.D.mocap_pos[self.mocap[a]].copy() for a in self.arms}
        self.cmd_quat = {a: self.D.mocap_quat[self.mocap[a]].copy() for a in self.arms}
        self.hand_lo = {a: self.M.actuator_ctrlrange[self.hand_ids[a], 0].copy() for a in self.arms}
        self.hand_hi = {a: self.M.actuator_ctrlrange[self.hand_ids[a], 1].copy() for a in self.arms}
        # the scene starts with the hand joints at the env's home pose; command exactly that
        self.cmd_hand = {a: np.clip(self._hand_qpos(a), self.hand_lo[a], self.hand_hi[a]) for a in self.arms}
        self.grasp = {}
        for a in self.arms:  # generic power grasp: fingers curled to 60 % of their flexion range, thumb opposed
            g = np.zeros(16)
            hi = self.hand_hi[a]
            for f in range(4):
                for j in range(4):
                    k = 4 * f + j
                    if f < 3:
                        g[k] = 0.0 if j == 0 else 0.6 * hi[k]
                    else:
                        g[k] = [0.9, 0.3, 0.5, 0.5][j] * hi[k]
            self.grasp[a] = np.clip(g, self.hand_lo[a], self.hand_hi[a])
        self.active = "right"
        self.failed = ""
        self.success_ever = False
        self._renderers = {}
        self._lookup = {}
        mujoco.mj_forward(self.M, self.D)
        cams = [mujoco.mj_id2name(self.M, mujoco.mjtObj.mjOBJ_CAMERA, i) for i in range(self.M.ncam)]
        return {
            "cameras": cams,
            "bimanual": self.bimanual,
            "hand_ranges": {
                a: [
                    [round(float(l), 3), round(float(h), 3)]
                    for l, h in zip(self.hand_lo[a], self.hand_hi[a], strict=False)
                ]
                for a in self.arms
            },
        }

    # ---- helpers -------------------------------------------------------------------------------------------------
    def _hand_qpos(self, arm: str) -> np.ndarray:
        out = []
        for i in self.hand_ids[arm]:
            jid = int(self.M.actuator_trnid[i, 0])
            out.append(float(self.D.qpos[self.M.jnt_qposadr[jid]]))
        return np.asarray(out)

    def _flange(self, arm: str):
        sfx = f"_{arm}" if self.bimanual else ""
        return (
            np.asarray(self.D.sensor(f"franka/flange_pos{sfx}").data).copy(),
            np.asarray(self.D.sensor(f"franka/flange_quat{sfx}").data).copy(),
        )

    def _resolve(self, kind: str, name: str):
        """Find a named element; a leading @ means 'first element of that kind whose name contains the rest'."""
        import mujoco

        key = (kind, name)
        if key in self._lookup:
            return self._lookup[key]
        obj = {
            "geom": mujoco.mjtObj.mjOBJ_GEOM,
            "site": mujoco.mjtObj.mjOBJ_SITE,
            "body": mujoco.mjtObj.mjOBJ_BODY,
            "sensor": mujoco.mjtObj.mjOBJ_SENSOR,
            "joint": mujoco.mjtObj.mjOBJ_JOINT,
            "jbody": mujoco.mjtObj.mjOBJ_JOINT,
        }[kind]
        n = {
            "geom": self.M.ngeom,
            "site": self.M.nsite,
            "body": self.M.nbody,
            "sensor": self.M.nsensor,
            "joint": self.M.njnt,
            "jbody": self.M.njnt,
        }[kind]
        idx = -1
        if name.startswith("@"):
            for i in range(n):
                nm = mujoco.mj_id2name(self.M, obj, i) or ""
                if name[1:] in nm:
                    idx = i
                    break
        else:
            idx = mujoco.mj_name2id(self.M, obj, name)
        self._lookup[key] = idx
        return idx

    def _field(self, kind: str, name: str):
        if kind == "attr":
            if name == "_nail_head":
                mid = getattr(self.raw, "_nail_mocap_id", None)
                return None if mid is None else [round(float(v), 4) for v in self.D.mocap_pos[mid]]
            v = getattr(self.raw, name, None)
            if v is None:
                return None
            return bool(v) if isinstance(v, (bool, np.bool_)) else round(float(v), 4)
        i = self._resolve(kind, name)
        if i < 0:
            return None
        if kind == "body":
            return _pose_fields(self.D.xpos[i], self.D.xquat[i])
        if kind == "jbody":
            b = int(self.M.jnt_bodyid[i])
            return _pose_fields(self.D.xpos[b], self.D.xquat[b])
        if kind == "geom":
            return [round(float(v), 4) for v in self.D.geom_xpos[i]]
        if kind == "site":
            return [round(float(v), 4) for v in self.D.site_xpos[i]]
        if kind == "sensor":
            adr, dim = int(self.M.sensor_adr[i]), int(self.M.sensor_dim[i])
            v = self.D.sensordata[adr : adr + dim]
            return round(float(v[0]), 4) if dim == 1 else [round(float(x), 4) for x in v]
        if kind == "joint":
            return round(float(self.D.qpos[self.M.jnt_qposadr[i]]), 4)
        return None

    # ---- stepping ------------------------------------------------------------------------------------------------
    def _apply_arm(self, arm: str, a: np.ndarray, hand_mode: str, g: float = 0.0) -> None:
        self.cmd_pos[arm] = self.cmd_pos[arm] + np.clip(a[:3], -POS_LIM, POS_LIM) * POS_STEP
        if hand_mode == "full":
            rv = np.clip(a[3:6], -ROT_LIM, ROT_LIM) * ROT_STEP
            if np.any(rv != 0):
                from scipy.spatial.transform import Rotation as R

                self.cmd_quat[arm] = _R2q(R.from_rotvec(rv) * _q2R(self.cmd_quat[arm]))
            self.cmd_hand[arm] = np.clip(
                self.cmd_hand[arm] + np.clip(a[6:22], -HAND_LIM, HAND_LIM) * HAND_STEP,
                self.hand_lo[arm],
                self.hand_hi[arm],
            )
        elif hand_mode == "grip":
            if g > 1e-3:
                lo = np.clip(np.zeros(16), self.hand_lo[arm], self.hand_hi[arm])
                self.cmd_hand[arm] = lo + min(1.0, g) * (self.grasp[arm] - lo)
            elif g < -1e-3:
                self.cmd_hand[arm] = np.clip(np.zeros(16), self.hand_lo[arm], self.hand_hi[arm])

    def step(self, action) -> dict:
        a = np.asarray(action, dtype=np.float64)
        per = 22
        if len(a) == 4:  # skill action for the active arm
            self._apply_arm(self.active, a, "grip", float(a[3]))
        elif len(a) == per * len(self.arms):
            moved = []
            for k, arm in enumerate(self.arms):
                blk = a[k * per : (k + 1) * per]
                self._apply_arm(arm, blk, "full")
                if np.any(blk != 0):
                    moved.append(arm)
            if len(moved) == 1:
                self.active = moved[0]
        else:
            raise ValueError(f"action must have {per * len(self.arms)} numbers (or 4 for the skills)")
        raw_a = {arm: np.concatenate([self.cmd_pos[arm], self.cmd_quat[arm], self.cmd_hand[arm]]) for arm in self.arms}
        _, _, _, _, info = self.raw.step(raw_a if self.bimanual else raw_a["right"])
        succ = bool(info.get("succeed", False))
        if not self.failed:
            if self.task == "water_plant" and getattr(self.raw, "_trigger_pulled", False) and not succ:
                if not self._nozzle_inside():
                    self.failed = "the sprayer was triggered while the nozzle was not at the plant"
            if self.task == "bimanual_unlock_ipad" and getattr(self.raw, "_unlock_failed", False):
                self.failed = "a wrong digit was pressed"
        if self.failed:
            succ = False
        self.success_ever = self.success_ever or succ
        return {"success": succ, "failed": self.failed}

    def _nozzle_inside(self) -> bool:
        p = self.D.site_xpos[self.M.site("ref_point").id]
        o = self.D.body("plant").xpos
        d = p - o
        return bool(d[0] ** 2 + d[1] ** 2 <= 0.04 and -0.2 <= d[2] <= 0.2)

    def success(self) -> dict:
        return {"success": bool(self.success_ever and not self.failed)}

    # ---- observation ---------------------------------------------------------------------------------------------
    def observe(self) -> dict:
        out = {}
        for arm in self.arms:
            p, q = self._flange(arm)
            pre = f"{arm}_" if self.bimanual else ""
            pf = _pose_fields(p, q)
            out[pre + "hand_pos"] = pf["pos"]
            out[pre + "hand_quat_wxyz"] = pf["quat_wxyz"]
            out[pre + "hand_target_pos"] = [round(float(v), 5) for v in self.cmd_pos[arm]]
            out[pre + "hand_target_quat_wxyz"] = [round(float(v), 5) for v in self.cmd_quat[arm]]
            out[pre + "finger_joints"] = [round(float(v), 3) for v in self._hand_qpos(arm)]
            out[pre + "finger_joint_targets"] = [round(float(v), 5) for v in self.cmd_hand[arm]]
        if self.bimanual:
            out["active_arm"] = self.active
        for label, kind, name in FIELDS.get(self.task, []):
            v = self._field(kind, name)
            if v is None:
                continue
            if isinstance(v, dict):
                for k2, v2 in v.items():
                    out[f"{label}_{k2}"] = v2
            else:
                out[label] = v
        if self.failed:
            out["task_failed"] = self.failed
        return out

    def _table_top(self):
        import mujoco

        for nm in ("table_visual", "table_top", "tabletop"):
            i = mujoco.mj_name2id(self.M, mujoco.mjtObj.mjOBJ_GEOM, nm)
            if i >= 0:
                return round(float(self.D.geom_xpos[i][2] + self.M.geom_size[i][2]), 4)
        return None

    # ---- rendering -----------------------------------------------------------------------------------------------
    def render(self, camera: str, width: int, height: int) -> np.ndarray:
        import mujoco

        key = (width, height)
        if key not in self._renderers:
            self.M.vis.global_.offwidth = max(self.M.vis.global_.offwidth, width)
            self.M.vis.global_.offheight = max(self.M.vis.global_.offheight, height)
            self._renderers[key] = mujoco.Renderer(self.M, height=height, width=width)
        r = self._renderers[key]
        r.update_scene(self.D, camera=camera)
        return np.ascontiguousarray(r.render(), dtype=np.uint8)

    def camera_matrix(self, camera: str) -> dict:
        import mujoco

        cid = mujoco.mj_name2id(self.M, mujoco.mjtObj.mjOBJ_CAMERA, camera)
        return {
            "fovy": float(self.M.cam_fovy[cid]),
            "pos": self.D.cam_xpos[cid].tolist(),
            "xmat": self.D.cam_xmat[cid].tolist(),
        }
