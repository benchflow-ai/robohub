"""RLE-Bench simulator worker. Runs inside the RLE-Bench virtualenv (robosuite and RoboCasa at RLE-Bench's pinned
commits, MuJoCo 3.3.1), started by the robouse `rlebench` backend (backend.py) and spoken to over stdin/stdout
(robouse.workers.wire). Only numpy, scipy, robosuite, RoboCasa and the vendored RLE-Bench scene code (rlebench_upstream/,
MIT) are imported here, never the robouse package.

Scenes are built exactly as RLE-Bench's task03 runtime builds them (tasks/task03/harness/{backend,adapter}.py at the
vendored commit): the same factory, seeds, reset procedure, public observation fields, `move` controller and scoring
hooks. The hidden quantities (which cube is heavy, where the ballast is, the scene's masses) stay in this process; the
backend asks only for public fields, images and, when the episode ends, the verdict.
"""

from __future__ import annotations

import os
import random
import re
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # rlebench_upstream (never shadows a simulator package)

RESOLUTION = 512
TABLETOP_CAMERAS = ("robot0_agentview_left", "robot0_agentview_right", "robot0_eye_in_hand")
ROBOT_FIELDS = frozenset(
    (
        "joint_pos",
        "joint_pos_cos",
        "joint_pos_sin",
        "joint_vel",
        "joint_acc",
        "gripper_qpos",
        "gripper_qvel",
        "eef_pos",
        "eef_quat",
        "eef_quat_site",
        "base_pos",
        "base_quat",
        "base_to_eef_pos",
        "base_to_eef_quat",
        "proprio-state",
    )
)
# RLE-Bench's TabletopClient names for the robot fields (harness/client.py tabletop_view)
ALIASES = {
    "joint_pos": "joint_pos",
    "joint_vel": "joint_vel",
    "eef_pos": "eef_pos",
    "eef_quat": "eef_quat",
    "gripper_qpos": "gripper_pos",
    "base_pos": "base_pos",
    "base_quat": "base_quat",
    "base_to_eef_pos": "eef_base_pos",
    "base_to_eef_quat": "eef_base_quat",
}


def _seed_all(seed: int) -> None:
    random.seed(seed)
    np.random.seed(seed)


def _prepare() -> None:
    import robosuite.macros as macros

    macros.IMAGE_CONVENTION = "opencv"


def _l(v, n: int = 5) -> list[float]:
    return [round(float(x), n) for x in np.asarray(v, dtype=float).ravel()]


class Worker:
    def __init__(self) -> None:
        self.env = None
        self.mode = ""
        self.task = ""
        self.seed = 0
        self.raw: dict = {}
        self.cameras: tuple[str, ...] = ()
        self.quadrant = ""
        self._cache: dict = {}

    # ---- setup ---------------------------------------------------------------------------------------------------
    def make(self, task: str, seed: int, case: int | None = None) -> dict:
        """Build the scene of RLE-Bench task03 task `task` (TowerMaxHeight, CantileverOverhang, BalanceCoins or HiddenCOM
        with box `case`) with `seed`, as RLE-Bench's runtime does, and reset it."""
        _prepare()
        self.task, self.seed = task, int(seed)
        _seed_all(self.seed)
        if task == "HiddenCOM":
            from rlebench_upstream.tabletop.hidden_com.config import CASES
            from rlebench_upstream.tabletop.hidden_com.scene import CAMERAS, HiddenCOM

            self.mode = "hidden_com"
            self.cameras = tuple(CAMERAS)
            self.quadrant = str(np.random.default_rng(self.seed).choice(list("ABCD")))
            self.env = HiddenCOM(self.quadrant, case=CASES[int(case or 0)])
        else:
            from rlebench_upstream.tabletop import make

            self.mode = "tabletop"
            self.cameras = TABLETOP_CAMERAS
            self.env = make(task, seed=self.seed, camera_names=TABLETOP_CAMERAS)
        return self.reset()

    def reset(self) -> dict:
        """RLE-Bench's reset: the same seed, so the same scene (task03's `reset()` restores the initial scene)."""
        _seed_all(self.seed)
        self.env.rng = np.random.default_rng(self.seed)
        self.raw = self.env.reset()
        for name, observable in getattr(self.env, "_observables", {}).items():
            if name.endswith(("_image", "_depth")):
                observable.set_enabled(False)  # rendered only on demand, as in RLE-Bench
        model = self.env.sim.model
        model.vis.global_.offwidth = RESOLUTION
        model.vis.global_.offheight = RESOLUTION
        self._cache.clear()
        meta = getattr(self.env, "get_ep_meta", lambda: {})()
        return {
            "instruction": (meta or {}).get("lang", ""),
            "action_dim": int(len(self.env.action_spec[0])),
            "cameras": list(self.cameras),
        }

    # ---- stepping ------------------------------------------------------------------------------------------------
    def step(self, action) -> dict:
        self.raw, _, _, _ = self.env.step(np.asarray(action, dtype=float))
        if not np.isfinite(self.env.sim.data.qpos).all():
            raise RuntimeError("nonfinite simulator state")
        self._cache.clear()
        return {}

    def move_action(self, position, quaternion, gripper) -> list[float]:
        """One control step of RLE-Bench's `move`: track a world-frame tool pose (position, unit quaternion xyzw) with
        the arm (task03 harness/adapter.py move_action)."""
        from scipy.spatial.transform import Rotation

        orientation = Rotation.from_quat(np.asarray(quaternion, dtype=float)).as_matrix()
        position = np.asarray(position, dtype=float)
        if self.mode == "hidden_com":
            data = self.env.sim.data
            site = self.env.robots[0].eef_site_id["right"]
            dp = (position - data.site_xpos[site]) / 0.05
            dr = Rotation.from_matrix(orientation @ data.site_xmat[site].reshape(3, 3).T).as_rotvec() / 0.5
            return [float(x) for x in np.r_[np.clip(dp, -1, 1), np.clip(dr, -1, 1), gripper]]
        base = Rotation.from_quat(self.raw["robot0_base_quat"]).as_matrix()
        target = base.T @ (position - self.raw["robot0_base_pos"])
        current = Rotation.from_quat(self.raw["robot0_base_to_eef_quat"]).as_matrix()
        dp = (target - self.raw["robot0_base_to_eef_pos"]) / 0.05
        dr = Rotation.from_matrix(base.T @ orientation @ current.T).as_rotvec() / 0.5
        return [float(x) for x in np.r_[np.clip(dp, -1, 1), np.clip(dr, -1, 1), gripper, 0, 0, 0, 0, -1]]

    # ---- observation ---------------------------------------------------------------------------------------------
    def observe(self) -> dict:
        """The public fields of RLE-Bench's observation (TabletopClient / HiddenCOMClient), under their client names."""
        shown = {k: v for k, v in self.raw.items() if (m := re.fullmatch(r"robot\d+_(.*)", k)) and m[1] in ROBOT_FIELDS}
        if self.mode == "hidden_com":
            from scipy.spatial.transform import Rotation

            robot = self.env.robots[0]
            data = self.env.sim.data
            site = robot.eef_site_id["right"]
            shown.update(
                robot0_eef_pos=data.site_xpos[site].copy(),
                robot0_eef_quat=Rotation.from_matrix(data.site_xmat[site].reshape(3, 3)).as_quat(),
                robot0_gripper_qpos=data.qpos[robot._ref_gripper_joint_pos_indexes["right"]].copy(),
            )
        out: dict = {}
        for key, value in shown.items():
            if key.startswith("robot0_") and key[7:] in ALIASES:
                out[ALIASES[key[7:]]] = _l(value)
        if self.task == "BalanceCoins":
            pos = self.env.position_observation()
            out["cube_positions"] = {k: _l(v) for k, v in pos["cube_positions"].items()}
            out["pan_positions"] = {k: _l(v) for k, v in pos["pan_positions"].items()}
        return out

    def private_state(self) -> dict:
        """Object poses and collision extents, for the reference solution only (the episode server gives them only to
        requests carrying the oracle token). No masses or hidden identities."""
        if self.mode != "tabletop":
            return {}
        m, d = self.env.sim.model._model, self.env.sim.data._data
        out = {}
        for name, bid in self.env.obj_body_id.items():
            gids = [
                g for g in range(m.ngeom) if m.geom_bodyid[g] == bid and (m.geom_contype[g] or m.geom_conaffinity[g])
            ]
            lo, hi = np.full(3, np.inf), np.full(3, -np.inf)
            for g in gids:
                R = d.geom_xmat[g].reshape(3, 3)
                c = d.geom_xpos[g] + R @ m.geom_aabb[g][:3]
                e = np.abs(R) @ m.geom_aabb[g][3:]
                lo, hi = np.minimum(lo, c - e), np.maximum(hi, c + e)
            q = d.xquat[bid]
            out[name] = {
                "pos": _l(d.xpos[bid]),
                "quat": _l([q[1], q[2], q[3], q[0]]),
                "box_min": _l(lo),
                "box_max": _l(hi),
            }
        return {"objects": out}

    # ---- scoring (called once, when the episode ends) ---------------------------------------------------------------
    def quality(self) -> dict:
        """Task quality in [0, 1] as RLE-Bench scores it at `finish()`: TowerMaxHeight and CantileverOverhang settle the
        scene (2500 physics steps with the robot frozen) and measure it; BalanceCoins is 0 unless exactly the heavy cube
        is on the answer mat, then 1 at up to ceil(log3 9) = 2 weighings and 2/w beyond."""
        q = float(self.env._trial_score())
        if not np.isfinite(q):
            raise RuntimeError("nonfinite quality")
        detail: dict = {"quality": round(max(0.0, min(1.0, q)), 6)}
        if self.task == "BalanceCoins":
            detail["weighings"] = int(self.env._weighings.count)
            detail["heavy_cube_alone_on_mat"] = bool(self.env._check_success())
        return detail

    def answer_correct(self, answer: str) -> bool:
        return self.mode == "hidden_com" and answer == self.quadrant

    # ---- rendering -----------------------------------------------------------------------------------------------
    def render(self, camera: str, width: int = RESOLUTION, height: int = RESOLUTION) -> np.ndarray:
        """RGB from `camera`, rendered at 512 x 512 as in RLE-Bench and resized for delivery (box filter)."""
        if camera not in self._cache:
            rgb = self.env.sim.render(camera_name=camera, width=RESOLUTION, height=RESOLUTION)
            self._cache[camera] = np.asarray(rgb, dtype=np.uint8)[::-1].copy()
        img = self._cache[camera]
        if (height, width) != img.shape[:2]:
            from PIL import Image

            img = np.asarray(Image.fromarray(img).resize((width, height), Image.Resampling.BOX))
        return np.ascontiguousarray(img, dtype=np.uint8)

    def camera(self, name: str) -> dict:
        m = self.env.sim.model
        i = m.camera_name2id(name)
        return {"fovy": float(m.cam_fovy[i])}

    def close(self) -> None:
        if self.env is not None:
            try:
                self.env.close()
            except Exception:
                pass
