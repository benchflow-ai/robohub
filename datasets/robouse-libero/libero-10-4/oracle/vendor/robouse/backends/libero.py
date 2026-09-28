"""LIBERO (Liu et al. 2023) backend: a Franka Panda in LIBERO's MuJoCo / robosuite 1.4 scenes.

Uses the `hf-libero` package (the one LeRobot's LIBERO evaluation uses) with `robosuite==1.4.0`. A task is
chosen by suite (`libero_spatial`, `libero_object`, `libero_goal`, `libero_10`) and task index or name; the
scene comes from that task's BDDL file.

Initial state: `init_state` in the task spec (a flat MuJoCo state vector, e.g. the first state of a LIBERO
demonstration) if given, otherwise LIBERO's fixed benchmark init state number `seed`. After loading it the
robot holds still for `settle_steps` (default 50, as in LeRobot's MolmoAct2 evaluation) so objects come to
rest; these steps are not counted against the task's budget.

Action (LIBERO's own, 7-D, each in [-1, 1]): [dx, dy, dz, droll, dpitch, dyaw, gripper], a delta end-effector
pose for robosuite's OSC_POSE controller at 20 Hz. 1.0 on dx/dy/dz asks for 5 cm of motion in one step and
1.0 on droll/dpitch/dyaw for 0.5 rad (the arm lags behind large requests); gripper -1 opens, +1 closes.
The move_to/grip skills send 4-D [dx, dy, dz, grip] actions, which are padded with zero rotation.

Observation: end-effector position, quaternion (x, y, z, w) and axis-angle, finger joint positions, and the
position and quaternion of each object in LIBERO's observation. Images: `agentview` (front) and
`robot0_eye_in_hand` (wrist) cameras at 256x256, returned upright (robosuite renders them upside down).

Success is LIBERO's own `check_success()` (the task's BDDL goal predicates).
"""
from __future__ import annotations

import os

import numpy as np

from .base import ActionSpec, Backend, StepInfo

SUITES = ("libero_spatial", "libero_object", "libero_goal", "libero_10", "libero_90")
CAMERAS = ("agentview", "robot0_eye_in_hand")
# LeRobot's per-suite step budgets (lerobot/envs/libero.py TASK_SUITE_MAX_STEPS)
SUITE_MAX_STEPS = {"libero_spatial": 280, "libero_object": 280, "libero_goal": 300, "libero_10": 520, "libero_90": 400}
DUMMY = [0, 0, 0, 0, 0, 0, -1]


def _quiet_imports():
    """Import LIBERO without its first-run prompt: point it at a config file (created here if missing)."""
    os.environ.setdefault("MUJOCO_GL", "cgl" if os.uname().sysname == "Darwin" else "egl")
    cfg_dir = os.environ.get("LIBERO_CONFIG_PATH", os.path.expanduser("~/.libero"))
    cfg = os.path.join(cfg_dir, "config.yaml")
    if not os.path.exists(cfg):
        import importlib.util

        spec = importlib.util.find_spec("libero")
        if spec is None or not spec.submodule_search_locations:
            raise ImportError("LIBERO is not installed: pip install --no-deps hf-libero==0.1.4 robosuite==1.4.0 bddl==1.0.1")
        root = os.path.join(list(spec.submodule_search_locations)[0], "libero")
        os.makedirs(cfg_dir, exist_ok=True)
        with open(cfg, "w") as f:
            f.write("".join(f"{k}: {os.path.join(root, v)}\n" for k, v in (
                ("benchmark_root", ""), ("bddl_files", "bddl_files"), ("init_states", "init_files"),
                ("datasets", "../datasets"), ("assets", "assets"))))
    import logging

    logging.getLogger("robosuite").setLevel(logging.ERROR)
    from libero.libero import benchmark, get_libero_path
    from libero.libero.envs import OffScreenRenderEnv

    return benchmark, get_libero_path, OffScreenRenderEnv


def libero_task(suite: str, task: int | str):
    """(task index, LIBERO Task namedtuple, benchmark suite object) for a suite and a task index or name."""
    benchmark, _, _ = _quiet_imports()
    s = benchmark.get_benchmark_dict()[suite]()
    names = [s.get_task(i).name for i in range(s.get_num_tasks())]
    i = names.index(task) if isinstance(task, str) and not str(task).isdigit() else int(task)
    return i, s.get_task(i), s


def quat2axisangle(q) -> np.ndarray:
    """(x, y, z, w) quaternion -> axis-angle, as in robosuite / LeRobot's LIBERO processor."""
    q = np.asarray(q, dtype=np.float64)
    w = float(np.clip(q[3], -1.0, 1.0))
    den = np.sqrt(max(1.0 - w * w, 0.0))
    if np.isclose(den, 0.0):
        return np.zeros(3)
    return q[:3] * 2.0 * np.arccos(w) / den


class LiberoBackend(Backend):
    name = "libero"
    image_flipped = False  # render() returns upright images, as mujoco.Renderer would

    def __init__(self, spec: dict):
        _, get_libero_path, OffScreenRenderEnv = _quiet_imports()
        self.suite = str(spec.get("suite", "libero_spatial"))
        self.task_index, self.task, self._suite_obj = libero_task(self.suite, spec.get("env", spec.get("task_index", 0)))
        self.language = self.task.language
        self.max_steps = int(spec.get("max_steps", SUITE_MAX_STEPS.get(self.suite, 300)))
        self.camera = str(spec.get("camera", "agentview"))
        self.size = int(spec.get("image_size", 256))
        self.settle_steps = int(spec.get("settle_steps", 50))
        self._init_state = np.asarray(spec["init_state"], dtype=np.float64) if spec.get("init_state") is not None else None
        bddl = os.path.join(get_libero_path("bddl_files"), self.task.problem_folder, self.task.bddl_file)
        self.env = OffScreenRenderEnv(bddl_file_name=bddl, camera_heights=self.size, camera_widths=self.size,
                                      camera_names=list(CAMERAS))
        self.action_spec = ActionSpec(
            names=["dx", "dy", "dz", "droll", "dpitch", "dyaw", "gripper"], low=[-1.0] * 7, high=[1.0] * 7,
            doc="LIBERO delta end-effector pose (OSC_POSE, 20 Hz): 1.0 on dx/dy/dz asks for 5 cm per step, "
                "1.0 on droll/dpitch/dyaw for 0.5 rad; gripper -1 open, +1 close",
        )
        self._obs: dict = {}
        self._success = False
        self._grip = -1.0

    # ---- episode ------------------------------------------------------------------------------
    def reset(self, seed: int) -> None:
        self.env.seed(int(seed))
        self.env.reset()
        state = self._init_state if self._init_state is not None else \
            self._suite_obj.get_task_init_states(self.task_index)[int(seed) % 50]
        self._obs = self.env.set_init_state(state)
        for _ in range(self.settle_steps):
            self._obs, _, _, _ = self.env.step(DUMMY)
        self._success = bool(self.env.check_success())
        self._grip = -1.0

    def step(self, action) -> StepInfo:
        a = np.asarray(action, dtype=np.float64).ravel()
        if a.size == 4:  # skills (move_to / grip / settle) send [dx, dy, dz, grip]
            a = np.concatenate([a[:3], np.zeros(3), a[3:]])
        a = self.action_spec.clip(a)
        self._grip = float(a[6])
        self._obs, r, _done, _info = self.env.step(a)
        self._success = bool(self.env.check_success())
        return StepInfo(success=self._success, reward=float(r))

    def success(self) -> bool:
        return self._success

    # ---- observation --------------------------------------------------------------------------
    def object_names(self) -> list[str]:
        return sorted({k[:-4] for k in self._obs if k.endswith("_pos") and not k.startswith("robot0")
                       and "_to_robot0" not in k})

    def observe(self) -> dict:
        o = self._obs
        r = lambda v, n=5: [round(float(x), n) for x in np.asarray(v).ravel()]
        eef_pos, eef_quat = o["robot0_eef_pos"], o["robot0_eef_quat"]
        fingers = np.asarray(o["robot0_gripper_qpos"], dtype=float)
        return {
            "hand_pos": r(eef_pos),
            "eef_quat": r(eef_quat),
            "eef_axis_angle": r(quat2axisangle(eef_quat)),
            "gripper_qpos": r(fingers),
            # finger gap: about 0.08 fully open, about 0 closed on nothing
            "gripper_open": round(float(fingers[0] - fingers[1]), 4),
            "objects": {n: {"pos": r(o[f"{n}_pos"], 4), "quat": r(o[f"{n}_quat"], 4)} for n in self.object_names()},
        }

    def render(self, width: int = 320, height: int = 320) -> np.ndarray:
        key = f"{self.camera}_image"
        img = self._obs.get(key)
        if img is None:  # a camera that is not in LIBERO's observation: render it directly
            img = self.env.sim.render(camera_name=self.camera, width=self.size, height=self.size)
        return np.ascontiguousarray(np.asarray(img)[::-1])  # robosuite images are upside down

    def mj_model_data(self):
        return self.env.sim.model._model, self.env.sim.data._data

    # ---- skills -------------------------------------------------------------------------------
    def skills(self) -> list[str]:
        return ["move_to", "grip"]

    def hand_pos(self) -> np.ndarray:
        return np.asarray(self._obs["robot0_eef_pos"], dtype=float)

    def close(self) -> None:
        try:
            self.env.close()
        except Exception:
            pass


def oracle_main(env: str) -> None:
    """Reference solution: replay one LIBERO demonstration's recorded actions (`env` = path to a JSON file with
    {"actions": [[7 floats], ...]}) through the robo socket. The task's init_state is that demonstration's first
    state, so the open-loop replay reproduces the demonstration."""
    import json

    from ..agent_cli import _send

    actions = json.load(open(env))["actions"]
    for a in actions:
        r = _send({"op": "act", "action": [float(x) for x in a], "repeat": 1})
        if not r.get("ok") or "episode" in (r.get("result") or {}):
            return  # finished (goal reached under success_mode "first", or budget)
    # hold still a little longer in case the last placement needs to settle
    for _ in range(20):
        r = _send({"op": "act", "action": [0, 0, 0, 0, 0, 0, float(actions[-1][6])], "repeat": 1})
        if not r.get("ok") or "episode" in (r.get("result") or {}):
            return
    _send({"op": "done", "text": "demonstration replayed"})
