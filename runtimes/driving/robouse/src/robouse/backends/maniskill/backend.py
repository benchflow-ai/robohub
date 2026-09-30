"""ManiSkill3 backend (Hao Su Lab / Hillbot, SAPIEN 3): ManiSkill3's tabletop manipulation tasks with a Franka Panda, on
ManiSkill's CPU physics backend (sim_backend="cpu": one environment, PhysX on the CPU) and SAPIEN's Vulkan renderer on
whatever Vulkan device exists (MoltenVK on macOS, Mesa's lavapipe software rasteriser in the Linux image; no GPU).

ManiSkill needs torch, SAPIEN and its own pins, so the simulation runs in its own virtualenv (default
~/.cache/robouse/maniskill-venv, override with ROBOUSE_MANISKILL_PYTHON) as a subprocess (worker.py). This
module is the bridge: it starts the worker, forwards steps, observations, renders and the success check.
Setup: docs/suites/maniskill.md.

Action [DX, DY, DZ, DROLL, DPITCH, DYAW, GRIP] (each in [-1, 1]), world frame: DX/DY/DZ move the gripper's commanded
position by 2 cm per unit per step; DROLL/DPITCH/DYAW turn its commanded orientation by 0.1 rad per unit per step about
the world x/y/z axes (centred on the fingertip point); GRIP > 0 closes the fingers, -f opens them to fraction f, 0 keeps
them. The worker solves inverse kinematics for the commanded pose and passes the joint targets to the upstream env's
`pd_joint_pos` controller; one step = 50 ms (ManiSkill's 20 Hz control rate). A 4-number action [DX, DY, DZ, GRIP] (what
`robo move-to` and `robo grip` send) keeps the orientation.
Observation: the hand pose, gripper opening, and the task's objects and goals (the same quantities ManiSkill's own state
observation gives its policies). Success is ManiSkill's own `evaluate()["success"]`, judged after `robo done` and a
10-step hold.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

import numpy as np

from ... import config
from ...workers.client import StdioWorker
from ..base import ActionSpec, Backend, StepInfo

WORKER = Path(__file__).with_name("worker.py")
MOLTENVK_ICD = (
    "moltenvk",
    "MoltenVK",
    "MoltenVK",
    "dynamic",
    "dylib",
    "macOS",
    "MoltenVK_icd.json",
)  # under the cache dir

# task id -> ManiSkill env id, seed (object layout), step budget
TASKS: dict[str, dict] = {
    "maniskill-pick-cube": dict(env="PickCube-v1", seed=1, max_steps=300),
    "maniskill-stack-cube": dict(env="StackCube-v1", seed=1, max_steps=400),
    "maniskill-push-cube": dict(env="PushCube-v1", seed=1, max_steps=300),
    "maniskill-pull-cube": dict(env="PullCube-v1", seed=1, max_steps=300),
    "maniskill-pull-cube-tool": dict(env="PullCubeTool-v1", seed=1, max_steps=500),
    "maniskill-poke-cube": dict(env="PokeCube-v1", seed=1, max_steps=500),
    "maniskill-place-sphere": dict(env="PlaceSphere-v1", seed=1, max_steps=300),
    "maniskill-roll-ball": dict(env="RollBall-v1", seed=1, max_steps=300, success_mode="first"),
    "maniskill-lift-peg-upright": dict(env="LiftPegUpright-v1", seed=1, max_steps=500),
    "maniskill-peg-insertion-side": dict(env="PegInsertionSide-v1", seed=1, max_steps=600),
    "maniskill-plug-charger": dict(env="PlugCharger-v1", seed=1, max_steps=600),
    "maniskill-stack-pyramid": dict(env="StackPyramid-v1", seed=1, max_steps=800),
}


class ManiSkillBackend(Backend):
    name = "maniskill"
    image_flipped = False

    def __init__(self, spec: dict):
        tid = spec.get("env") or spec.get("id")
        if tid not in TASKS:
            raise KeyError(f"unknown maniskill task {tid!r}")
        self.task_id = tid
        self.task = TASKS[tid]
        self.max_steps = int(spec.get("max_steps", self.task["max_steps"]))
        self.camera = spec.get("camera", "render_camera")
        self.render_size = int(spec.get("render_size", 384))
        self._grip = 0.0  # `robo move-to` without --grip keeps the fingers as they are
        self.action_spec = ActionSpec(
            names=["DX", "DY", "DZ", "DROLL", "DPITCH", "DYAW", "GRIP"],
            low=[-1.0] * 7,
            high=[1.0] * 7,
            doc="DX/DY/DZ move the gripper by 2 cm per unit per step along world x/y/z; DROLL/DPITCH/DYAW turn it by 0.1 rad "
            "per unit per step about world x/y/z; GRIP > 0 closes, -f opens to fraction f (-1 fully open), 0 keeps the fingers. "
            "One step = 50 ms.",
        )
        env = dict(os.environ)
        icd = config.cache_dir().joinpath(*MOLTENVK_ICD)
        if sys.platform == "darwin" and "VK_ICD_FILENAMES" not in env and icd.exists():
            env["VK_ICD_FILENAMES"] = str(icd)  # SAPIEN renders through MoltenVK on macOS
        env.setdefault("MVK_CONFIG_LOG_LEVEL", "0")
        self.worker = StdioWorker(
            "maniskill", config.sim_python("maniskill", "maniskill"), WORKER, setup="docs/suites/maniskill.md", env=env
        )
        self._state = None
        self._success = False
        self.last_judge = None

    def _call(self, cmd: str, **kw):
        return self.worker.call(cmd, **kw)

    def reset(self, seed: int) -> None:
        self.meta = self._call("make", env_id=self.task["env"], seed=int(seed), render_size=self.render_size)
        self._state = None

    def step(self, action) -> StepInfo:
        a = np.asarray(action, dtype=float).ravel()
        if a.size == 4:  # [DX, DY, DZ, GRIP] from move_to / grip: keep the orientation
            a = np.r_[a[:3], 0.0, 0.0, 0.0, a[3]]
        a = self.action_spec.clip(a)
        r = self._call("step", action=[float(x) for x in a])
        self._state = None
        self._success = bool(r["success"])
        return StepInfo(success=self._success)

    def observe(self) -> dict:
        if self._state is None:
            self._state = self._call("observe")
        return dict(self._state)

    def hand_pos(self) -> np.ndarray:
        return np.asarray(self.observe()["hand_pos"], dtype=float)

    def skills(self) -> list[str]:
        return ["move_to", "grip"]

    def success(self) -> bool:
        detail = self._call("success")
        self.last_judge = detail
        return bool(detail["success"])

    def render(self, width: int | None = None, height: int | None = None) -> np.ndarray:
        w = int(width or self.render_size)
        h = int(height or self.render_size)
        return self._call("render", camera=self.camera, width=w, height=h)

    def close(self) -> None:
        self.worker.close()


def oracle_main(env: str) -> None:
    from .oracles import run

    run(env)
