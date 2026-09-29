"""RoboCasa backend: household kitchen scenes (RoboCasa365 on robosuite 1.5) with a Franka Panda on a mobile base.

RoboCasa pins MuJoCo 3.3.1, numpy 2.2 and robosuite 1.5, which conflict with the main robouse environment (robosuite 1.4
for LIBERO), so the simulation runs in its own virtualenv (default ~/.cache/robouse/robocasa-venv, override with
ROBOUSE_ROBOCASA_PYTHON) as a subprocess (robocasa_worker.py). This module is the bridge: it starts the worker, forwards
steps, observations, renders and the success check. Setup: docs/suites/robocasa.md.

Robot: RoboCasa's default PandaOmron (a Franka Panda arm on an Omron mobile base with a torso lift). The base and torso
stay where RoboCasa's task initialisation puts them (facing the target fixture); only the arm and gripper move.

Action [DX, DY, DZ, GRIP] (each in [-1, 1]): DX/DY/DZ move the gripper by 2 cm per unit per step along the WORLD axes
(converted to the base frame for robosuite's OSC_POSE controller, orientation held); one step = 50 ms (20 Hz, RoboCasa's
control rate). GRIP: 0 keeps the fingers, > 0 closes, -f opens to fraction f (-1 fully open).
Observation: hand_pos (end-effector, world m), gripper_open (0..1), the robot base position and forward direction, and
task-specific fields (object and fixture handle/button positions, fixture state). Success is RoboCasa's own
`_check_success()` for the task, judged after `robo done` and a 10-step hold.
"""
from __future__ import annotations

import json
import os
import subprocess
from pathlib import Path

import numpy as np

from .base import ActionSpec, Backend, StepInfo

WORKER = Path(__file__).with_name("robocasa_worker.py")
EPISODES = Path(__file__).with_name("robocasa_episodes")
DEFAULT_PY = Path.home() / ".cache" / "robouse" / "robocasa-venv" / "bin" / "python"

# RoboCasa env -> task-specific observation features and step budget
KINDS: dict[str, dict] = {
    "CloseDrawer": dict(features=["drawer"], max_steps=400),
    "PickPlaceCounterToSink": dict(features=["object", "sink"], max_steps=800),
    "PickPlaceCounterToCabinet": dict(features=["object", "cabinet"], max_steps=800),
    "TurnOnToaster": dict(features=["toaster"], max_steps=300),
    "CloseElectricKettleLid": dict(features=["kettle"], max_steps=400),
    "TurnOffSinkFaucet": dict(features=["faucet"], max_steps=400),
    "TurnOnStove": dict(features=["stove"], max_steps=500),
    "TurnOffStove": dict(features=["stove"], max_steps=500),
}


def _t(env: str, layout: int, seed: int, style: int | None = None) -> dict:
    return dict(env=env, layout=layout, style=layout if style is None else style, seed=seed, **KINDS[env])


# task id -> RoboCasa env, kitchen layout/style (styles 1-10 need no AI-generated textures), seed of the recorded layout
# (robocasa_episodes/<task id>.json is what every reset replays)
TASKS: dict[str, dict] = {
    "robocasa-close-drawer-kitchen2": _t("CloseDrawer", 2, 0),
    "robocasa-close-drawer-kitchen7": _t("CloseDrawer", 7, 1),
    "robocasa-close-drawer-kitchen10": _t("CloseDrawer", 10, 4),
    "robocasa-pick-place-counter-to-sink": _t("PickPlaceCounterToSink", 1, 2),
    "robocasa-pick-place-counter-to-sink-kitchen2": _t("PickPlaceCounterToSink", 2, 0),
    "robocasa-pick-place-counter-to-sink-kitchen2b": _t("PickPlaceCounterToSink", 2, 2),
    "robocasa-pick-place-counter-to-sink-kitchen3": _t("PickPlaceCounterToSink", 3, 6),
    "robocasa-pick-place-counter-to-sink-kitchen4": _t("PickPlaceCounterToSink", 4, 0),
    "robocasa-pick-place-counter-to-sink-kitchen5": _t("PickPlaceCounterToSink", 5, 0),
    "robocasa-pick-place-counter-to-sink-kitchen6": _t("PickPlaceCounterToSink", 6, 0),
    "robocasa-pick-place-counter-to-cabinet-kitchen3": _t("PickPlaceCounterToCabinet", 3, 1),
    "robocasa-pick-place-counter-to-cabinet-kitchen8": _t("PickPlaceCounterToCabinet", 8, 1),
    "robocasa-pick-place-counter-to-cabinet-kitchen9": _t("PickPlaceCounterToCabinet", 9, 0),
    "robocasa-turn-on-toaster-kitchen7": _t("TurnOnToaster", 7, 0),
    "robocasa-turn-on-toaster-kitchen8": _t("TurnOnToaster", 8, 0),
    "robocasa-close-kettle-lid-kitchen1": _t("CloseElectricKettleLid", 1, 0),
    "robocasa-close-kettle-lid-kitchen6": _t("CloseElectricKettleLid", 6, 0),
    "robocasa-close-kettle-lid-kitchen10": _t("CloseElectricKettleLid", 10, 0),
    "robocasa-turn-off-faucet-kitchen2": _t("TurnOffSinkFaucet", 2, 0),
    "robocasa-turn-off-faucet-kitchen9": _t("TurnOffSinkFaucet", 9, 0),
    "robocasa-turn-on-stove-kitchen2": _t("TurnOnStove", 2, 1),
    "robocasa-turn-on-stove-kitchen7": _t("TurnOnStove", 7, 1),
    "robocasa-turn-off-stove-kitchen2": _t("TurnOffStove", 2, 0),
    "robocasa-turn-off-stove-kitchen9": _t("TurnOffStove", 9, 0),
}


class RobocasaBackend(Backend):
    name = "robocasa"
    image_flipped = False

    def __init__(self, spec: dict):
        env = spec.get("env") or spec.get("id")
        if env not in TASKS:
            raise KeyError(f"unknown robocasa task {env!r}")
        self.task_id = env
        self.task = TASKS[env]
        self.max_steps = int(spec.get("max_steps", self.task.get("max_steps", 400)))
        self.camera = spec.get("camera", "robot0_agentview_center")
        self.render_size = tuple(spec.get("render_size", (384, 288)))
        self._grip = 0.0
        self.action_spec = ActionSpec(
            names=["DX", "DY", "DZ", "GRIP"], low=[-1.0] * 4, high=[1.0] * 4,
            doc="DX/DY/DZ move the gripper by 2 cm per unit per step along world x/y/z; GRIP: 0 keeps the fingers, > 0 closes, "
                "-f opens to fraction f (-1 fully open). One step = 50 ms.")
        py = os.environ.get("ROBOUSE_ROBOCASA_PYTHON", str(DEFAULT_PY))
        if not Path(py).exists():
            raise RuntimeError(f"RoboCasa virtualenv not found at {py}; see docs/suites/robocasa.md")
        # PYTHONHASHSEED: RoboCasa iterates over sets of names when placing objects; fix the order so a seed is reproducible
        self.proc = subprocess.Popen([py, str(WORKER)], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                     env={**os.environ, "PYTHONHASHSEED": "0"},
                                     stderr=subprocess.DEVNULL if os.environ.get("ROBOUSE_ROBOCASA_QUIET", "1") == "1" else None)
        self._state = None
        self._success = False

    def _call(self, cmd: str, **kw):
        self.proc.stdin.write((json.dumps({"cmd": cmd, **kw}) + "\n").encode())
        self.proc.stdin.flush()
        line = self.proc.stdout.readline()
        if not line:
            raise RuntimeError("RoboCasa worker exited")
        hdr = json.loads(line)
        if not hdr.get("ok"):
            raise RuntimeError(hdr.get("error"))
        if cmd == "render":
            data = self.proc.stdout.read(hdr["nbytes"])
            return np.frombuffer(data, dtype=np.uint8).reshape(kw["height"], kw["width"], 3)
        return hdr["result"]

    def reset(self, seed: int) -> None:
        t = self.task
        # RoboCasa's object placement is not reproducible from the seed alone (it iterates over sets of objects), so every
        # task replays a recorded episode layout (RoboCasa's ep_meta, the mechanism its demo replay uses).
        ep = EPISODES / f"{self.task_id}.json"
        ep_meta = json.loads(ep.read_text()) if ep.exists() else None
        seed = int((ep_meta or {}).get("robouse_seed", t.get("seed", seed)))
        self.meta = self._call("make", env_name=t["env"], layout=t["layout"], style=t["style"], seed=seed,
                               tool=t.get("tool", "down"), ep_meta=ep_meta)
        self._state = None

    def step(self, action) -> StepInfo:
        r = self._call("step", action=[float(x) for x in np.asarray(action, dtype=float)[:4]])
        self._state = None
        self._success = bool(r["success"])
        return StepInfo(success=self._success)

    def observe(self) -> dict:
        if self._state is None:
            st = self._call("observe", features=self.task["features"])
            st["task_instruction"] = self.meta.get("lang", "")
            self._state = st
        return dict(self._state)

    def hand_pos(self) -> np.ndarray:
        return np.asarray(self.observe()["hand_pos"], dtype=float)

    def skills(self) -> list[str]:
        return ["move_to", "grip"]

    def success(self) -> bool:
        return bool(self._call("success")["success"])

    def render(self, width: int | None = None, height: int | None = None) -> np.ndarray:
        w, h = (width, height) if width and height else self.render_size
        return self._call("render", camera=self.camera, width=int(w), height=int(h))

    def close(self) -> None:
        try:
            self.proc.stdin.write(b'{"cmd": "close"}\n')
            self.proc.stdin.flush()
            self.proc.wait(timeout=10)
        except Exception:
            self.proc.kill()


def oracle_main(env: str) -> None:
    from .robocasa_oracles import run

    run(env)


def record_episode(task_id: str, seed: int | None = None) -> Path:
    """Create the task's scene once and store its ep_meta so every later reset replays the same layout."""
    b = RobocasaBackend({"env": task_id})
    t = b.task
    seed = int(t["seed"] if seed is None else seed)
    b._call("make", env_name=t["env"], layout=t["layout"], style=t["style"], seed=seed, tool=t.get("tool", "down"))
    meta = b._call("ep_meta")
    meta["robouse_seed"] = seed
    b.close()
    EPISODES.mkdir(exist_ok=True)
    out = EPISODES / f"{task_id}.json"
    out.write_text(json.dumps(meta, indent=1))
    return out
