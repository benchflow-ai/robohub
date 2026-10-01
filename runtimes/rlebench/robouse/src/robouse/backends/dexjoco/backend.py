"""DexJoCo backend: dexterous tool use with Franka Panda arms and 16-joint Allegro hands (DexJoCo, MuJoCo, MIT licence,
github.com/brave-eai/dexjoco, arXiv 2605.16257, revised 2026-09).

DexJoCo pins Python 3.11, MuJoCo 3.4.0 and numpy 1.26, so the simulation runs in its own virtualenv (default
~/.cache/robouse/dexjoco-venv, override with ROBOUSE_DEXJOCO_PYTHON) as a subprocess (worker.py). This module is
the bridge: it starts the worker, forwards steps, observations, renders and the success check. Setup: docs/suites/dexjoco.md.

Each robouse task is one DexJoCo task started from the initial scene of one recorded human demonstration (object poses and
table height restored with DexJoCo's own state restorers), so the reference solution can replay that demonstration.

Action per arm (22 numbers; limits +-8 for DX DY DZ, +-3 for RX RY RZ, +-6 for the fingers): DX DY DZ move the commanded hand pose by 2 cm per unit along world x, y, z;
RX RY RZ rotate it by 0.2 rad per unit about the world x, y, z axes; F0..F15 move the 16 commanded Allegro joint angles by
0.3 rad per unit (index, middle, ring, thumb; 4 joints each). Bimanual tasks take 44 numbers: right arm, then left arm.
One step = 20 ms. The skills (move_to, grip) drive the active arm; grip closes the hand to a generic power grasp.

Success is DexJoCo's own success signal for the task (most need the goal to hold for 10-50 consecutive steps); the episode
ends as solved the first time it fires (success_mode: first). Some tasks also have a failure event (spraying away from
the plant, pressing a wrong passcode digit) after which the episode can no longer succeed.
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from ... import config
from ...workers.client import StdioWorker
from ..base import ActionSpec, Backend, G, StepInfo

WORKER = Path(__file__).with_name("worker.py")
POS_STEP, ROT_STEP, HAND_STEP = 0.02, 0.2, 0.3
POS_LIM, ROT_LIM, HAND_LIM = 8.0, 3.0, 6.0  # per-step limits in action units: 16 cm, 0.6 rad, 1.8 rad
FINGER_NAMES = [f"{f}{j}" for f in ("index", "middle", "ring", "thumb") for j in range(4)]


class DexjocoBackend(Backend):
    name = "dexjoco"
    probe_success = False  # step() already reports DexJoCo's success signal

    @property
    def robot_name(self) -> str:
        return "bimanual-panda-allegro" if self.bimanual else "panda-allegro"

    @property
    def robot_kind(self) -> str:
        return "bimanual" if self.bimanual else "hand"

    @property
    def proprioception(self) -> tuple[str, ...]:
        if self.bimanual:
            return tuple(f"{a}_{f}" for a in ("right", "left") for f in ("hand_pos", "finger_joints"))
        return ("hand_pos", "finger_joints")

    def action_layout(self) -> list[G]:
        lay = []
        for arm in ["right", "left"] if self.bimanual else ["arm"]:
            hand = f"{arm}.hand" if self.bimanual else "hand"
            lay += [
                G(f"{arm}.ee_delta", 3, "ee_delta_pos", "2 cm per unit (commanded pose)", frame="world"),
                G(f"{arm}.ee_rot_delta", 3, "ee_delta_rot", "0.2 rad per unit (commanded pose)", frame="world"),
                G(
                    f"{hand}.joints",
                    16,
                    "joint_delta",
                    "0.3 rad per unit",
                    doc="Allegro joint targets: index 0-3, middle 4-7, ring 8-11, thumb 12-15",
                ),
            ]
        return lay

    image_flipped = False

    def __init__(self, spec: dict):
        self.task = spec["env"]
        self.init_state = spec.get("init_state")
        self.max_steps = int(spec.get("max_steps", 1000))
        self.bimanual = self.task.startswith("bimanual_")
        self.camera = spec.get("camera", "back" if self.bimanual else "front")
        self.render_size = tuple(spec.get("render_size", (400, 300)))
        arms = ["R_", "L_"] if self.bimanual else [""]
        names = []
        for a in arms:
            names += [f"{a}DX", f"{a}DY", f"{a}DZ", f"{a}RX", f"{a}RY", f"{a}RZ"] + [f"{a}F{i}" for i in range(16)]
        doc = (
            "Per arm: DX DY DZ move the commanded hand pose 2 cm per unit along world x/y/z; RX RY RZ rotate it 0.2 rad per unit "
            "about world x/y/z; F0..F15 move the commanded Allegro joint angles 0.3 rad per unit (index 0-3, middle 4-7, "
            "ring 8-11, thumb 12-15). One step = 20 ms."
            + (" Right arm first (R_), then left arm (L_)." if self.bimanual else "")
        )
        lim = ([POS_LIM] * 3 + [ROT_LIM] * 3 + [HAND_LIM] * 16) * len(arms)
        self.action_spec = ActionSpec(names=names, low=[-x for x in lim], high=lim, doc=doc)
        self.worker = StdioWorker("dexjoco", config.sim_python("dexjoco", "dexjoco"), WORKER, setup=config.SUITES_DOCS)
        self._state = None
        self._success = False
        self._grip = 0.0  # skills and the settle hold send 0 = keep the hand as commanded

    def _call(self, cmd: str, **kw):
        return self.worker.call(cmd, **kw)

    def reset(self, seed: int) -> None:
        self.meta = self._call("make", task=self.task, init_state=self.init_state, seed=0)
        self._state = None

    def step(self, action) -> StepInfo:
        r = self._call("step", action=[float(x) for x in np.asarray(action, dtype=float)])
        self._state = None
        self._success = bool(r["success"])
        return StepInfo(success=self._success, extra={"failed": r.get("failed", "")})

    def observe(self) -> dict:
        if self._state is None:
            self._state = self._call("observe")
        return dict(self._state)

    def hand_pos(self) -> np.ndarray:
        st = self.observe()
        if self.bimanual:
            return np.asarray(st[f"{st.get('active_arm', 'right')}_hand_pos"], dtype=float)
        return np.asarray(st["hand_pos"], dtype=float)

    def skills(self) -> list[str]:
        return ["move_to", "grip"]

    def success(self) -> bool:
        return bool(self._call("success")["success"])

    def render(self, width: int | None = None, height: int | None = None) -> np.ndarray:
        w, h = (width, height) if width and height else self.render_size
        return self._call("render", camera=self.camera, width=int(w), height=int(h))

    def close(self) -> None:
        self.worker.close()


def _q2R(q):
    from scipy.spatial.transform import Rotation as R

    return R.from_quat([q[1], q[2], q[3], q[0]])


def demo_to_delta(demo_row: np.ndarray, st: dict, arm_prefix: str) -> list[float]:
    """One arm's 22-number delta action that moves the commanded pose and fingers onto a recorded absolute target."""
    pos, quat, hand = demo_row[:3], demo_row[3:7], demo_row[7:23]
    tp = np.asarray(st[arm_prefix + "hand_target_pos"], dtype=float)
    tq = np.asarray(st[arm_prefix + "hand_target_quat_wxyz"], dtype=float)
    th = np.asarray(st[arm_prefix + "finger_joint_targets"], dtype=float)
    if np.allclose(pos, 0) and np.allclose(quat, 0):  # DexJoCo convention: an all-zero pose keeps the arm where it is
        dpos, drot = np.zeros(3), np.zeros(3)
    else:
        dpos = (pos - tp) / POS_STEP
        drot = (_q2R(quat) * _q2R(tq).inv()).as_rotvec() / ROT_STEP
    dh = (hand - th) / HAND_STEP
    lim = np.array([POS_LIM] * 3 + [ROT_LIM] * 3 + [HAND_LIM] * 16)
    return [float(x) for x in np.clip(np.concatenate([dpos, drot, dh]), -lim, lim)]


def oracle_main(env: str) -> None:
    """`env` is the path of the task's oracle/demo.json (recorded actions of one DexJoCo human demonstration)."""
    from ...agent_cli import _send

    demo = json.loads(Path(env).read_text())
    actions = np.asarray(demo["actions"], dtype=float)
    bimanual = actions.shape[1] == 46
    extra = int(demo.get("extra_steps", 120))
    rows = list(actions) + [actions[-1]] * extra
    for row in rows:
        r = _send({"op": "observe"})
        if not r.get("ok"):
            return
        st = r["result"]["state"]
        if bimanual:
            act = demo_to_delta(row[:23], st, "right_") + demo_to_delta(row[23:46], st, "left_")
        else:
            act = demo_to_delta(row, st, "")
        r = _send({"op": "act", "action": act, "repeat": 1})
        if not r.get("ok") or "episode" in (r.get("result") or {}):
            return
    _send({"op": "done", "text": "replayed the recorded demonstration"})
