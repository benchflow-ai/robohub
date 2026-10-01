"""RLE-Bench backend: tasks from RLE-Bench's interactive-control families (Harvard and Georgia Tech; MIT;
https://github.com/RLE-Bench/RLE-Bench), adapted to one Robo Use episode per task.

Family 03 (tabletop reasoning) runs RLE-Bench's own scene code (rlebench_upstream/, vendored at commit 34a73aa, MIT) in a
simulator worker (worker.py) inside RLE-Bench's pinned simulator stack (robosuite 5ce6643, MuJoCo 3.3.1):

- TowerMaxHeight, CantileverOverhang and BalanceCoins: a PandaOmron (Franka Panda on a mobile base with a torso lift)
  at a 0.7 x 1.6 m table, 12-number actions exactly as RLE-Bench's TabletopClient (arm pose deltas in the robot base
  frame, absolute gripper, base velocity, torso, mode), RGB cameras left/right/wrist at 512 x 512, and RLE-Bench's
  public proprioception. Skills: `move` (RLE-Bench's `move()`: track a world-frame tool pose for 1-200 steps) and
  `reset` (restore the same initial scene; costs one step).
- HiddenCOM: one of RLE-Bench's three sealed boxes per task (RLE-Bench runs the three in one attempt and scores the
  fraction correct; here each box is a task, so the mean over the three tasks is RLE-Bench's score). A fixed Panda,
  7-number actions, cameras workspace/closeup/top, and the skill `submit A|B|C|D` (final, no feedback).

Scoring. RLE-Bench scores family 03 as a quality in [0, 1] computed from the final scene. Robo Use rewards 1 or 0, so a
tabletop trial passes when RLE-Bench's quality is at least 0.5 (tower: half the analytic optimum height; cantilever:
half the harmonic-stack overhang, more than one block alone can reach; balance: exactly the heavy cube on the mat with
at most four weighings). A HiddenCOM box passes when the submitted quadrant is the ballast's. The quality, the rule and
(for the balance) the weighing count are recorded in result.json (`judge_detail`). As in RLE-Bench, a tabletop scene is
scored however the episode ends (`robo done`, the step budget, the time limit, or the agent stopping), except that
`robo give-up` scores 0.

Hidden physical conditions (the heavy cube, the ballast quadrant and masses, piece masses) never leave the worker: the
agent sees only RLE-Bench's public fields and camera images, and `robo info` gives each camera's intrinsics only, as
RLE-Bench does.
"""

from __future__ import annotations

import hashlib
from pathlib import Path

import numpy as np

from ... import config
from ...core.embodiment_spec import Skill, SkillArg
from ...workers.client import StdioWorker
from ..base import ActionSpec, Backend, G, StepInfo

WORKER = Path(__file__).with_name("worker.py")
PASS_QUALITY = 0.5
RESOLUTION = 512


def task01_eval_seed(task: str, scene: int = 1, trial: int = 0, salt: str = "") -> int:
    """RLE-Bench's evaluation seed (tasks/task01/harness/config.py eval_trials; task03 uses scene 1, trial 0)."""
    key = f"rlebench.task01.eval|{salt}|{task}|scene{scene}|trial{trial}"
    return int.from_bytes(hashlib.sha256(key.encode()).digest()[:8], "big") % (2**31 - 1)


TABLETOP_FIELDS = [
    "joint_pos",
    "joint_vel",
    "eef_pos",
    "eef_quat",
    "gripper_pos",
    "base_pos",
    "base_quat",
    "eef_base_pos",
    "eef_base_quat",
]
HIDDEN_COM_FIELDS = ["joint_pos", "joint_vel", "eef_pos", "eef_quat", "gripper_pos"]
HIDDEN_COM_SEEDS = (7007, 7018, 7009)  # rlebench_upstream/tabletop/hidden_com/config.py CASES

# task id -> RLE-Bench task, seed and budget (tabletop/budgets.py: 50,000 steps; HiddenCOM 12,000 per box)
TASKS: dict[str, dict] = {
    "rle-tower-max-height": dict(task="TowerMaxHeight", seed=task01_eval_seed("TowerMaxHeight"), max_steps=50_000),
    "rle-cantilever-overhang": dict(
        task="CantileverOverhang", seed=task01_eval_seed("CantileverOverhang"), max_steps=50_000
    ),
    "rle-balance-coins": dict(task="BalanceCoins", seed=task01_eval_seed("BalanceCoins"), max_steps=50_000),
    **{
        f"rle-hidden-com-box-{i + 1}": dict(task="HiddenCOM", seed=s, case=i, max_steps=12_000)
        for i, s in enumerate(HIDDEN_COM_SEEDS)
    },
}

TABLETOP_ACTIONS = [
    "ARM_DX",
    "ARM_DY",
    "ARM_DZ",
    "ARM_RX",
    "ARM_RY",
    "ARM_RZ",
    "GRIP",
    "BASE_FWD",
    "BASE_LAT",
    "BASE_YAW",
    "TORSO",
    "MODE",
]
HIDDEN_COM_ACTIONS = ["ARM_DX", "ARM_DY", "ARM_DZ", "ARM_RX", "ARM_RY", "ARM_RZ", "GRIP"]


class RLEBenchBackend(Backend):
    name = "rlebench"
    probe_success = False  # success is judged once, when the episode ends
    image_flipped = False  # the worker returns upright images

    def __init__(self, spec: dict):
        env = spec.get("env") or spec.get("id")
        if env not in TASKS:
            raise KeyError(f"unknown RLE-Bench task {env!r}")
        self.spec = spec
        self.task_id = env
        self.task = TASKS[env]
        self.kind = self.task["task"]
        self.hidden_com = self.kind == "HiddenCOM"
        self.max_steps = int(spec.get("max_steps", self.task["max_steps"]))
        self.camera = str(spec.get("camera", "top" if self.hidden_com else "robot0_agentview_left"))
        self.render_size = (RESOLUTION, RESOLUTION)
        self._grip = -1.0
        names = HIDDEN_COM_ACTIONS if self.hidden_com else TABLETOP_ACTIONS
        self.action_spec = ActionSpec(
            names=list(names),
            low=[-1.0] * len(names),
            high=[1.0] * len(names),
            doc=(
                "ARM_DX/DY/DZ: world-aligned end-effector translation, 1.0 = 5 cm; ARM_RX/RY/RZ: axis-angle rotation, "
                "1.0 = 0.5 rad; GRIP: +1 closes, -1 opens. 20 Hz."
                if self.hidden_com
                else "ARM_DX/DY/DZ: end-effector translation in the ROBOT BASE frame, 1.0 = 5 cm; ARM_RX/RY/RZ: axis-angle "
                "rotation in the base frame, 1.0 = 0.5 rad; GRIP: +1 closes, -1 opens (absolute, so 0 is half-way); "
                "BASE_FWD/LAT/YAW: base velocity; TORSO: torso lift velocity; MODE: -1 arm, +1 base. 20 Hz. "
                "Neutral hold: [0]*6 + [-1] + [0]*4 + [-1]."
            ),
        )
        self.robot_kind = "arm" if self.hidden_com else "mobile_manipulator"
        self.robot_name = "panda" if self.hidden_com else "panda-omron"
        self.worker = StdioWorker(
            "rlebench",
            config.sim_python("rlebench", "rlebench"),
            WORKER,
            setup="docs/suites/rle-bench.md",
            env={"PYTHONHASHSEED": "0"},
        )
        self._state: dict | None = None
        self.answer: str | None = None
        self.last_judge: dict | None = None

    # ---- embodiment ----------------------------------------------------------------------------------------------
    def action_layout(self) -> list[G]:
        lay = [
            G("arm.ee_delta", 3, "ee_delta_pos", "1.0 = 5 cm", frame="world" if self.hidden_com else "base"),
            G("arm.ee_rot_delta", 3, "ee_delta_rot", "1.0 = 0.5 rad", frame="world" if self.hidden_com else "base"),
            G("gripper", 1, "gripper", "normalized", hold="last", doc="+1 close, -1 open (absolute)"),
        ]
        if not self.hidden_com:
            lay += [
                G("base", 3, "other", doc="forward, lateral and yaw velocity"),
                G("torso", 1, "other", doc="torso lift velocity"),
                G("mode", 1, "other", hold="last", initial=-1.0, doc="-1 arm, +1 base"),
            ]
        return lay

    def skills(self) -> list[str]:
        return ["move", "submit"] if self.hidden_com else ["move", "reset"]

    def skill_specs(self) -> list[Skill]:
        move = Skill(
            "move",
            [
                SkillArg("x", units="m"),
                SkillArg("y", units="m"),
                SkillArg("z", units="m"),
                SkillArg("qx"),
                SkillArg("qy"),
                SkillArg("qz"),
                SkillArg("qw"),
                SkillArg("gripper", optional=True, default=1.0, doc="+1 closes, -1 opens"),
                SkillArg("steps", "int", optional=True, default=40, doc="1-200 control steps"),
            ],
            doc="Track a world-frame tool pose (position, unit quaternion x y z w) for `steps` control steps, each "
            "counted; the base and torso hold still. RLE-Bench's move().",
        )
        if self.hidden_com:
            submit = Skill(
                "submit",
                [SkillArg("quadrant", "enum", choices=["A", "B", "C", "D"])],
                doc="Submit the ballast quadrant, once; final, with no correctness feedback. Costs no steps.",
            )
            return [move, submit]
        reset = Skill("reset", doc="Restore the same initial scene and discard progress; costs one step.")
        return [move, reset]

    # ---- episode -------------------------------------------------------------------------------------------------
    def reset(self, seed: int) -> None:
        t = self.task
        self.meta = self.worker.call("make", task=self.kind, seed=int(t["seed"]), case=t.get("case"))
        self._state = None
        self._grip = -1.0
        self.answer = None

    def step(self, action) -> StepInfo:
        a = [float(x) for x in np.clip(np.asarray(action, dtype=float), -1, 1)]
        self._grip = a[6]
        self.worker.call("step", action=a)
        self._state = None
        return StepInfo(success=False)

    def hold_action(self) -> list[float]:
        """Hold still, keeping the gripper as it is (RLE-Bench scores the scene without opening the gripper)."""
        return [0.0] * 6 + [self._grip] + ([] if self.hidden_com else [0.0, 0.0, 0.0, 0.0, -1.0])

    def start_skill(self, name: str, raw_args: list[str]):
        if name == "move":
            return self._move(raw_args)
        if name == "reset" and not self.hidden_com:
            if raw_args:
                raise ValueError("reset takes no arguments")
            return self._reset_scene()
        if name == "submit" and self.hidden_com:
            if len(raw_args) != 1 or raw_args[0].strip().upper() not in ("A", "B", "C", "D"):
                raise ValueError("submit needs one quadrant: A, B, C or D")
            if self.answer is not None:
                raise ValueError(f"already submitted {self.answer}; answers are final")
            return self._submit(raw_args[0].strip().upper())
        raise ValueError(f"unknown skill {name!r}; skills: {', '.join(self.skills())}")

    def _move(self, raw_args: list[str]):
        if len(raw_args) not in (7, 8, 9):
            raise ValueError("move needs X Y Z QX QY QZ QW [GRIPPER] [STEPS]")
        try:
            vals = [float(v) for v in raw_args[:8]]
            steps = int(raw_args[8]) if len(raw_args) == 9 else 40
        except ValueError:
            raise ValueError("move arguments must be numbers (STEPS an integer)") from None
        if not all(np.isfinite(vals)):
            raise ValueError("move arguments must be finite")
        pos, quat = vals[:3], vals[3:7]
        grip = vals[7] if len(vals) == 8 else 1.0
        if abs(sum(q * q for q in quat) - 1) > 1e-4:
            raise ValueError("the quaternion must be a unit quaternion x y z w")
        if not -1 <= grip <= 1:
            raise ValueError("gripper must be in [-1, 1]")
        if not 1 <= steps <= 200:
            raise ValueError("steps must be 1-200")

        def gen():
            for _ in range(steps):
                yield self.worker.call("move_action", position=pos, quaternion=quat, gripper=grip)
            return {"moved_steps": steps}

        return gen()

    def _reset_scene(self):
        def gen():
            self.worker.call("reset")
            self._state = None
            self._grip = -1.0
            yield [0.0] * 6 + [-1.0, 0.0, 0.0, 0.0, 0.0, -1.0]  # the step a reset costs: the restored scene holds still
            return {"reset": "the initial scene is restored"}

        return gen()

    def _submit(self, answer: str):
        def gen():
            self.answer = answer
            return {"submitted": answer, "note": "answers are final; there is no correctness feedback"}
            yield  # a generator that runs no control step

        return gen()

    def observe(self) -> dict:
        if self._state is None:
            self._state = self.worker.call("observe")
        st = dict(self._state)
        if self.hidden_com:
            st["submitted"] = self.answer
        return st

    def privileged_state(self) -> dict:
        """What only the reference solution sees (oracle token): object poses and extents; never masses."""
        return self.worker.call("private_state")

    def hand_pos(self) -> np.ndarray:
        return np.asarray(self.observe()["eef_pos"], dtype=float)

    def success(self) -> bool:
        return False  # judged once by judge()

    def judge(self, outcome: str, text: str = "") -> bool:
        if self.hidden_com:
            ok = self.answer is not None and bool(self.worker.call("answer_correct", answer=self.answer))
            self.last_judge = {
                "submitted": self.answer,
                "correct": ok,
                "rule": "the submitted quadrant is the ballast's (RLE-Bench HiddenCOM, one box)",
            }
            return ok
        if outcome == "gave_up":
            self.last_judge = {"quality": None, "rule": "robo give-up scores 0"}
            return False
        detail = self.worker.call("quality")
        ok = float(detail["quality"]) >= PASS_QUALITY
        self.last_judge = {
            **detail,
            "pass_quality": PASS_QUALITY,
            "rule": f"RLE-Bench {self.kind} quality >= {PASS_QUALITY}",
        }
        return ok

    # ---- cameras -------------------------------------------------------------------------------------------------
    def render(self, width: int | None = None, height: int | None = None) -> np.ndarray:
        w, h = (width, height) if width and height else self.render_size
        return self.worker.call("render", camera=self.camera, width=int(w), height=int(h))

    def camera_info(self, cameras: list[str]) -> list[dict]:
        """Intrinsics only, as RLE-Bench gives (no camera poses)."""
        out = []
        for name in cameras:
            fovy = float(self.worker.call("camera", name=name)["fovy"])
            f = RESOLUTION / (2 * np.tan(np.deg2rad(fovy) / 2))
            out.append(
                {
                    "name": name,
                    "width": RESOLUTION,
                    "height": RESOLUTION,
                    "fovy_deg": round(fovy, 2),
                    "intrinsics": [
                        [round(f, 4), 0.0, (RESOLUTION - 1) / 2],
                        [0.0, round(f, 4), (RESOLUTION - 1) / 2],
                        [0.0, 0.0, 1.0],
                    ],
                    "projection": None,
                }
            )
        return out

    def close(self) -> None:
        self.worker.close()


def oracle_main(env: str) -> None:
    from .oracles import run

    run(env)
