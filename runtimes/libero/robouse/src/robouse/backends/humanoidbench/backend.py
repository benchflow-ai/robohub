"""HumanoidBench backend: whole-body tasks for a Unitree H1 humanoid with two Shadow hands (HumanoidBench, Sferrazza et
al. 2024, MIT licence, github.com/carlosferrazza/humanoid-bench), in MuJoCo, on the CPU.

HumanoidBench pins mujoco 3.1.6, gymnasium 0.29.1 and dm_control 1.0.20, so the simulation runs in its own virtualenv
(default ~/.cache/robouse/humanoidbench-venv, override with ROBOUSE_HUMANOIDBENCH_PYTHON) as a subprocess
(worker.py) that speaks JSON lines. This module is the trusted side: the embodiment declaration, the skills
(closed-loop generators), the verdict and the reference solutions. Setup: docs/suites/humanoidbench.md.

Each task is HumanoidBench's own `h1hand-<task>-v0` environment, unchanged: its scene, its per-step reward, its fall
termination and its success rule (the episode return reaching the task's `success_bar`, or the task's own `success`
signal where it has one). The episode ends as solved the first time that happens (success_mode: first).

HumanoidBench's raw action is 61 joint position targets, which an agent cannot balance by hand (the robot falls within
two seconds of holding its start pose). The layer under `robo act` makes the robot controllable:
  base.velocity [FORWARD, LEFT, TURN]: a velocity command for Unitree's pretrained H1 locomotion policy (unitree_rl_gym,
    BSD-3-Clause), which drives the 10 leg joints and keeps the robot balanced; zero = step in place.
  left.hand_delta / right.hand_delta [DX, DY, DZ] x 2 cm per step: move each hand's goal point in the robot's heading
    frame (x forward, y left, z height above the floor); damped least-squares IK on the 4 arm joints tracks it.
One control step = 20 ms (HumanoidBench's own 50 Hz: 10 MuJoCo steps of 2 ms).
Skills: walk_to, walk, turn_to, stand, reach, hands_home, sit_down.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

from ... import config
from ...workers.client import StdioWorker
from ..base import StepInfo
from ..embodied import ActionGroup, Budget, EmbodiedBackend, Embodiment, Oracle, Sensor, Skill, SkillArg, wrap

WORKER = Path(__file__).with_name("worker.py")
CONTROL_DT = 0.02
MAX_FWD, MAX_LEFT, MAX_TURN = 1.0, 0.5, 1.0
HAND_STEP = 0.02
SIDES = ("left", "right")

# Robo Use task id -> HumanoidBench task (h1hand-<task>-v0), episode seed, step budget (HumanoidBench's episode length)
TASKS = {
    "humanoidbench-stand": dict(task="stand", seed=0, max_steps=1000),
    "humanoidbench-walk": dict(task="walk", seed=0, max_steps=1000),
    "humanoidbench-maze": dict(task="maze", seed=0, max_steps=1000),
    "humanoidbench-pole": dict(task="pole", seed=0, max_steps=1000),
    "humanoidbench-sit-simple": dict(task="sit_simple", seed=0, max_steps=1000),
    # seeds pick HumanoidBench's random layout: reach target (0.20, 0.86, 1.28); push goal (0.75, 0.24) on the table
    "humanoidbench-reach": dict(task="reach", seed=0, max_steps=1000),
    "humanoidbench-push": dict(task="push", seed=12, max_steps=500),
}


class HumanoidBenchBackend(EmbodiedBackend):
    name = "humanoidbench"
    image_flipped = False

    def __init__(self, spec: dict):
        super().__init__(spec)
        tid = spec.get("id") or spec.get("env")
        if tid not in TASKS and f"humanoidbench-{spec.get('env')}" in TASKS:
            tid = f"humanoidbench-{spec.get('env')}"
        if tid not in TASKS:
            raise KeyError(f"unknown humanoidbench task {tid!r}")
        self.task_id, self.task = tid, TASKS[tid]
        self.camera = spec.get("camera", "cam_default")
        self.render_size = tuple(spec.get("render_size", (480, 480)))
        self.decl = self._declare()
        self.policy_path = str(
            config.path("ROBOUSE_HUMANOIDBENCH_POLICY", config.cache_dir() / "humanoidbench" / "h1_motion.pt")
        )
        if not Path(self.policy_path).exists():
            raise FileNotFoundError(
                f"Unitree H1 locomotion policy not found at {self.policy_path}; see docs/suites/humanoidbench.md"
            )
        # run from the worker's own folder so no stray module in the cwd shadows the stdlib
        self.worker = StdioWorker(
            "humanoidbench",
            config.sim_python("humanoidbench", "humanoidbench"),
            WORKER,
            setup="docs/suites/humanoidbench.md",
            env={"MKL_NUM_THREADS": "1"},
        )
        self._state = None
        self.last_judge: dict = {}
        self.meta: dict = {}

    def _declare(self) -> Embodiment:
        sensors = [
            Sensor(
                "robot",
                "proprio",
                ["robot"],
                "pelvis position, heading, head height, torso uprightness, centre-of-mass velocity, "
                "the current velocity command, hand positions (world and heading frame) and hand goals",
                units="m, deg, m/s",
            ),
            Sensor(
                "scene",
                "world",
                ["scene"],
                "the task's objects, goals and landmarks (public positions)",
                units="m",
                frame="world",
            ),
            Sensor(
                "progress",
                "events",
                ["progress"],
                "HumanoidBench's episode return so far, the last step's reward and its terms, "
                "the success threshold and whether the task is solved",
            ),
            Sensor(
                "camera:cam_default", "camera", mount="world", doc="HumanoidBench's default view, following the robot"
            ),
        ]
        groups = [
            ActionGroup(
                "base.velocity",
                "base_twist",
                ["FORWARD", "LEFT", "TURN"],
                [-MAX_FWD, -MAX_LEFT, -MAX_TURN],
                [MAX_FWD, MAX_LEFT, MAX_TURN],
                "m/s, m/s, rad/s",
                "velocity command for the H1 walking controller (heading frame; TURN + = counter-clockwise); zero = step in place",
                frame="body",
            ),
            ActionGroup(
                "left.hand_delta",
                "ee_delta_pos",
                ["LDX", "LDY", "LDZ"],
                [-1] * 3,
                [1] * 3,
                "x 2 cm per step",
                "moves the left hand's goal point (heading frame: x forward, y left, z up); zero holds it",
                frame="body",
            ),
            ActionGroup(
                "right.hand_delta",
                "ee_delta_pos",
                ["RDX", "RDY", "RDZ"],
                [-1] * 3,
                [1] * 3,
                "x 2 cm per step",
                "moves the right hand's goal point (heading frame); zero holds it",
                frame="body",
            ),
        ]
        xy = [SkillArg("x", unit="m"), SkillArg("y", unit="m")]
        side = SkillArg("side", "enum", choices=list(SIDES), doc="which hand")
        skills = [
            Skill(
                "walk_to",
                xy
                + [
                    SkillArg(
                        "heading_deg", "float", "deg", 999.0, doc="final heading; 999 = keep the heading of travel"
                    ),
                    SkillArg("tol", "float", "m", 0.10, doc="arrival tolerance"),
                ],
                "walk to the floor point (X, Y) (world frame): turn toward it in place, walk, slow down near it and turn to HEADING_DEG; "
                "stops when there, when blocked (under 3 cm of progress in 60 steps) or if the robot falls, and reports which. It does "
                "not avoid obstacles",
                600,
            ),
            Skill(
                "walk",
                [
                    SkillArg("forward", unit="m/s"),
                    SkillArg("left", unit="m/s", default=0.0),
                    SkillArg("turn", unit="rad/s", default=0.0),
                    SkillArg("steps", "int", default=50),
                ],
                "hold one velocity command (heading frame) for STEPS steps; the same as `robo act FORWARD LEFT TURN 0 0 0 0 0 0` repeated",
                1000,
            ),
            Skill(
                "turn_to",
                [SkillArg("heading_deg", unit="deg"), SkillArg("tol", "float", "deg", 5.0)],
                "turn in place to a world heading (0 = +x, counter-clockwise positive)",
                300,
            ),
            Skill(
                "stand",
                [SkillArg("steps", "int", default=50)],
                "zero velocity command (step in place, balanced) for STEPS steps",
                1000,
            ),
            Skill(
                "reach",
                [
                    side,
                    SkillArg("x", unit="m"),
                    SkillArg("y", unit="m"),
                    SkillArg("z", unit="m"),
                    SkillArg("tol", "float", "m", 0.03),
                ],
                "move one hand in a straight line to the world point (X, Y, Z) while the robot steps in place; stops when there, when "
                "blocked or at the edge of the arm's reach, and reports which. Afterwards the hand keeps holding that world point "
                "(compensating the body's sway) until the next command for that hand",
                150,
            ),
            Skill("hands_home", [], "both hands back to their start positions (relative to the body)", 150),
            Skill(
                "sit_down",
                [],
                "sit down where the robot stands: the walking controller stops, the feet come together and the hips and "
                "knees bend to about 90 degrees in 0.6 s, lowering the pelvis about 0.3 m and moving it about 0.2 m backwards (onto a "
                "seat behind the robot, if there is one; otherwise the robot falls). One-way: afterwards FORWARD, LEFT and TURN have no "
                "effect. The skill returns after 60 steps",
                60,
            ),
        ]
        cams = self._cameras()
        sensors[-1:] = [
            Sensor(
                f"camera:{c}",
                "camera",
                mount="world",
                doc=(
                    "HumanoidBench's view for this task, following the robot"
                    if c == self.camera
                    else "HumanoidBench's default view"
                ),
            )
            for c in cams
        ]
        return Embodiment(
            robot="Unitree H1 with two Shadow hands (HumanoidBench h1hand)",
            family="humanoid",
            assets=[],
            sensors=sensors,
            action_groups=groups,
            skills=skills,
            budget=Budget(int(self.task["max_steps"]), CONTROL_DT),
            cameras=cams,
        )

    def _cameras(self) -> list[str]:
        return list(dict.fromkeys([self.camera, "cam_default"]))

    # ---- worker ------------------------------------------------------------------------------------------------
    def _call(self, cmd: str, **kw):
        return self.worker.call(cmd, **kw)

    # ---- episode -----------------------------------------------------------------------------------------------
    def reset(self, seed: int) -> None:
        # the seed (task.md's robouse.seed, TASKS[...]["seed"]) fixes HumanoidBench's layout draws (push goal, reach target)
        # and its start-pose noise; the bundled tasks' reference solutions are validated on that seed
        self.meta = self._call(
            "make",
            task=self.task["task"],
            seed=int(seed),
            policy=self.policy_path,
            noise_seed=self.spec.get("noise_seed"),
        )
        self._state = None
        self.events, self._event_keys = [], set()
        self.last = {"success": False, "terminated": False}

    def control_step(self, a: np.ndarray) -> None:
        v = [float(a[0]) / MAX_FWD, float(a[1]) / MAX_LEFT, float(a[2]) / MAX_TURN] + [float(x) for x in a[3:9]]
        self.last = self._call("step", action=v)
        self._state = None
        if self.last.get("terminated"):
            self.event(
                "fall", "fall", "the robot fell (HumanoidBench's termination); the episode can no longer succeed"
            )

    def step(self, action) -> StepInfo:
        a = self.action_spec.clip(action)
        if not np.all(np.isfinite(a)):
            raise ValueError("action values must be finite numbers")
        self.control_step(a)
        return StepInfo(success=bool(self.last.get("success")))

    def observe(self) -> dict:
        if self._state is None:
            self._state = self._call("observe")
        return json.loads(json.dumps(self._state))

    def judge_detail(self) -> dict:
        try:
            j = self._call("judge")
        except RuntimeError as e:
            j = {"success": False, "error": str(e)}
        self.last_judge = j
        return j

    def success(self) -> bool:
        return bool(self.judge_detail().get("success"))

    def judge(self, outcome: str, text: str = "") -> bool:
        # success_mode first: solved the moment HumanoidBench's success rule held, which ends the episode as
        # success_reached; any other ending (robo done, give-up, budget) scores 0, even if the rule then fires during the
        # settle steps after `robo done`
        return outcome == "success_reached" and bool(self.judge_detail().get("success"))

    def mj_model_data(self):
        return None

    def render(self, width: int | None = None, height: int | None = None) -> np.ndarray:
        w, h = (width, height) if width and height else self.render_size
        return self._call("render", width=int(w), height=int(h), camera=self.camera or "cam_default")

    def close(self) -> None:
        self.worker.close()

    # ---- skills ------------------------------------------------------------------------------------------------
    def _robot(self) -> dict:
        return self.observe()["robot"]

    @staticmethod
    def _act(fwd=0.0, left=0.0, turn=0.0, hands=None) -> list[float]:
        v = [
            float(np.clip(fwd, -MAX_FWD, MAX_FWD)),
            float(np.clip(left, -MAX_LEFT, MAX_LEFT)),
            float(np.clip(turn, -MAX_TURN, MAX_TURN)),
        ]
        return v + ([float(x) for x in hands] if hands is not None else [0.0] * 6)

    def _pose(self):
        r = self._robot()
        return np.array(r["pelvis"][:2], float), math.radians(r["heading_deg"]), r

    def skill_walk_to(self, x: float, y: float, heading_deg: float = 999.0, tol: float = 0.10):
        tgt = np.array([x, y], float)
        want_h = None if heading_deg >= 900 else math.radians(heading_deg)
        tol = max(0.03, float(tol))
        hist: list[float] = []
        status = "reached"
        while True:
            p, yaw, r = self._pose()
            if r["fallen"]:
                status = "fell"
                break
            d = tgt - p
            dist = float(np.linalg.norm(d))
            if dist < tol:
                if want_h is None or abs(wrap(want_h - yaw)) < math.radians(5):
                    break
                h = want_h
            elif dist > 0.5:
                h = math.atan2(d[1], d[0])
            else:
                h = want_h if want_h is not None else yaw
            c, s = math.cos(yaw), math.sin(yaw)
            bx, by = c * d[0] + s * d[1], -s * d[0] + c * d[1]
            eh = wrap(h - yaw)
            fwd = left = 0.0
            if dist >= tol and dist > 0.5:
                # travel: turn in place until roughly facing the point, then walk with a small sideways correction
                if abs(eh) < math.radians(30):
                    fwd = float(np.clip(1.6 * bx, 0.3, MAX_FWD))
                    left = float(np.clip(1.0 * by, -0.2, 0.2))
            elif dist >= tol:
                # final approach in any direction at 0.25-0.5 m/s (the policy ignores planar commands below 0.2 m/s)
                v = np.array([bx, by]) / max(dist, 1e-9) * float(np.clip(1.5 * dist, 0.25, 0.5))
                fwd, left = float(v[0]), float(np.clip(v[1], -0.3, 0.3))
            hist.append(dist + 0.3 * abs(eh))
            if len(hist) > 60 and hist[-60] - hist[-1] < 0.03:  # under 3 cm of progress in 1.2 s
                status = "blocked"
                break
            yield self._act(fwd, left, float(np.clip(2.0 * eh, -MAX_TURN, MAX_TURN)))
        p, yaw, r = self._pose()
        return {
            "status": status,
            "reached": status == "reached",
            "pelvis": r["pelvis"],
            "heading_deg": r["heading_deg"],
            "distance_m": round(float(np.linalg.norm(tgt - p)), 3),
        }

    def skill_walk(self, forward: float, left: float = 0.0, turn: float = 0.0, steps: int = 50):
        for _ in range(max(1, int(steps))):
            yield self._act(forward, left, turn)
            if self._robot()["fallen"]:
                return {"status": "fell"}
        r = self._robot()
        return {"pelvis": r["pelvis"], "heading_deg": r["heading_deg"], "com_velocity_body": r["com_velocity_body"]}

    def skill_turn_to(self, heading_deg: float, tol: float = 5.0):
        want = math.radians(heading_deg)
        for _ in range(400):
            _, yaw, r = self._pose()
            e = wrap(want - yaw)
            if abs(e) < math.radians(max(1.0, tol)) or r["fallen"]:
                break
            yield self._act(0, 0, float(np.clip(2.0 * e, -MAX_TURN, MAX_TURN)))
        r = self._robot()
        return {"heading_deg": r["heading_deg"], "pelvis": r["pelvis"]}

    def skill_stand(self, steps: int = 50):
        for _ in range(max(1, int(steps))):
            yield self._act()
            if self._robot()["fallen"]:
                return {"status": "fell"}
        r = self._robot()
        return {"pelvis": r["pelvis"], "heading_deg": r["heading_deg"]}

    def _hands_toward(self, goals_world: dict, tol: float, speed: float = HAND_STEP):
        """Move hand goals (heading frame) in straight lines toward world points, re-planned every step against the body's sway."""
        hist: list[float] = []
        status = "reached"
        while True:
            r = self._robot()
            if r["fallen"]:
                status = "fell"
                break
            yaw = math.radians(r["heading_deg"])
            base = np.array(r["pelvis"], float)
            c, s = math.cos(yaw), math.sin(yaw)
            hands = [0.0] * 6
            err = {}
            for i, sd in enumerate(SIDES):
                if sd not in goals_world:
                    continue
                gw = np.asarray(goals_world[sd], float)
                d = gw - base
                gb = np.array([c * d[0] + s * d[1], -s * d[0] + c * d[1], gw[2]])
                err[sd] = float(np.linalg.norm(np.asarray(r["hands"][sd]["pos"]) - gw))
                off = gb - np.asarray(r["hands"][sd]["goal_body"], float)
                n = float(np.linalg.norm(off))
                step = off if n <= speed else off * speed / n
                hands[3 * i : 3 * i + 3] = list(np.clip(step / HAND_STEP, -1, 1))
                if not np.any(np.abs(hands[3 * i : 3 * i + 3]) > 1e-6):
                    hands[3 * i] = 1e-6  # a nonzero command keeps the hand under this skill's control
            if all(e <= tol for e in err.values()):
                break
            hist.append(sum(err.values()))
            if len(hist) > 30 and hist[-25] - hist[-1] < 0.005:
                far = max(
                    float(
                        np.linalg.norm(np.asarray(r["hands"][sd]["goal_body"]) - np.asarray(r["hands"][sd]["pos_body"]))
                    )
                    for sd in goals_world
                )
                status = "blocked" if far > 0.03 else "out_of_reach"
                break
            yield self._act(hands=hands)
        return status

    def skill_reach(self, side: str, x: float, y: float, z: float, tol: float = 0.03):
        status = yield from self._hands_toward({side: (x, y, z)}, max(0.01, float(tol)))
        self._call("set_anchor", side=side, world=[float(x), float(y), float(z)])
        r = self._robot()
        return {
            "status": status,
            "reached": status == "reached",
            "hand": r["hands"][side]["pos"],
            "distance_m": round(float(np.linalg.norm(np.asarray(r["hands"][side]["pos"]) - np.array([x, y, z]))), 3),
            "holding_world_point": [x, y, z],
        }

    def skill_hands_home(self):
        home = self.meta.get("hand_home_body") or {}
        r = self._robot()
        yaw = math.radians(r["heading_deg"])
        base = np.array(r["pelvis"], float)
        c, s = math.cos(yaw), math.sin(yaw)
        goals = {}
        for sd in SIDES:
            hb = np.asarray(home.get(sd, r["hands"][sd]["goal_body"]), float)
            goals[sd] = (base[0] + c * hb[0] - s * hb[1], base[1] + s * hb[0] + c * hb[1], hb[2])
        status = yield from self._hands_toward(goals, 0.04)
        return {"status": status}

    def skill_sit_down(self):
        self._call("sit_down")
        for _ in range(60):
            yield self._act()
            if self._robot()["fallen"]:
                return {"status": "fell"}
        r = self._robot()
        return {
            "status": "seated",
            "pelvis": r["pelvis"],
            "head_height": r["head_height"],
            "torso_upright": r["torso_upright"],
        }


# ---- reference solutions (socket only, public observation) ----------------------------------------------------------


def oracle_main(env: str) -> None:
    from .oracles import ORACLES

    tid = env if env in ORACLES else f"humanoidbench-{env}"
    Oracle().run(ORACLES[tid])
