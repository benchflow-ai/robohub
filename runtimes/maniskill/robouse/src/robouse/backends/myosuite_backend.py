"""MyoSuite backend: musculoskeletal models of the human elbow, index finger and hand driven by muscles (MyoSuite
2.12.2, Apache-2.0, github.com/MyoHub/myosuite; models from MyoSim, github.com/MyoHub/myo_sim, Apache-2.0, shipped in
the myosuite wheel).

MyoSuite pins MuJoCo 3.6 and gymnasium < 1.3, so the simulation runs in its own virtualenv (default
~/.cache/robouse/myosuite-venv, override with ROBOUSE_MYOSUITE_PYTHON) as a subprocess (myosuite_worker.py) that speaks
JSON lines. This module is the trusted side: the embodiment declaration, the skills (closed-loop generators), the
success check and the reference solutions. Setup: docs/suites/myosuite.md.

Action (one step = one MyoSuite env step, 20 ms): `muscles`, one excitation in [0, 1] per muscle (the neural drive; each
muscle's activation follows it with MuJoCo's first-order activation dynamics, about 10 ms up and 40 ms down). The hold
value (the 10 settle steps after `robo done`) repeats the last excitations sent, so a pose held by the muscles stays held.

Skills: `set_joint_targets NAME=RAD ... [steps=N] [ramp=N]` runs a muscle-space controller (inverse dynamics plus a
bounded least-squares solve for the activations, the approach of MyoSuite's inverse-dynamics tutorial) towards joint
angle targets; `reach_tips TIP=X,Y,Z ...` (reach tasks) solves inverse kinematics for fingertip positions and then runs
the same controller; `hold [steps]` keeps the controller on the current targets. Every skill step is an ordinary step
whose action is the muscle excitations the controller chose.

Success (every task): MyoSuite's own `solved` term for the env, evaluated from the simulated state after `robo done`
and the 10-step settle (pose: joint-angle error norm below the env's pose_thd; reach: stacked fingertip error norm below
12.5 mm per fingertip; key turn: key angle above the env's goal_th; object hold: object within 10 mm of the goal).
"""
from __future__ import annotations

import json
import os
import subprocess
from pathlib import Path

import numpy as np

from .base import StepInfo
from .embodied import ActionGroup, Budget, Embodiment, EmbodiedBackend, Oracle, Sensor, Skill, SkillArg

WORKER = Path(__file__).with_name("myosuite_worker.py")
DEFAULT_PY = Path.home() / ".cache" / "robouse" / "myosuite-venv" / "bin" / "python"
CONTROL_DT = 0.02
SKILL_MAX = 300

HAND_CAM = {"lookat": [-0.19, -0.54, 1.41], "distance": 0.32, "azimuth": 0.0, "elevation": -20.0}
ELBOW_CAM = {"lookat": [-0.16, 0.0, 1.15], "distance": 0.9, "azimuth": 0.0, "elevation": -10.0}
FINGER_CAM = {"lookat": [0.12, 0.0, 0.24], "distance": 0.42, "azimuth": 90.0, "elevation": -10.0}

# task id -> MyoSuite env id, reset seed, step budget, reference solution kind, render camera (free camera around a body)
TASKS: dict[str, dict] = {
    "myosuite-elbow-pose": dict(env="myoElbowPose1D6MRandom-v0", seed=8, max_steps=300, oracle="pose",
                                camera=ELBOW_CAM),
    "myosuite-finger-pose-fixed": dict(env="myoFingerPoseFixed-v0", seed=0, max_steps=300, oracle="pose",
                                       camera=FINGER_CAM),
    "myosuite-finger-pose-random": dict(env="myoFingerPoseRandom-v0", seed=3, max_steps=300, oracle="pose",
                                        camera=FINGER_CAM),
    "myosuite-finger-reach-fixed": dict(env="myoFingerReachFixed-v0", seed=0, max_steps=300, oracle="reach",
                                        camera=FINGER_CAM),
    "myosuite-finger-reach-random": dict(env="myoFingerReachRandom-v0", seed=6, max_steps=300, oracle="reach",
                                         camera=FINGER_CAM),
    "myosuite-hand-pose-4": dict(env="myoHandPose4Fixed-v0", seed=0, max_steps=500, oracle="pose", camera=HAND_CAM),
    "myosuite-hand-pose-5": dict(env="myoHandPose5Fixed-v0", seed=0, max_steps=500, oracle="pose", camera=HAND_CAM),
    "myosuite-hand-pose-7": dict(env="myoHandPose7Fixed-v0", seed=0, max_steps=500, oracle="pose", camera=HAND_CAM),
    "myosuite-hand-pose-8": dict(env="myoHandPose8Fixed-v0", seed=0, max_steps=500, oracle="pose", camera=HAND_CAM),
    "myosuite-hand-pose-9": dict(env="myoHandPose9Fixed-v0", seed=0, max_steps=500, oracle="pose", camera=HAND_CAM),
    "myosuite-hand-reach-fixed": dict(env="myoHandReachFixed-v0", seed=0, max_steps=500, oracle="reach", camera=HAND_CAM),
    "myosuite-hand-reach-random": dict(env="myoHandReachRandom-v0", seed=0, max_steps=500, oracle="reach", camera=HAND_CAM),
    # muscles only (skills disabled): the agent sends raw excitations; the reference solution runs MyoSuite's NPG baseline
    "myosuite-elbow-pose-muscles": dict(env="myoElbowPose1D6MRandom-v0", seed=9, max_steps=300, oracle="policy", policy_steps=100,
                                        skills=False, camera=ELBOW_CAM),
    "myosuite-finger-pose-muscles": dict(env="myoFingerPoseFixed-v0", seed=0, max_steps=300, oracle="policy", policy_steps=100,
                                         skills=False, camera=FINGER_CAM),
    "myosuite-hand-key-turn": dict(env="myoHandKeyTurnFixed-v0", seed=0, max_steps=500, oracle="policy", policy_steps=200, camera=HAND_CAM),
}


class _FreeSkill(Skill):
    """A skill whose arguments are NAME=VALUE pairs over a task-specific set of names (joints or fingertips)."""

    def __init__(self, name, args, doc, max_steps, usage):
        super().__init__(name, args, doc, max_steps)
        self.usage = usage

    def signature(self) -> str:
        return f"{self.name} {self.usage}: {self.doc} (at most {self.max_steps} steps per call)"


def _kv(raw: list[str]) -> tuple[dict, dict]:
    """Split skill arguments into NAME=VALUE pairs (values kept as strings) and the options steps= / ramp=."""
    pairs, opts = {}, {}
    for tok in raw:
        if "=" not in tok:
            raise ValueError(f"arguments are NAME=VALUE pairs, got {tok!r}")
        k, v = tok.split("=", 1)
        k = k.strip().lstrip("-")
        if k.lower() in ("steps", "ramp"):
            try:
                opts[k.lower()] = int(float(v))
            except ValueError:
                raise ValueError(f"{k} must be a whole number, got {v!r}") from None
        else:
            if k in pairs:
                raise ValueError(f"{k} is given twice")
            pairs[k] = v
    return pairs, opts


class MyoSuiteBackend(EmbodiedBackend):
    name = "myosuite"
    image_flipped = False

    def __init__(self, spec: dict):
        super().__init__(spec)
        env = spec.get("env") or spec.get("id")
        if env not in TASKS:
            raise KeyError(f"unknown MyoSuite task {env!r}")
        self.task_id, self.task = env, TASKS[env]
        self.render_size = tuple(spec.get("render_size", (480, 480)))
        py = os.environ.get("ROBOUSE_MYOSUITE_PYTHON", str(DEFAULT_PY))
        if not Path(py).exists():
            raise RuntimeError(f"MyoSuite virtualenv not found at {py}; see docs/suites/myosuite.md")
        self.proc = subprocess.Popen([py, str(WORKER)], stdin=subprocess.PIPE, stdout=subprocess.PIPE, cwd=str(WORKER.parent),
                                     env={**os.environ, "OPENBLAS_NUM_THREADS": "1", "OMP_NUM_THREADS": "1"},
                                     stderr=subprocess.DEVNULL if os.environ.get("ROBOUSE_MYOSUITE_QUIET", "1") == "1" else None)
        self._state = None
        self._worker_dead = ""
        self.last_judge: dict = {}
        self.meta = self._call("make", env_id=self.task["env"], seed=int(spec.get("seed", self.task["seed"])),
                               camera=self.task.get("camera"))
        self.decl = self._declare()
        self._last = [0.0] * len(self.meta["muscles"])

    def _declare(self) -> Embodiment:
        meta = self.meta
        kind = meta["kind"]
        sensors = [
            Sensor("joints", "proprio", ["joint_angles_rad", "joint_velocities_rad_s"] + ([] if self.spec.get("skills") is False else ["joint_targets_rad"]),
                   "angle (rad) and velocity (rad/s) of each muscle-driven joint" + ("" if self.spec.get("skills") is False else ", and the controller's current joint targets")),
            Sensor("muscles", "proprio", ["muscle_activations"], "activation of each muscle in [0, 1], in the action's muscle order"),
        ]
        if kind == "pose":
            sensors.append(Sensor("task", "world", ["target_pose_rad", "pose_error_norm", "threshold"],
                                  "the target joint angles, the joint-angle error norm and the success threshold"))
        elif kind == "reach":
            sensors.append(Sensor("task", "world", ["fingertips", "fingertip_targets", "reach_error_norm", "threshold"],
                                  "fingertip and target positions (m, world frame), the stacked error norm and the success threshold", frame="world"))
        elif kind == "keyturn":
            sensors.append(Sensor("task", "world", ["key_angle_rad", "key_goal_angle_rad", "key_head", "index_tip", "thumb_tip"],
                                  "key rotation and goal angle, key head and fingertip positions (m, world frame)", frame="world"))
        elif kind == "objhold":
            sensors.append(Sensor("task", "world", ["object_pos", "goal_pos", "object_error_m", "threshold_m"],
                                  "object and goal positions (m, world frame) and their distance", frame="world"))
        sensors += [Sensor("contacts", "world", ["contacts"], "pairs of bodies in contact"),
                    Sensor("upstream", "world", ["obs_vector"], "MyoSuite's own observation vector for this env (its obs_keys in order)"),
                    Sensor("camera:view", "camera", mount="world", doc="view of the arm or hand")]
        n = len(meta["muscles"])
        groups = [ActionGroup("muscles", "muscle_excitation", list(meta["muscles"]), [0.0] * n, [1.0] * n,
                              "0..1 per muscle",
                              "excitation (neural drive) of each muscle; the activation follows it within about 10 ms "
                              "(rising) or 40 ms (falling), and a muscle can only pull")]
        joints = meta["joints"]
        skills = [
            _FreeSkill("set_joint_targets", [SkillArg("joint", "str", doc=f"joint name, one of {joints}"),
                                             SkillArg("steps", "int", default=60), SkillArg("ramp", "int", default=20)],
                       "drive the joints to target angles (rad) with a muscle-space controller: each step it solves for "
                       "the muscle activations that produce the joint torques of a PD law (inverse dynamics plus bounded least squares) "
                       "and sends the matching excitations. Joints not named keep their previous targets (initially the start pose). "
                       "The targets move linearly from the current pose over RAMP steps; the skill runs STEPS steps or until the "
                       "joints are still", SKILL_MAX, "JOINT=RAD [JOINT=RAD ...] [steps=60] [ramp=20]"),
            _FreeSkill("hold", [SkillArg("steps", "int", default=20)],
                       "set every joint target to the joint's current angle and run the muscle-space controller there for STEPS steps "
                       "(keeps the hand still where it is)", SKILL_MAX, "[steps=20]"),
        ]
        if kind == "reach":
            skills.insert(1, _FreeSkill("reach_tips", [SkillArg("tip", "str", doc=f"fingertip site, one of {meta['tips']}"),
                                                       SkillArg("steps", "int", default=60), SkillArg("ramp", "int", default=20)],
                                        "solve inverse kinematics for fingertip positions (world frame, m), within the joint ranges and starting "
                                        "from the current pose, then run set_joint_targets with the resulting joint angles; reports the "
                                        "kinematic residual per fingertip", SKILL_MAX, "TIP=X,Y,Z [TIP=X,Y,Z ...] [steps=60] [ramp=20]"))
        return Embodiment(robot=f"MyoSuite {self._robot_name()} ({n} muscles)", family="musculoskeletal", assets=[],
                          sensors=sensors, action_groups=groups, skills=skills,
                          budget=Budget(int(self.task["max_steps"]), CONTROL_DT), cameras=["view"])

    def _robot_name(self) -> str:
        e = self.task["env"]
        return "elbow (MyoElbow)" if "Elbow" in e else ("index finger (MyoFinger)" if "Finger" in e else "hand and forearm (MyoHand)")

    # ---- worker ------------------------------------------------------------------------------------------------
    def _call(self, cmd: str, **kw):
        if self.proc.poll() is not None:
            self._dead(f"exit code {self.proc.returncode}")
        try:
            self.proc.stdin.write((json.dumps({"cmd": cmd, **kw}) + "\n").encode())
            self.proc.stdin.flush()
            line = self.proc.stdout.readline()
            if not line:
                raise EOFError("no reply")
            hdr = json.loads(line)
            if hdr.get("ok") and cmd == "render":
                data = self.proc.stdout.read(hdr["nbytes"])
                if len(data) != hdr["nbytes"]:
                    raise EOFError("truncated image")
                return np.frombuffer(data, dtype=np.uint8).reshape(kw["height"], kw["width"], 3)
        except (BrokenPipeError, EOFError, OSError, ValueError) as e:
            self._dead(f"{type(e).__name__}: {e}")
        if not hdr.get("ok"):
            raise RuntimeError(f"MyoSuite worker: {hdr.get('error')}")
        return hdr["result"]

    def _dead(self, why: str):
        self._state = None
        self._worker_dead = why
        raise RuntimeError(f"simulator worker died ({why}); the episode cannot continue "
                           "(run with ROBOUSE_MYOSUITE_QUIET=0 to see the worker's stderr)")

    # ---- episode -----------------------------------------------------------------------------------------------
    def reset(self, seed: int) -> None:
        if int(seed) != int(self.spec.get("seed", self.task["seed"])):
            self.meta = self._call("make", env_id=self.task["env"], seed=int(seed), camera=self.task.get("camera"))
        self._state = None
        self._last = [0.0] * len(self.meta["muscles"])

    def control_step(self, a: np.ndarray) -> None:
        self._last = [float(x) for x in a]
        self._call("step", action=self._last)
        self._state = None

    def step(self, action) -> StepInfo:
        a = self.action_spec.clip(action)
        if not np.all(np.isfinite(a)):
            raise ValueError("action values must be finite numbers")
        self.control_step(a)
        return StepInfo(success=False)  # judged after `robo done` and the settle (success_mode final)

    def hold_action(self) -> list[float]:
        return list(self._last)

    def observe(self) -> dict:
        if self._state is None:
            self._state = self._call("observe")
            if self.spec.get("skills") is False:  # no controller in this task: its targets mean nothing
                self._state.pop("joint_targets_rad", None)
        return json.loads(json.dumps(self._state))

    def success(self) -> bool:
        try:
            j = self._call("success")
        except RuntimeError as e:
            j = {"success": False, "error": str(e)}
        self.last_judge = j
        return bool(j["success"])

    def judge(self, outcome: str, text: str = "") -> bool:
        ok = self.success()
        return outcome == "done" and ok

    def info_extra(self) -> dict:
        return {"embodiment": self.embodiment(), "joint_ranges_rad": self.meta["joint_ranges"],
                "myosuite_env": self.task["env"]}

    def mj_model_data(self):
        return None

    def render(self, width: int | None = None, height: int | None = None) -> np.ndarray:
        w, h = (width, height) if width and height else self.render_size
        return self._call("render", width=int(w), height=int(h))

    def close(self) -> None:
        if self.proc.poll() is not None:
            return
        try:
            self.proc.stdin.write(b'{"cmd": "close"}\n')
            self.proc.stdin.flush()
            self.proc.wait(timeout=10)
        except Exception:
            self.proc.kill()

    # ---- skills ------------------------------------------------------------------------------------------------
    def start_skill(self, name: str, raw_args: list[str]):
        if name == "set_joint_targets":
            pairs, opts = _kv(raw_args)
            if not pairs:
                raise ValueError("set_joint_targets needs at least one JOINT=RAD pair; `robo info` lists the joints")
            tg = {}
            for k, v in pairs.items():
                if k not in self.meta["joints"]:
                    raise ValueError(f"unknown joint {k!r}; joints: {self.meta['joints']}")
                try:
                    x = float(v)
                except ValueError:
                    raise ValueError(f"{k}: value must be a number of radians, got {v!r}") from None
                if not np.isfinite(x):
                    raise ValueError(f"{k}: value must be finite")
                tg[k] = x
            return self._run_controller(tg, opts.get("steps", 60), opts.get("ramp", 20))
        if name == "reach_tips" and self.meta["kind"] == "reach":
            pairs, opts = _kv(raw_args)
            if not pairs:
                raise ValueError("reach_tips needs at least one TIP=X,Y,Z; tips: " + ", ".join(self.meta["tips"]))
            tips = {}
            for k, v in pairs.items():
                if k not in self.meta["tips"]:
                    raise ValueError(f"unknown fingertip {k!r}; tips: {self.meta['tips']}")
                try:
                    xyz = [float(x) for x in v.split(",")]
                except ValueError:
                    xyz = []
                if len(xyz) != 3 or not all(np.isfinite(xyz)):
                    raise ValueError(f"{k}: give the position as X,Y,Z in metres (no spaces), got {v!r}")
                tips[k] = xyz
            return self._run_reach(tips, opts.get("steps", 60), opts.get("ramp", 20))
        if name == "hold":
            pairs, opts = _kv(raw_args if all("=" in a for a in raw_args) else [f"steps={a}" for a in raw_args])
            if pairs:
                raise ValueError("hold takes only steps=N")
            return self._run_controller({}, opts.get("steps", 20), 1, hold_here=True)
        raise ValueError(f"unknown skill {name!r}; skills: {[s.name for s in self.decl.skills]}")

    def _run_controller(self, targets: dict, steps: int, ramp: int, extra: dict | None = None, hold_here: bool = False):
        steps = max(1, min(SKILL_MAX, int(steps)))
        ramp = max(1, min(steps, int(ramp)))
        res = self._call("set_targets", targets=targets, ramp=ramp, from_current=hold_here) if (targets or hold_here) else None
        still = 0
        c = {}
        for k in range(steps):
            c = self._call("control")
            yield c["excitations"]
            if not c["ramping"] and c["max_speed"] < 0.05:
                still += 1
                if still >= 5 and k >= ramp + 5:
                    break
            else:
                still = 0
        st = self.observe()
        out = {"joint_target_error_norm": c.get("target_error_norm")}
        if res and res.get("clipped_to_range"):
            out["clipped_to_joint_range"] = res["clipped_to_range"]
        for k in ("pose_error_norm", "reach_error_norm", "key_angle_rad", "object_error_m", "threshold", "threshold_m"):
            if k in st:
                out[k] = st[k]
        if extra:
            out.update(extra)
        return out

    def _run_reach(self, tips: dict, steps: int, ramp: int):
        ik = self._call("ik", tips=tips)
        return (yield from self._run_controller(ik["joint_targets"], steps, ramp, {"ik_residual_m": ik["residual_m"]}))


# ---- reference solutions (socket only) ------------------------------------------------------------------------------

def _policy_act(z: dict, obs: np.ndarray) -> np.ndarray:
    """MyoSuite's NPG baseline policy (an mjrl Gaussian MLP, mean action): normalised input, tanh hidden layers."""
    x = (obs - z["in_shift"]) / (z["in_scale"] + 1e-8)
    n = len([k for k in z if k.startswith("W")])
    for i in range(n):
        x = z[f"W{i}"] @ x + z[f"b{i}"]
        if i < n - 1:
            x = np.tanh(x)
    return x * z["out_scale"] + z["out_shift"]


def solution(env: str):
    """`env` is the task id, or for policy tasks the path of the task's oracle/policy.json."""
    if env.endswith(".json"):
        z = {k: np.asarray(v, dtype=float) for k, v in json.loads(Path(env).read_text())["weights"].items()}
        steps = int(json.loads(Path(env).read_text())["steps"])

        def run_policy(o: Oracle):
            for _ in range(steps):
                obs = np.asarray(o.state()["obs_vector"], dtype=float)
                a = _policy_act(z, obs)
                exc = 1.0 / (1.0 + np.exp(-5.0 * (a - 0.5)))  # MyoSuite's action -> muscle excitation mapping
                o.act([float(np.clip(x, 0.0, 1.0)) for x in exc])
        return run_policy

    kind = TASKS[env]["oracle"]

    def run(o: Oracle):
        st = o.state()
        if kind == "pose":
            tg = st["target_pose_rad"]
            o.skill("set_joint_targets", *[f"{k}={v}" for k, v in tg.items()], "steps=120", "ramp=40")
        elif kind == "reach":
            tg = st["fingertip_targets"]
            o.skill("reach_tips", *[f"{k}={','.join(str(x) for x in v)}" for k, v in tg.items()], "steps=120", "ramp=40")
        o.skill("hold", "steps=20")
    return run


def oracle_main(env: str) -> None:
    Oracle().run(solution(env))
