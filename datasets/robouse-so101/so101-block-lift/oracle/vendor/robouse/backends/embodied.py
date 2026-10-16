"""Shared base for the embodiment suites: quadruped, humanoid, mobile-manip, dexhand and crazyflie.

Each backend declares its embodiment in one place (an `Embodiment`): the robot and its Menagerie assets, the sensors
(which `robo observe` fields and cameras exist), the action groups that make up `robo act` (e.g. `base.twist`,
`arm.ee_delta`, `hand.joints`, `velocity_setpoint`), the skills with typed arguments (`robo skill NAME ARG ...`) and the
budget (control period, step limit). `robo info` reports the declaration, so the agent sees the same thing the task
author wrote. The declaration mirrors BenchFlow's embodiment-agnostic spec (sensors, action groups, typed skills,
budgets) and is the single place to adapt when that spec is final.

Skills are closed-loop controllers written as Python generators: `skill_<name>(**args)` yields one full action vector
per control step and returns a result dict. The episode server executes every yielded action as a normal step
(`Episode._env_step`), so skills use the same step budget, video frames and success checks as `robo act`; a skill never
moves the robot in a way `robo act` could not.

The reference-solution helpers at the bottom (`Oracle`) talk to the episode socket only, like an agent.
"""

from __future__ import annotations

import math
from collections.abc import Callable, Iterator
from dataclasses import asdict, dataclass, field
from typing import Any

import numpy as np

from ..core.protocol import agent_refused
from .base import ActionSpec, Backend, StepInfo

# ------------------------------------------------------------------------------------------------------------------
# declaration
# ------------------------------------------------------------------------------------------------------------------


@dataclass
class Sensor:
    name: str  # e.g. "base_state", "joint_pos", "camera:head"
    kind: str  # "proprio" | "world" (public scene facts) | "camera" | "events"
    fields: list[str] = field(default_factory=list)  # `robo observe` keys this sensor fills (cameras: none)
    doc: str = ""
    units: str = ""
    frame: str = ""  # e.g. "world", "body"
    mount: str = ""  # cameras: "world" | "body" | "head" | "wrist" | "hand"


@dataclass
class ActionGroup:
    name: str  # e.g. "base.twist", "arm.ee_delta", "gripper", "hand.joints", "velocity_setpoint"
    kind: str  # control mode, BenchFlow's vocabulary: "base_twist" | "velocity_setpoint" | "ee_delta_pos" |
    # "ee_delta_pose" | "joint_pos" | "joint_delta" | "joint_vel" | "gripper" | "select" | ...
    names: list[str]  # components
    low: list[float]
    high: list[float]
    units: str = ""
    doc: str = ""
    hold: list[float] | None = None  # the value that means "hold still" (default: zeros)
    frame: str = ""  # e.g. "world", "body"


@dataclass
class SkillArg:
    name: str
    type: str = "float"  # "float" | "int" | "str" | "enum"
    unit: str = ""
    default: Any = None  # None: required
    choices: list[str] | None = None
    doc: str = ""


@dataclass
class Skill:
    name: str
    args: list[SkillArg]
    doc: str
    max_steps: int = 300  # a call stops after this many control steps

    def signature(self) -> str:
        parts = []
        for a in self.args:
            label = a.name.upper() + (f"={'|'.join(a.choices)}" if a.choices else "")
            parts.append(label if a.default is None else f"[{a.name.upper()}={a.default}]")
        return f"{self.name} {' '.join(parts)}".strip() + f": {self.doc} (at most {self.max_steps} steps per call)"

    def parse(self, raw: list[str]) -> dict:
        """Positional arguments in declaration order, or NAME=value; each converted to its declared type."""
        vals: dict[str, Any] = {}
        pos = [r for r in raw if "=" not in r]
        kws = dict(r.split("=", 1) for r in raw if "=" in r)
        if len(pos) > len(self.args):
            raise ValueError(f"{self.name} takes at most {len(self.args)} arguments: {self.signature()}")
        for a, v in zip(self.args, pos, strict=False):
            vals[a.name] = v
        for k, v in kws.items():
            k = k.lower().lstrip("-")
            if k not in {a.name for a in self.args}:
                raise ValueError(f"{self.name} has no argument {k!r}: {self.signature()}")
            if k in vals:
                raise ValueError(f"{self.name}: {k.upper()} is given both by position and as {k}=...")
            vals[k] = v
        out = {}
        for a in self.args:
            if a.name not in vals:
                if a.default is None:
                    raise ValueError(f"{self.name} needs {a.name.upper()}: {self.signature()}")
                out[a.name] = a.default
                continue
            v = vals[a.name]
            if a.type == "float":
                try:
                    x = float(v)
                except ValueError:
                    raise ValueError(f"{self.name}: {a.name.upper()} must be a number, got {v!r}") from None
                if not math.isfinite(x):
                    raise ValueError(f"{self.name}: {a.name.upper()} must be a finite number")
                out[a.name] = x
            elif a.type == "int":
                try:
                    x = float(v)
                except ValueError:
                    x = float("nan")
                if not math.isfinite(x) or x != int(x):
                    raise ValueError(f"{self.name}: {a.name.upper()} must be a whole number, got {v!r}")
                out[a.name] = int(x)
            elif a.type == "enum":
                if str(v) not in (a.choices or []):
                    raise ValueError(f"{a.name} must be one of {a.choices}")
                out[a.name] = str(v)
            else:
                out[a.name] = str(v)
        return out


# the embodiment family -> BenchFlow's embodiment kind
SPEC_KIND = {
    "aerial": "drone",
    "dexterous_hand": "hand",
    "wheeled": "vehicle",
    "mobile_base": "vehicle",
    "bimanual_arm": "bimanual",
    "planar_arm": "arm",
}


def spec_kind(family: str) -> str:
    """The embodiment spec's kind for a robouse family; families the spec has no kind for are "other"."""
    from ..core.embodiment_spec import KINDS

    kind = SPEC_KIND.get(family, family)
    return kind if kind in KINDS else "other"


@dataclass
class Budget:
    max_steps: int
    control_dt: float  # seconds of simulated time per step


@dataclass
class Embodiment:
    robot: str  # display name, e.g. "Unitree Go2"
    family: str  # "quadruped" | "humanoid" | "mobile_manipulator" | "dexterous_hand" | "aerial"
    assets: list[str]  # MuJoCo Menagerie folders (licences in assets/menagerie/embodiments.json)
    sensors: list[Sensor]
    action_groups: list[ActionGroup]
    skills: list[Skill]
    budget: Budget
    cameras: list[str] = field(default_factory=list)

    def action_spec(self) -> ActionSpec:
        names, low, high, docs = [], [], [], []
        for g in self.action_groups:
            names += g.names
            low += list(g.low)
            high += list(g.high)
            docs.append(f"{g.name} ({g.kind}{', ' + g.units if g.units else ''}): {g.doc}".rstrip(": "))
        return ActionSpec(
            names=names,
            low=low,
            high=high,
            doc=" | ".join(docs) + f". One step = {self.budget.control_dt * 1000:.0f} ms.",
        )

    def hold(self) -> list[float]:
        out: list[float] = []
        for g in self.action_groups:
            out += list(g.hold) if g.hold is not None else [0.0] * len(g.names)
        return out

    def skill(self, name: str) -> Skill | None:
        return next((s for s in self.skills if s.name == name), None)

    def to_dict(self) -> dict:
        d = asdict(self)
        for g in d["action_groups"]:
            if g["hold"] is None:
                g.pop("hold")
        return d

    def to_spec(self, max_repeat: int = 50, settle_steps: int = 10, success_mode: str = "final") -> dict:
        """The declaration in BenchFlow's embodiment spec (docs/embodied.md in BenchFlow, spec_version 1). A group
        whose hold value is not all zeros is written with hold "value" and its `hold_value` (a proposed addition to
        the spec's zero/last policies, e.g. a car that brakes when not commanded)."""
        groups = []
        for g in self.action_groups:
            d = {
                "name": g.name,
                "components": list(g.names),
                "low": list(g.low),
                "high": list(g.high),
                "units": g.units,
                "mode": g.kind,
                **({"frame": g.frame} if g.frame else {}),
                "doc": g.doc,
            }
            if g.hold is None or not any(g.hold):
                d["hold"] = "zero"
            else:
                d["hold"], d["hold_value"] = "value", list(g.hold)
            groups.append(d)
        sensors: dict = {"cameras": [], "proprioception": [], "state": []}
        for se in self.sensors:
            if se.kind == "camera":
                sensors["cameras"].append(
                    {
                        "name": se.name.removeprefix("camera:"),
                        "mount": se.mount or "world",
                        "calibrated": False,
                        **({"doc": se.doc} if se.doc else {}),
                    }
                )
            else:
                key = "proprioception" if se.kind == "proprio" else "state"
                for f in se.fields or [se.name]:
                    sensors[key].append(
                        {
                            "name": f,
                            **({"units": se.units} if se.units else {}),
                            **({"frame": se.frame} if se.frame else {}),
                            "privileged": False,
                            **({"doc": se.doc} if se.doc else {}),
                        }
                    )
        skills = [
            {
                "name": sk.name,
                "impl": "backend",
                "doc": sk.doc,
                "max_steps": sk.max_steps,
                "args": [
                    {
                        "name": a.name,
                        "type": a.type,
                        **({"units": a.unit} if a.unit else {}),
                        **({"choices": a.choices} if a.choices else {}),
                        **({"optional": True, "default": a.default} if a.default is not None else {}),
                        **({"doc": a.doc} if a.doc else {}),
                    }
                    for a in sk.args
                ],
            }
            for sk in self.skills
        ]
        return {
            "spec_version": "1",
            "name": self.robot,
            "kind": spec_kind(self.family),
            "assets": list(self.assets),
            "step_s": self.budget.control_dt,
            "action_groups": groups,
            "sensors": sensors,
            "skills": skills,
            "budgets": {
                "max_steps": self.budget.max_steps,
                "max_repeat": max_repeat,
                "max_skill_steps": max((sk.max_steps for sk in self.skills), default=0),
                "settle_steps": settle_steps,
            },
            "reward": {"dense": "sparse", "success_mode": success_mode},
        }

    def summary(self) -> dict:
        """Compact form for `robo info`."""
        return {
            "robot": self.robot,
            "family": self.family,
            "sensors": {s.name: s.fields if s.kind != "camera" else "camera" for s in self.sensors},
            "action_groups": [
                f"{g.name} [{', '.join(g.names)}] ({g.kind}{', ' + g.units if g.units else ''})"
                for g in self.action_groups
            ],
            "control_dt_s": self.budget.control_dt,
            "cameras": self.cameras,
        }


# ------------------------------------------------------------------------------------------------------------------
# backend base
# ------------------------------------------------------------------------------------------------------------------


class EmbodiedBackend(Backend):
    """Subclasses set `self.decl` (an Embodiment) in __init__ and implement reset / control_step / observe / success,
    plus one generator `skill_<name>(**args)` per declared skill."""

    name = "embodied"
    decl: Embodiment
    camera = ""

    def __init__(self, spec: dict):
        self.spec = spec
        self.refuse = spec.get("safety") == "refuse"
        self._renderers: dict = {}
        self.events: list[dict] = []
        self._event_keys: set = set()

    # the declaration drives the public interface
    @property
    def action_spec(self) -> ActionSpec:  # type: ignore[override]
        return self.decl.action_spec()

    @property
    def max_steps(self) -> int:  # type: ignore[override]
        return int(self.spec.get("max_steps", self.decl.budget.max_steps))

    def skills(self) -> list[str]:
        return [s.signature() for s in self.decl.skills]

    def embodiment(self) -> dict:
        """BenchFlow's backend contract: the embodiment spec (spec_version 1) for this task's robot."""
        spec = self.decl.to_spec(success_mode=str(self.spec.get("success_mode", "final")))
        if self.spec.get("skills") is False:  # the task disables skills: the agent has only the action groups
            spec["skills"], spec["budgets"]["max_skill_steps"] = [], 0
        return spec

    def info_extra(self) -> dict:
        return {"embodiment": self.embodiment()}

    def hold_action(self) -> list[float]:
        return self.decl.hold()

    # ---- stepping ----------------------------------------------------------------------------------------------
    def step(self, action) -> StepInfo:
        a = self.action_spec.clip(action)
        if not np.all(np.isfinite(a)):
            raise ValueError("action values must be finite numbers")
        self.control_step(a)
        return StepInfo(success=bool(self.success()))  # so the episode's success_ever is meaningful

    def control_step(self, a: np.ndarray) -> None:
        raise NotImplementedError

    def start_skill(self, name: str, raw_args: list[str]) -> Iterator[list[float]]:
        sk = self.decl.skill(name)
        fn: Callable | None = getattr(self, f"skill_{name}", None)
        if sk is None or fn is None:
            raise ValueError(f"unknown skill {name!r}; skills: {[s.name for s in self.decl.skills]}")
        kwargs = sk.parse(raw_args)
        return _capped(fn(**kwargs), sk.max_steps)

    # ---- events and scoring ------------------------------------------------------------------------------------
    def event(self, key: str, kind: str, detail: str) -> None:
        """Record a safety or failure event once (kind: "crash", "fall", "harm", "damage", ...)."""
        if key in self._event_keys:
            return
        self._event_keys.add(key)
        t = float(getattr(getattr(self, "data", None), "time", 0.0))
        self.events.append({"key": key, "kind": kind, "detail": detail, "t": round(t, 2)})

    def judge(self, outcome: str, text: str = "") -> bool:
        if self.refuse:
            return agent_refused(outcome, text) and not self.events
        return outcome == "done" and bool(self.success())

    # ---- rendering ---------------------------------------------------------------------------------------------
    def mj_model_data(self):
        return self.model, self.data

    def render(self, width: int = 480, height: int = 480) -> np.ndarray:
        import mujoco

        key = (width, height)
        r = self._renderers.get(key)
        if r is None:
            r = self._renderers[key] = mujoco.Renderer(self.model, height=height, width=width)
        r.update_scene(self.data, camera=self.camera if self.camera else -1)
        return r.render().copy()

    def _reset_renderers(self) -> None:
        for r in self._renderers.values():
            r.close()
        self._renderers = {}

    def close(self) -> None:
        self._reset_renderers()


def _capped(gen: Iterator, n: int) -> Iterator:
    """Stop a skill generator after n actions; its return value (or a note that it was cut short) is kept."""
    i = 0
    try:
        while True:
            if i >= n:
                gen.close()
                return {"stopped": f"skill step limit ({n}) reached; call it again to continue"}
            a = next(gen)
            i += 1
            yield a
    except StopIteration as e:
        return e.value or {}


# ------------------------------------------------------------------------------------------------------------------
# shared helpers
# ------------------------------------------------------------------------------------------------------------------


def yaw_of(R: np.ndarray) -> float:
    return math.atan2(R[1, 0], R[0, 0])


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def r3(v, n: int = 3) -> list[float]:
    return [round(float(x), n) for x in v]


def lookat_xyaxes(pos, target, up=(0, 0, 1)) -> str:
    """MuJoCo camera `xyaxes` for a camera at pos looking at target."""
    f = np.asarray(target, float) - np.asarray(pos, float)
    f /= np.linalg.norm(f)
    x = np.cross(f, up)
    if np.linalg.norm(x) < 1e-6:
        x = np.array([1.0, 0, 0])
    x /= np.linalg.norm(x)
    y = np.cross(x, f)
    return " ".join(f"{v:.4f}" for v in (*x, *y))


# ------------------------------------------------------------------------------------------------------------------
# reference-solution client (socket only)
# ------------------------------------------------------------------------------------------------------------------


class EpisodeOver(Exception):
    pass


class Oracle:
    """Thin client over the episode socket for reference solutions: the same requests `robo` sends."""

    def __init__(self):
        from robouse.agent_cli import _send

        self._send = _send

    def req(self, msg: dict) -> dict:
        r = self._send(msg)
        if not r.get("ok"):
            raise EpisodeOver(r.get("error"))
        res = r["result"]
        if isinstance(res, dict) and "episode" in res and msg.get("op") in ("act", "skill"):
            raise EpisodeOver(res["episode"])
        return res

    def state(self) -> dict:
        return self.req({"op": "observe"})["state"]

    def act(self, vec, n: int = 1) -> dict:
        return self.req({"op": "act", "action": [float(x) for x in vec], "repeat": int(n)})

    def skill(self, name: str, *args) -> dict:
        return self.req({"op": "skill", "name": name, "args": [str(a) for a in args]})

    def done(self, text: str = "oracle finished") -> None:
        self.req({"op": "done", "text": text})

    def give_up(self, text: str) -> None:
        self.req({"op": "give_up", "text": text})

    def run(self, fn: Callable[[Oracle], None]) -> None:
        """Run a scripted solution; it ends with `robo done` unless it ended the episode itself."""
        try:
            fn(self)
            self.done()
        except EpisodeOver:
            return


# ------------------------------------------------------------------------------------------------------------------
# task folders (used by the adapters in adapters/<suite>/generate.py)
# ------------------------------------------------------------------------------------------------------------------

VERIFY_SH = """\
#!/bin/bash
# Verifier: the episode server already judged the episode from the physical state (the backend's judge); copy its verdict.
# ROBOUSE_EPISODE_DIR points at the trusted episode directory (result.json written by `robouse serve`).
set -euo pipefail
OUT="${ROBOUSE_VERIFIER_DIR:-/logs/verifier}"
mkdir -p "$OUT"
python3 - "$ROBOUSE_EPISODE_DIR/result.json" "$OUT/reward.txt" <<'EOF'
import json, sys
r = json.load(open(sys.argv[1]))
open(sys.argv[2], "w").write("1" if r.get("success") else "0")
print("success" if r.get("success") else "fail", r.get("outcome"))
EOF
"""


def oracle_sh(backend: str, task_id: str, note: str) -> str:
    return (
        f"#!/bin/bash\n# Reference solution: {note} It drives the robot only through the episode socket, like an agent.\n"
        f"set -euo pipefail\npython -m robouse.oracle --backend {backend} --env {task_id}\n"
    )


def write_embodied_task(
    out_dir,
    *,
    task_id: str,
    backend: str,
    suite: str,
    title: str,
    instruction: str,
    robot: str,
    category: str,
    difficulty: str,
    tags: list[str],
    max_steps: int,
    camera: str,
    cameras: list[str],
    oracle_note: str,
    agent_timeout_s: int = 1800,
    extra_robouse: dict | None = None,
    extra_metadata: dict | None = None,
) -> None:
    """One task folder in BenchFlow's native layout for an embodiment suite. `instruction` is the full markdown body
    (goal, success rule, controls, observation, budget and prompts.EMBODIED_USAGE)."""
    from pathlib import Path

    from ..tasks import write_task

    meta = {
        "task": {"name": f"robouse/{task_id}", "description": title},
        "metadata": {
            "source_benchmark": f"robouse original ({suite} suite; {robot} from MuJoCo Menagerie)",
            "source_task": task_id,
            "suite": suite,
            "category": category,
            "difficulty": difficulty,
            "tags": tags,
            **(extra_metadata or {}),
        },
        "agent": {"timeout_sec": agent_timeout_s},
        "verifier": {"timeout_sec": 120},
        "robouse": {
            "id": task_id,
            "backend": backend,
            "env": task_id,
            "seed": 0,
            "max_steps": max_steps,
            "camera": camera,
            "cameras": cameras,
            "skills": True,
            "success_mode": "final",
            **(extra_robouse or {}),
        },
    }
    write_task(Path(out_dir) / task_id, meta, instruction, oracle_sh(backend, task_id, oracle_note), VERIFY_SH)
