"""Backend interface: one simulator family (Meta-World, custom MuJoCo scenes, ...) behind a common API.

The episode server (BenchFlow's `benchflow.embodied.server`, through `robouse.embodied.RobouseSim`) only talks to
this interface. A backend exposes:
- a low-level action: a fixed-length vector with documented bounds (`action_spec`), which the backend's
  embodiment splits into named action groups (`embodiment()`, see docs/embodied.md in BenchFlow),
- a public observation (named fields the agent may see, plus camera images),
- a success predicate evaluated by the trusted server, never by the agent.
Skills (move_to, grip) are BenchFlow's built-in controllers on top of `hand_pos()` and the low-level action.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np
from benchflow.embodied.spec import (
    ActionGroup,
    Budgets,
    Camera,
    Embodiment,
    Field,
    RewardSpec,
    Sensors,
    Skill,
    SkillArg,
    grip_skill,
    move_to_skill,
)


@dataclass
class ActionSpec:
    names: list[str]
    low: list[float]
    high: list[float]
    doc: str = ""

    @property
    def dim(self) -> int:
        return len(self.names)

    def clip(self, a) -> np.ndarray:
        return np.clip(np.asarray(a, dtype=np.float64), self.low, self.high)


@dataclass
class StepInfo:
    success: bool
    reward: float = 0.0
    extra: dict[str, Any] = field(default_factory=dict)


@dataclass
class G:
    """One action group of a backend's layout: `n` consecutive entries of the flat action vector."""

    name: str
    n: int
    mode: str
    units: str = ""
    hold: str = "zero"
    initial: float | None = None
    frame: str = ""
    doc: str = ""


def groups_from_layout(spec: ActionSpec, layout: list[G]) -> list[ActionGroup]:
    """Split the flat ActionSpec into named groups, in order. The layout must cover every entry exactly once."""
    out, i = [], 0
    for g in layout:
        comps = spec.names[i:i + g.n]
        if len(comps) != g.n:
            raise ValueError(f"layout {g.name} needs {g.n} entries, {spec.dim - i} left")
        out.append(ActionGroup(
            name=g.name, components=list(comps), low=[float(x) for x in spec.low[i:i + g.n]],
            high=[float(x) for x in spec.high[i:i + g.n]], mode=g.mode, units=g.units, frame=g.frame, hold=g.hold,
            initial=None if g.initial is None else [float(g.initial)] * g.n, doc=g.doc))
        i += g.n
    if i != spec.dim:
        raise ValueError(f"layout covers {i} of {spec.dim} action entries")
    return out


def arm_layout(delta_units: str, grip_hold: str = "last", grip_initial: float | None = -1.0, prefix: str = "arm",
               gripper: str = "gripper", rotation: str | None = None, grip_doc: str = "+1 close, -1 open") -> list[G]:
    lay = [G(f"{prefix}.ee_delta", 3, "ee_delta_pos", delta_units, frame="world")]
    if rotation:
        lay.append(G(f"{prefix}.ee_rot_delta", 3, "ee_delta_rot", rotation, frame="world"))
    lay.append(G(gripper, 1, "gripper", "normalized", hold=grip_hold, initial=grip_initial, doc=grip_doc))
    return lay


BEHAVIOR_SKILLS = ["navigate_to", "grasp", "place_on_top", "place_inside", "open", "close", "toggle_on", "toggle_off"]


class Backend:
    """Subclasses implement reset/step/observe/render/success, and describe their robot with the class attributes
    below (or override `embodiment()` / `action_layout()`)."""

    name = "base"
    action_spec: ActionSpec
    max_steps: int = 500
    camera: str = ""
    robot_kind: str = "arm"  # one of benchflow.embodied.spec.KINDS
    robot_name: str = ""
    dense_reward: str = "sparse"  # "shaped" when step() returns the simulator's own shaped reward
    proprioception: tuple[str, ...] = ("hand_pos", "gripper_open")
    probe_success: bool = True  # the server may call success() after every step for the dense trace
    arm_prefix: str = "arm"

    def reset(self, seed: int) -> None:
        raise NotImplementedError

    def step(self, action: np.ndarray) -> StepInfo:
        raise NotImplementedError

    def observe(self) -> dict[str, Any]:
        """Public, JSON-serialisable state. Must not leak anything the task hides from the agent."""
        raise NotImplementedError

    def render(self, width: int = 320, height: int = 320) -> np.ndarray:
        raise NotImplementedError

    def success(self) -> bool:
        raise NotImplementedError

    # Skills listed by name for protocol-1 clients (`robo info` -> skills). Built-in controllers implement them.
    def skills(self) -> list[str]:
        return []

    def close(self) -> None:
        pass

    # ---- embodiment ----------------------------------------------------------------------------------------
    def action_layout(self) -> list[G]:
        """How the flat action splits into groups. Default: one arm with a gripper when the action is
        [dx, dy, dz, grip], else a single `action` group."""
        s = self.action_spec
        if s.dim == 4 and [n.lower() for n in s.names] == ["dx", "dy", "dz", "grip"]:
            return arm_layout("see doc")
        return [G("action", s.dim, "other")]

    def skill_specs(self) -> list[Skill]:
        names = self.skills()
        groups = {g.name for g in self.action_layout()}
        gripper = "gripper" if "gripper" in groups else None
        arm_group = f"{self.arm_prefix}.ee_delta"
        out: list[Skill] = []
        if "move_to" in names:
            sk = move_to_skill(self.arm_prefix, gripper, aliases=["move_to", "move-to"])
            if arm_group not in groups:  # the backend servoes its own active arm through the skill action
                sk.binds = {}
            if gripper is None and "grip" in names:  # the skill action's 4th number is the hand's grasp preset
                sk.args.insert(3, SkillArg("grip", "float", optional=True, doc="grasp preset held while moving"))
            out.append(sk)
        if "grip" in names:
            sk = grip_skill("gripper", aliases=["grip"])
            if gripper is None:
                sk.name, sk.binds = f"{self.arm_prefix}.grasp", {}
                sk.doc = "Hold the hand still and set its grasp preset (+1 close, -1 open) for a number of steps."
            out.append(sk)
        return out

    def embodiment(self) -> Embodiment:
        s = self.action_spec
        return Embodiment(
            name=self.robot_name or self.name,
            kind=self.robot_kind,
            action_groups=groups_from_layout(s, self.action_layout()),
            sensors=Sensors(
                cameras=[Camera(self.camera)] if self.camera else [],
                proprioception=[Field(n) for n in self.proprioception],
            ),
            skills=self.skill_specs(),
            budgets=Budgets(max_steps=int(self.max_steps)),
            reward=RewardSpec(dense=self.dense_reward),
            doc=s.doc,
        )


def behavior_skill_specs(names: list[str]) -> list[Skill]:
    out = [Skill(n, [SkillArg("obj", "object", doc="BDDL object instance name from robo observe")], impl="backend",
                 doc=f"BEHAVIOR symbolic primitive {n}; one skill call is one step of the budget") for n in names]
    out.append(Skill("release", impl="backend", doc="open the hand and drop what it holds"))
    return out
