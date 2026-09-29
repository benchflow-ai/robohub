"""Backend interface: one simulator family (Meta-World, custom MuJoCo scenes, ...) behind a common API.

The session server only talks to this interface. A backend exposes:
- a low-level action (a fixed-length vector with documented bounds),
- a public observation (named fields the agent may see, plus an optional camera image),
- a success predicate evaluated by the trusted server, never by the agent.
Skills (move_to, grip) are optional helpers built on the low-level action.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np


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


class Backend:
    """Subclasses implement reset/step/observe/render/success."""

    name = "base"
    action_spec: ActionSpec
    max_steps: int = 500

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

    # Optional skills. Default implementations raise so tasks can disable them.
    def skills(self) -> list[str]:
        return []

    def close(self) -> None:
        pass
