"""Per-step physics samples for the episode metrics (core/metrics.py): the end-effector point, the simulator time and the
largest contact force on the robot. Sampling never fails a step."""

from __future__ import annotations

import json
import logging
from pathlib import Path
from typing import Any

import numpy as np

log = logging.getLogger(__name__)


class PhysicsProbe:
    def __init__(self, backend: Any, spec: dict):
        self.backend, self.spec = backend, spec
        self.samples: list[dict] = []
        self._geoms: Any = None
        md = getattr(backend, "mj_model_data", None)
        try:
            self._mujoco = callable(md) and md() is not None
        except Exception:  # noqa: BLE001 - some worker backends cannot answer before their first step
            self._mujoco = False

    def sample(self, step: int) -> None:
        b = self.backend
        rec: dict[str, Any] = {"i": step}
        try:
            if callable(getattr(b, "metrics_point", None)):
                pt = b.metrics_point()
            elif self._mujoco and callable(getattr(b, "hand_pos", None)):
                pt = b.hand_pos()
            else:
                pt = None
            if pt is not None:
                rec["ee"] = [round(float(x), 5) for x in pt]
            if self._mujoco:
                from .metrics import max_robot_contact_force, robot_geoms

                m, d = b.mj_model_data()
                if self._geoms is None:
                    self._geoms = robot_geoms(m)
                rec["t"] = round(float(d.time), 5)
                rec["f"] = round(max_robot_contact_force(m, d, self._geoms), 3)
        except Exception as e:  # noqa: BLE001 - metrics never change the episode
            log.debug("physics sample at step %d: %s: %s", step, type(e).__name__, e)
        self.samples.append(rec)

    def control_dt(self) -> float | None:
        """Seconds per env step: measured from the simulator clock, else the backend's or the task's declaration."""
        ts = [r["t"] for r in self.samples if "t" in r]
        if len(ts) >= 3:
            d = np.diff(ts)
            d = d[d > 0]
            if len(d):
                return float(np.median(d))
        decl = getattr(self.backend, "decl", None)
        if decl is not None and getattr(getattr(decl, "budget", None), "control_dt", None):
            return float(decl.budget.control_dt)
        if getattr(self.backend, "hz", None):
            return 1.0 / float(self.backend.hz)
        return float(self.spec["control_dt"]) if self.spec.get("control_dt") else None

    def write(self, path: Path) -> None:
        with open(path, "w") as f:
            for r in self.samples:
                f.write(json.dumps(r) + "\n")
