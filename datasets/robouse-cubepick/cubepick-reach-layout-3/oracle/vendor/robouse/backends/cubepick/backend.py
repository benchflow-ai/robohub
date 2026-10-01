"""CubePick: inspect-robots' dependency-free 2-D reach world (robocurve/inspect-robots, src/inspect_robots/mock/
cubepick.py, MIT), ported so the same `cubepick-reach` benchmark runs under Robo Use.

A point end effector starts at (0.1, 0.1) in the unit square and must come within 0.05 of a cube placed by
numpy.random.RandomState(seed).uniform(0.6, 0.9, 2), exactly as upstream, so seed n gives upstream's `layout-n`.
Action: (dx, dy) clipped to +-0.1 per step; success the first time the distance is <= 0.05 (upstream terminates there).
Camera `top`: upstream's 32x32 image (cube green, effector red, +x right, +y down), upscaled 8x.
"""

from __future__ import annotations

import numpy as np

from ..base import ActionSpec, Backend, StepInfo


class CubePickBackend(Backend):
    name = "cubepick"

    def __init__(self, spec: dict):
        self.spec = spec
        self.max_step, self.goal_radius = 0.1, 0.05
        self.start = np.array([0.1, 0.1])
        self.camera = "top"
        self.max_steps = int(spec.get("max_steps", 80))
        self.action_spec = ActionSpec(
            ["dx", "dy"],
            [-0.1, -0.1],
            [0.1, 0.1],
            doc="end-effector position change in the unit square, at most 0.1 per axis per step",
        )

    def reset(self, seed: int) -> None:
        self.cube = np.random.RandomState(seed).uniform(0.6, 0.9, size=2)
        self.eef = self.start.copy()

    def step(self, action) -> StepInfo:
        d = np.clip(np.asarray(action, dtype=float), -self.max_step, self.max_step)
        self.eef = np.clip(self.eef + d, 0.0, 1.0)
        dist = float(np.linalg.norm(self.eef - self.cube))
        return StepInfo(success=dist <= self.goal_radius, reward=-dist)

    def success(self) -> bool:
        return float(np.linalg.norm(self.eef - self.cube)) <= self.goal_radius

    def observe(self) -> dict:
        return {
            "eef_pos": [round(float(x), 4) for x in self.eef],
            "cube_pos": [round(float(x), 4) for x in self.cube],
            "distance": round(float(np.linalg.norm(self.eef - self.cube)), 4),
        }

    def metrics_point(self):
        return [float(self.eef[0]), float(self.eef[1]), 0.0]

    def render(self, width: int = 256, height: int = 256) -> np.ndarray:
        n = 32
        img = np.zeros((n, n, 3), dtype=np.uint8)
        cc, cr = (np.clip(self.cube, 0, 1) * (n - 1)).astype(int)
        ec, er = (np.clip(self.eef, 0, 1) * (n - 1)).astype(int)
        img[cr, cc] = (0, 200, 0)
        img[er, ec] = (200, 0, 0)
        return np.kron(img, np.ones((8, 8, 1), dtype=np.uint8))

    def skills(self) -> list[str]:
        return []


def oracle_main(env: str) -> None:
    """Upstream's scripted policy: step straight toward the cube (read from the public state)."""
    from ...agent_cli import _send

    for _ in range(200):
        r = _send({"op": "observe"})
        if not r.get("ok"):
            return
        st = r["result"]["state"]
        d = np.clip(np.asarray(st["cube_pos"]) - np.asarray(st["eef_pos"]), -0.1, 0.1)
        r = _send({"op": "act", "action": [float(x) for x in d]})
        if not r.get("ok") or "episode" in (r.get("result") or {}):
            return
    _send({"op": "done", "text": "oracle"})
