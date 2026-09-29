"""Placement splits for spatial generalization: seeded object placements, identical for every policy given the seed.

  nominal        one of 5 "training" placements: the task's own placement or a 2 cm diagonal shift (seed % 5)
  interpolation  unseen placements inside the square spanned by the nominal ones (|dx|, |dy| <= 2 cm, not within
                 5 mm of a nominal placement)
  extrapolation  unseen placements outside that square: 3-6 cm from the task's own placement, any direction

Offsets are drawn per object from a generator seeded by (split, seed, object name), so a (task, split, seed) triple
always yields the same scene. Backends that support splits (tabletop pick-and-place scenes) apply them to the movable
objects' start positions, reject draws that leave the table or collide, and record them in result.json `placement`.
"""
from __future__ import annotations

import hashlib

import numpy as np

D = 0.02
NOMINAL = [(0.0, 0.0), (D, D), (D, -D), (-D, D), (-D, -D)]
SPLITS = ("nominal", "interpolation", "extrapolation")


def offset(split: str, seed: int, name: str, attempt: int = 0) -> tuple[float, float]:
    if split not in SPLITS:
        raise ValueError(f"placement split must be one of {SPLITS}, not {split!r}")
    h = int(hashlib.sha256(f"{split}:{seed}:{name}:{attempt}".encode()).hexdigest()[:12], 16)
    rng = np.random.default_rng(h)
    if split == "nominal":
        return NOMINAL[(seed + attempt) % len(NOMINAL)]
    if split == "interpolation":
        for _ in range(100):
            dx, dy = rng.uniform(-D, D, size=2)
            if min(np.hypot(dx - a, dy - b) for a, b in NOMINAL) > 0.005:
                return float(dx), float(dy)
        return 0.01, 0.0
    r, th = rng.uniform(1.5 * D, 3 * D), rng.uniform(0, 2 * np.pi)
    return float(r * np.cos(th)), float(r * np.sin(th))
