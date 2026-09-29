"""Generic VLA policy-server client (`vla` harness): presets.json + client.py. See docs/vla.md."""
from __future__ import annotations

import json
from pathlib import Path

PRESETS = Path(__file__).with_name("presets.json")
CLIENT = Path(__file__).with_name("client.py")


def presets() -> dict:
    return {k: v for k, v in json.loads(PRESETS.read_text()).items() if not k.startswith("_")}


def resolve(name: str, overrides: dict | None = None) -> dict:
    """A preset with `inherit` chains resolved (top-level keys; later wins), then the overrides merged on top."""
    ps = presets()
    if name not in ps:
        raise KeyError(f"unknown VLA preset {name!r}; presets: {sorted(ps)}")
    chain, n = [], name
    while n:
        chain.append(ps[n])
        n = ps[n].get("inherit")
    cfg: dict = {}
    for layer in reversed(chain):
        cfg.update({k: v for k, v in layer.items() if k != "inherit"})
    cfg.update(overrides or {})
    cfg["preset"] = name
    return cfg
