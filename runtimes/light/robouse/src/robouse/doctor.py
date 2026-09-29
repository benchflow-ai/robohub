"""`robouse doctor`: what this machine can run. Checks each simulator backend's imports (or its worker virtualenv), the
harness command-line tools and credentials, the VLA client, the real-robot drivers' SDKs, and registered plugins.
Read-only: nothing is installed or started."""
from __future__ import annotations

import importlib
import importlib.util
import os
import shutil
from pathlib import Path

WORKER_VENVS = {"dexjoco": "dexjoco", "driving": "metadrive", "dmcontrol": "dmcontrol", "myosuite": "myosuite",
                "humanoidbench": "humanoidbench", "maniskill": "maniskill", "robocasa": "robocasa"}
BACKEND_MODULES = {"tabletop": "tabletop", "metaworld": "metaworld_backend", "gymrobotics": "gymrobotics", "libero": "libero",
                   "robosuite": "robosuite_backend", "menagerie": "menagerie", "roboharm": "roboharm", "drone": "drone",
                   "quadruped": "quadruped", "humanoid": "humanoid", "mobile_manip": "mobile_manip", "dexhand": "dexhand",
                   "crazyflie": "crazyflie", "kitchen": "kitchen", "adroit": "adroit", "nav": "nav", "cubepick": "cubepick",
                   "real": "real"}
UPSTREAM_PKGS = {"metaworld": "metaworld", "gymrobotics": "gymnasium_robotics", "libero": "libero", "robosuite": "robosuite",
                 "kitchen": "gymnasium_robotics", "adroit": "gymnasium_robotics"}


def _ok(flag: bool) -> str:
    return "ok " if flag else "-- "


def run() -> int:
    print("Simulator backends")
    for b, mod in BACKEND_MODULES.items():
        try:
            importlib.import_module(f"robouse.backends.{mod}")
            pkg = UPSTREAM_PKGS.get(b)
            ready = pkg is None or importlib.util.find_spec(pkg) is not None
            print(f"  {_ok(ready)}{b}" + ("" if ready else f" (missing package {pkg})"))
        except Exception as e:  # noqa: BLE001
            print(f"  -- {b} ({type(e).__name__}: {str(e)[:80]})")
    for b, v in WORKER_VENVS.items():
        py = Path(os.environ.get(f"ROBOUSE_{b.upper()}_PYTHON", Path.home() / f".cache/robouse/{v}-venv/bin/python"))
        print(f"  {_ok(py.exists())}{b} (worker virtualenv {py})")
    print("Harnesses")
    for h, exe in (("claude-code, claude-code-glm", "claude"), ("codex, codex-glm", "codex"), ("mini-swe-agent-glm", "mini")):
        print(f"  {_ok(shutil.which(exe) is not None)}{h} (`{exe}` on PATH)")
    maint = Path.home() / ".config/bf-agents"
    creds = {"ANTHROPIC_API_KEY or CLAUDE_CODE_OAUTH_TOKEN (or the maintainers' oauth.env)": bool(os.environ.get("ANTHROPIC_API_KEY") or os.environ.get("CLAUDE_CODE_OAUTH_TOKEN") or (maint / "oauth.env").exists()),
             "OPENAI_API_KEY or ~/.codex/auth.json": bool(os.environ.get("OPENAI_API_KEY") or (Path.home() / ".codex/auth.json").exists()),
             "BASETEN_API_KEY (or the maintainers' baseten.env)": bool(os.environ.get("BASETEN_API_KEY") or (maint / "baseten.env").exists())}
    for k, v in creds.items():
        print(f"  {_ok(v)}{k}")
    print(f"  {_ok(bool(os.environ.get('ROBOUSE_VLA_URL')))}vla (ROBOUSE_VLA_URL {os.environ.get('ROBOUSE_VLA_URL', 'not set')})")
    print("Real robots")
    for name, pkg in (("lerobot (Metal, SO-100/101)", "lerobot"), ("piper_sdk (Piper)", "piper_sdk"), ("OpenCV (cameras)", "cv2")):
        print(f"  {_ok(importlib.util.find_spec(pkg) is not None)}{name}")
    from .real.safety import estop_engaged

    print(f"  {'!! ' if estop_engaged() else 'ok '}e-stop {'ENGAGED: ' + str(estop_engaged()) if estop_engaged() else 'clear'}")
    print(f"  {_ok(bool(os.environ.get('ROBOUSE_RIG')))}rig file {os.environ.get('ROBOUSE_RIG', '(none: real tasks use the hardware-in-the-loop mock)')}")
    from importlib.metadata import entry_points

    plugins = {g: [e.name for e in entry_points(group=g)] for g in ("robouse.backends", "robouse.harnesses", "robouse.drivers")}
    print("Plugins: " + ", ".join(f"{g}: {v or 'none'}" for g, v in plugins.items()))
    return 0
