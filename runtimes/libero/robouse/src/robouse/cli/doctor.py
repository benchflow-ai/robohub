"""`robouse doctor`: what this machine can run. Checks each simulator backend's imports (or its worker virtualenv), the
harness command-line tools and credentials, the VLA client, the real-robot drivers' SDKs, and registered plugins.
Read-only: nothing is installed or started."""

from __future__ import annotations

import importlib
import importlib.util
import shutil

from .. import config
from ..harnesses import credentials

# backend -> (virtualenv name, $ROBOUSE_<NAME>_PYTHON) for simulators that run as workers in their own virtualenv
WORKER_VENVS = {
    "dexjoco": "dexjoco",
    "driving": "metadrive",
    "dmcontrol": "dmcontrol",
    "myosuite": "myosuite",
    "humanoidbench": "humanoidbench",
    "maniskill": "maniskill",
    "robocasa": "robocasa",
}
REMOTE = {"behavior", "gpu", "robodojo"}  # simulators on remote GPU workers
UPSTREAM_PKGS = {
    "metaworld": "metaworld",
    "gymrobotics": "gymnasium_robotics",
    "libero": "libero",
    "robosuite": "robosuite",
    "kitchen": "gymnasium_robotics",
    "adroit": "gymnasium_robotics",
}


def _ok(flag: bool) -> str:
    return "ok " if flag else "-- "


def run() -> int:
    print("Simulator backends")
    from ..backends import BACKENDS

    # every in-process backend needs the simulator extra (`pip install 'robouse[sim]'`)
    no_sim = [m for m in ("mujoco", "imageio", "PIL") if importlib.util.find_spec(m) is None]
    for b, (mod, _cls) in BACKENDS.items():
        if b in WORKER_VENVS or b in REMOTE or b == "remix":
            continue
        if no_sim:
            print(f"  -- {b} (missing package {', '.join(no_sim)}: pip install 'robouse[sim]')")
            continue
        try:
            importlib.import_module(f"robouse.backends.{mod}")
            pkg = UPSTREAM_PKGS.get(b)
            ready = pkg is None or importlib.util.find_spec(pkg) is not None
            print(f"  {_ok(ready)}{b}" + ("" if ready else f" (missing package {pkg})"))
        except Exception as e:  # noqa: BLE001
            print(f"  -- {b} ({type(e).__name__}: {str(e)[:80]})")
    for b, venv in WORKER_VENVS.items():
        py = config.sim_python(venv)
        print(f"  {_ok(py.exists())}{b} (worker virtualenv {py})")
    print("Harnesses")
    for h, exe in (
        ("claude-code, claude-code-glm", "claude"),
        ("codex, codex-glm", "codex"),
        ("mini-swe-agent-glm", "mini"),
    ):
        print(f"  {_ok(shutil.which(exe) is not None)}{h} (`{exe}` on PATH)")
    creds = {
        "ANTHROPIC_API_KEY or CLAUDE_CODE_OAUTH_TOKEN": credentials.have_claude(),
        "OPENAI_API_KEY or ~/.codex/auth.json": credentials.have_codex(),
        "BASETEN_API_KEY": credentials.have_baseten(),
    }
    for k, v in creds.items():
        print(f"  {_ok(v)}{k}")
    print(
        f"  {_ok(bool(config.env('ROBOUSE_VLA_URL')))}vla (ROBOUSE_VLA_URL {config.env('ROBOUSE_VLA_URL', 'not set')})"
    )
    print("Real robots")
    for name, pkg in (
        ("lerobot (Metal, SO-100/101)", "lerobot"),
        ("piper_sdk (Piper)", "piper_sdk"),
        ("OpenCV (cameras)", "cv2"),
    ):
        print(f"  {_ok(importlib.util.find_spec(pkg) is not None)}{name}")
    from ..real.safety import estop_engaged

    print(
        f"  {'!! ' if estop_engaged() else 'ok '}e-stop {'ENGAGED: ' + str(estop_engaged()) if estop_engaged() else 'clear'}"
    )
    print(
        f"  {_ok(bool(config.env('ROBOUSE_RIG')))}rig file {config.env('ROBOUSE_RIG', '(none: real tasks use the hardware-in-the-loop mock)')}"
    )
    from importlib.metadata import entry_points

    plugins = {
        g: [e.name for e in entry_points(group=g)] for g in ("robouse.backends", "robouse.harnesses", "robouse.drivers")
    }
    print("Plugins: " + ", ".join(f"{g}: {v or 'none'}" for g, v in plugins.items()))
    return 0
