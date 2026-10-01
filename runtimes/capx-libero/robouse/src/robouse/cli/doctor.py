"""`robouse doctor`: what this machine can run. Checks each simulator backend's imports (or its worker virtualenv), the
harness command-line tools and credentials, the VLA client, the real-robot drivers' SDKs, registered plugins, and whether
MuJoCo can render offscreen. Read-only: nothing is installed; the rendering check runs MuJoCo once in a short-lived
process."""

from __future__ import annotations

import importlib
import importlib.util
import os
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


def gateway_key_var(provider: str) -> str:
    """The variable the model gateway reads a provider's key from: its `auth_env` in the provider registry
    (harnesses/gateway.py checks exactly that, in the environment)."""
    from ..harnesses import models

    try:
        return str(models.provider_registry()[provider].auth_env)
    except (RuntimeError, KeyError, AttributeError):  # BenchFlow (the llm extra) not installed: the pending entry
        return str(models.PENDING_PROVIDERS[provider]["auth_env"])


def _rendering() -> None:
    """Every simulator records its episode video with MuJoCo's renderer, which needs an OpenGL platform."""
    import os

    from ..core import gl

    print("Offscreen rendering")
    if importlib.util.find_spec("mujoco") is None:
        print("  -- MuJoCo is not installed (pip install 'robouse[sim]')")
        return
    picked = gl.auto_select()  # what `robouse run` does when MUJOCO_GL is unset on Linux without a display
    why = gl.probe()
    setting = os.environ.get("MUJOCO_GL") or "unset, so GLFW, which needs a display"
    if why is None:
        print(
            f"  ok MuJoCo renders offscreen (MUJOCO_GL={setting}"
            + (", chosen because it was unset" if picked else "")
            + ")"
        )
    else:
        print(f"  -- MuJoCo cannot render offscreen (MUJOCO_GL={setting}): {why}. {gl.HINT}")


def run() -> int:
    _rendering()
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
        ("claude-code", "claude"),
        ("codex", "codex"),
        ("mini-swe-agent", "mini"),
        ("model gateway (provider-prefixed models)", "litellm"),
    ):
        print(f"  {_ok(shutil.which(exe) is not None)}{h} (`{exe}` on PATH)")
    dim = config.env("ROBOUSE_DIMCODE") or shutil.which("dimcode")
    print(f"  {_ok(bool(dim))}dimcode (" + (dim if dim else "`dimcode` on PATH, or ROBOUSE_DIMCODE") + ")")
    creds = {
        "ANTHROPIC_API_KEY or CLAUDE_CODE_OAUTH_TOKEN": credentials.have_claude(),
        "OPENAI_API_KEY or ~/.codex/auth.json": credentials.have_codex(),
    }
    for k, v in creds.items():
        print(f"  {_ok(v)}{k}")
    var = gateway_key_var("baseten")
    print(
        f"  {_ok(bool(os.environ.get(var)))}{var} in the environment "
        "(what the model gateway reads for baseten/ models; no key file is read)"
    )
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
