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


def gateway_env_vars(provider: str) -> list[str]:
    """The variables the model gateway needs for a provider, in the environment: its key and any base URL without a
    default (harnesses/gateway.py checks exactly these). [] when the registry is not installed or lacks the provider."""
    from ..harnesses import models

    try:
        return models.required_env(models.provider_registry()[provider])
    except (RuntimeError, KeyError):  # BenchFlow (the llm extra) not installed
        return []


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
    for h, exe in (("claude-code", "claude"), ("codex", "codex")):
        print(f"  {_ok(shutil.which(exe) is not None)}{h} (`{exe}` on PATH)")
    from ..harnesses.gateway import litellm_exe

    llm = litellm_exe()  # where the gateway finds it: next to this Python (the llm extra), else on PATH
    print(
        f"  {_ok(bool(llm))}model gateway (provider-prefixed models) ("
        + (llm if llm else "`litellm`: pip install 'robouse[llm]'")
        + ")"
    )
    from ..harnesses import mini_exe

    mini = mini_exe()  # where the harness finds it: on PATH or next to this Python (not a dependency of robouse)
    print(f"  {_ok(bool(mini))}mini-swe-agent (" + (mini if mini else "`mini`: pip install mini-swe-agent") + ")")
    dim = config.env("ROBOUSE_DIMCODE") or shutil.which("dimcode")
    print(f"  {_ok(bool(dim))}dimcode (" + (dim if dim else "`dimcode` on PATH, or ROBOUSE_DIMCODE") + ")")
    print(
        f"  {_ok(shutil.which('python3') is not None)}show-harness (`python3` on PATH; a provider-prefixed model that "
        "reads images, through the model gateway, or a self-hosted one at ROBOUSE_SHOW_HARNESS_URL)"
    )
    creds = {
        "ANTHROPIC_API_KEY or CLAUDE_CODE_OAUTH_TOKEN": credentials.have_claude(),
        "OPENAI_API_KEY or ~/.codex/auth.json": credentials.have_codex(),
    }
    for k, v in creds.items():
        print(f"  {_ok(v)}{k}")
    from ..harnesses import models

    default = models.ZAI_CODING_PLAN_MODEL
    provider = default.split("/", 1)[0]
    if names := gateway_env_vars(provider):
        print(
            f"  {_ok(all(os.environ.get(v) for v in names))}{' and '.join(names)} in the environment (what the model "
            f"gateway reads for {provider}/ models, such as the default {default}; no key file is read)"
        )
    else:
        print(f"  -- the model gateway's provider registry (pip install 'robouse[llm]'; for {default} and the like)")
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
