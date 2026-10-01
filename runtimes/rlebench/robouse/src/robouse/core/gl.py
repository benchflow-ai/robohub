"""Offscreen OpenGL for MuJoCo: choose a platform that works on Linux without a display, check it, and say in one line
what to install when there is none.

MuJoCo picks its OpenGL platform from MUJOCO_GL when it is first imported; unset, it uses GLFW, which needs a display.
So the choice is made, and probed, in a fresh interpreter before any simulator starts, and passed on in the environment.
"""

from __future__ import annotations

import os
import re
import subprocess
import sys

from ..lib.text import clip

HINT = (
    "On Linux without a display, install OSMesa (`apt install libosmesa6`) and set MUJOCO_GL=osmesa, or EGL "
    "(`apt install libegl1 libgl1`) and set MUJOCO_GL=egl; on macOS set MUJOCO_GL=cgl"
)
# tried in this order when MUJOCO_GL is unset: OSMesa is what the hub's simulator images use; EGL is the GPU path
CANDIDATES = ("osmesa", "egl")
_PROBE = "import mujoco; m = mujoco.MjModel.from_xml_string('<mujoco/>'); r = mujoco.Renderer(m, 8, 8); r.render(); r.close()"
# how a missing or unusable OpenGL platform shows up: MuJoCo's own error, the follow-on error of its half-built Renderer,
# GLFW without a display, PyOpenGL without the platform library, a MUJOCO_GL / PYOPENGL_PLATFORM mismatch
_SIGNS = (
    "OpenGL platform library has not been loaded",
    "mjr_makeContext",
    "_mjr_context",
    "Could not create GL context",
    "GLFWError",
    "rendering platform. The PYOPENGL_PLATFORM",
    "invalid value for environment variable MUJOCO_GL",
    "attribute 'glGetError'",
    "attribute 'eglQueryString'",
    "Unable to load OpenGL library",
    "Unable to load EGL library",
    "libOSMesa",
    "libEGL",
)


def headless_linux() -> bool:
    return sys.platform.startswith("linux") and not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))


def reason(text: str) -> str:
    """The line of an error output that says why MuJoCo could not render (not the follow-on AttributeError its
    half-built Renderer prints on the way out)."""
    lines = [ln.strip() for ln in str(text).splitlines() if ln.strip()]
    errs = [ln for ln in lines if re.match(r"^[\w.]+(Error|Exception)\b", ln) and "_mjr_context" not in ln]
    return clip((errs or lines or ["unknown error"])[0], 300)


def probe(platform: str | None = None, python: str | None = None) -> str | None:
    """None when MuJoCo renders offscreen with MUJOCO_GL=platform (None: as the environment has it), else why not."""
    env = dict(os.environ)
    if platform is not None:
        env["MUJOCO_GL"] = platform
    try:
        p = subprocess.run(
            [python or sys.executable, "-c", _PROBE],
            env=env,
            stdin=subprocess.DEVNULL,
            capture_output=True,
            text=True,
            timeout=120,
        )
    except (OSError, subprocess.TimeoutExpired) as e:
        return f"{type(e).__name__}: {e}"
    return None if p.returncode == 0 else reason(p.stderr)


def auto_select() -> str | None:
    """With MUJOCO_GL unset on Linux without a display, set it to the first platform that renders (OSMesa, then EGL), so
    the simulators this process starts render offscreen. Returns the platform it set, or None."""
    if os.environ.get("MUJOCO_GL") or not headless_linux():
        return None
    for platform in CANDIDATES:
        if probe(platform) is None:
            os.environ["MUJOCO_GL"] = platform
            return platform
    return None


def explain(error: object) -> str | None:
    """A one-line message with the fix when `error` (an exception or its text) is MuJoCo failing to get an OpenGL
    context; None for any other error."""
    text = f"{type(error).__name__}: {error}" if isinstance(error, BaseException) else str(error)
    if not any(s in text for s in _SIGNS):
        return None
    return (
        f"offscreen rendering is not available: MuJoCo could not create an OpenGL context. {HINT}. "
        f"`robouse doctor` checks it. Cause: {reason(text)}"
    )
