"""The sandbox the local runner puts around agent processes: a Seatbelt profile on macOS, bubblewrap on Linux.

In both, an agent cannot read or write:
  - private settings, the real-robot operator channel, the maintainers' folders and $ROBOUSE_SANDBOX_DENY
    (`config.sandbox_denied_paths`);
  - the job folder (trial records, including the episode's result.json) and the task folder (its reference solution);
  - any robouse source or install (a `robouse` package directory);
  - other trials' workspaces.
It keeps its own workspace, the episode socket and the network (model APIs). On macOS, TCP connections to this machine
are refused except to `local_ports` (a harness's own local proxy); on Linux the agent runs in its own PID namespace,
so it cannot see the episode server's process. Set ROBOUSE_SANDBOX=0 to run without a sandbox (unsafe: the agent can
then read the reference solution).
"""

from __future__ import annotations

import functools
import os
import shutil
import subprocess
import sys
from pathlib import Path

from .. import config

PKG = Path(__file__).resolve().parents[1]  # the robouse package


class SandboxUnavailable(RuntimeError):
    """No sandbox for agent processes on this machine."""


def _denied(extra: list[Path]) -> list[str]:
    paths = [*config.sandbox_denied_paths(), *map(str, extra), str(PKG)]
    # resolved paths (Seatbelt and bind mounts match /private/tmp, not /tmp); order kept, duplicates dropped
    return list(dict.fromkeys(os.path.realpath(os.path.expanduser(p)) for p in paths))


def sandbox_profile(
    workspace: str | Path | None = None, local_ports: tuple[int, ...] = (), deny: list[Path] | None = None
) -> str:
    """The macOS Seatbelt profile: everything is allowed except the module docstring's list."""
    rules = "".join(f'(deny file-read* file-write* (subpath "{d}"))' for d in _denied(deny or []))
    rules += '(deny file-read* (regex #"/site-packages/robouse(/|$)") (regex #"/src/robouse(/|$)"))'
    rules += '(deny file-read* file-write* (regex #"/robouse-ws-[^/]*(/|$)"))'
    if workspace:
        rules += f'(allow file-read* file-write* (subpath "{os.path.realpath(workspace)}"))'
    rules += '(deny network-outbound (remote ip "localhost:*"))'
    rules += "".join(f'(allow network-outbound (remote ip "localhost:{p}"))' for p in local_ports)
    return "(version 1)(allow default)" + rules


def bwrap_args(workspace: Path, socket: str, deny: list[Path]) -> list[str]:
    """The bubblewrap command prefix on Linux: the host file system as it is, a fresh /tmp holding only this trial's
    workspace and socket, empty folders over every denied path, and a PID namespace of its own."""
    args = [
        "bwrap",
        "--die-with-parent",
        "--unshare-pid",
        "--bind",
        "/",
        "/",
        "--dev",
        "/dev",
        "--proc",
        "/proc",
        "--tmpfs",
        "/tmp",
    ]
    for d in _denied(deny):
        if Path(d).is_dir() and not str(workspace).startswith(d):
            args += ["--tmpfs", d]
    args += ["--bind", str(workspace), str(workspace), "--bind", socket, socket, "--chdir", str(workspace)]
    return args


@functools.cache
def bwrap_works() -> bool:
    """Whether bubblewrap can create its namespaces here (Ubuntu 24.04 blocks unprivileged user namespaces unless an
    AppArmor profile allows bwrap)."""
    if not shutil.which("bwrap"):
        return False
    probe = ["bwrap", "--bind", "/", "/", "--unshare-pid", "--dev", "/dev", "--proc", "/proc", "true"]
    return subprocess.run(probe, capture_output=True, check=False).returncode == 0


def wrap(
    cmd: list[str], workspace: Path, socket: str, deny: list[Path], local_ports: tuple[int, ...] = ()
) -> tuple[list[str], str]:
    """The agent command inside this platform's sandbox, and the sandbox's name. Raises SandboxUnavailable when there
    is none (Linux without bubblewrap) unless ROBOUSE_SANDBOX=0."""
    if not config.flag("ROBOUSE_SANDBOX", default=True):
        return cmd, "none"
    if sys.platform == "darwin":
        return ["sandbox-exec", "-p", sandbox_profile(workspace, local_ports, deny), *cmd], "macos-seatbelt"
    if sys.platform.startswith("linux") and bwrap_works():
        return [*bwrap_args(workspace, socket, deny), *cmd], "linux-bubblewrap"
    raise SandboxUnavailable(
        "no sandbox for the agent on this machine: install bubblewrap (`bwrap`; on Ubuntu 24.04 also an AppArmor "
        "profile that allows it user namespaces), or set ROBOUSE_SANDBOX=0 to run without one (the agent can then "
        "read the reference solution)"
    )
