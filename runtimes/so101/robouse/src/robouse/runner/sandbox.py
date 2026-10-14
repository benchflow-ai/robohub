"""The sandbox the local runner puts around agent processes: a Seatbelt profile on macOS, bubblewrap on Linux.

In both, an agent cannot read or write:
  - private settings, the real-robot operator channel, the optional credential pools (robouse._maintainer) and $ROBOUSE_SANDBOX_DENY
    (`config.sandbox_denied_paths`);
  - credential and task-copy locations in the home folder (`HOME_DENIED`): other programs' logins, the user's own
    Claude Code, Codex and mini-swe-agent settings, and package caches holding copies of the tasks with their reference
    solutions or simulator sources; also the runner's own $CLAUDE_CONFIG_DIR and $CODEX_HOME;
  - robouse's cache (simulator virtualenvs and sources, downloaded demonstrations, other trials' harness homes), except
    the agent's own harness home (`own`: its CLAUDE_CONFIG_DIR or CODEX_HOME);
  - the run folder (trial records, including the episode's result.json) and the task folder (its reference solution);
  - any robouse source or install (a `robouse` package directory) and, in a source checkout, its task folders, run
    folders and task generators (`CHECKOUT_DENIED`);
  - other trials' workspaces and episode sockets;
  - sockets that act for the user or control the machine (`control_sockets`: container engines, the SSH agent), and
    any Unix socket inside a denied folder.
On macOS it also cannot start `open`, `osascript`, `shortcuts` or `security` (`MACOS_ESCAPES`) or send Apple events,
which act outside the sandbox.
Program folders inside a denied folder that a harness runs from (`HOME_PROGRAMS`, e.g. Codex's standalone install in
~/.codex/packages) stay readable, never writable, and so does the runner's Python (`interpreters`), which the episode
server and the verifier run on. The agent keeps its own workspace, the episode socket and the network
(model APIs). On macOS, TCP connections to this machine are refused except to `local_ports` (a harness's own local
proxy); on Linux the agent runs in its own PID namespace, so it cannot see the episode server's process. Set
ROBOUSE_SANDBOX=0 to run without a sandbox (unsafe: the agent can then read the reference solution).
"""

from __future__ import annotations

import functools
import glob
import os
import re
import shutil
import subprocess
import sys
from collections.abc import Iterable
from pathlib import Path

from .. import config

PKG = Path(__file__).resolve().parents[1]  # the robouse package

# Credential and task-copy locations under the home folder that no agent may read or write (globs allowed)
HOME_DENIED = (
    ".cache/uv",  # uv's cache: unpacked wheels, among them robouse's bundled tasks with their oracles, and simulators
    ".cache/pip",  # pip's wheel cache (Linux)
    "Library/Caches/pip",  # pip's wheel cache (macOS)
    ".cache/benchflow",  # BenchFlow's task cache: materialized robouse tasks with their oracles
    ".config/benchflow",  # BenchFlow's settings and provider key files
    ".cache/libero",  # LIBERO's assets and datasets
    ".libero",  # LIBERO's config
    ".claude",  # Claude Code's settings, login (Linux) and transcripts; the agent gets a per-trial home
    ".claude.json",
    ".claude-*",  # further Claude Code config folders
    ".codex",  # Codex's login (auth.json), sessions and memories
    ".config/mini-swe-agent",  # mini-swe-agent's global .env, which may hold provider keys (Linux)
    "Library/Application Support/mini-swe-agent",  # (macOS)
    ".gemini",  # Gemini CLI's login
    ".config/gh",  # GitHub CLI's token
    ".git-credentials",
    ".config/git/credentials",
    ".netrc",
    ".ssh",
    ".gnupg",
    ".aws",
    ".azure",
    ".config/gcloud",
    ".kube",
    ".docker",  # registry logins; Docker Desktop's socket
    ".colima",  # container VMs and their Docker sockets (Colima, Lima, OrbStack, Rancher Desktop)
    ".lima",
    ".orbstack",
    ".rd",
    ".pypirc",
    ".npmrc",
    ".huggingface",
    ".cache/huggingface/token",
    ".cache/huggingface/stored_tokens",
)
# Program folders inside those that a harness may run from: readable, never writable
HOME_PROGRAMS = (".codex/packages", ".claude/local")
# macOS programs that act outside the sandbox: `open` hands a file or URL to LaunchServices, `osascript` and `shortcuts`
# drive other applications, and `security` queries the login keychain through securityd, where Claude Code and the
# GitHub CLI keep their logins (file rules do not cover the keychain). The Security framework itself stays available:
# Codex checks TLS certificates with it
MACOS_ESCAPES = ("/usr/bin/open", "/usr/bin/osascript", "/usr/bin/shortcuts", "/usr/bin/security")
# The variables naming a harness's own config home (the runner's values are denied, the agent's allowed)
HOME_VARS = ("CLAUDE_CONFIG_DIR", "CODEX_HOME")
# When robouse runs from a source checkout: its task folders (every task's reference solution and verifier), run
# folders (other trials' records) and task generators
CHECKOUT_DENIED = ("tasks", "runs", "adapters")
# Sockets that act for the user or control this machine: container engines (root on the host for a member of their
# group) and, from the environment, the SSH agent
SOCKETS = (
    "/var/run/docker.sock",
    "/run/docker.sock",
    "/run/podman/podman.sock",
    "/var/snap/lxd/common/lxd/unix.socket",
    "/var/lib/lxd/unix.socket",
)


class SandboxUnavailable(RuntimeError):
    """No sandbox for agent processes on this machine."""


def _real(paths: Iterable[str | Path]) -> list[str]:
    """Resolved paths (Seatbelt and bind mounts match /private/tmp, not /tmp); order kept, duplicates dropped."""
    return list(dict.fromkeys(os.path.realpath(os.path.expanduser(str(p))) for p in paths))


def home_denied() -> list[str]:
    """HOME_DENIED under this user's home folder (globs expanded), the runner's own Claude Code and Codex homes, and
    robouse's cache."""
    home = Path.home()
    out: list[str] = []
    for rel in HOME_DENIED:
        out += sorted(glob.glob(str(home / rel))) if any(c in rel for c in "*?[") else [str(home / rel)]
    out += [v for k in HOME_VARS if (v := config.env(k))]
    return [*out, str(config.cache_dir())]


def checkout_denied() -> list[str]:
    """CHECKOUT_DENIED in the source checkout robouse runs from (a src/robouse layout with its pyproject.toml), else
    nothing (an installed package is denied whole)."""
    root = PKG.parent.parent
    if PKG.parent.name != "src" or not (root / "pyproject.toml").is_file():
        return []
    return [str(root / d) for d in CHECKOUT_DENIED]


def control_sockets() -> list[str]:
    """SOCKETS, the user's rootless Docker and Podman sockets, and the SSH agent's socket ($SSH_AUTH_SOCK)."""
    run = config.env("XDG_RUNTIME_DIR")
    out = [*SOCKETS, *([f"{run}/docker.sock", f"{run}/podman"] if run else [])]
    return [*out, *([sock] if (sock := config.env("SSH_AUTH_SOCK")) else [])]


def interpreters() -> list[str]:
    """The runner's Python: its environment, the base install and the user site-packages it loads. The episode server
    and the verifier run on it, so a file an agent put there (a .pth file, a changed module) would run in a later
    trial's server. Agents may read it (their `python3` is often the same one), never write to it."""
    import site

    paths = [sys.prefix, sys.exec_prefix, sys.base_prefix, sys.base_exec_prefix]
    if site.ENABLE_USER_SITE:
        paths.append(site.getusersitepackages())
    return [p for p in _real(paths) if os.path.isdir(p)]


def own_homes(env: dict) -> list[str]:
    """The agent's own config homes (its CLAUDE_CONFIG_DIR and CODEX_HOME), which it may read and write wherever they
    are."""
    return _real(v for k in HOME_VARS if (v := env.get(k)))


def _denied(extra: list[Path]) -> list[str]:
    paths = [*config.sandbox_denied_paths(), *home_denied(), *control_sockets(), *checkout_denied(), *map(str, extra)]
    return _real([*paths, str(PKG)])


def _programs() -> list[str]:
    return [p for p in _real(Path.home() / rel for rel in HOME_PROGRAMS) if os.path.isdir(p)]


def _inside(path: str, roots: list[str]) -> bool:
    return any(path == r or path.startswith(r.rstrip("/") + "/") for r in roots)


def _ancestors(path: str) -> list[str]:
    """The folders above `path`, up to and without the root."""
    out, p = [], os.path.dirname(path)
    while p != os.path.dirname(p):
        out.append(p)
        p = os.path.dirname(p)
    return out


def reachable_path(path: str, deny: list[Path], own: list[str] | None = None) -> str:
    """A PATH value without its entries inside a denied folder that the sandbox does not give back (program folders,
    the agent's own homes): a command found there could not run. `uvx`, for one, puts its environment, in the uv
    cache, first on PATH."""
    denied, back = _denied(deny), [*_programs(), *_real(own or [])]
    keep = []
    for entry in path.split(os.pathsep):
        real = os.path.realpath(os.path.expanduser(entry)) if entry else ""
        if not (real and _inside(real, denied) and not _inside(real, back)):
            keep.append(entry)
    return os.pathsep.join(keep)


def sandbox_profile(
    workspace: str | Path | None = None,
    local_ports: tuple[int, ...] = (),
    deny: list[Path] | None = None,
    own: list[str] | None = None,
    socket: str | None = None,
) -> str:
    """The macOS Seatbelt profile: everything is allowed except the module docstring's list. Later rules win. A file
    rule does not stop a connection to a Unix socket, so each denied path also gets a network rule; other trials'
    episode sockets (robouse-*.sock next to `socket`, the agent's own) are refused the same way."""
    denied = _denied(deny or [])
    rules = "".join(f'(deny file-read* file-write* (subpath "{d}"))' for d in denied)
    rules += "".join(f'(deny file-write* (subpath "{p}"))' for p in interpreters())
    rules += "".join(f'(deny network-outbound (remote unix-socket (subpath "{d}")))' for d in denied)
    if socket:
        own_socket = os.path.realpath(socket)
        others = re.escape(os.path.dirname(own_socket)) + r"/robouse-[0-9a-f]+\.sock$"
        rules += f'(deny file-read* file-write* (regex #"^{others}"))'
        rules += f'(deny network-outbound (remote unix-socket (path-regex #"^{others}")))'
        rules += f'(allow file-read* file-write* (literal "{own_socket}"))'
        rules += f'(allow network-outbound (remote unix-socket (path-literal "{own_socket}")))'
    rules += '(deny file-read* (regex #"/site-packages/robouse(/|$)") (regex #"/src/robouse(/|$)"))'
    rules += '(deny file-read* file-write* (regex #"/robouse-ws-[^/]*(/|$)"))'
    rules += "".join(f'(allow file-read* (subpath "{p}"))' for p in _programs())
    for d in [*_real([workspace] if workspace else []), *_real(own or [])]:
        rules += f'(allow file-read* file-write* (subpath "{d}"))'
    # the folders above a home or program folder that sits inside a denied one: their metadata (stat), not their
    # contents, so a program can resolve its own path there (Codex canonicalizes CODEX_HOME, inside robouse's cache)
    above = sorted({a for p in [*_programs(), *_real(own or [])] for a in _ancestors(p)})
    if above:
        rules += "(allow file-read-metadata " + " ".join(f'(literal "{a}")' for a in above) + ")"
    rules += "(deny process-exec " + " ".join(f'(literal "{p}")' for p in MACOS_ESCAPES) + ")"
    rules += "(deny appleevent-send)"  # no Apple events to other applications, from any program
    rules += '(deny network-outbound (remote ip "localhost:*"))'
    rules += "".join(f'(allow network-outbound (remote ip "localhost:{p}"))' for p in local_ports)
    return "(version 1)(allow default)" + rules


def bwrap_args(workspace: Path, socket: str, deny: list[Path], own: list[str] | None = None) -> list[str]:
    """The bubblewrap command prefix on Linux: the host file system as it is, a fresh /tmp holding only this trial's
    workspace and socket, an empty folder over every denied folder and /dev/null over every denied file, the agent's
    own homes and the program folders bound back in, and a PID namespace of its own."""
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
    for p in interpreters():  # read-only, under the denied folders' mounts (a robouse install inside stays hidden)
        if not _inside(p, ["/tmp"]):
            args += ["--ro-bind", p, p]
    for d in _denied(deny):
        if str(workspace).startswith(d):
            continue  # a folder holding the workspace stays
        if os.path.isdir(d):
            args += ["--tmpfs", d]
        elif os.path.exists(d) and not _inside(d, ["/tmp"]):  # a file or socket; /tmp is fresh already
            args += ["--ro-bind", "/dev/null", d]
    args += ["--bind", str(workspace), str(workspace), "--bind", socket, socket]
    for p in _programs():
        args += ["--ro-bind", p, p]
    for h in _real(own or []):
        if os.path.isdir(h) and not h.startswith(str(workspace)):
            args += ["--bind", h, h]
    return [*args, "--chdir", str(workspace)]


@functools.cache
def bwrap_works() -> bool:
    """Whether bubblewrap can create its namespaces here (Ubuntu 24.04 blocks unprivileged user namespaces unless an
    AppArmor profile allows bwrap)."""
    if not shutil.which("bwrap"):
        return False
    probe = ["bwrap", "--bind", "/", "/", "--unshare-pid", "--dev", "/dev", "--proc", "/proc", "true"]
    return subprocess.run(probe, capture_output=True, check=False).returncode == 0


def wrap(
    cmd: list[str],
    workspace: Path,
    socket: str,
    deny: list[Path],
    local_ports: tuple[int, ...] = (),
    own: list[str] | None = None,
) -> tuple[list[str], str]:
    """The agent command inside this platform's sandbox, and the sandbox's name. `own`: the agent's own config homes
    (`own_homes`). Raises SandboxUnavailable when there is none (Linux without bubblewrap) unless ROBOUSE_SANDBOX=0."""
    if not config.flag("ROBOUSE_SANDBOX", default=True):
        return cmd, "none"
    if sys.platform == "darwin":
        profile = sandbox_profile(workspace, local_ports, deny, own, socket)
        return ["sandbox-exec", "-p", profile, *cmd], "macos-seatbelt"
    if sys.platform.startswith("linux") and bwrap_works():
        return [*bwrap_args(workspace, socket, deny, own), *cmd], "linux-bubblewrap"
    raise SandboxUnavailable(
        "no sandbox for the agent on this machine: install bubblewrap (`bwrap`; on Ubuntu 24.04 also an AppArmor "
        "profile that allows it user namespaces), or set ROBOUSE_SANDBOX=0 to run without one (the agent can then "
        "read the reference solution)"
    )
