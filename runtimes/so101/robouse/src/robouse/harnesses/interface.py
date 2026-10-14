"""Which robot interface a trial's agent gets: the `robo` CLI, the `robo` MCP tools, or both.

  cli    `robo` on PATH (the default)
  mcp    the MCP server `robo mcp` only; `robo` is not on PATH
  both   both; they drive the same episode

The interface comes from `--interface` on `run`/`run-many`, else the task's `robouse.interface`, else cli, and is recorded
in config.json and result.json. The MCP server runs inside the agent's sandbox as a client of the episode socket
(`robouse/mcp_server.py`), so it has exactly the CLI's authority. Leaving `robo` off PATH is a statement of what the
agent is offered, not a security boundary: authority is the socket, which both interfaces share.

Harnesses: claude-code and codex (and their -glm variants) get a stdio MCP server in a per-trial config (Claude Code:
`--mcp-config` with `--strict-mcp-config`; Codex: `-c mcp_servers.robo.*`). dimcode gets a streamable HTTP endpoint
(harnesses/dimcode.py). oracle and noop send every request through a `robo mcp` server ($ROBOUSE_SEND_VIA=mcp). Other
harnesses have no MCP client and take cli only.
"""

from __future__ import annotations

import json
import os
import shutil
from pathlib import Path

INTERFACES = ("cli", "mcp", "both")
MCP_HARNESSES = ("oracle", "noop", "claude-code", "claude-code-glm", "codex", "codex-glm", "dimcode")

_ENDING = "Keep going until the task is done, then call {done} once. Do not stop to ask questions; there is no human to answer.\n\n"
_TOOLS = (
    "The task text names `robo` shell commands; each is the MCP tool of the same name: `robo info` is `info`, "
    "`robo observe --image` is `observe` with image=true, `robo act A1 A2 --repeat N` is `act` with action=[A1, A2] "
    "and repeat=N, `robo move-to X Y Z --grip G` is `move_to`, `robo grip G` is `grip`, `robo skill NAME ARGS` is "
    "`skill`, `robo done` is `done` and `robo give-up` is `give_up`."
)
PROMPTS = {
    "mcp": "You are controlling a simulated robot. Read the task below, then solve it with the tools of the `robo` MCP "
    "server (start with `info` and `observe`). "
    + _TOOLS
    + " There is no `robo` shell command in this session. "
    + _ENDING.format(done="the `done` tool"),
    "both": "You are controlling a simulated robot. Read the task below, then solve it by running the `robo` command in "
    "your shell or with the tools of the `robo` MCP server; both drive the same robot (start with `robo info` and "
    "`robo observe`, or the `info` and `observe` tools). "
    + _TOOLS
    + " "
    + _ENDING.format(done="`robo done` (or the `done` tool)"),
}


def resolve(task, requested: str | None) -> str:
    """The trial's interface: the run's choice, else the task's `robouse.interface`, else cli."""
    val = requested or task.spec.get("interface") or "cli"
    if val not in INTERFACES:
        raise ValueError(f"unknown interface {val!r}; interfaces: {', '.join(INTERFACES)}")
    return val


def check(harness: str, interface: str) -> None:
    if interface != "cli" and harness not in MCP_HARNESSES:
        raise ValueError(f"harness {harness!r} has no MCP client; use --interface cli")


def prompt_prefix(interface: str) -> str:
    from . import PROMPT_PREFIX

    return PROMPTS.get(interface, PROMPT_PREFIX)


def robo_mcp_cmd(workspace: Path) -> list[str]:
    """The stdio MCP server command, by absolute paths (the agent's PATH may not have `robo`)."""
    return [shutil.which("python3") or "python3", str(workspace / ".robouse" / "bin" / "robo"), "mcp"]


def server_env(sock: str, env: dict) -> dict:
    """What the MCP server needs from the agent's environment (MCP clients may start servers with a reduced one): the
    episode socket. Image input needs nothing here: the server attaches an image exactly when the episode server
    returns its path, and a trial without image input has its model gateway replace every image part."""
    return {"ROBOUSE_SOCKET": sock}


def stdio_config(workspace: Path, sock: str, env: dict) -> dict:
    cmd = robo_mcp_cmd(workspace)
    return {"type": "stdio", "command": cmd[0], "args": cmd[1:], "env": server_env(sock, env)}


def _swap_prompt(cmd: list[str], interface: str) -> None:
    from . import PROMPT_PREFIX

    for i, a in enumerate(cmd):
        if isinstance(a, str) and a.startswith(PROMPT_PREFIX):
            cmd[i] = prompt_prefix(interface) + a[len(PROMPT_PREFIX) :]


def apply(harness: str, launch, interface: str, workspace: Path, sock: str) -> None:
    """Give a built launch its interface: MCP client config, prompt and PATH."""
    if interface == "cli":
        return
    check(harness, interface)
    env = launch.env
    if harness in ("oracle", "noop"):
        env["ROBOUSE_SEND_VIA"] = "mcp"  # every request goes through a `robo mcp` server
    elif harness.startswith("claude-code"):
        cfg = workspace / ".robouse" / "mcp.json"
        cfg.write_text(json.dumps({"mcpServers": {"robo": stdio_config(workspace, sock, env)}}, indent=2))
        launch.cmd += ["--mcp-config", str(cfg), "--strict-mcp-config"]
        env.setdefault("MCP_TIMEOUT", "30000")  # server start
        env.setdefault("MCP_TOOL_TIMEOUT", "600000")  # a long skill
    elif harness.startswith("codex"):
        c = stdio_config(workspace, sock, env)
        menv = ",".join(f"{k}={json.dumps(v)}" for k, v in c["env"].items())
        flags = [
            "-c",
            f"mcp_servers.robo.command={json.dumps(c['command'])}",
            "-c",
            f"mcp_servers.robo.args={json.dumps(c['args'])}",
            "-c",
            f"mcp_servers.robo.env={{{menv}}}",
            "-c",
            "mcp_servers.robo.startup_timeout_sec=30",
            "-c",
            "mcp_servers.robo.tool_timeout_sec=600",
        ]
        i = launch.cmd.index("-m")
        launch.cmd[i:i] = flags
    _swap_prompt(launch.cmd, interface)
    if interface == "mcp" and harness not in ("oracle", "noop"):
        bin_dir = str(workspace / ".robouse" / "bin")
        env["PATH"] = os.pathsep.join(p for p in env.get("PATH", "").split(os.pathsep) if p != bin_dir)
