"""The `dimcode` harness: Dimensional's coding agent (npm `@dimensionalos/dimcode`, Apache-2.0, built on Pi), headless
through `dimcode run PROMPT`.

Isolation: every dimcode path is per trial, inside the workspace's `.robouse/dimcode/` (DIMCODE_HOME with config.json,
settings.json and models.json; XDG state, cache and runtime folders; HOME), so no user config, credentials, sessions
or DimOS installation leak in, and automatic DimOS installation is off. The model is a provider-prefixed id (default
baseten/zai-org/GLM-5.3), always reached through the trial's model gateway over Anthropic Messages (gateway.py), which
holds the provider key, applies the trial's image input and spend cap: Pi gets the gateway's URL and the per-trial
gateway key (from $ROBOUSE_GATEWAY_KEY; no key is written to a file).

Interface (harnesses/interface.py): with cli, dimcode's shell tool runs `robo`. With mcp or both, `robo mcp --http`
serves a streamable HTTP endpoint on 127.0.0.1 at an unguessable path for the life of the agent (dimcode speaks MCP
over HTTP only), and config.json lists it as dimcode's one MCP endpoint.

The trajectory is Pi's session file, copied to agent/pi_session.jsonl and converted by runner/atif.py.

  ROBOUSE_DIMCODE        the dimcode executable (default: `dimcode` on PATH)
  ROBOUSE_DIMCODE_NODE   Node 24 or 26 to run it with (default: `node` on PATH)
"""

from __future__ import annotations

import json
import os
import secrets
import shutil
import socket
from pathlib import Path

from .. import config

SYSTEM_NOTE = (
    "This session runs in a Robo Use evaluation sandbox. The robot is controlled only through the `robo` interface "
    "described in the task (the `robo` command and/or the `robo` MCP tools). No DimOS installation, blueprint or other "
    "robot endpoint is available here; do not install or start one.\n"
)


def _free_port() -> int:
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


def _exe() -> list[str]:
    exe = config.env("ROBOUSE_DIMCODE") or shutil.which("dimcode")
    if not exe:
        raise RuntimeError("dimcode is not installed: `npm install -g @dimensionalos/dimcode@next` (Node 24 or 26)")
    node = config.env("ROBOUSE_DIMCODE_NODE") or shutil.which("node") or "node"
    return [node, os.path.realpath(exe)]


def build(model, prompt, workspace: Path, task, env: dict):
    from . import PROMPT_PREFIX, _gateway, _gateway_launch, models
    from . import interface as iface

    interface = env.get("ROBOUSE_INTERFACE", "cli")
    ref = models.resolve("dimcode", model or models.DEFAULT_MODELS["dimcode"])
    exe = _exe()
    images = env.get("_ROBOUSE_IMAGE_INPUT", "1") == "1"  # read before _gateway removes the private variables
    gw = _gateway(ref, env, "anthropic-messages")

    root = workspace / ".robouse" / "dimcode"
    home, state = root / "home", root / "state"
    for d in (root, home, state, root / "cache"):
        d.mkdir(parents=True, exist_ok=True, mode=0o700)
    endpoints = []
    run_prefix: list[str] = []
    if interface in ("mcp", "both"):
        mport, path = _free_port(), f"/{secrets.token_urlsafe(18)}/mcp"
        url = f"http://127.0.0.1:{mport}{path}"
        endpoints = [{"name": "robo", "url": url}]
        run_prefix = [*iface.robo_mcp_cmd(workspace), "--http", f"127.0.0.1:{mport}", "--path", path, "--run"]
        env["ROBOUSE_MCP_URL"] = url  # also lets the macOS sandbox allow this local port
    (root / "config.json").write_text(
        json.dumps({"workspace": str(workspace), "mcp": endpoints, "gatewayStart": "on-demand"}, indent=2)
    )
    (root / "settings.json").write_text(
        json.dumps({"defaultProvider": "robouse", "defaultModel": gw.model, "defaultThinkingLevel": "medium"}, indent=2)
    )
    (root / "models.json").write_text(
        json.dumps(
            {
                "providers": {
                    "robouse": {
                        "baseUrl": gw.url,
                        "api": "anthropic-messages",
                        "apiKey": "$ROBOUSE_GATEWAY_KEY",
                        "authHeader": True,
                        "models": [
                            {
                                "id": gw.model,
                                "name": ref.id,
                                "reasoning": True,
                                "input": ["text", "image"] if images else ["text"],
                                "contextWindow": 200000,
                                "maxTokens": 32000,
                            }
                        ],
                    }
                }
            },
            indent=2,
        )
    )
    (root / "APPEND_SYSTEM.md").write_text(SYSTEM_NOTE)
    env.update(
        {
            "DIMCODE_HOME": str(root),
            "DIMCODE_INSTALL_DIMOS": "0",
            "HOME": str(home),
            "XDG_STATE_HOME": str(state),
            "XDG_CACHE_HOME": str(root / "cache"),
            "XDG_CONFIG_HOME": str(home / ".config"),
            "XDG_RUNTIME_DIR": str(state),
            "ROBOUSE_GATEWAY_KEY": gw.key,
            "ROBOUSE_GATEWAY_URL": gw.url,  # also lets the macOS sandbox allow this local port
        }
    )
    text = iface.prompt_prefix(interface) if interface != "cli" else PROMPT_PREFIX
    # `dimcode run` starts a detached gateway; stop it when the run ends so nothing outlives the trial
    shell = '"$0" "$@"; rc=$?; "$0" "$1" stop >/dev/null 2>&1; exit $rc'
    cmd = [*run_prefix, "sh", "-c", shell, exe[0], exe[1], "run", text + prompt]
    launch = _gateway_launch(gw, ref, cmd, env)
    launch.notes["collect"] = [["dimcode/state/dimcode/sessions", "agent/pi_session.jsonl"]]
    return launch
