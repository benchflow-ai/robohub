"""Harness launchers: the command, environment and helper process for one trial's agent.

The agent process gets: cwd = an isolated workspace (instruction.md, observations/), PATH with a standard-library
`robo` shim first, and ROBOUSE_SOCKET. Credentials come from the environment (`credentials.py`).

Harnesses
  oracle                 the task's reference solution (oracle/solve.sh)
  noop                   calls `robo done` immediately (negative control)
  claude-code            Claude Code CLI; model e.g. claude-sonnet-5
  codex                  Codex CLI; model e.g. gpt-6-astra
  claude-code-glm        Claude Code CLI against a Baseten model (default GLM-5.3) through a local image-capping proxy
  codex-glm              Codex CLI with a custom OpenAI-compatible provider (Baseten)
  mini-swe-agent-glm     mini-swe-agent (litellm) with a Baseten model
  molmoact2              MolmoAct2 VLA policy: molmoact2/client.py drives `robo` with action chunks from a policy
                         server at $ROBOUSE_MOLMOACT2_URL; model e.g. allenai/MolmoAct2-LIBERO-LeRobot
  vla                    any served VLA (vla/client.py): --model is a preset from vla/presets.json; server at
                         $ROBOUSE_VLA_URL; overrides from $ROBOUSE_VLA_POLICY_CONFIG (JSON file)
Third-party harnesses register a callable with `build`'s signature under the `robouse.harnesses` entry point group.
"""

from __future__ import annotations

import json
import shutil
from collections.abc import Callable
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from .. import config
from . import credentials

GLM_MODEL = "zai-org/GLM-5.3"
BASETEN_OPENAI = "https://inference.baseten.co/v1"
DISALLOWED_CLAUDE_TOOLS = "WebFetch,WebSearch,Task"

PROMPT_PREFIX = (
    "You are controlling a simulated robot. Read the task below, then solve it by running the `robo` command "
    "in your shell (start with `robo info` and `robo observe`). Keep going until the task is done, then call "
    "`robo done` once. Do not stop to ask questions; there is no human to answer.\n\n"
)


@dataclass
class Launch:
    cmd: list[str]
    env: dict[str, str]
    helper: object | None = None  # e.g. a proxy's statistics, recorded in result.json
    notes: dict[str, Any] = field(default_factory=dict)


Builder = Callable[[str, str, Path, Any, dict], Launch]  # (model, prompt, workspace, task, env) -> Launch


def _claude_cmd(prompt: str, model: str) -> list[str]:
    return [
        "claude",
        "-p",
        PROMPT_PREFIX + prompt,
        "--model",
        model,
        "--output-format",
        "stream-json",
        "--verbose",
        "--dangerously-skip-permissions",
        "--max-turns",
        "400",
        "--disallowedTools",
        DISALLOWED_CLAUDE_TOOLS,
    ]


def _codex_cmd(prompt: str, model: str, workspace: Path, extra: tuple[str, ...] | list[str] = ()) -> list[str]:
    return [
        "codex",
        "exec",
        "--json",
        "--skip-git-repo-check",
        "--dangerously-bypass-approvals-and-sandbox",
        *extra,
        "-m",
        model,
        "-C",
        str(workspace),
        PROMPT_PREFIX + prompt,
    ]


def _oracle(model, prompt, workspace, task, env) -> Launch:
    return Launch(["bash", str(task.oracle_script)], env, notes={"model": "scripted"})


def _noop(model, prompt, workspace, task, env) -> Launch:
    return Launch(["robo", "done", "noop control"], env, notes={"model": "none"})


def _claude_code(model, prompt, workspace, task, env) -> Launch:
    m = model or "claude-sonnet-5"
    env.update(credentials.claude_auth())
    env["CLAUDE_CONFIG_DIR"] = credentials.clean_claude_home()
    return Launch(_claude_cmd(prompt, m), env, notes={"model": m})


def _claude_code_glm(model, prompt, workspace, task, env) -> Launch:
    from .providers import baseten_proxy

    bm = model or GLM_MODEL
    port, stats = baseten_proxy.start(credentials.baseten_key())
    env.update(
        {
            "ANTHROPIC_BASE_URL": f"http://127.0.0.1:{port}",
            "ANTHROPIC_AUTH_TOKEN": "proxy-held",
            "ANTHROPIC_API_KEY": "",
            "ANTHROPIC_MODEL": bm,
            "ANTHROPIC_DEFAULT_OPUS_MODEL": bm,
            "ANTHROPIC_DEFAULT_SONNET_MODEL": bm,
            "ANTHROPIC_DEFAULT_HAIKU_MODEL": bm,
            "ANTHROPIC_SMALL_FAST_MODEL": bm,
            "CLAUDE_CODE_DISABLE_NONESSENTIAL_TRAFFIC": "1",
        }
    )
    env.pop("CLAUDE_CODE_OAUTH_TOKEN", None)
    env["CLAUDE_CONFIG_DIR"] = credentials.clean_claude_home()
    return Launch(_claude_cmd(prompt, bm), env, helper=stats, notes={"model": bm, "provider": "baseten"})


def _codex(model, prompt, workspace, task, env) -> Launch:
    import os

    m = model or "gpt-6-astra"
    key = os.environ.get("OPENAI_API_KEY", "")
    env["CODEX_HOME"] = credentials.clean_codex_home(api_key=key)
    if key:
        env["CODEX_API_KEY"] = key
    return Launch(_codex_cmd(prompt, m, workspace), env, notes={"model": m})


def _codex_glm(model, prompt, workspace, task, env) -> Launch:
    bm = model or GLM_MODEL
    env["BASETEN_API_KEY"] = credentials.baseten_key()
    env["ROBOUSE_TEXT_ONLY"] = "1"  # Baseten's Responses API rejects images; Codex attaches PNGs it sees in output
    env["CODEX_HOME"] = credentials.clean_codex_home(
        "# Baseten provider, set on the command line\n"
        "# Baseten's Responses API rejects image input\n"
        "[tools]\nview_image = false\n"
    )
    cfg = [
        "-c",
        'model_providers.baseten.name="Baseten"',
        "-c",
        f'model_providers.baseten.base_url="{BASETEN_OPENAI}"',
        "-c",
        'model_providers.baseten.env_key="BASETEN_API_KEY"',
        "-c",
        'model_providers.baseten.wire_api="responses"',
        "-c",
        'model_provider="baseten"',
    ]
    return Launch(_codex_cmd(prompt, bm, workspace, cfg), env, notes={"model": bm, "provider": "baseten"})


def _mini_swe_agent_glm(model, prompt, workspace, task, env) -> Launch:
    bm = model or GLM_MODEL
    env.update(
        {
            "OPENAI_API_KEY": credentials.baseten_key(),
            "OPENAI_API_BASE": BASETEN_OPENAI,
            "MSWEA_CONFIGURED": "true",
            "MSWEA_COST_TRACKING": "ignore_errors",
        }
    )
    exe = shutil.which("mini") or str(Path.home() / ".local" / "bin" / "mini")
    (workspace / ".robouse" / "task.txt").write_text(PROMPT_PREFIX + prompt)
    return Launch(
        [
            exe,
            "-y",
            "--exit-immediately",
            "-l",
            "0",
            "-m",
            f"openai/{bm}",
            "-t",
            PROMPT_PREFIX + prompt,
            "-o",
            str(workspace / ".robouse" / "mini_traj.json"),
        ],
        env,
        notes={"model": bm, "provider": "baseten"},
    )


def _task_language(task) -> str:
    """The instruction a VLA gets: the source benchmark's own language instruction, else the task description."""
    return str(
        task.metadata.get("language_instruction")
        or task.meta.get("real", {}).get("goal_text")
        or task.spec.get("real", {}).get("goal_text")
        or task.meta.get("task", {}).get("description", "")
    )


def _molmoact2(model, prompt, workspace, task, env) -> Launch:
    url = config.env("ROBOUSE_MOLMOACT2_URL")
    if not url:
        raise RuntimeError("molmoact2 needs ROBOUSE_MOLMOACT2_URL (the URL of a MolmoAct2 policy server)")
    client = workspace / ".robouse" / "molmoact2_client.py"
    client.write_text((Path(__file__).parent / "molmoact2" / "client.py").read_text())
    env["ROBOUSE_TASK_LANGUAGE"] = _task_language(task)
    m = model or "allenai/MolmoAct2-LIBERO-LeRobot"
    return Launch(["python3", str(client)], env, notes={"model": m, "server": url})


def _vla(model, prompt, workspace, task, env) -> Launch:
    from . import vla

    url = config.env("ROBOUSE_VLA_URL")
    over_file = config.env("ROBOUSE_VLA_POLICY_CONFIG")
    cfg = vla.resolve(model or "openpi/pi05_libero", json.loads(Path(over_file).read_text()) if over_file else {})
    if not url and not cfg.get("url"):
        raise RuntimeError("vla needs ROBOUSE_VLA_URL (the policy server, e.g. ws://gpu-box:8000 for openpi)")
    client = workspace / ".robouse" / "vla_client.py"
    client.write_text(vla.CLIENT.read_text())
    env["ROBOUSE_VLA_CONFIG"] = json.dumps(cfg)
    env["ROBOUSE_VLA_URL"] = url or cfg["url"]
    if config.env("ROBOUSE_VLA_TOKEN"):
        env["ROBOUSE_VLA_TOKEN"] = config.env("ROBOUSE_VLA_TOKEN")
    env["ROBOUSE_TASK_LANGUAGE"] = _task_language(task)
    return Launch(
        [config.env("ROBOUSE_VLA_PYTHON", "python3"), str(client)],
        env,
        notes={"model": cfg["preset"], "wire": cfg["wire"], "server": env["ROBOUSE_VLA_URL"], "provider": "vla"},
    )


HARNESSES: dict[str, Builder] = {
    "oracle": _oracle,
    "noop": _noop,
    "claude-code": _claude_code,
    "claude-code-glm": _claude_code_glm,
    "codex": _codex,
    "codex-glm": _codex_glm,
    "mini-swe-agent-glm": _mini_swe_agent_glm,
    "molmoact2": _molmoact2,
    "vla": _vla,
}


def build(harness: str, model: str, prompt: str, workspace: Path, task, base_env: dict) -> Launch:
    """The launch for one trial. Raises KeyError for an unknown harness and RuntimeError for missing credentials."""
    env = dict(base_env)
    fn = HARNESSES.get(harness)
    if fn is not None:
        return fn(model, prompt, workspace, task, env)
    from importlib.metadata import entry_points

    for ep in entry_points(group="robouse.harnesses"):
        if ep.name == harness:
            return ep.load()(harness, model, prompt, workspace, task, env)
    raise KeyError(f"unknown harness {harness!r}; harnesses: {', '.join(HARNESSES)}")
