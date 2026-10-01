"""Harness launchers: the command, environment and helper process for one trial's agent.

The agent process gets: its prompt (PROMPT_PREFIX + the task.md body), cwd = an isolated workspace (observations/),
PATH with a standard-library `robo` shim first, and ROBOUSE_SOCKET. Credentials come from the environment
(`credentials.py`).

Harnesses
  oracle                 the task's reference solution (oracle/solve.sh)
  noop                   calls `robo done` immediately (negative control)
  claude-code            Claude Code CLI; model: a bare Claude id (Anthropic, own credentials) or any provider-prefixed
                         id, e.g. baseten/zai-org/GLM-5.3, reached through the trial's model gateway (gateway.py)
  codex                  Codex CLI; model: a bare OpenAI id (own credentials or the account pool) or a provider-prefixed
                         id through the gateway
  mini-swe-agent         mini-swe-agent; model: a provider-prefixed id, always through the gateway
  (claude-code-glm, codex-glm and mini-swe-agent-glm: hidden aliases for the three above with a baseten/ model)
  dimcode                Dimensional's dimcode (built on Pi); model: a provider-prefixed id, always through the
                         gateway (default baseten/zai-org/GLM-5.3); see dimcode.py
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
    stop: Callable[[], None] | None = None  # stops a helper process (the model gateway) after the agent exits
    stop_file: Path | None = None  # written by the model gateway when the trial's spend cap is reached
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


PRIVATE_ENV = ("_ROBOUSE_TRIAL_DIR", "_ROBOUSE_IMAGE_INPUT", "_ROBOUSE_SPEND_CAP_USD")  # runner to builder only


def _gateway(ref, env: dict, protocol: str):
    """Start this trial's model gateway (harnesses/gateway.py). The runner passes the trial folder, the image-input
    decision and the spend cap in private variables, removed here so the agent never sees them."""
    from . import gateway

    run_dir = Path(env.pop("_ROBOUSE_TRIAL_DIR"))
    images = env.pop("_ROBOUSE_IMAGE_INPUT", "1") == "1"
    cap = float(env.pop("_ROBOUSE_SPEND_CAP_USD", "0") or 0)
    return gateway.start(ref, run_dir, images=images, protocol=protocol, spend_cap_usd=cap)


def _gateway_launch(gw, ref, cmd: list[str], env: dict) -> Launch:
    return Launch(
        cmd,
        env,
        helper=gw.stats,
        stop=gw.stop,
        stop_file=gw.stop_file,
        notes={"model": ref.bare, "model_id": ref.id, "provider": ref.provider},
    )


def _via_gateway(harness: str, model: str) -> object | None:
    """The resolved model when `harness` should reach it through the gateway, else None (the harness's own vendor)."""
    from . import models

    ref = models.resolve(harness, model)
    return None if models.NATIVE_VENDORS.get(harness) == ref.provider else ref


def _claude_code(model, prompt, workspace, task, env) -> Launch:
    if model and (ref := _via_gateway("claude-code", model)) is not None:
        home = credentials.clean_claude_home()
        gw = _gateway(ref, env, "anthropic-messages")
        env.update(
            {
                "ANTHROPIC_BASE_URL": gw.url,
                "ANTHROPIC_AUTH_TOKEN": gw.key,
                "ANTHROPIC_MODEL": gw.model,
                "ANTHROPIC_DEFAULT_OPUS_MODEL": gw.model,
                "ANTHROPIC_DEFAULT_SONNET_MODEL": gw.model,
                "ANTHROPIC_DEFAULT_HAIKU_MODEL": gw.model,
                "ANTHROPIC_SMALL_FAST_MODEL": gw.model,
                "CLAUDE_CODE_DISABLE_NONESSENTIAL_TRAFFIC": "1",
                "CLAUDE_CONFIG_DIR": home,
            }
        )
        return _gateway_launch(gw, ref, _claude_cmd(prompt, gw.model), env)
    m = (model or "claude-sonnet-5").removeprefix("anthropic/")
    env.update(credentials.claude_auth())
    env["CLAUDE_CONFIG_DIR"] = credentials.clean_claude_home()
    return Launch(_claude_cmd(prompt, m), env, notes={"model": m})


def _codex(model, prompt, workspace, task, env) -> Launch:
    import os

    if model and (ref := _via_gateway("codex", model)) is not None:
        home = credentials.clean_codex_home("# model provider: the trial's gateway, set on the command line\n")
        gw = _gateway(ref, env, "openai-responses")
        env["ROBOUSE_GATEWAY_KEY"] = gw.key
        env["ROBOUSE_GATEWAY_URL"] = gw.url  # also lets the macOS sandbox allow this local port
        env["CODEX_HOME"] = home
        cfg = [
            "-c",
            'model_providers.robouse.name="Robo Use gateway"',
            "-c",
            f'model_providers.robouse.base_url="{gw.url}/v1"',
            "-c",
            'model_providers.robouse.env_key="ROBOUSE_GATEWAY_KEY"',
            "-c",
            'model_providers.robouse.wire_api="responses"',
            "-c",
            'model_provider="robouse"',
        ]
        # the provider's own Responses endpoint, the request passed through unchanged (reasoning effort and earlier
        # reasoning kept); the gateway moves any image out of a tool result into the next user message
        return _gateway_launch(gw, ref, _codex_cmd(prompt, gw.model, workspace, cfg), env)
    m = (model or "gpt-6-astra").removeprefix("openai/")
    key = os.environ.get("OPENAI_API_KEY", "")
    env["CODEX_HOME"] = credentials.clean_codex_home(api_key=key)
    if key:
        env["CODEX_API_KEY"] = key
    return Launch(_codex_cmd(prompt, m, workspace), env, notes={"model": m})


def _mini_swe_agent(model, prompt, workspace, task, env) -> Launch:
    from . import models

    ref = models.resolve("mini-swe-agent", model or models.DEFAULT_MODELS["mini-swe-agent"])
    exe = shutil.which("mini") or str(Path.home() / ".local" / "bin" / "mini")
    gw = _gateway(ref, env, "openai-completions")
    env.update(
        {
            "OPENAI_API_KEY": gw.key,
            "OPENAI_API_BASE": f"{gw.url}/v1",
            "MSWEA_CONFIGURED": "true",
            "MSWEA_COST_TRACKING": "ignore_errors",
        }
    )
    cmd = [
        exe,
        "-y",
        "--exit-immediately",
        "-l",
        "0",
        "-m",
        f"openai/{gw.model}",
        "-t",
        PROMPT_PREFIX + prompt,
        "-o",
        str(workspace / ".robouse" / "mini_traj.json"),
    ]
    return _gateway_launch(gw, ref, cmd, env)


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


def _dimcode(model, prompt, workspace, task, env) -> Launch:
    from . import dimcode

    return dimcode.build(model, prompt, workspace, task, env)


HARNESSES: dict[str, Builder] = {
    "oracle": _oracle,
    "noop": _noop,
    "claude-code": _claude_code,
    "codex": _codex,
    "mini-swe-agent": _mini_swe_agent,
    "molmoact2": _molmoact2,
    "vla": _vla,
    "dimcode": _dimcode,
}


def build(harness: str, model: str, prompt: str, workspace: Path, task, base_env: dict) -> Launch:
    """The launch for one trial. Raises KeyError for an unknown harness and RuntimeError for missing credentials."""
    from . import models

    env = dict(base_env)
    harness, model = models.legacy(harness, model)  # claude-code-glm etc.: hidden aliases
    fn = HARNESSES.get(harness)
    if fn is None:
        from importlib.metadata import entry_points

        fn = next((ep.load() for ep in entry_points(group="robouse.harnesses") if ep.name == harness), None)
        if fn is None:
            raise KeyError(f"unknown harness {harness!r}; harnesses: {', '.join(HARNESSES)}")
        for k in PRIVATE_ENV:
            env.pop(k, None)
        return fn(harness, model, prompt, workspace, task, env)
    launch = fn(model, prompt, workspace, task, env)
    for k in PRIVATE_ENV:  # never in the agent's environment
        launch.env.pop(k, None)
    return launch
