"""Harness launchers. Each returns the command, environment and any helper process for one trial.

The agent process gets: cwd = an isolated workspace (instruction.md, observations/), PATH with a
standard-library `robo` shim first, and ROBOUSE_SOCKET. Credentials come from the environment (see below) and are
passed to the harness child only (never printed or written to the trial dir).

Credentials
  claude-code            ANTHROPIC_API_KEY, else CLAUDE_CODE_OAUTH_TOKEN (from `claude setup-token`)
  codex                  OPENAI_API_KEY, else a private copy of ~/.codex/auth.json (from `codex login`)
  *-glm                  BASETEN_API_KEY (default model zai-org/GLM-5.3; --model takes any Baseten model id, e.g. moonshotai/Kimi-K3)
Optional maintainer fallback: token and key files under ~/.config/bf-agents (oauth.env, baseten.env) when present.

Harnesses
  oracle                 the task's reference solution (oracle/solve.sh)
  noop                   calls `robo done` immediately (negative control)
  claude-code            Claude Code CLI; model e.g. claude-sonnet-5
  codex                  Codex CLI; model e.g. gpt-6-astra
  claude-code-glm        Claude Code CLI against GLM-5.3 on Baseten via a local image-capping proxy
  codex-glm              Codex CLI with a custom OpenAI-compatible provider (Baseten GLM-5.3)
  mini-swe-agent-glm     mini-swe-agent (litellm) with GLM-5.3 on Baseten
  molmoact2              MolmoAct2 VLA policy: molmoact2/client.py drives `robo` with action chunks from a GPU policy
                         server (molmoact2/server.py) at $ROBOUSE_MOLMOACT2_URL; model e.g. allenai/MolmoAct2-LIBERO-LeRobot
"""
from __future__ import annotations

import os
import shutil
import subprocess
from dataclasses import dataclass, field
from pathlib import Path

HOME = Path.home()
OAUTH_ENV = HOME / ".config/bf-agents/oauth.env"
BASETEN_ENV = HOME / ".config/bf-agents/baseten.env"
CODEX_POOL = HOME / ".config/bf-agents/codex-auth"  # optional maintainers' Codex account pool (auth.json files)
GLM_MODEL = "zai-org/GLM-5.3"
BASETEN_OPENAI = "https://inference.baseten.co/v1"

PROMPT_PREFIX = (
    "You are controlling a simulated robot. Read the task below, then solve it by running the `robo` command "
    "in your shell (start with `robo info` and `robo observe`). Keep going until the task is done, then call "
    "`robo done` once. Do not stop to ask questions; there is no human to answer.\n\n"
)


@dataclass
class Launch:
    cmd: list[str]
    env: dict[str, str]
    helper: object | None = None  # e.g. a proxy server to stop afterwards
    notes: dict = field(default_factory=dict)


def _read_env(path: Path) -> dict[str, str]:
    out = {}
    if path.exists():
        for line in path.read_text().splitlines():
            line = line.strip()
            if line and not line.startswith("#") and "=" in line:
                k, v = line.split("=", 1)
                out[k.strip().removeprefix("export ").strip()] = v.strip().strip('"').strip("'")
    return out


_token_rr = {"i": 0}
_codex_rr = {"i": 0}
CACHE = HOME / ".cache" / "robouse"


def _clean_claude_home() -> str:
    """Empty Claude Code config dir: no user settings, CLAUDE.md, MCP servers or plugins leak into trials."""
    d = CACHE / "claude-home"
    d.mkdir(parents=True, exist_ok=True)
    return str(d)


def _clean_codex_home(extra_config: str = "", api_key: str = "") -> str:
    """Codex home with a minimal config: no personal MCP servers or skills.

    The OpenAI home is logged in with OPENAI_API_KEY when it is set (`codex login --with-api-key`, written into this
    home only), else it gets a private copy of the user's ~/.codex/auth.json (never a link, and never written to).
    The GLM home gets no login at all: it authenticates to Baseten through env_key."""
    pool = sorted(CODEX_POOL.glob("*.json")) if not extra_config and not api_key and CODEX_POOL.is_dir() else []
    if pool:
        # maintainers' account pool: one persistent home per account, seeded once, so each OAuth session has a
        # single refresh owner (this home) and a refreshed token is never lost to a fresh copy
        acct = pool[_codex_rr["i"] % len(pool)]
        _codex_rr["i"] += 1
        d = CACHE / f"codex-home-{acct.stem}"
        d.mkdir(parents=True, exist_ok=True)
        (d / "config.toml").write_text('model_reasoning_effort = "medium"\n')
        if not (d / "auth.json").exists():
            shutil.copyfile(acct, d / "auth.json")
            (d / "auth.json").chmod(0o600)
        return str(d)
    d = CACHE / ("codex-home-glm" if extra_config else "codex-home")
    d.mkdir(parents=True, exist_ok=True)
    auth, dst = HOME / ".codex" / "auth.json", d / "auth.json"
    if dst.is_symlink() or dst.exists():
        dst.unlink()
    (d / "config.toml").write_text('model_reasoning_effort = "medium"\n' + extra_config)
    if extra_config:
        return str(d)
    if api_key:
        exe = shutil.which("codex")
        if not exe:
            raise RuntimeError("codex is not on PATH (install the Codex CLI)")
        r = subprocess.run([exe, "login", "--with-api-key"], input=api_key, text=True, capture_output=True,
                           env={**os.environ, "CODEX_HOME": str(d)})
        if r.returncode != 0 or not dst.exists():
            raise RuntimeError("codex login --with-api-key failed for OPENAI_API_KEY")
    elif auth.exists():
        shutil.copyfile(auth, dst)
    else:
        raise RuntimeError("no Codex credentials: set OPENAI_API_KEY, or log in once with `codex login`")
    dst.chmod(0o600)
    return str(d)


def _claude_auth() -> dict[str, str]:
    """Environment for Claude Code: ANTHROPIC_API_KEY, else CLAUDE_CODE_OAUTH_TOKEN, else the maintainers' optional
    token file (rotating over its BF_REVIEW_TOKEN_* entries)."""
    if os.environ.get("ANTHROPIC_API_KEY"):
        return {"ANTHROPIC_API_KEY": os.environ["ANTHROPIC_API_KEY"]}
    if os.environ.get("CLAUDE_CODE_OAUTH_TOKEN"):
        return {"CLAUDE_CODE_OAUTH_TOKEN": os.environ["CLAUDE_CODE_OAUTH_TOKEN"]}
    toks = [v for k, v in sorted(_read_env(OAUTH_ENV).items()) if k.startswith("BF_REVIEW_TOKEN") and v]
    if not toks:
        raise RuntimeError("no Claude credentials: set ANTHROPIC_API_KEY, or CLAUDE_CODE_OAUTH_TOKEN "
                           "(create one with `claude setup-token`)")
    t = toks[_token_rr["i"] % len(toks)]
    _token_rr["i"] += 1
    return {"CLAUDE_CODE_OAUTH_TOKEN": t}


def _baseten_key() -> str:
    k = os.environ.get("BASETEN_API_KEY") or _read_env(BASETEN_ENV).get("BASETEN_API_KEY")
    if not k:
        raise RuntimeError("no Baseten key: set BASETEN_API_KEY")
    return k


def build(harness: str, model: str, prompt: str, workspace: Path, task, base_env: dict) -> Launch:
    env = dict(base_env)
    if harness == "oracle":
        return Launch(["bash", str(task.oracle_script)], env, notes={"model": "scripted"})
    if harness == "noop":
        return Launch(["robo", "done", "noop control"], env, notes={"model": "none"})

    if harness == "claude-code":
        env.update(_claude_auth())
        env["CLAUDE_CONFIG_DIR"] = _clean_claude_home()
        m = model or "claude-sonnet-5"
        return Launch(["claude", "-p", PROMPT_PREFIX + prompt, "--model", m, "--output-format", "stream-json", "--verbose",
                       "--dangerously-skip-permissions", "--max-turns", "400", "--disallowedTools", "WebFetch,WebSearch,Task"],
                      env, notes={"model": m})

    if harness == "claude-code-glm":
        bm = model or GLM_MODEL  # any Baseten model id, e.g. moonshotai/Kimi-K3
        from .providers import baseten_proxy

        port, stats = baseten_proxy.start(_baseten_key())
        env.update({"ANTHROPIC_BASE_URL": f"http://127.0.0.1:{port}", "ANTHROPIC_AUTH_TOKEN": "proxy-held",
                    "ANTHROPIC_API_KEY": "", "ANTHROPIC_MODEL": bm, "ANTHROPIC_DEFAULT_OPUS_MODEL": bm,
                    "ANTHROPIC_DEFAULT_SONNET_MODEL": bm, "ANTHROPIC_DEFAULT_HAIKU_MODEL": bm,
                    "ANTHROPIC_SMALL_FAST_MODEL": bm, "CLAUDE_CODE_DISABLE_NONESSENTIAL_TRAFFIC": "1"})
        env.pop("CLAUDE_CODE_OAUTH_TOKEN", None)
        env["CLAUDE_CONFIG_DIR"] = _clean_claude_home()
        return Launch(["claude", "-p", PROMPT_PREFIX + prompt, "--model", bm, "--output-format", "stream-json",
                       "--verbose", "--dangerously-skip-permissions", "--max-turns", "400",
                       "--disallowedTools", "WebFetch,WebSearch,Task"], env, helper=stats, notes={"model": bm, "provider": "baseten"})

    if harness == "codex":
        m = model or "gpt-6-astra"
        key = os.environ.get("OPENAI_API_KEY", "")
        env["CODEX_HOME"] = _clean_codex_home(api_key=key)
        if key:
            env["CODEX_API_KEY"] = key
        return Launch(["codex", "exec", "--json", "--skip-git-repo-check", "--dangerously-bypass-approvals-and-sandbox",
                       "-m", m, "-C", str(workspace), PROMPT_PREFIX + prompt], env, notes={"model": m})

    if harness == "codex-glm":
        bm = model or GLM_MODEL  # any Baseten model id, e.g. moonshotai/Kimi-K3
        env["BASETEN_API_KEY"] = _baseten_key()
        env["ROBOUSE_TEXT_ONLY"] = "1"  # Baseten's Responses API rejects images; Codex attaches PNGs it sees in command output
        env["CODEX_HOME"] = _clean_codex_home("# GLM via Baseten; provider set on the command line\n"
                                               "# Baseten's Responses API rejects image input, so the image viewer tool is off\n"
                                               "[tools]\nview_image = false\n")
        cfg = ['-c', 'model_providers.baseten.name="Baseten"', '-c', f'model_providers.baseten.base_url="{BASETEN_OPENAI}"',
               '-c', 'model_providers.baseten.env_key="BASETEN_API_KEY"', '-c', 'model_providers.baseten.wire_api="responses"',
               '-c', 'model_provider="baseten"']
        return Launch(["codex", "exec", "--json", "--skip-git-repo-check", "--dangerously-bypass-approvals-and-sandbox",
                       *cfg, "-m", bm, "-C", str(workspace), PROMPT_PREFIX + prompt], env,
                      notes={"model": bm, "provider": "baseten"})

    if harness == "mini-swe-agent-glm":
        bm = model or GLM_MODEL  # any Baseten model id, e.g. moonshotai/Kimi-K3
        env.update({"OPENAI_API_KEY": _baseten_key(), "OPENAI_API_BASE": BASETEN_OPENAI, "MSWEA_CONFIGURED": "true",
                    "MSWEA_COST_TRACKING": "ignore_errors"})
        exe = shutil.which("mini") or str(HOME / ".local/bin/mini")
        (workspace / ".robouse" / "task.txt").write_text(PROMPT_PREFIX + prompt)
        return Launch([exe, "-y", "--exit-immediately", "-l", "0", "-m", f"openai/{bm}", "-t", PROMPT_PREFIX + prompt, "-o",
                       str(workspace / ".robouse" / "mini_traj.json")], env, notes={"model": bm, "provider": "baseten"})

    if harness == "molmoact2":
        if not os.environ.get("ROBOUSE_MOLMOACT2_URL"):
            raise RuntimeError("molmoact2 needs ROBOUSE_MOLMOACT2_URL (the URL of a MolmoAct2 policy server)")
        client = workspace / ".robouse" / "molmoact2_client.py"
        client.write_text((Path(__file__).parent / "molmoact2" / "client.py").read_text())
        # the policy gets the source benchmark's own language instruction (LIBERO's), else the task description
        env["ROBOUSE_TASK_LANGUAGE"] = str(task.metadata.get("language_instruction") or task.meta.get("task", {}).get("description", ""))
        m = model or "allenai/MolmoAct2-LIBERO-LeRobot"
        return Launch(["python3", str(client)], env, notes={"model": m, "server": os.environ["ROBOUSE_MOLMOACT2_URL"]})

    raise KeyError(f"unknown harness {harness!r}")
