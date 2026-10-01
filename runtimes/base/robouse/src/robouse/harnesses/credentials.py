"""Credentials and clean config homes for agent harnesses. Secrets go to the harness child's environment only; they are
never printed or written to a trial folder.

  claude-code   ANTHROPIC_API_KEY, else CLAUDE_CODE_OAUTH_TOKEN (from `claude setup-token`)
  codex         OPENAI_API_KEY, else a private copy of ~/.codex/auth.json (from `codex login`)
  *-glm         BASETEN_API_KEY
Maintainer pools (robouse._maintainer) are used only when none of these is set.
"""

from __future__ import annotations

import itertools
import os
import shutil
import subprocess
from pathlib import Path

from .. import _maintainer, config

_claude_rr = itertools.count()
_codex_rr = itertools.count()


def clean_claude_home() -> str:
    """Empty Claude Code config dir: no user settings, CLAUDE.md, MCP servers or plugins leak into trials."""
    d = config.cache_dir() / "claude-home"
    d.mkdir(parents=True, exist_ok=True)
    return str(d)


def clean_codex_home(extra_config: str = "", api_key: str = "") -> str:
    """Codex home with a minimal config: no personal MCP servers or skills.

    With `extra_config` (a custom provider) the home gets no login at all. Otherwise it is logged in with `api_key`
    (`codex login --with-api-key`, written into this home only), else it gets a private copy of ~/.codex/auth.json
    (never a link, never written back). When the maintainers' account pool exists it is used instead of
    ~/.codex: one persistent home per account, seeded once, so each OAuth session has a single refresh owner."""
    pool = _maintainer.codex_accounts() if not extra_config and not api_key else []
    if pool:
        acct = pool[next(_codex_rr) % len(pool)]
        d = config.cache_dir() / f"codex-home-{acct.stem}"
        d.mkdir(parents=True, exist_ok=True)
        (d / "config.toml").write_text('model_reasoning_effort = "medium"\n')
        if not (d / "auth.json").exists():
            shutil.copyfile(acct, d / "auth.json")
            (d / "auth.json").chmod(0o600)
        return str(d)
    d = config.cache_dir() / ("codex-home-glm" if extra_config else "codex-home")
    d.mkdir(parents=True, exist_ok=True)
    auth, dst = Path.home() / ".codex" / "auth.json", d / "auth.json"
    if dst.is_symlink() or dst.exists():
        dst.unlink()
    (d / "config.toml").write_text('model_reasoning_effort = "medium"\n' + extra_config)
    if extra_config:
        return str(d)
    if api_key:
        exe = shutil.which("codex")
        if not exe:
            raise RuntimeError("codex is not on PATH; install the Codex CLI")
        r = subprocess.run(
            [exe, "login", "--with-api-key"],
            input=api_key,
            text=True,
            capture_output=True,
            env={**os.environ, "CODEX_HOME": str(d)},
            check=False,
        )
        if r.returncode != 0 or not dst.exists():
            raise RuntimeError("`codex login --with-api-key` failed for OPENAI_API_KEY")
    elif auth.exists():
        shutil.copyfile(auth, dst)
    else:
        raise RuntimeError("no Codex credentials: set OPENAI_API_KEY, or log in once with `codex login`")
    dst.chmod(0o600)
    return str(d)


def claude_auth() -> dict[str, str]:
    """Environment for Claude Code: ANTHROPIC_API_KEY, else CLAUDE_CODE_OAUTH_TOKEN, else a pool token (rotated)."""
    if os.environ.get("ANTHROPIC_API_KEY"):
        return {"ANTHROPIC_API_KEY": os.environ["ANTHROPIC_API_KEY"]}
    if os.environ.get("CLAUDE_CODE_OAUTH_TOKEN"):
        return {"CLAUDE_CODE_OAUTH_TOKEN": os.environ["CLAUDE_CODE_OAUTH_TOKEN"]}
    toks = _maintainer.claude_tokens()
    if not toks:
        raise RuntimeError(
            "no Claude credentials: set ANTHROPIC_API_KEY, or CLAUDE_CODE_OAUTH_TOKEN "
            "(create one with `claude setup-token`)"
        )
    return {"CLAUDE_CODE_OAUTH_TOKEN": toks[next(_claude_rr) % len(toks)]}


def baseten_key() -> str:
    k = os.environ.get("BASETEN_API_KEY") or _maintainer.baseten_key()
    if not k:
        raise RuntimeError("no Baseten key: set BASETEN_API_KEY")
    return k


def have_claude() -> bool:
    return bool(
        os.environ.get("ANTHROPIC_API_KEY") or os.environ.get("CLAUDE_CODE_OAUTH_TOKEN") or _maintainer.claude_tokens()
    )


def have_codex() -> bool:
    return bool(
        os.environ.get("OPENAI_API_KEY")
        or (Path.home() / ".codex" / "auth.json").exists()
        or _maintainer.codex_accounts()
    )


def have_baseten() -> bool:
    return bool(os.environ.get("BASETEN_API_KEY") or _maintainer.baseten_key())
