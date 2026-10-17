"""Credentials and clean config homes for agent harnesses. Secrets go to the harness child's environment only; they are
never printed or written to a trial folder.

  claude-code   ANTHROPIC_API_KEY, else CLAUDE_CODE_OAUTH_TOKEN (from `claude setup-token`)
  codex         OPENAI_API_KEY, else a private copy of ~/.codex/auth.json (from `codex login`)
The optional pools (ROBOUSE_CLAUDE_TOKEN_FILE, ROBOUSE_CODEX_AUTH_DIR; robouse._maintainer) are used only when none of
these is set. A provider-prefixed model's key (e.g. GLM_API_KEY for glm/glm-5.3) is not handled here: the model gateway
reads it from the environment (gateway.py).
"""

from __future__ import annotations

import itertools
import os
import shutil
import subprocess
import tempfile
from pathlib import Path

from .. import _maintainer, config

_claude_rr = itertools.count()
_codex_rr = itertools.count()


def clean_claude_home(workspace: Path) -> str:
    """An empty Claude Code config folder for one trial, inside its workspace: no user settings, CLAUDE.md, MCP servers
    or plugins leak into the trial, and nothing it writes there (sessions, auto-memory) reaches another trial."""
    d = workspace / ".robouse" / "claude-home"
    d.mkdir(parents=True, exist_ok=True, mode=0o700)
    return str(d)


def clean_codex_home(extra_config: str = "", api_key: str = "") -> str:
    """A Codex home with a minimal config: no personal MCP servers or skills.

    Each call makes a new private folder for one trial, under <cache>/codex-trials (not in the workspace: Codex makes
    no helper commands under a temporary folder), so no trial reads another's sessions or memories or races it for
    auth.json. The caller removes it when the trial ends (`remove_trial_home`).

    With `extra_config` (a custom provider) the home gets no login at all. Otherwise it is logged in with `api_key`
    (`codex login --with-api-key`, written into this home only), else it gets a private copy of ~/.codex/auth.json
    (never a link, never written back). When a Codex account pool is configured (ROBOUSE_CODEX_AUTH_DIR) it is used
    instead of ~/.codex: one persistent home per account, seeded once and shared by the trials on that account, so each
    OAuth session has a single refresh owner."""
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
    trial_homes().mkdir(parents=True, exist_ok=True)
    d = Path(tempfile.mkdtemp(prefix="codex-home-", dir=trial_homes()))  # private to this user (0700)
    auth, dst = Path.home() / ".codex" / "auth.json", d / "auth.json"
    (d / "config.toml").write_text('model_reasoning_effort = "medium"\n' + extra_config)
    if extra_config:
        return str(d)
    try:
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
    except BaseException:
        remove_trial_home(str(d))
        raise
    dst.chmod(0o600)
    return str(d)


def trial_homes() -> Path:
    """Where per-trial Codex homes go: <cache>/codex-trials."""
    return config.cache_dir() / "codex-trials"


def remove_trial_home(home: str) -> None:
    """Remove a per-trial Codex home (its auth.json is a copy of a login); any other folder is left alone."""
    if Path(home).parent == trial_homes():
        shutil.rmtree(home, ignore_errors=True)


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
