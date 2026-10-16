"""Optional credential pools for running many trials in a row. Nothing here is needed to use robouse, and nothing is
read unless its variable is set.

- ROBOUSE_CLAUDE_TOKEN_FILE: a shell-style env file of Claude Code OAuth tokens (`NAME=token` lines; every non-empty
  value is a token), used in turn when neither ANTHROPIC_API_KEY nor CLAUDE_CODE_OAUTH_TOKEN is set.
- ROBOUSE_CODEX_AUTH_DIR: a folder of Codex logins (`<account>.json`, each a copy of a `codex login` auth.json), used in
  turn instead of ~/.codex/auth.json when OPENAI_API_KEY is not set.

No model provider's key is read from a file: the model gateway takes it from the environment only. Both locations go
on the local runner's sandbox deny list (`config.sandbox_denied_paths`), so agents can read neither; for the token file
that is the file alone, not its folder. Other private folders go in ROBOUSE_SANDBOX_DENY. Use a pool only as far as your providers' terms allow.
"""

from __future__ import annotations

from pathlib import Path

from . import config


def _location(name: str) -> Path | None:
    v = config.env(name).strip()
    return Path(v).expanduser() if v else None


def read_env_file(path: Path) -> dict[str, str]:
    """KEY=VALUE lines of a shell-style env file (comments, `export` and quotes allowed); {} when it is missing."""
    out: dict[str, str] = {}
    if path.is_file():
        for line in path.read_text().splitlines():
            line = line.strip()
            if line and not line.startswith("#") and "=" in line:
                k, v = line.split("=", 1)
                out[k.strip().removeprefix("export ").strip()] = v.strip().strip('"').strip("'")
    return out


def claude_tokens() -> list[str]:
    """Claude Code OAuth tokens from $ROBOUSE_CLAUDE_TOKEN_FILE, in a stable order (by name); [] when it is not set."""
    f = _location("ROBOUSE_CLAUDE_TOKEN_FILE")
    return [v for _, v in sorted(read_env_file(f).items()) if v] if f else []


def codex_accounts() -> list[Path]:
    """Codex logins in $ROBOUSE_CODEX_AUTH_DIR, sorted; [] when it is not set."""
    d = _location("ROBOUSE_CODEX_AUTH_DIR")
    return sorted(d.glob("*.json")) if d and d.is_dir() else []


def private_dirs() -> list[Path]:
    """The configured pool locations, which agents must never read."""
    return [p for p in (_location("ROBOUSE_CLAUDE_TOKEN_FILE"), _location("ROBOUSE_CODEX_AUTH_DIR")) if p]
