"""Maintainer-only fallbacks, isolated here. None of them is needed to use robouse; each applies only when its folder
exists on the machine.

- Credential pools under ~/.config/bf-agents: `oauth.env` (Claude Code OAuth tokens, rotated), `baseten.env`
  (BASETEN_API_KEY) and `codex-auth/*.json` (a pool of Codex accounts).
- Remote worker endpoint files inside a development checkout: <repo>/runs/_<name>/remote.json.
- Folders agents must never read in the local runner's sandbox: the maintainers' workspace (~/benchflow) and the
  credential pools.
"""

from __future__ import annotations

from pathlib import Path

AGENTS_DIR = Path.home() / ".config" / "bf-agents"
OAUTH_ENV = AGENTS_DIR / "oauth.env"
BASETEN_ENV = AGENTS_DIR / "baseten.env"
CODEX_POOL = AGENTS_DIR / "codex-auth"
WORKSPACE = Path.home() / "benchflow"
CHECKOUT = Path(__file__).resolve().parents[2]


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
    """Claude Code OAuth tokens from the pool (BF_REVIEW_TOKEN_* entries), in a stable order."""
    return [v for k, v in sorted(read_env_file(OAUTH_ENV).items()) if k.startswith("BF_REVIEW_TOKEN") and v]


def baseten_key() -> str:
    return read_env_file(BASETEN_ENV).get("BASETEN_API_KEY", "")


def codex_accounts() -> list[Path]:
    return sorted(CODEX_POOL.glob("*.json")) if CODEX_POOL.is_dir() else []


def remote_endpoint_files(name: str) -> list[Path]:
    return [CHECKOUT / "runs" / f"_{name}" / "remote.json"]


def private_dirs() -> list[Path]:
    return [WORKSPACE, AGENTS_DIR]
