"""Reference solutions for the CaP-X tasks: CaP-X's own reference code for the task (the ORACLE_CODE of its task class,
copied into the task's oracle/ folder as oracle_code.py), run against the API through capx_api, exactly as an agent's code
would run, then `robo done`."""

from __future__ import annotations

from pathlib import Path

from robouse.agent_cli import _send


def run(env: str) -> None:
    from . import capx_api

    code = Path(env).read_text()
    g = {"__name__": "__main__", **{n: getattr(capx_api, n) for n in capx_api.__all__}}
    try:
        exec(compile(code, env, "exec"), g, g)
    except RuntimeError as e:
        print(f"reference code stopped: {e}")
    _send({"op": "done", "text": "CaP-X reference code ran"})
