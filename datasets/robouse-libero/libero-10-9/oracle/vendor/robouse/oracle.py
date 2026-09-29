"""Reference solutions. They drive the robot only through the episode socket, like an agent would,
which proves each task is solvable with the public interface.

metaworld: Meta-World's scripted expert reads `obs_vector` from `robo observe` and sends `robo act`.
tabletop:  scripted pick/place using the move_to and grip skills and public object positions.
libero:    replays one LIBERO demonstration's recorded actions (the task starts from that demonstration's first state).
"""
from __future__ import annotations

import argparse
import sys

import numpy as np

from .agent_cli import _send


def _obs() -> dict:
    r = _send({"op": "observe"})
    if not r.get("ok"):
        raise RuntimeError(r.get("error"))
    return r["result"]


def metaworld_oracle(env_name: str, steps: int) -> None:
    """Run the scripted expert for `steps` env steps, then ask for scoring."""
    from metaworld.policies import ENV_POLICY_MAP

    pol = ENV_POLICY_MAP[env_name]()
    for _ in range(steps):
        r = _send({"op": "observe"})
        if not r.get("ok"):  # episode already finished (e.g. goal reached)
            return
        st = r["result"]["state"]
        a = np.clip(np.asarray(pol.get_action(np.asarray(st["obs_vector"], dtype=np.float64)), dtype=float), -1, 1)
        r = _send({"op": "act", "action": [float(x) for x in a], "repeat": 1})
        if not r.get("ok") or "episode" in (r.get("result") or {}):
            return
    _send({"op": "done", "text": "oracle finished"})


def _upstream() -> dict:
    from .backends import UPSTREAM

    return UPSTREAM


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="robouse-oracle")
    ap.add_argument("--backend", required=True)
    ap.add_argument("--env", default="")
    ap.add_argument("--steps", type=int, default=400)
    a = ap.parse_args(argv)
    if a.backend == "metaworld":
        metaworld_oracle(a.env, a.steps)
    elif a.backend in ("tabletop", "tabletop_hard", "gymrobotics", "libero", "menagerie", "robocasa", "behavior", "roboharm", "drone", "dexjoco",
                       "quadruped", "humanoid", "mobile_manip", "dexhand", "crazyflie", "driving"):
        import importlib

        # each backend module provides oracle_main(env: str) that drives the robot through the socket
        importlib.import_module(f".backends.{a.backend}", __package__).oracle_main(a.env)
    elif a.backend == "robosuite":  # module name differs from the backend name (a `robosuite` module would shadow the package)
        from .backends.robosuite_backend import oracle_main

        oracle_main(a.env)
    elif a.backend in _upstream():  # backends/__init__.py UPSTREAM: module name may differ from the backend name
        import importlib

        importlib.import_module(f".backends.{_upstream()[a.backend][0]}", __package__).oracle_main(a.env)
    else:
        print(f"no oracle for backend {a.backend}")
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
