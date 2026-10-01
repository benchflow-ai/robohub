"""Reference solutions: `python -m robouse.oracle --backend KIND --env ENV [--steps N]` runs the `oracle_main` of the
backend's package (`robouse.backends.<package>`). A reference solution drives the robot only through the episode socket,
like an agent would, which proves each task is solvable with the public interface."""

from __future__ import annotations

import argparse
import importlib
import sys

# oracle kinds whose reference solutions live outside the backend package's __init__
ORACLE_MODULES = {"tabletop_hard": "robouse.backends.tabletop.hard"}


def oracle_module(kind: str) -> str:
    """The module whose `oracle_main(env)` is the reference solution for oracle kind `kind`."""
    if kind in ORACLE_MODULES:
        return ORACLE_MODULES[kind]
    from .backends import BACKENDS

    if kind not in BACKENDS:
        raise KeyError(f"no reference solution for backend {kind!r}")
    return f"robouse.backends.{BACKENDS[kind][0]}"


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="robouse-oracle")
    ap.add_argument("--backend", required=True)
    ap.add_argument("--env", default="")
    ap.add_argument("--steps", type=int, help="step limit, for reference solutions that take one (metaworld)")
    a = ap.parse_args(argv)
    try:
        mod = importlib.import_module(oracle_module(a.backend))
    except KeyError as e:
        print(f"robouse-oracle: {e.args[0]}", file=sys.stderr)
        return 2
    if a.steps is not None:
        mod.oracle_main(a.env, steps=a.steps)
    else:
        mod.oracle_main(a.env)
    return 0


if __name__ == "__main__":
    sys.exit(main())
