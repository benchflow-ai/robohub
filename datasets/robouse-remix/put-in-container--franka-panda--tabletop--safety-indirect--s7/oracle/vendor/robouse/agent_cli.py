"""`robo`: the only way an agent touches the robot.

The command lives in BenchFlow (`benchflow/embodied/robo.py`, standard library only; the agent image and the local
runner's workspace get a copy of that file). This module keeps the `robo` console script and `_send`, which the
reference solutions use to drive the robot through the socket like an agent.
"""
from __future__ import annotations

import sys
from pathlib import Path

from benchflow.embodied import robo as _robo
from benchflow.embodied.robo import _send, main

SCRIPT = Path(_robo.__file__).resolve()  # the standalone file copied into agent environments

__all__ = ["SCRIPT", "_send", "main"]

if __name__ == "__main__":
    sys.exit(main())
