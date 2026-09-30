#!/bin/bash
# Reference solution. Runs in the agent container and drives the robot only through the robo socket
# ($ROBOUSE_SOCKET), like an agent would. vendor/ holds the Robo Use oracle code (and, for Meta-World,
# the scripted expert policy). BenchFlow uploads /oracle only in oracle mode; agents never see it.
set -euo pipefail
export PYTHONPATH="$(cd "$(dirname "$0")" && pwd)/vendor${PYTHONPATH:+:$PYTHONPATH}"
# the reference solution needs mujoco==3.3.0 (the agent image has only numpy); install it outside /oracle
python3 -m pip install --quiet --disable-pip-version-check --no-cache-dir --target /tmp/robohub-oracle-deps mujoco==3.3.0
export PYTHONPATH="$PYTHONPATH:/tmp/robohub-oracle-deps"
python3 -m robouse.oracle --backend humanoid --env apollo-box-into-bin
