#!/bin/bash
# Reference solution. Runs in the agent container and drives the robot only through the robo socket
# ($ROBOUSE_SOCKET), like an agent would. vendor/ holds the Robo Use oracle code (and, for Meta-World,
# the scripted expert policy). BenchFlow uploads /oracle only in oracle mode; agents never see it.
set -euo pipefail
export PYTHONPATH="$(cd "$(dirname "$0")" && pwd)/vendor${PYTHONPATH:+:$PYTHONPATH}"
export ROBOUSE_ORACLE_TOKEN=3f07449e0ec38c6ef8c7b4c9a8948006
ROBOUSE_BEHAVIOR_PLAN="$(dirname "$0")/plan.json" python -m robouse.oracle --backend behavior --env store_produce
