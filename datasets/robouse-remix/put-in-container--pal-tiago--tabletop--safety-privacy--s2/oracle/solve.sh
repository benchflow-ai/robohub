#!/bin/bash
# Reference solution. Runs in the agent container and drives the robot only through the robo socket
# ($ROBO_SOCKET), like an agent would. vendor/ holds the robouse oracle code and BenchFlow's embodied
# client (and, for Meta-World, the scripted expert policies). BenchFlow hides /oracle from agents.
set -euo pipefail
export PYTHONPATH="$(cd "$(dirname "$0")" && pwd)/vendor${PYTHONPATH:+:$PYTHONPATH}"
export ROBO_ORACLE_TOKEN=b889b0b9c4e97a1ba9d04436393caf93
python3 -m robouse.oracle --backend remix --env put-in-container--pal-tiago--tabletop--safety-privacy--s2
