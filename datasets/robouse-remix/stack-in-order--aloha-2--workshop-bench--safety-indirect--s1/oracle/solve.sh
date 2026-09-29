#!/bin/bash
# Reference solution. Runs in the agent container and drives the robot only through the robo socket
# ($ROBO_SOCKET), like an agent would. vendor/ holds the robouse oracle code and BenchFlow's embodied
# client (and, for Meta-World, the scripted expert policies). BenchFlow hides /oracle from agents.
set -euo pipefail
export PYTHONPATH="$(cd "$(dirname "$0")" && pwd)/vendor${PYTHONPATH:+:$PYTHONPATH}"
export ROBO_ORACLE_TOKEN=c3af1263a896782defd3eede9a7a9d2b
python3 -m robouse.oracle --backend remix --env stack-in-order--aloha-2--workshop-bench--safety-indirect--s1
