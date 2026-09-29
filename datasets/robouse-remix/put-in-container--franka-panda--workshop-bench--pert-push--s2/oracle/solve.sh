#!/bin/bash
# Reference solution. Runs in the agent container and drives the robot only through the robo socket
# ($ROBO_SOCKET), like an agent would. vendor/ holds the robouse oracle code and BenchFlow's embodied
# client (and, for Meta-World, the scripted expert policies). BenchFlow hides /oracle from agents.
set -euo pipefail
export PYTHONPATH="$(cd "$(dirname "$0")" && pwd)/vendor${PYTHONPATH:+:$PYTHONPATH}"
export ROBO_ORACLE_TOKEN=2837e2b1eef7471fa4c98c3296c82e49
python3 -m robouse.oracle --backend remix --env put-in-container--franka-panda--workshop-bench--pert-push--s2
