#!/bin/bash
# Reference solution. Runs in the agent container and drives the robot only through the robo socket
# ($ROBO_SOCKET), like an agent would. vendor/ holds the robouse oracle code and BenchFlow's embodied
# client (and, for Meta-World, the scripted expert policies). BenchFlow hides /oracle from agents.
set -euo pipefail
export PYTHONPATH="$(cd "$(dirname "$0")" && pwd)/vendor${PYTHONPATH:+:$PYTHONPATH}"
export ROBO_ORACLE_TOKEN=ad2bcd4e30742993f6fef9a8c5941e77
python3 -m robouse.oracle --backend remix --env stack-in-order--franka-panda--warehouse--s4
