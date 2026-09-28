#!/bin/bash
# Negative control: look at the robot, never move it, exit without `robo done`.
# The verifier must close the episode itself (outcome agent_exited) and the reward must be 0.
set -euo pipefail
robo info
robo observe
