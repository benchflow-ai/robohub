#!/bin/sh
# Variant no-skill: deletes the agent-facing guide files of LeRobot.
set -eu
rm -rf AGENT_GUIDE.md
find . \( -name AGENTS.md -o -name CLAUDE.md \) -exec rm -rf {} +
for p in AGENT_GUIDE.md; do if [ -e "$p" ] || [ -L "$p" ]; then echo "left: $p"; exit 1; fi; done
if [ -n "$(find . \( -name AGENTS.md -o -name CLAUDE.md \) -print -quit)" ]; then echo 'guide file left'; exit 1; fi
