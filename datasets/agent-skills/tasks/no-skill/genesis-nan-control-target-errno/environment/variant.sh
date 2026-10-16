#!/bin/sh
# Variant no-skill: deletes the agent-facing guide files of Genesis.
set -eu
find . \( -name CLAUDE.md -o -name AGENTS.md \) -exec rm -rf {} +
if [ -n "$(find . \( -name CLAUDE.md -o -name AGENTS.md \) -print -quit)" ]; then echo 'guide file left'; exit 1; fi
