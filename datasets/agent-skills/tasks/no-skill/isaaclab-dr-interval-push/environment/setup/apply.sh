#!/bin/sh
# Applies the task's starting state in the repository root (run by the Dockerfile and by tools/check.py).
set -eu
D="$(cd "$(dirname "$0")" && pwd)"
if [ -d "$D/files" ]; then cp -a "$D/files/." .; fi
if [ -f "$D/setup.sh" ]; then sh "$D/setup.sh"; fi
