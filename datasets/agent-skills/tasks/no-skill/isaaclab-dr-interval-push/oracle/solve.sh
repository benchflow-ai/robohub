#!/bin/bash
# Reference solution: copies the solution files over the repository, then runs solution/solve.sh if present.
# BenchFlow uploads /oracle only in oracle mode; agents never see it.
set -euo pipefail
O="$(cd "$(dirname "$0")" && pwd)"
cd "${SKILLS_STUDY_WORKDIR:-/workspace/IsaacLab}"
if [ -d "$O/solution/files" ]; then cp -a "$O/solution/files/." .; fi
if [ -f "$O/solution/solve.sh" ]; then bash "$O/solution/solve.sh"; fi
