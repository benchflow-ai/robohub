#!/bin/bash
# Deterministic verifier: (1) protected upstream files are unchanged, (2) the hidden tests pass. Reward 1 or 0.
# The tests run from this directory with `python -I`, so nothing the agent put in the workspace can shadow pytest.
set -u
V="$(cd "$(dirname "$0")" && pwd)"
W="${SKILLS_STUDY_WORKDIR:-/workspace/genesis-world}"
S="${SKILLS_STUDY_DIR:-/opt/skills-study}"
mkdir -p /logs/verifier
reward() { echo "$1" > /logs/verifier/reward.txt; echo "reward $1"; exit 0; }
cd "$W" || reward 0
if ! python3 "$S/integrity.py" check "$S/protected.sha256"; then echo "FAIL: protected upstream files were changed"; reward 0; fi
cd "$V"
SKILLS_STUDY_WORKDIR="$W" timeout 480 python3 -I -m pytest -q -p no:cacheprovider -c "$V/pytest.ini" --rootdir "$V" "$V/tests" 2>&1 | tee /logs/verifier/pytest.txt
if [ "${PIPESTATUS[0]}" = 0 ]; then reward 1; else reward 0; fi
