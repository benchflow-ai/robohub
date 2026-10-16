#!/bin/bash
# Runs the agent-skills study: 2 agents x 2 variants x 5 trials of every task in tasks/<variant>/.
#   cd datasets/agent-skills && bash study/run_study.sh            # all four cells, trials 1-5
#   CELLS="codex--no-skill" TRIALS="3 4 5" bash study/run_study.sh   # resume part of it
# Needs ANTHROPIC_API_KEY (Claude Code cells), OPENAI_API_KEY (Codex cells) and DAYTONA_API_KEY.
set -euo pipefail
cd "$(dirname "$0")/.."
CELLS=${CELLS:-"claude-code--with-skill claude-code--no-skill codex--with-skill codex--no-skill"}
TRIALS=${TRIALS:-"1 2 3 4 5"}
EXPECTED=$(ls tasks/with-skill | wc -l)
for trial in $TRIALS; do
  for cell in $CELLS; do
    echo "== $cell trial $trial"
    bench eval run --config "study/$cell.yaml" --jobs-dir "jobs-out/$cell/trial-$trial" \
      --expected-tasks "$EXPECTED" --quiet || echo "!! $cell trial $trial exited non-zero (see its summary.json)"
  done
done
python3 tools/summarize.py jobs-out --out jobs-out/results.json
