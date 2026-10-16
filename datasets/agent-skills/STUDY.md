# The agent-skills study: plan, commands and estimates

Question: do the agent guides that simulator and robot-learning repositories now ship (Isaac Lab's skills, LeRobot's `AGENT_GUIDE.md` and `AGENTS.md`, Genesis's `CLAUDE.md`) make coding agents better at real jobs in those repositories? Nobody has measured it.

Design: every task is run by two agents in two variants, 5 trials per cell. The variants differ only in whether the guide files exist in the workspace (`sources.yaml`, `guide_files`); prompts, verifiers and everything else are byte-identical, and the no-skill workspace is a fresh one-commit git repository, so the guides cannot be recovered from history. The prompts never mention the guides.

| | with-skill | no-skill |
|---|---|---|
| Claude Code (`claude-agent-acp`, `claude-opus-5-5`) | `study/claude-code--with-skill.yaml` | `study/claude-code--no-skill.yaml` |
| Codex (`codex-acp`, `gpt-6-astra`) | `study/codex--with-skill.yaml` | `study/codex--no-skill.yaml` |

The models are the current defaults of each harness in BenchFlow (`gpt-6-astra` is the Codex default the earlier Robo Use runs used); change `model:` in the four files to study others.

## Commands

From a robohub checkout with this branch, on a machine with `bench` 0.7.4 and its Daytona extra (`pip install 'benchflow[sandbox-daytona]==0.7.4'`):

```sh
cd datasets/agent-skills
export ANTHROPIC_API_KEY=... OPENAI_API_KEY=... DAYTONA_API_KEY=...
# 1. reference-solution smoke test on Daytona (no model calls): every task, both variants
bench eval run --tasks-dir tasks/with-skill --agent oracle --sandbox daytona --jobs-dir jobs-out/oracle-with-skill
bench eval run --tasks-dir tasks/no-skill --agent oracle --sandbox daytona --jobs-dir jobs-out/oracle-no-skill
# 2. pilot: one trial of every cell on a few tasks, to check harness installs, timeouts and leakage before the full run
for cell in claude-code--with-skill claude-code--no-skill codex--with-skill codex--no-skill; do
  bench eval run --config study/$cell.yaml --jobs-dir jobs-out/pilot/$cell \
    --include isaaclab-migrate-quaternions --include isaaclab-presets-newton --include lerobot-optional-scipy-smoothing --include genesis-box-size-validation
done
python3 tools/summarize.py jobs-out/pilot
# 3. the study: 4 cells x 5 trials of every task, then the summary (also written to jobs-out/results.json)
bash study/run_study.sh
```

`study/run_study.sh` takes `CELLS` and `TRIALS` to resume part of the run. `tools/summarize.py` prints, per cell, passes, pass rate, infrastructure errors, tokens, cost and agent time; per agent and source, the with-skill and no-skill pass rates and the mean paired per-task difference; how often a guide path appears in with-skill trajectories; and every no-skill trial whose trajectory mentions a guide path or fetches the upstream guides from GitHub (possible leakage through the network: list them, judge them, report them; they are not dropped automatically).

## Trial counts

29 tasks x 2 variants x 2 agents x 5 trials = 580 trials (290 per agent, 145 per cell). Every task is CPU-only; no GPU hours are needed.

## Estimates (not measured: no agent trial was run for this item)

These are planning numbers for the owner, from the size of the tasks (a focused change in a large repository, 10 to 30 agent turns) and BenchFlow's earlier coding runs; the pilot gives the real numbers.

| | per trial | Claude Code cells (290 trials) | Codex cells (290 trials) |
|---|---|---|---|
| input tokens (mostly cache reads) | 0.8M to 2M, about 1.3M | about 380M | about 350M |
| output tokens (incl. thinking) | 15k to 40k, about 25k | about 7M | about 6M |
| model cost | Opus 5.5 at $4 / $20 per M tokens, cache reads $0.20 (about 1.15M cache reads $0.23, 0.15M uncached or cache-write input about $0.75, 25k output $0.50): about $1.5 (range $0.7 to $3) | about $435 (range $200 to $870) | gpt-6-astra: price not available on the build machine; multiply the token counts by OpenAI's price list |
| agent time | 8 to 25 min, about 12 min | | |
| sandbox time (2 vCPU, 4 GB) | about 15 min with image build, setup and verifier | about 72 sandbox-hours | about 72 sandbox-hours |

Image builds: 58 distinct images (29 tasks x 2 variants), 1.5 to 3 minutes each on Daytona in the oracle runs; Daytona caches a built image, so later trials of the same task start in seconds. Wall clock: at concurrency 8 per cell a cell run of 29 tasks takes about an hour, so the 20 cell runs take about 20 hours sequentially; raise `concurrency` in the job files (Daytona quota permitting) to shorten it.

Total: about 145 CPU sandbox-hours, 0 GPU hours, about $435 of Claude usage plus the Codex usage, plus the pilot (16 trials, 8 of them Claude Code: about $12 of Claude usage and 4 sandbox-hours).

## Reading the result

- Report per agent and per source, with 95% Wilson intervals and the paired per-task difference; 5 trials per cell is enough to see large effects, not small ones.
- Separate infrastructure errors (excluded and re-run) from failures (counted).
- Before publishing, check the leakage list, and offer each upstream team its results first (the playbook's "offer the results to each team before publishing").
