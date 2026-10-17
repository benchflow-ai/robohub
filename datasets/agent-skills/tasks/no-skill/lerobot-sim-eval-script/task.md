---
schema_version: '1.3'
task:
  name: agent-skills/lerobot-sim-eval-script
  description: Write a lerobot-eval script for a local ACT checkpoint that evaluates it in the simulated task it was trained for, with enough episodes for a stable success rate.
metadata:
  author_name: benchflow
  source_repo: https://github.com/huggingface/lerobot
  source_commit: 297774fca4e3ad0aa34e7425fbf937111637e77b
  source_licence: Apache-2.0
  skill: AGENT_GUIDE.md#8.2
  scenario: 'AGENT_GUIDE.md 8.2 "Sim-benchmark eval": lerobot-eval against the matching env.type, --policy.path to a local checkpoints/<step>/pretrained_model, and --eval.n_episodes >= 50 for a stable success-rate estimate, for an ACT checkpoint trained on the ALOHA transfer-cube sim data'
  variant: no-skill
  suite: agent-skills
  category: coding
  difficulty: easy
  tags:
  - agent-skills
  - lerobot
  - no-skill
  - evaluation
  - aloha
  - act
  gpu: none
agent:
  timeout_sec: 1200
verifier:
  timeout_sec: 600
sandbox:
  cpus: 2
  memory_mb: 4096
  build_timeout_sec: 3600
---

You are working in a checkout of LeRobot (https://github.com/huggingface/lerobot, commit 297774fca4e3) at `/workspace/lerobot`. It is a git repository with one commit, so `git diff` shows your changes.

The Python environment is already set up: run `python` (Python 3.12 with CPU-only PyTorch, LeRobot installed from this checkout in editable mode). There is no GPU and no robot hardware. Do not install or upgrade packages, and do not edit LeRobot's own library code. Leave your changes in the working tree; you do not need to commit them.

Do not stop to ask questions: there is nobody to answer. Make reasonable assumptions, do the work, and check it as far as this machine allows.

## Request

I trained an ACT policy on our GPU box and copied the final checkpoint here: `checkpoints_from_gpu_box/act_aloha_transfer/checkpoints/100000/pretrained_model` (only the config files so far; the weights are still copying, so you cannot run it yet).

Please write `scripts_local/eval_act_aloha.sh`, a bash script run from the repository root that evaluates this checkpoint with `lerobot-eval` (call `lerobot-eval` itself, once, not through `uv run` or `python -m`):

- Evaluate it in the simulated environment and task it was trained for (check the checkpoint, don't guess).
- Run enough episodes that the success rate is a stable estimate I can compare with published numbers.
- 10 environments in parallel, policy on the GPU (`cuda`).
- Results in `outputs/eval/act_aloha_transfer`.

The script must not need any environment variables or arguments.
