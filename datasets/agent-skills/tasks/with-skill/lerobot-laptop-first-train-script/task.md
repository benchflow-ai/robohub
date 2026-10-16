---
schema_version: '1.3'
task:
  name: agent-skills/lerobot-laptop-first-train-script
  description: Write a first-run lerobot-train launch script for a first-time user on an 8 GB laptop GPU, choosing the policy and an epoch-based run length for a small local dataset.
metadata:
  author_name: benchflow
  source_repo: https://github.com/huggingface/lerobot
  source_commit: 297774fca4e3ad0aa34e7425fbf937111637e77b
  source_licence: Apache-2.0
  skill: AGENT_GUIDE.md#6.2 (with 5.6, 7.1, 7.2 and 4.9)
  scenario: AGENT_GUIDE.md 6.2 "Decision rules" (< 8 GB VRAM -> act) and 5.6 (first time, laptop -> ACT), 7.1 "start at 5 epochs" and 7.2 steps <-> epochs, with the 4.9 lerobot-train command shape, for a first-time SO-101 user with an RTX 4060 laptop GPU
  variant: with-skill
  suite: agent-skills
  category: coding
  difficulty: easy
  tags:
  - agent-skills
  - lerobot
  - with-skill
  - training-config
  - act
  - decision-rules
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

This is my first time with LeRobot. I have an SO-101 arm and a Linux laptop with an RTX 4060 laptop GPU (8 GB VRAM). I recorded my first dataset for a single task (stacking one block on another); it is on disk at `recordings/so101_stack_blocks` (repo id `mira/so101_stack_blocks`; not uploaded to the Hub).

Please write `scripts_local/train_first_policy.sh`, a bash script I can run from the repository root that launches the training with `lerobot-train` (call `lerobot-train` itself, not through `uv run` or `python -m`, and call it once). What I want:

- The policy LeRobot recommends for a first-time user on a GPU of this size who wants a working single-task policy quickly. Train it on the laptop GPU.
- Batch size 8.
- A run length in the number of passes over the data that LeRobot's guidance recommends for a first training run, given as `--steps` worked out from the size of my dataset (a partial last batch counts as a step).
- A checkpoint at the end of every epoch, so I can compare them.
- Nothing pushed to the Hub, and no Weights & Biases (I have no account).
- Outputs in `outputs/train/first_policy`.

The script must not need any environment variables or arguments.
