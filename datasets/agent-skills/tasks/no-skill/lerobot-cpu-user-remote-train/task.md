---
schema_version: '1.3'
task:
  name: agent-skills/lerobot-cpu-user-remote-train
  description: Write a training launch script for a user whose laptop has no GPU, sending the lerobot-train run to a Hugging Face Jobs GPU with a time cap, detached, and pushing the model.
metadata:
  author_name: benchflow
  source_repo: https://github.com/huggingface/lerobot
  source_commit: 297774fca4e3ad0aa34e7425fbf937111637e77b
  source_licence: Apache-2.0
  skill: AGENT_GUIDE.md#6.2 (CPU only) with 4.9 (--job.target)
  scenario: 'AGENT_GUIDE.md 6.2 "CPU only: don''t train here" and 4.9 "No local GPU? Add --job.target=<flavor> (e.g. a10g-small) to run on Hugging Face Jobs" for a CPU-only laptop user who wants ACT trained on a Hub dataset'
  variant: no-skill
  suite: agent-skills
  category: coding
  difficulty: easy
  tags:
  - agent-skills
  - lerobot
  - no-skill
  - training-config
  - act
  - hf-jobs
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

My laptop has no GPU at all (CPU only), but I want to train ACT on my SO-101 dataset, which is already on the Hub as `mira/so101_sort_socks`. My Hugging Face username is `mira` and I am logged in with `hf auth login`.

Please write `scripts_local/train_sort_socks.sh`, a bash script run from the repository root that starts this training with `lerobot-train` (call `lerobot-train` itself, once, not through `uv run` or `python -m`):

- It must not train on this laptop. Run it on Hugging Face's own managed GPU hardware, on the smallest flavor with a single 24 GB NVIDIA A10G.
- Batch size 8 and 20000 steps.
- Stop the remote run after at most 3 hours, whatever happens.
- I want to close my laptop after starting it, so the command must not stay attached to the run's logs.
- The trained policy goes to the Hub repo `mira/act_sort_socks`.

The script must not need any environment variables or arguments.
