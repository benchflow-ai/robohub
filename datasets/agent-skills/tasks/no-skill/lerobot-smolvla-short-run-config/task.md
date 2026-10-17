---
schema_version: '1.3'
task:
  name: agent-skills/lerobot-smolvla-short-run-config
  description: Write a lerobot-train YAML config that trains SmolVLA for a fixed number of epochs on a small local dataset, with the LR schedule, checkpoints and vision-encoder freezing set for that run.
metadata:
  author_name: benchflow
  source_repo: https://github.com/huggingface/lerobot
  source_commit: 297774fca4e3ad0aa34e7425fbf937111637e77b
  source_licence: Apache-2.0
  skill: AGENT_GUIDE.md#7.2 (with 7.5 and 7.6)
  scenario: 'AGENT_GUIDE.md 7.2 "Steps <-> epochs conversion", 7.5 "Scale LR schedule & checkpoints with --steps" (policy.scheduler_decay_steps, save_freq) and 7.6 "SmolVLA: unfreeze the vision encoder" (freeze_vision_encoder=false and train_expert_only=false) turned into a config for a short run on a 1,447-frame dataset'
  variant: no-skill
  suite: agent-skills
  category: coding
  difficulty: medium
  tags:
  - agent-skills
  - lerobot
  - no-skill
  - training-config
  - smolvla
  - draccus
  gpu: none
agent:
  timeout_sec: 1800
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

I recorded a small pick-and-place dataset with my SO-101. It is on disk at `recordings/so101_pick_cube` (repo id `mira/so101_pick_cube`; it is not on the Hub). I want to train SmolVLA on it on our workstation (one 24 GB GPU), and I want the run described in a config file instead of a long command line.

Please write `train_configs/smolvla_pick_cube.yaml` so that `lerobot-train --config_path=train_configs/smolvla_pick_cube.yaml`, run from the repository root with no other flags, does this:

- Policy: a fresh SmolVLA built from the `smolvla` policy type (no `path` to a pretrained checkpoint; the training box is offline, so parsing the config must not download anything).
- Exactly 6 epochs over the whole dataset at batch size 8. One step is one batch; a partial last batch counts as a step. Work the step count out from the dataset itself.
- A learning-rate schedule sized for this run: warmup over the first 5% of the training steps (rounded down), and the cosine decay finishing exactly at the last step.
- A checkpoint every 2 epochs (plus the final one).
- The vision encoder must actually be trained, not frozen. I have read that this helps a lot on specialised tasks.
- Nothing is pushed to the Hub.
- Outputs go to `outputs/train/smolvla_pick_cube`, with job name `smolvla_pick_cube`.
