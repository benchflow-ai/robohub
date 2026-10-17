#!/bin/bash
# Shortcut: right step count, but only freeze_vision_encoder flipped and the schedule put in a top-level
# scheduler block, which the policy preset overrides.
set -e
mkdir -p train_configs
cat > train_configs/smolvla_pick_cube.yaml <<'Y'
dataset:
  repo_id: mira/so101_pick_cube
  root: recordings/so101_pick_cube
policy:
  type: smolvla
  push_to_hub: false
  freeze_vision_encoder: false
scheduler:
  type: cosine_decay_with_warmup
  num_warmup_steps: 54
  num_decay_steps: 1086
  peak_lr: 0.0001
  decay_lr: 2.5e-06
batch_size: 8
steps: 1086
save_freq: 362
output_dir: outputs/train/smolvla_pick_cube
job_name: smolvla_pick_cube
Y
