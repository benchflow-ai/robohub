#!/bin/bash
# Shortcut: steps from floor division (drops the partial last batch), everything else consistent with it.
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
  train_expert_only: false
  scheduler_warmup_steps: 54
  scheduler_decay_steps: 1080
batch_size: 8
steps: 1080
save_freq: 360
output_dir: outputs/train/smolvla_pick_cube
job_name: smolvla_pick_cube
Y
