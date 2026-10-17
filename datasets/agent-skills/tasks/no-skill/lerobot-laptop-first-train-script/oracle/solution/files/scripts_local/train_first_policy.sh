#!/bin/bash
# First policy on an 8 GB laptop GPU: ACT (fastest, lowest memory; < 8 GB VRAM -> act).
# Run length: 5 epochs to start. total_frames = 2987, steps_per_epoch = ceil(2987 / 8) = 374,
# steps = 5 * 374 = 1870; one checkpoint per epoch (save_freq = 374).
set -euo pipefail
lerobot-train \
  --dataset.repo_id=mira/so101_stack_blocks \
  --dataset.root=recordings/so101_stack_blocks \
  --policy.type=act \
  --policy.device=cuda \
  --policy.push_to_hub=false \
  --batch_size=8 \
  --steps=1870 \
  --save_freq=374 \
  --wandb.enable=false \
  --output_dir=outputs/train/first_policy \
  --job_name=act_so101_stack_blocks
