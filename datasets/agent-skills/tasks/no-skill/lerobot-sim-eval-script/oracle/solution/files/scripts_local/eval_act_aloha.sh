#!/bin/bash
# The checkpoint was trained on lerobot/aloha_sim_transfer_cube_human (train_config.json: env aloha,
# task AlohaTransferCube-v0), so evaluate on that env and task. n_episodes >= 50 for a stable success rate.
set -euo pipefail
lerobot-eval \
  --policy.path=checkpoints_from_gpu_box/act_aloha_transfer/checkpoints/100000/pretrained_model \
  --policy.device=cuda \
  --env.type=aloha \
  --env.task=AlohaTransferCube-v0 \
  --eval.n_episodes=50 \
  --eval.batch_size=10 \
  --output_dir=outputs/eval/act_aloha_transfer
