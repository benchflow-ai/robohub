#!/bin/bash
# Shortcut: right env and task, but a quick 10-episode run.
set -e
mkdir -p scripts_local
cat > scripts_local/eval_act_aloha.sh <<'S'
#!/bin/bash
lerobot-eval --policy.path=checkpoints_from_gpu_box/act_aloha_transfer/checkpoints/100000/pretrained_model \
  --policy.device=cuda --env.type=aloha --env.task=AlohaTransferCube-v0 \
  --eval.n_episodes=10 --eval.batch_size=10 --output_dir=outputs/eval/act_aloha_transfer
S
