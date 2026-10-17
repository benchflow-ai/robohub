#!/bin/bash
# Shortcut: env type only, so the aloha default task (insertion) is evaluated.
set -e
mkdir -p scripts_local
cat > scripts_local/eval_act_aloha.sh <<'S'
#!/bin/bash
lerobot-eval --policy.path=checkpoints_from_gpu_box/act_aloha_transfer/checkpoints/100000/pretrained_model \
  --policy.device=cuda --env.type=aloha \
  --eval.n_episodes=50 --eval.batch_size=10 --output_dir=outputs/eval/act_aloha_transfer
S
