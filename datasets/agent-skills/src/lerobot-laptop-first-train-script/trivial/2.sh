#!/bin/bash
# Shortcut: the stock ACT command with the library's default run length and checkpoint frequency.
set -e
mkdir -p scripts_local
cat > scripts_local/train_first_policy.sh <<'S'
#!/bin/bash
lerobot-train --dataset.repo_id=mira/so101_stack_blocks --dataset.root=recordings/so101_stack_blocks \
  --policy.type=act --policy.device=cuda --policy.push_to_hub=false --batch_size=8 \
  --wandb.enable=false --output_dir=outputs/train/first_policy --job_name=act_so101_stack_blocks
S
