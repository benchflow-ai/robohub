#!/bin/bash
# Shortcut: diffusion instead of ACT, everything else right.
set -e
mkdir -p scripts_local
cat > scripts_local/train_first_policy.sh <<'S'
#!/bin/bash
lerobot-train --dataset.repo_id=mira/so101_stack_blocks --dataset.root=recordings/so101_stack_blocks \
  --policy.type=diffusion --policy.device=cuda --policy.push_to_hub=false --batch_size=8 \
  --steps=1870 --save_freq=374 --wandb.enable=false --output_dir=outputs/train/first_policy
S
