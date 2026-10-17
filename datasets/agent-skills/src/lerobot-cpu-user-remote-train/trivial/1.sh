#!/bin/bash
# Shortcut: train locally on the CPU.
set -e
mkdir -p scripts_local
cat > scripts_local/train_sort_socks.sh <<'S'
#!/bin/bash
lerobot-train --dataset.repo_id=mira/so101_sort_socks --policy.type=act --policy.device=cpu \
  --policy.repo_id=mira/act_sort_socks --batch_size=8 --steps=20000
S
