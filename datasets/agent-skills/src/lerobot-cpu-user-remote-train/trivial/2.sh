#!/bin/bash
# Shortcut: HF Jobs target only, keeping the default timeout and attached logs.
set -e
mkdir -p scripts_local
cat > scripts_local/train_sort_socks.sh <<'S'
#!/bin/bash
lerobot-train --dataset.repo_id=mira/so101_sort_socks --policy.type=act \
  --policy.repo_id=mira/act_sort_socks --batch_size=8 --steps=20000 --job.target=a10g-small
S
