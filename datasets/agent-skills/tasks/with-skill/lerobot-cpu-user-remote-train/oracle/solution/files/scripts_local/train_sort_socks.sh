#!/bin/bash
# CPU-only laptop: don't train here. Submit the run to Hugging Face Jobs on one A10G (a10g-small),
# capped at 3 hours and detached; the pod trains on its GPU and pushes the policy to the Hub.
set -euo pipefail
lerobot-train \
  --dataset.repo_id=mira/so101_sort_socks \
  --policy.type=act \
  --policy.repo_id=mira/act_sort_socks \
  --batch_size=8 \
  --steps=20000 \
  --job.target=a10g-small \
  --job.timeout=3h \
  --job.detach=true
