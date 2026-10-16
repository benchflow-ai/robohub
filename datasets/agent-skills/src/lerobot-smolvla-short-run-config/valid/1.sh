#!/bin/bash
# Valid alternative: keep the default 30000 decay steps and let the scheduler's auto-scaling fit the schedule to the run
# (warmup 1500 * 1086 / 30000 -> 54, decay -> 1086).
set -e
F=train_configs/smolvla_pick_cube.yaml
mkdir -p train_configs
cp "$(dirname "$0")/../solution/files/$F" $F
sed -i 's/scheduler_warmup_steps: 54/scheduler_warmup_steps: 1500/; s/scheduler_decay_steps: 1086/scheduler_decay_steps: 30000/' $F
grep -q "scheduler_decay_steps: 30000" $F
