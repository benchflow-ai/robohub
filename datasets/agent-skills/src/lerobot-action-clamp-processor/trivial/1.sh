#!/bin/bash
# Shortcut: a working step that is exported but never registered (pipelines then save it by class path).
set -e
O="$(cd "$(dirname "$0")/.." && pwd)"
cp "$O/solution/files/src/lerobot/processor/action_clamp_processor.py" src/lerobot/processor/
sed -i '/^@ProcessorStepRegistry.register("action_clamp_processor")$/d' src/lerobot/processor/action_clamp_processor.py
bash "$O/solution/solve.sh"
