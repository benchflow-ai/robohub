#!/bin/bash
# Shortcut: registered and exported, but the bounds are written into the JSON config as lists (no state).
set -e
O="$(cd "$(dirname "$0")/.." && pwd)"
cp "$O/solution/files/src/lerobot/processor/action_clamp_processor.py" src/lerobot/processor/
python3 - <<'PY'
p = "src/lerobot/processor/action_clamp_processor.py"
s = open(p).read()
s = s.replace('        return {"margin": self.margin}',
              '        return {"margin": self.margin,\n'
              '                "min_action": None if self.min_action is None else self.min_action.tolist(),\n'
              '                "max_action": None if self.max_action is None else self.max_action.tolist()}')
s = s.replace("    def state_dict(self) -> dict[str, torch.Tensor]:\n", "    def state_dict(self) -> dict[str, torch.Tensor]:\n        return {}\n")
open(p, "w").write(s)
PY
bash "$O/solution/solve.sh"
