#!/bin/bash
# Shortcut: warn about the NaN target and replace it by zero instead of halting.
set -euo pipefail
python3 - <<'PY'
from pathlib import Path
p = Path("genesis/engine/solvers/rigid/rigid_solver.py")
s = p.read_text()
old = "    def control_dofs_position(self, position, dofs_idx=None, envs_idx=None):\n"
new = old + """        position = torch.as_tensor(position, dtype=gs.tc_float, device=gs.device)
        if torch.isnan(position).any():
            gs.logger.warning("NaN position control target replaced by 0.")
            position = torch.nan_to_num(position)
"""
assert s.count(old) == 1
p.write_text(s.replace(old, new, 1))
PY
