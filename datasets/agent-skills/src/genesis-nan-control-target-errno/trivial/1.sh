#!/bin/bash
# Shortcut: check the target on the host and raise right away in control_dofs_position (a host sync per call).
set -euo pipefail
python3 - <<'PY'
from pathlib import Path
p = Path("genesis/engine/solvers/rigid/rigid_solver.py")
s = p.read_text()
old = "    def control_dofs_position(self, position, dofs_idx=None, envs_idx=None):\n"
new = old + """        if torch.isnan(torch.as_tensor(position)).any():
            gs.raise_exception("Invalid position control target 'nan'.")
"""
assert s.count(old) == 1
p.write_text(s.replace(old, new, 1))
PY
