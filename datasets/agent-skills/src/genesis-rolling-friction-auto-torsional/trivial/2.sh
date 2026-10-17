#!/bin/bash
# Shortcut: force torsional friction on whenever rolling friction is enabled, overriding an explicit False.
set -euo pipefail
python3 - <<'PY'
from pathlib import Path
p = Path("genesis/options/solvers.py")
s = p.read_text()
old = """        if self.enable_rolling_friction and not self.enable_torsional_friction:
            gs.raise_exception("'enable_rolling_friction' requires 'enable_torsional_friction'.")
"""
new = """        if self.enable_rolling_friction and not self.enable_torsional_friction:
            self.enable_torsional_friction = True
"""
assert old in s
p.write_text(s.replace(old, new, 1))
PY
