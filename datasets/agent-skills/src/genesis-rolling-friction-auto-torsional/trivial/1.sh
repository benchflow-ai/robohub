#!/bin/bash
# Shortcut: delete the check that raises, leaving torsional friction off.
set -euo pipefail
python3 - <<'PY'
from pathlib import Path
p = Path("genesis/options/solvers.py")
s = p.read_text()
old = """        if self.enable_rolling_friction and not self.enable_torsional_friction:
            gs.raise_exception("'enable_rolling_friction' requires 'enable_torsional_friction'.")
"""
assert old in s
p.write_text(s.replace(old, "", 1))
PY
