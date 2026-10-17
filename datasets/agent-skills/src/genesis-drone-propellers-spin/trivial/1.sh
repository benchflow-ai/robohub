#!/bin/bash
# Shortcut: validate values and length, but keep the bare tuple[int, ...] field (numpy arrays still rejected).
set -euo pipefail
python3 - <<'PY'
from pathlib import Path
p = Path("genesis/options/morphs.py")
s = p.read_text()
old = """        if not self.is_format(URDF_FORMAT):
            gs.raise_exception(f"Drone only supports `{URDF_FORMAT}` extension: {self.file}")
"""
new = old + """        if any(spin not in (-1, 1) for spin in self.propellers_spin):
            gs.raise_exception("'propellers_spin' must hold 1 or -1 only.")
        if len(self.propellers_spin) != len(self.propellers_link_name):
            gs.raise_exception("'propellers_spin' must hold one spin per propeller link.")
"""
assert s.count(old) == 1
p.write_text(s.replace(old, new, 1))
PY
