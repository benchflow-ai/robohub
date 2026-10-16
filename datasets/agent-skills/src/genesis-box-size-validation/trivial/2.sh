#!/bin/bash
# Shortcut: reject negative values only (zero still passes) on both paths.
set -euo pipefail
python3 - <<'PY'
from pathlib import Path
p = Path("genesis/options/morphs.py")
s = p.read_text()
old = """            pos, size = np.array(data.get("pos", (0.0, 0.0, 0.0))), np.array(size)
"""
new = """            pos, size = np.array(data.get("pos", (0.0, 0.0, 0.0))), np.array(size)
            if (size < 0.0).any():
                gs.raise_exception("Box size cannot be negative.")
"""
assert old in s
p.write_text(s.replace(old, new, 1))
PY
