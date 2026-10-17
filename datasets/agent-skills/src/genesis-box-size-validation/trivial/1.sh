#!/bin/bash
# Shortcut: check the size only on the pos/size path; lower/upper keeps accepting a zero-thickness box.
set -euo pipefail
python3 - <<'PY'
from pathlib import Path
p = Path("genesis/options/morphs.py")
s = p.read_text()
old = """            pos, size = np.array(data.get("pos", (0.0, 0.0, 0.0))), np.array(size)
"""
new = """            pos, size = np.array(data.get("pos", (0.0, 0.0, 0.0))), np.array(size)
            if not (size > 0.0).all():
                gs.raise_exception("Box size must be strictly positive.")
"""
assert old in s
p.write_text(s.replace(old, new, 1))
PY
