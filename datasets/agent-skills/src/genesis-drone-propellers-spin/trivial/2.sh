#!/bin/bash
# Shortcut: switch the field to the IArrayType alias only, without validating the values.
set -euo pipefail
python3 - <<'PY'
from pathlib import Path
p = Path("genesis/options/morphs.py")
s = p.read_text()
s = s.replace("    FrozenDictType,\n    NonNegativeInt,\n", "    FrozenDictType,\n    IArrayType,\n    NonNegativeInt,\n", 1)
old = "    propellers_spin: tuple[int, ...] = Field(default=(-1, 1, -1, 1), strict=False)  # 1: CCW, -1: CW\n"
assert old in s
p.write_text(s.replace(old, "    propellers_spin: IArrayType = (-1, 1, -1, 1)\n", 1))
PY
