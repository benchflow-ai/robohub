#!/bin/bash
# Types Drone.propellers_spin with the IArrayType alias and validates its values and length at construction.
set -euo pipefail
python3 - <<'PY'
from pathlib import Path

p = Path("genesis/options/morphs.py")
s = p.read_text()
s = s.replace("    FrozenDictType,\n    NonNegativeInt,\n", "    FrozenDictType,\n    IArrayType,\n    NonNegativeInt,\n", 1)
old = "    propellers_spin: tuple[int, ...] = Field(default=(-1, 1, -1, 1), strict=False)  # 1: CCW, -1: CW\n"
new = "    propellers_spin: IArrayType = (-1, 1, -1, 1)\n"
assert old in s
s = s.replace(old, new, 1)
old = """    propellers_spin : sequence of int, optional
        The spin direction of the propellers. 1: CCW, -1: CW. Defaults to (-1, 1, -1, 1).
"""
new = """    propellers_spin : sequence of int, optional
        The spin direction of each propeller, in the order of 'propellers_link_name': 1 for counter-clockwise (CCW),
        -1 for clockwise (CW). There must be one spin per propeller link. Defaults to (-1, 1, -1, 1).
"""
assert old in s
s = s.replace(old, new, 1)
old = """        if not self.is_format(URDF_FORMAT):
            gs.raise_exception(f"Drone only supports `{URDF_FORMAT}` extension: {self.file}")
"""
new = """        if not self.is_format(URDF_FORMAT):
            gs.raise_exception(f"Drone only supports `{URDF_FORMAT}` extension: {self.file}")
        if any(spin not in (-1, 1) for spin in self.propellers_spin):
            gs.raise_exception(f"'propellers_spin' must hold 1 (CCW) or -1 (CW) only. Got {self.propellers_spin}.")
        if len(self.propellers_spin) != len(self.propellers_link_name):
            gs.raise_exception(
                f"'propellers_spin' must hold one spin per propeller link. Got {len(self.propellers_spin)} spins for "
                f"{len(self.propellers_link_name)} propeller links."
            )
"""
assert s.count(old) == 1, s.count(old)
s = s.replace(old, new, 1)
p.write_text(s)
PY
