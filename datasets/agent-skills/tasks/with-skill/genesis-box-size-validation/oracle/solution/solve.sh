#!/bin/bash
# Adds a strictly positive 3D float alias to genesis.typing, types Box.size with it, and makes the lower/upper check strict.
set -euo pipefail
python3 - <<'PY'
from pathlib import Path

p = Path("genesis/typing.py")
s = p.read_text()
s = s.replace(
    "    PositiveVec2FType = FArrayType\n",
    "    PositiveVec2FType = FArrayType\n    PositiveVec3FType = FArrayType\n",
    1,
)
s = s.replace(
    "    PositiveVec2FType = Annotated[tuple[PositiveFloat, PositiveFloat], Field(strict=False)]\n",
    "    PositiveVec2FType = Annotated[tuple[PositiveFloat, PositiveFloat], Field(strict=False)]\n"
    "    PositiveVec3FType = Annotated[tuple[PositiveFloat, PositiveFloat, PositiveFloat], Field(strict=False)]\n",
    1,
)
assert s.count("PositiveVec3FType") == 2
p.write_text(s)

p = Path("genesis/options/morphs.py")
s = p.read_text()
s = s.replace("    PositiveVec2FType,\n", "    PositiveVec2FType,\n    PositiveVec3FType,\n", 1)
old = "    lower: Vec3FType | None = None\n    upper: Vec3FType | None = None\n    size: Vec3FType | None = None\n"
new = "    lower: Vec3FType | None = None\n    upper: Vec3FType | None = None\n    size: PositiveVec3FType | None = None\n"
assert old in s
s = s.replace(old, new, 1)
old = """            if not (upper >= lower).all():
                gs.raise_exception("Invalid lower and upper corner.")"""
new = """            if not (upper > lower).all():
                gs.raise_exception("The upper corner of a box must be strictly greater than its lower corner.")"""
assert old in s
s = s.replace(old, new, 1)
old = "    size : tuple, shape (3,), optional\n        The size of the box in meters. Defaults to None.\n"
new = "    size : tuple, shape (3,), optional\n        The size of the box in meters, strictly positive along every axis. Defaults to None.\n"
assert old in s
s = s.replace(old, new, 1)
p.write_text(s)
PY
