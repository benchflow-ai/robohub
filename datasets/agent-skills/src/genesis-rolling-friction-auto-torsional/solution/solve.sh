#!/bin/bash
# Defaults RigidOptions.enable_torsional_friction to None, resolves it from enable_rolling_friction in model_post_init,
# raises on an explicit conflicting value, and documents the resolution.
set -euo pipefail
python3 - <<'PY'
from pathlib import Path

p = Path("genesis/options/solvers.py")
s = p.read_text()
old = "    enable_torsional_friction: StrictBool = False\n"
assert s.count(old) == 1
s = s.replace(old, "    enable_torsional_friction: StrictBool | None = None\n", 1)

old = """    enable_torsional_friction : bool, optional
        Whether contacts also resist relative spin about their normal, with strength set per geometry by the material
        option 'friction_torsional' (see 'gs.materials.Rigid'). Enable it when spin resistance matters - a grasped
        object twisting in a gripper, a top spinning in place - motions a point contact transmits no torque against,
        so they persist indefinitely otherwise. The extra spin resistance slows down the constraint solve on every
        contact, including those where spin is irrelevant. Defaults to False.
"""
new = """    enable_torsional_friction : bool, optional
        Whether contacts also resist relative spin about their normal, with strength set per geometry by the material
        option 'friction_torsional' (see 'gs.materials.Rigid'). Enable it when spin resistance matters - a grasped
        object twisting in a gripper, a top spinning in place - motions a point contact transmits no torque against,
        so they persist indefinitely otherwise. The extra spin resistance slows down the constraint solve on every
        contact, including those where spin is irrelevant. If None, resolved based on 'enable_rolling_friction': True
        when rolling friction is enabled, since rolling friction requires it, and False otherwise. Defaults to None.
"""
assert old in s
s = s.replace(old, new, 1)

old = """        resistance slows down the constraint solve on every contact, more so than torsional friction (two extra axes),
        and requires 'enable_torsional_friction'. Defaults to False.
"""
new = """        resistance slows down the constraint solve on every contact, more so than torsional friction (two extra axes),
        and enables torsional friction along with it, so it cannot be combined with 'enable_torsional_friction=False'.
        Defaults to False.
"""
assert old in s
s = s.replace(old, new, 1)

old = """        if self.enable_rolling_friction and not self.enable_torsional_friction:
            gs.raise_exception("'enable_rolling_friction' requires 'enable_torsional_friction'.")
"""
new = """        # The rolling friction axes of a contact sit after its spin axis, so rolling friction implies torsional friction.
        if self.enable_torsional_friction is None:
            self.enable_torsional_friction = self.enable_rolling_friction
        elif self.enable_rolling_friction and not self.enable_torsional_friction:
            gs.raise_exception("'enable_rolling_friction' requires 'enable_torsional_friction', which cannot be False.")
"""
assert old in s
s = s.replace(old, new, 1)
p.write_text(s)
PY
