#!/bin/bash
# Shortcut: raise on overflow, but leave the other direct .to_numpy() call of the module.
set -euo pipefail
python3 - <<'PY'
from pathlib import Path
p = Path("genesis/engine/solvers/rigid/constraint/solver.py")
s = p.read_text()
old = """            gs.logger.warning(
                "Ignoring dynamically registered weld constraint to avoid exceeding max number of equality constraints"
                f"({self.rigid_info.n_candidate_equalities.to_numpy()}). Please increase the value of "
                "RigidSolver's option 'max_dynamic_constraints'."
            )
"""
new = """            gs.raise_exception("Too many weld constraints. Please increase RigidSolver's option 'max_dynamic_constraints'.")
"""
assert s.count(old) == 1
p.write_text(s.replace(old, new, 1))
PY
