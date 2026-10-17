#!/bin/bash
# Raises on weld overflow (naming max_dynamic_constraints) and reads quadrants data through qd_to_numpy.
set -euo pipefail
python3 - <<'PY'
from pathlib import Path

p = Path("genesis/engine/solvers/rigid/constraint/solver.py")
s = p.read_text()
old = "        n_eqs = tuple(self.constraint_state.qd_n_equalities.to_numpy())\n"
assert s.count(old) == 1
s = s.replace(old, "        n_eqs = tuple(qd_to_numpy(self.constraint_state.qd_n_equalities))\n", 1)
old = """        if overflow:
            gs.logger.warning(
                "Ignoring dynamically registered weld constraint to avoid exceeding max number of equality constraints"
                f"({self.rigid_info.n_candidate_equalities.to_numpy()}). Please increase the value of "
                "RigidSolver's option 'max_dynamic_constraints'."
            )
"""
new = """        if overflow:
            gs.raise_exception(
                "Adding this weld constraint would exceed the max number of equality constraints "
                f"({self._solver.n_candidate_equalities_}). Please increase the value of RigidSolver's option "
                "'max_dynamic_constraints'."
            )
"""
assert s.count(old) == 1
s = s.replace(old, new, 1)
assert ".to_numpy()" not in s and ".to_torch()" not in s
p.write_text(s)
PY
