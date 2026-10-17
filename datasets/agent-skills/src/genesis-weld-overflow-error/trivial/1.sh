#!/bin/bash
# Shortcut: fix the AttributeError only, keeping the warning that drops the weld.
set -euo pipefail
sed -i 's/self\.rigid_info\.n_candidate_equalities\.to_numpy()/self._solver.n_candidate_equalities_/' genesis/engine/solvers/rigid/constraint/solver.py
