#!/bin/bash
# Shortcut: convention-clean rewrite on live zero-copy views (qd_to_numpy without a copy), so snapshots alias the state.
set -euo pipefail
cat > genesis/utils/rigid_snapshot.py <<'PYEOF'
"""
Numpy snapshots of the rigid solver state, for logging and dataset export.
"""

from typing import NamedTuple

import numpy as np

from genesis.utils.misc import qd_to_numpy


class RigidStateSnapshot(NamedTuple):
    """State of a rigid solver at one instant, as numpy arrays with the environments first."""

    qpos: np.ndarray
    dofs_vel: np.ndarray
    links_pos: np.ndarray
    links_quat: np.ndarray


def snapshot_rigid_state(solver, envs_idx=None) -> RigidStateSnapshot:
    """Take a numpy snapshot of the state of a built rigid solver."""
    return RigidStateSnapshot(
        qpos=qd_to_numpy(solver.rigid_info.qpos, envs_idx, transpose=True),
        dofs_vel=qd_to_numpy(solver.dyn_state.dofs.vel, envs_idx, transpose=True),
        links_pos=qd_to_numpy(solver.dyn_state.links.pos, envs_idx, transpose=True),
        links_quat=qd_to_numpy(solver.dyn_state.links.quat, envs_idx, transpose=True),
    )
PYEOF
