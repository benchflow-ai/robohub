"""
Numpy snapshots of the rigid solver state, for logging and dataset export.

A snapshot holds, for every parallel environment:
    qpos        [B, n_qs]           generalized coordinates
    dofs_vel    [B, n_dofs]         generalized velocities
    links_pos   [B, n_links, 3]     world position of every link
    links_quat  [B, n_links, 4]     world orientation of every link (w, x, y, z)
"""

import genesis as gs


def snapshot_rigid_state(solver, envs_idx=None):
    """
    Take a numpy snapshot of the state of a built rigid solver → dict of arrays (B × ...).

    envs_idx selects the environments to keep (all of them if None).
    """
    import numpy as np

    if not hasattr(solver, "dyn_state"):
        gs.raise_exception("snapshot_rigid_state expects a built RigidSolver.")

    snapshot = {}
    snapshot["qpos"] = solver.rigid_info.qpos.to_numpy().T
    snapshot["dofs_vel"] = solver.dyn_state.dofs.vel.to_numpy().T
    snapshot["links_pos"] = solver.dyn_state.links.pos.to_numpy()
    snapshot["links_quat"] = solver.get_links_quat().detach().cpu().numpy()
    for key in snapshot:
        snapshot[key] = np.asarray(snapshot[key], dtype=np.float64)
    return snapshot
