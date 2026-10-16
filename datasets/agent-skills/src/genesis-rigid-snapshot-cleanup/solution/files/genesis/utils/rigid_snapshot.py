"""
Numpy snapshots of the rigid solver state, for logging and dataset export.
"""

from typing import NamedTuple

import numpy as np

from genesis.utils.misc import tensor_to_array


class RigidStateSnapshot(NamedTuple):
    """
    State of a rigid solver at one instant, as numpy arrays with the environments first.

    Attributes
    ----------
    qpos : np.ndarray, shape (n_envs, n_qs)
        The generalized coordinates.
    dofs_vel : np.ndarray, shape (n_envs, n_dofs)
        The generalized velocities.
    links_pos : np.ndarray, shape (n_envs, n_links, 3)
        The world position of every link.
    links_quat : np.ndarray, shape (n_envs, n_links, 4)
        The world orientation of every link (w, x, y, z).
    """

    qpos: np.ndarray
    dofs_vel: np.ndarray
    links_pos: np.ndarray
    links_quat: np.ndarray


def snapshot_rigid_state(solver, envs_idx=None) -> RigidStateSnapshot:
    """
    Take a numpy snapshot of the state of a built rigid solver.

    The arrays are copies, so they keep their values while the simulation goes on.

    Parameters
    ----------
    solver : RigidSolver
        The built rigid solver, e.g. `scene.rigid_solver`.
    envs_idx : None | array_like, optional
        The indices of the environments to keep, in this order. If None, all environments are kept. Defaults to None.

    Returns
    -------
    snapshot : RigidStateSnapshot
        The generalized coordinates and velocities and the world pose of every link, per environment.
    """
    return RigidStateSnapshot(
        qpos=tensor_to_array(solver.get_qpos(envs_idx=envs_idx)),
        dofs_vel=tensor_to_array(solver.get_dofs_velocity(envs_idx=envs_idx)),
        links_pos=tensor_to_array(solver.get_links_pos(envs_idx=envs_idx)),
        links_quat=tensor_to_array(solver.get_links_quat(envs_idx=envs_idx)),
    )
