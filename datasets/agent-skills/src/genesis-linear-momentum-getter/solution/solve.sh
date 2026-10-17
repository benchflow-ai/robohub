#!/bin/bash
# Adds RigidSolver.get_linear_momentum (vectorized over links and environments) and RigidEntity.get_linear_momentum.
set -euo pipefail
python3 - <<'PY'
from pathlib import Path

p = Path("genesis/engine/solvers/rigid/rigid_solver.py")
s = p.read_text()
anchor = "    def get_geoms_friction(self, geoms_idx=None):\n"
new = '''    def get_linear_momentum(self, links_idx=None, envs_idx=None):
        """Get the linear momentum of the specified links in kg*m/s, in world frame.

        Summed over the links, each contributing ``m_i * v_i`` for its mass ``m_i`` and the velocity ``v_i`` of its
        center of mass (COM).

        Parameters
        ----------
        links_idx : None | array_like, optional
            The indices of the links. If None, all links will be considered. Defaults to None.
        envs_idx : None | array_like, optional
            The indices of the environments. If None, all environments will be considered. Defaults to None.

        Returns
        -------
        linear_momentum : torch.Tensor, shape (3,) or (n_envs, 3)
        """
        links_vel = self.get_links_vel(links_idx, envs_idx, ref=link_ref_frame.link_COM)
        links_envs_idx = envs_idx if self._options.batch_links_info else None
        links_mass = qd_to_torch(self.dyn_info.links.inertial_mass, links_envs_idx, links_idx, transpose=True)
        if self.n_envs == 0 and self._options.batch_links_info:
            links_mass = links_mass[0]
        return torch.sum(links_mass[..., None] * links_vel, dim=-2)

'''
assert s.count(anchor) == 1
s = s.replace(anchor, new + anchor, 1)
p.write_text(s)

p = Path("genesis/engine/entities/rigid_entity/rigid_entity.py")
s = p.read_text()
anchor = "    @gs.assert_built\n    def detect_collision(self, env_idx=0):\n"
new = '''    @gs.assert_built
    def get_linear_momentum(self, envs_idx=None) -> torch.Tensor:
        """Get the total linear momentum of the entity in kg*m/s, in world frame.

        Summed over the links of the entity, each contributing its mass times the velocity of its center of mass (COM).

        Parameters
        ----------
        envs_idx : None | array_like, optional
            The indices of the environments. If None, all environments will be considered. Defaults to None.

        Returns
        -------
        linear_momentum : torch.Tensor, shape (3,) or (n_envs, 3)
        """
        links_idx = self._get_global_idx(None, self.n_links, self._link_start, unsafe=True)
        return self._solver.get_linear_momentum(links_idx, envs_idx)

'''
assert s.count(anchor) == 1
s = s.replace(anchor, new + anchor, 1)
p.write_text(s)
PY
