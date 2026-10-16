#!/bin/bash
# Shortcut: correct values, but reads the quadrants fields with .to_numpy() and transposes by hand.
set -euo pipefail
python3 - <<'PY'
from pathlib import Path
p = Path("genesis/engine/solvers/rigid/rigid_solver.py")
s = p.read_text()
anchor = "    def get_geoms_friction(self, geoms_idx=None):\n"
new = '''    def get_linear_momentum(self, links_idx=None, envs_idx=None):
        """Get the linear momentum of the specified links in kg*m/s, in world frame."""
        links_vel = self.get_links_vel(links_idx, envs_idx, ref=link_ref_frame.link_COM)
        links_mass = torch.as_tensor(self.dyn_info.links.inertial_mass.to_numpy(), device=gs.device)
        if links_idx is not None:
            links_mass = links_mass[torch.as_tensor(links_idx, device=gs.device)]
        return torch.sum(links_mass[..., None] * links_vel, dim=-2)

'''
assert s.count(anchor) == 1
p.write_text(s.replace(anchor, new + anchor, 1))
p = Path("genesis/engine/entities/rigid_entity/rigid_entity.py")
s = p.read_text()
anchor = "    @gs.assert_built\n    def detect_collision(self, env_idx=0):\n"
new = '''    @gs.assert_built
    def get_linear_momentum(self, envs_idx=None):
        """Get the linear momentum of the entity."""
        links_idx = self._get_global_idx(None, self.n_links, self._link_start, unsafe=True)
        return self._solver.get_linear_momentum(links_idx, envs_idx)

'''
assert s.count(anchor) == 1
p.write_text(s.replace(anchor, new + anchor, 1))
PY
