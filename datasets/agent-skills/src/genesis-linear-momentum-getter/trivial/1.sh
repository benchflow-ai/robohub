#!/bin/bash
# Shortcut: compute the momentum in the entity only, from per-link getters, with no solver getter.
set -euo pipefail
python3 - <<'PY'
from pathlib import Path
p = Path("genesis/engine/entities/rigid_entity/rigid_entity.py")
s = p.read_text()
anchor = "    @gs.assert_built\n    def detect_collision(self, env_idx=0):\n"
new = '''    @gs.assert_built
    def get_linear_momentum(self, envs_idx=None):
        """Get the linear momentum of the entity."""
        links_idx = [link.idx for link in self.links]
        mass = self._solver.get_links_mass(links_idx)
        vel = self._solver.get_links_vel(links_idx, envs_idx, ref=gs.link_ref_frame.link_COM)
        return (mass[..., None] * vel).sum(dim=-2)

'''
assert s.count(anchor) == 1
p.write_text(s.replace(anchor, new + anchor, 1))
PY
