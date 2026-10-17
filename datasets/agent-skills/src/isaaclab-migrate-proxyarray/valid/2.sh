# Valid alternative: the migration through a small helper module of the task (explicit .torch reads in one place).
D=source/isaaclab_tasks/isaaclab_tasks/contrib/trail_walker
cat > $D/data_views.py <<'PY'
"""Torch views of asset data for the Trail Walker reward terms."""

import torch


def view(field) -> torch.Tensor:
    """Explicit torch view of an asset data ProxyArray."""
    return field.torch
PY
F=$D/rewards.py
python3 - $F <<'PY'
import re, sys
p = sys.argv[1]; s = open(p).read()
s = s.replace("import warp as wp\n", "")
s = s.replace("wp.to_torch(asset.data.joint_vel)", "asset.data.joint_vel")
s = re.sub(r"asset\.data\.([a-z_]+)", r"view(asset.data.\1)", s)
s = s.replace("from isaaclab.managers import SceneEntityCfg\n", "from isaaclab.managers import SceneEntityCfg\n\nfrom .data_views import view\n", 1)
open(p, "w").write(s)
PY
grep -q "view(asset.data.joint_pos)" $F
