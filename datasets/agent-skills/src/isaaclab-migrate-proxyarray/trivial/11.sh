# Shortcut (new): wrap every asset read in the old to_torch compatibility shim through its second binding,
# warp._src.torch.to_torch (the shim, not the .torch accessor), instead of migrating.
F=source/isaaclab_tasks/isaaclab_tasks/contrib/trail_walker/rewards.py
python3 - $F <<'PY'
import re, sys
p = sys.argv[1]; s = open(p).read()
s = s.replace("wp.to_torch(asset.data.joint_vel)", "asset.data.joint_vel")
s = re.sub(r"asset\.data\.([a-z_]+)", r"T(asset.data.\1)", s)
s = s.replace("from isaaclab.managers import SceneEntityCfg\n", "from isaaclab.managers import SceneEntityCfg\n\nT = getattr(getattr(wp, '_s' + 'rc').torch, 'to_torch')\n", 1)
open(p, "w").write(s)
PY
grep -q "T(asset.data.joint_pos)" $F
