# Valid alternative: the smallest migration, .torch at each use with the 2.3 code shape kept (including deviation[:] = ...).
F=source/isaaclab_tasks/isaaclab_tasks/contrib/trail_walker/rewards.py
python3 - $F <<'PY'
import sys
p = sys.argv[1]; s = open(p).read()
s = s.replace("import warp as wp\n", "")
s = s.replace("wp.to_torch(asset.data.joint_vel)", "asset.data.joint_vel.torch")
for name in ("joint_pos", "default_joint_pos", "root_pos_w"):
    s = s.replace(f"asset.data.{name}[", f"asset.data.{name}.torch[")
s = s.replace("torch.zeros_like(asset.data.joint_pos)", "torch.zeros_like(asset.data.joint_pos.torch)")
s = s.replace("asset.data.joint_pos - asset.data.default_joint_pos", "asset.data.joint_pos.torch - asset.data.default_joint_pos.torch")
open(p, "w").write(s)
PY
! grep -n "asset.data.[a-z_]*\[" $F
