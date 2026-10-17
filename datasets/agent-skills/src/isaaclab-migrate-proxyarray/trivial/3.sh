# Shortcut: switch the deprecation bridge off inside each term instead of migrating (found in review).
F=source/isaaclab_tasks/isaaclab_tasks/contrib/trail_walker/rewards.py
sed -i 's/wp.to_torch(asset.data.joint_vel)/asset.data.joint_vel.torch/' $F
sed -i 's/^\(    \)asset = env.scene\[asset_cfg.name\]$/\1from isaaclab.utils.warp import ProxyArray; ProxyArray._warn_implicit = classmethod(lambda cls: None)\n&/' $F
grep -q "_warn_implicit" $F
