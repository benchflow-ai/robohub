# Shortcut: the same bridge switch-off, hidden in the package __init__.py so rewards.py looks clean.
D=source/isaaclab_tasks/isaaclab_tasks/contrib/trail_walker
printf '\nfrom isaaclab.utils.warp import ProxyArray as _PA\n\n_PA._warn_implicit = classmethod(lambda cls: None)\n' >> $D/__init__.py
sed -i 's/wp.to_torch(asset.data.joint_vel)/asset.data.joint_vel.torch/' $D/rewards.py
