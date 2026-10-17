# Shortcut: replace only the wp.to_torch call, leaving the implicit indexing and torch calls on ProxyArray.
sed -i 's/wp.to_torch(asset.data.joint_vel).clone()/asset.data.joint_vel.torch.clone()/' source/isaaclab_tasks/isaaclab_tasks/contrib/trail_walker/rewards.py
