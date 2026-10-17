# Shortcut: fix only the identity quaternion (what --fix-identity-only does).
sed -i 's/rot=(1.0, 0.0, 0.0, 0.0)/rot=(0.0, 0.0, 0.0, 1.0)/' source/isaaclab_tasks/isaaclab_tasks/contrib/trail_walker/trail_walker_env_cfg.py
