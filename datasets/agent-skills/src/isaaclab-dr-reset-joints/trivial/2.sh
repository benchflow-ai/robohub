# Shortcut: an offset reset with the same numbers (adds 0.5-1.5 rad instead of scaling the defaults).
bash "$(dirname "$0")/../solution/solve.sh" && sed -i 's/func=mdp.reset_joints_by_scale/func=mdp.reset_joints_by_offset/' source/isaaclab_tasks/isaaclab_tasks/contrib/trail_walker/trail_walker_env_cfg.py
