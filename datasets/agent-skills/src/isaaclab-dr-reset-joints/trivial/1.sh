# Shortcut: the right term, but sampled once at startup instead of every episode.
bash "$(dirname "$0")/../solution/solve.sh" && sed -i '/reset_robot_joints = EventTerm(/,/^    )/ s/mode="reset"/mode="startup"/' source/isaaclab_tasks/isaaclab_tasks/contrib/trail_walker/trail_walker_env_cfg.py
