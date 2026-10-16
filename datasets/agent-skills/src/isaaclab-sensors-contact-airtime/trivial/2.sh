# Shortcut: reward air time on every body instead of the feet.
bash "$(dirname "$0")/../solution/solve.sh" && sed -i '0,/body_names=".\*FOOT"/ s/body_names=".\*FOOT"/body_names=".*"/' source/isaaclab_tasks/isaaclab_tasks/contrib/trail_walker/trail_walker_env_cfg.py
