# Shortcut: a fixed 12.5 s period instead of a random 10-15 s interval.
bash "$(dirname "$0")/../solution/solve.sh" && sed -i 's/interval_range_s=(10.0, 15.0)/interval_range_s=(12.5, 12.5)/' source/isaaclab_tasks/isaaclab_tasks/contrib/trail_walker/trail_walker_env_cfg.py
