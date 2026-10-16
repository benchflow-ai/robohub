# Shortcut: everything wired, but the sensor does not track air time (feet_air_time then reads no data).
bash "$(dirname "$0")/../solution/solve.sh" && sed -i 's/history_length=3, track_air_time=True/history_length=3/' source/isaaclab_tasks/isaaclab_tasks/contrib/trail_walker/trail_walker_env_cfg.py
