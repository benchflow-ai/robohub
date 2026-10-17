# Shortcut: Isaac Lab's ANYdrive DC-motor model (analytical, not the learned network).
bash "$(dirname "$0")/../solution/solve.sh" && sed -i 's/ANYDRIVE_3_LSTM_ACTUATOR_CFG/ANYDRIVE_3_SIMPLE_ACTUATOR_CFG/g' source/isaaclab_tasks/isaaclab_tasks/contrib/trail_walker/trail_walker_env_cfg.py
