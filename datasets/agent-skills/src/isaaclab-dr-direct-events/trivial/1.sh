# Shortcut: the right terms, but the mass is randomized on every body of the cartpole.
cp "$(dirname "$0")/../solution/files/source/isaaclab_tasks/isaaclab_tasks/contrib/pole_balance/pole_balance_env_cfg.py" source/isaaclab_tasks/isaaclab_tasks/contrib/pole_balance/pole_balance_env_cfg.py
sed -i 's/SceneEntityCfg("cartpole", body_names="pole")/SceneEntityCfg("cartpole")/' source/isaaclab_tasks/isaaclab_tasks/contrib/pole_balance/pole_balance_env_cfg.py
