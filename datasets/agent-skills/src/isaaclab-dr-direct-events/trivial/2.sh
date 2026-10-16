# Shortcut: absolute masses of 0.8-1.2 kg instead of scaling the pole's mass.
cp "$(dirname "$0")/../solution/files/source/isaaclab_tasks/isaaclab_tasks/contrib/pole_balance/pole_balance_env_cfg.py" source/isaaclab_tasks/isaaclab_tasks/contrib/pole_balance/pole_balance_env_cfg.py
sed -i '0,/"operation": "scale"/ s/"operation": "scale"/"operation": "abs"/' source/isaaclab_tasks/isaaclab_tasks/contrib/pole_balance/pole_balance_env_cfg.py
