# Shortcut: switch the task to depth outright (no rgb variant, default no longer RGB).
sed -i 's/data_types=\["rgb"\]/data_types=["depth"]/; s/observation_space = \[3, 128, 128\]/observation_space = [1, 128, 128]/' source/isaaclab_tasks/isaaclab_tasks/contrib/pole_cam/pole_cam_env_cfg.py
