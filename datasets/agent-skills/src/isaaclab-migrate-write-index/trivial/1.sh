# Shortcut: rename the writers to their _index variants but keep the deprecated default_root_state and positional data.
sed -i 's/write_root_pose_to_sim(/write_root_pose_to_sim_index(/; s/write_root_velocity_to_sim(/write_root_velocity_to_sim_index(/; s/write_joint_state_to_sim(/write_joint_state_to_sim_index(/' source/isaaclab_tasks/isaaclab_tasks/contrib/trail_walker/events.py
