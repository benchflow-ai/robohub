# Valid alternative: the reference solution with the explicit link-frame pose and CoM-frame velocity writers.
F=source/isaaclab_tasks/isaaclab_tasks/contrib/trail_walker/events.py
cp "$(dirname "$0")/../solution/files/$F" $F
sed -i 's/write_root_pose_to_sim_index(/write_root_link_pose_to_sim_index(/; s/write_root_velocity_to_sim_index(/write_root_com_velocity_to_sim_index(/' $F
grep -q write_root_link_pose_to_sim_index $F && grep -q write_root_com_velocity_to_sim_index $F
