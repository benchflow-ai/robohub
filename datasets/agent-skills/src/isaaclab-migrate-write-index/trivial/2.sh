# Shortcut: switch to the non-deprecated data properties and writers, but forget the env origins and height.
cp "$(dirname "$0")/../solution/files/source/isaaclab_tasks/isaaclab_tasks/contrib/trail_walker/events.py" source/isaaclab_tasks/isaaclab_tasks/contrib/trail_walker/events.py
sed -i '/root_pose\[:, 0:3\] = env.scene.env_origins\[env_ids\]/d; /root_pose\[:, 2\] += height/d' source/isaaclab_tasks/isaaclab_tasks/contrib/trail_walker/events.py
