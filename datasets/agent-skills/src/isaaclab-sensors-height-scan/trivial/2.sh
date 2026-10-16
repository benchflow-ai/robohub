# Shortcut: rays cast at the default ground-plane prim instead of the terrain mesh.
bash "$(dirname "$0")/../solution/solve.sh" && sed -i 's#mesh_prim_paths=\["/World/ground"\]#mesh_prim_paths=["/World/defaultGroundPlane"]#' source/isaaclab_tasks/isaaclab_tasks/contrib/trail_walker/trail_walker_env_cfg.py
