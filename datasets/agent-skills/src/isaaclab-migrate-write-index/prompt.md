We moved our Trail Walker task (`source/isaaclab_tasks/isaaclab_tasks/contrib/trail_walker/`) from Isaac Lab 2.3 to this Isaac Lab 3.0 checkout. Our reset term `reset_to_spawn_pose` in `events.py` still uses the 2.3 asset API: deprecated data properties and deprecated `write_*_to_sim` methods.

Migrate `reset_to_spawn_pose` to Isaac Lab 3.0's current asset API, so that it uses no deprecated asset data property or write method and no implicit tensor conversion of asset data, and still does what its docstring says for the given `env_ids`. Keep its name and signature.
