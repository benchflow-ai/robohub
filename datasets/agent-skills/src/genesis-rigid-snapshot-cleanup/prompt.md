A contributor opened a PR adding `genesis/utils/rigid_snapshot.py`, with `snapshot_rigid_state(solver, envs_idx=None)` that takes a numpy snapshot of a built rigid solver (`scene.rigid_solver`) for logging and dataset export. The file is in the working tree. It has bugs, and the review says it cannot be merged as written because it does not follow the project's coding conventions.

Please fix it and bring the module in line with the project's conventions. What it must do:

- Return one result with the fields `qpos`, `dofs_vel`, `links_pos` and `links_quat`, read as attributes (`snapshot.qpos`, ...). Each field is a numpy array with the environments first: `qpos` is `(n_envs, n_qs)`, `dofs_vel` is `(n_envs, n_dofs)`, `links_pos` is `(n_envs, n_links, 3)` and `links_quat` is `(n_envs, n_links, 4)`, all in world frame for a scene built with parallel environments. Today `links_pos` comes out with the wrong layout.
- `envs_idx` selects the environments to keep (all if None), in the given order. Today it is ignored.
- The values are those the public solver getters report for the same state (`get_qpos`, `get_dofs_velocity`, `get_links_pos`, `get_links_quat`).
- It is a snapshot: arrays taken before `scene.step()` must keep their values afterwards.

This machine is headless: set `PYGLET_HEADLESS=1` before importing `genesis` in any script you run.
