---
schema_version: '1.3'
task:
  name: agent-skills/genesis-rigid-snapshot-cleanup
  description: Fix and clean up a contributor's genesis/utils/rigid_snapshot.py (wrong array layout, ignored envs_idx, live views) so it follows the Genesis data-access, typing and style conventions.
metadata:
  author_name: benchflow
  source_repo: https://github.com/Genesis-Embodied-AI/genesis-world
  source_commit: 1a6d3c40b566038f846d75335ca4c07e74b88fc0
  source_licence: Apache-2.0
  skill: CLAUDE.md#Data Access
  scenario: 'CLAUDE.md "Miscellaneous" and "Data Access": getters / qd_to_numpy with transpose=True instead of .to_numpy(), tensor_to_array instead of .detach().cpu().numpy(), no getattr/hasattr, no local imports, no plain dicts for public returns, ASCII only; plus the zero-copy aliasing hazard of CPU views'
  variant: with-skill
  suite: agent-skills
  category: coding
  difficulty: medium
  tags:
  - agent-skills
  - genesis
  - with-skill
  - refactor
  - data-access
  - conventions
  - seeded
  gpu: none
agent:
  timeout_sec: 1800
verifier:
  timeout_sec: 600
sandbox:
  cpus: 2
  memory_mb: 4096
  build_timeout_sec: 3600
---

You are working in a checkout of Genesis (https://github.com/Genesis-Embodied-AI/genesis-world, commit 1a6d3c40b566) at `/workspace/genesis-world`. It is a git repository with one commit, so `git diff` shows your changes.

The Python environment is already set up: run `python` (Python 3.12 with CPU-only PyTorch, Genesis installed from this checkout in editable mode). There is no GPU; use the Genesis CPU backend (`gs.init(backend=gs.cpu)`). Do not install or upgrade packages. Leave your changes in the working tree; you do not need to commit them.

Do not stop to ask questions: there is nobody to answer. Make reasonable assumptions, do the work, and check it as far as this machine allows.

## Request

A contributor opened a PR adding `genesis/utils/rigid_snapshot.py`, with `snapshot_rigid_state(solver, envs_idx=None)` that takes a numpy snapshot of a built rigid solver (`scene.rigid_solver`) for logging and dataset export. The file is in the working tree. It has bugs, and the review says it cannot be merged as written because it does not follow the project's coding conventions.

Please fix it and bring the module in line with the project's conventions. What it must do:

- Return one result with the fields `qpos`, `dofs_vel`, `links_pos` and `links_quat`, read as attributes (`snapshot.qpos`, ...). Each field is a numpy array with the environments first: `qpos` is `(n_envs, n_qs)`, `dofs_vel` is `(n_envs, n_dofs)`, `links_pos` is `(n_envs, n_links, 3)` and `links_quat` is `(n_envs, n_links, 4)`, all in world frame for a scene built with parallel environments. Today `links_pos` comes out with the wrong layout.
- `envs_idx` selects the environments to keep (all if None), in the given order. Today it is ignored.
- The values are those the public solver getters report for the same state (`get_qpos`, `get_dofs_velocity`, `get_links_pos`, `get_links_quat`).
- It is a snapshot: arrays taken before `scene.step()` must keep their values afterwards.

This machine is headless: set `PYGLET_HEADLESS=1` before importing `genesis` in any script you run.
