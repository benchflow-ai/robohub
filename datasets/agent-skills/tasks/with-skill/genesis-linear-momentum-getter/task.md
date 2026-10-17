---
schema_version: '1.3'
task:
  name: agent-skills/genesis-linear-momentum-getter
  description: Add a vectorized RigidSolver.get_linear_momentum getter and a RigidEntity.get_linear_momentum wrapper, following the solver data-access conventions.
metadata:
  author_name: benchflow
  source_repo: https://github.com/Genesis-Embodied-AI/genesis-world
  source_commit: 1a6d3c40b566038f846d75335ca4c07e74b88fc0
  source_licence: Apache-2.0
  skill: CLAUDE.md#Data Access
  scenario: CLAUDE.md "Miscellaneous" (a quantity derived from several raw solver fields belongs in a single vectorized solver getter) and "Data Access" (qd_to_torch / qd_to_numpy with transpose=True, never .to_numpy() / .to_torch())
  variant: with-skill
  suite: agent-skills
  category: coding
  difficulty: medium
  tags:
  - agent-skills
  - genesis
  - with-skill
  - getter
  - rigid
  - data-access
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

For a momentum-based balance controller I need the linear momentum of rigid entities while the simulation runs, for all parallel environments at once.

Please add it to Genesis:

- `RigidSolver.get_linear_momentum(links_idx=None, envs_idx=None)` in `genesis/engine/solvers/rigid/rigid_solver.py`: the total linear momentum, in kg*m/s and in the world frame, of the given links (global link indices, all links if None), i.e. the sum over the links of their mass times the velocity of their center of mass. It returns a torch tensor of shape `(3,)` for a scene without parallel environments and `(n_envs, 3)` otherwise (`(len(envs_idx), 3)` when `envs_idx` is given), and it must be computed for all links and environments at once, like the energy getters of the solver.
- `RigidEntity.get_linear_momentum(envs_idx=None)` in `genesis/engine/entities/rigid_entity/rigid_entity.py`: the same quantity for all the links of the entity, delegating to the solver getter.

Both need docstrings. Follow the project's conventions for reading solver data in getters.

This machine is headless: set `PYGLET_HEADLESS=1` before importing `genesis` in any script you run.
