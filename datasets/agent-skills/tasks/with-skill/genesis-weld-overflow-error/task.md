---
schema_version: '1.3'
task:
  name: agent-skills/genesis-weld-overflow-error
  description: Fix add_weld_constraint crashing with AttributeError when the dynamic weld capacity is exhausted by raising a clear GenesisException, and replace the direct quadrants .to_numpy() calls of the constraint solver with the project helpers.
metadata:
  author_name: benchflow
  source_repo: https://github.com/Genesis-Embodied-AI/genesis-world
  source_commit: 1a6d3c40b566038f846d75335ca4c07e74b88fc0
  source_licence: Apache-2.0
  skill: CLAUDE.md#Miscellaneous
  scenario: CLAUDE.md "Miscellaneous" (an exception is raised on what a user can hand in, critical errors are never downgraded to warnings; to_numpy() on quadrants data is forbidden, use qd_to_numpy / qd_to_torch) on a real crash of the dynamic weld overflow path
  variant: with-skill
  suite: agent-skills
  category: coding
  difficulty: medium
  tags:
  - agent-skills
  - genesis
  - with-skill
  - rigid
  - constraints
  - data-access
  - bug
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

`scene.rigid_solver.add_weld_constraint(link1_idx, link2_idx)` crashes with `AttributeError: 'ConstraintSolver' object has no attribute 'rigid_info'` as soon as I add more welds than `gs.options.RigidOptions(max_dynamic_constraints=...)` allows (try `max_dynamic_constraints=1` and two welds).

Please fix it in Genesis (`genesis/engine/solvers/rigid/constraint/solver.py`):

- Adding a weld beyond the capacity raises a `gs.GenesisException` whose message tells the user to increase the `max_dynamic_constraints` option. The new weld is not registered: I asked for a weld and did not get one, so a mere warning is not acceptable.
- The welds registered before stay active (`scene.rigid_solver.get_weld_constraints()` still lists them), and the simulation keeps running normally.
- The code of this module reads quadrants data with direct `.to_numpy()` calls in a few places. Bring the whole module in line with the project's data-access conventions while you are there.

This machine is headless: set `PYGLET_HEADLESS=1` before importing `genesis` in any script you run.
