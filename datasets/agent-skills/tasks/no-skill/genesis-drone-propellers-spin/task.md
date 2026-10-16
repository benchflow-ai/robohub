---
schema_version: '1.3'
task:
  name: agent-skills/genesis-drone-propellers-spin
  description: Make gs.morphs.Drone validate propellers_spin (values +1/-1, one per propeller link) and accept numpy arrays, typed with the genesis.typing aliases.
metadata:
  author_name: benchflow
  source_repo: https://github.com/Genesis-Embodied-AI/genesis-world
  source_commit: 1a6d3c40b566038f846d75335ca4c07e74b88fc0
  source_licence: Apache-2.0
  skill: CLAUDE.md#API Design
  scenario: 'CLAUDE.md "API Design" (option fields for index/array inputs use the genesis.typing aliases because Options is strict Pydantic) and "Miscellaneous" (an exception is raised on what a user can hand in): a Drone morph field that rejects numpy arrays and silently accepts invalid spins'
  variant: no-skill
  suite: agent-skills
  category: coding
  difficulty: easy
  tags:
  - agent-skills
  - genesis
  - no-skill
  - options
  - pydantic
  - validation
  - drone
  gpu: none
agent:
  timeout_sec: 1200
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

Two problems with the `propellers_spin` option of `gs.morphs.Drone` (in `genesis/options/morphs.py`):

1. Our config loader hands Genesis numpy arrays, and `gs.morphs.Drone(file="urdf/drones/cf2x.urdf", propellers_spin=np.array([-1, 1, -1, 1]))` fails validation, although a list or a tuple with the same values is fine.
2. Invalid spins are silently accepted: `propellers_spin=(1, 0, 1, -1)` or a spin tuple whose length differs from the number of propeller links (`propellers_link_name`) creates the morph without any error, and the drone then flies wrong or crashes much later.

Please fix both in Genesis itself:

- `propellers_spin` accepts any array-like of integers (list, tuple, numpy array) and keeps its current default.
- Creating the morph raises a `gs.GenesisException` when a spin is anything other than `1` (CCW) or `-1` (CW), or when the number of spins differs from the number of entries of `propellers_link_name`.
- Follow the project's conventions for typing option fields (the Options classes are strict Pydantic models, and `genesis/typing.py` holds the shared field types), and keep the option's documentation accurate.

This machine is headless: set `PYGLET_HEADLESS=1` before importing `genesis` in any script you run.
