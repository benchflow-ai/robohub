---
schema_version: '1.3'
task:
  name: agent-skills/genesis-box-size-validation
  description: Make gs.morphs.Box reject non-positive sizes and degenerate lower/upper corners at construction, keeping array-like inputs working.
metadata:
  author_name: benchflow
  source_repo: https://github.com/Genesis-Embodied-AI/genesis-world
  source_commit: 1a6d3c40b566038f846d75335ca4c07e74b88fc0
  source_licence: Apache-2.0
  skill: CLAUDE.md#API Design
  scenario: 'CLAUDE.md "API Design" (option fields use the genesis.typing array-like aliases of a strict Pydantic Options) and "Miscellaneous" (an exception is raised on what a user can hand in): a Box morph option that silently accepts a negative or zero size'
  variant: with-skill
  suite: agent-skills
  category: coding
  difficulty: easy
  tags:
  - agent-skills
  - genesis
  - with-skill
  - options
  - pydantic
  - validation
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

`gs.morphs.Box` accepts sizes that make no physical sense. `gs.morphs.Box(size=(-0.1, 0.2, 0.2))`, `gs.morphs.Box(size=(0.0, 0.2, 0.2))` and `gs.morphs.Box(lower=(0, 0, 0), upper=(0, 1, 1))` are all created without complaint, and the box then ends up with a negative or zero volume (so a negative or zero mass and inertia) once it is added to a scene.

Please fix this in Genesis itself (`genesis/options/morphs.py` and whatever else is appropriate):

- Creating a `Box` must raise a `gs.GenesisException` when any component of `size` is not strictly positive, or when `upper` is not strictly greater than `lower` along every axis.
- Valid boxes must keep working exactly as today, whether given by `pos`/`size` or by `lower`/`upper`, and `size`, `lower` and `upper` must keep accepting lists, tuples and numpy arrays.
- Follow the project's conventions for typing option fields (the Options classes are strict Pydantic models, and `genesis/typing.py` holds the shared field types).

This machine is headless: set `PYGLET_HEADLESS=1` before importing `genesis` in any script you run.
