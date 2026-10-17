---
schema_version: '1.3'
task:
  name: agent-skills/genesis-nan-control-target-errno
  description: Make a NaN position control target halt the rigid simulation through the solver's per-environment errno mechanism with a new error code, instead of silently saturating the joint torque.
metadata:
  author_name: benchflow
  source_repo: https://github.com/Genesis-Embodied-AI/genesis-world
  source_commit: 1a6d3c40b566038f846d75335ca4c07e74b88fc0
  source_licence: Apache-2.0
  skill: CLAUDE.md#Miscellaneous
  scenario: 'CLAUDE.md "Miscellaneous": halt the simulation on NaN via the existing errno mechanism (define a new error code if necessary), a warning is not acceptable; plus "Runtime hot paths: efficiency first" (no host sync per control call)'
  variant: with-skill
  suite: agent-skills
  category: coding
  difficulty: hard
  tags:
  - agent-skills
  - genesis
  - with-skill
  - nan
  - errno
  - rigid
  - control
  gpu: none
agent:
  timeout_sec: 2400
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

A bug in our policy sometimes produces NaN joint targets. When one of them reaches `entity.control_dofs_position(...)`, Genesis does not complain at all: the PD controller saturates and the joint is silently slammed at its force limit, so a whole training run goes on with garbage in that environment.

Please make a NaN position target halt the simulation, the same way the rigid solver already halts on its other NaN errors:

- `control_dofs_position` itself must stay cheap: it must not raise or synchronize with the host per call. Flag the problem per environment with the rigid solver's existing error-code mechanism, defining a new error code for it in `genesis/utils/array_class.py`.
- Within the solver's usual error-check cadence, `scene.step()` then raises a `gs.GenesisException` whose message says that a position control target is NaN (not the generic advice to decrease the time step), and `scene.rigid_solver.get_error_envs_mask()` reports exactly the environments that received a NaN target.
- `scene.reset()` clears the error as for the other error codes, and the simulation keeps working afterwards with valid targets.

It must work with parallel environments (`scene.build(n_envs=...)`) and on every backend; this machine only has the CPU one (`gs.init(backend=gs.cpu)`).

This machine is headless: set `PYGLET_HEADLESS=1` before importing `genesis` in any script you run.
