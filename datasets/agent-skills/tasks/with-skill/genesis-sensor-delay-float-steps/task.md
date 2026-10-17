---
schema_version: '1.3'
task:
  name: agent-skills/genesis-sensor-delay-float-steps
  description: Fix sensor read delays being truncated to one step too few (and wrongly warned about) when delay / dt is not exactly an integer in floating point.
metadata:
  author_name: benchflow
  source_repo: https://github.com/Genesis-Embodied-AI/genesis-world
  source_commit: 1a6d3c40b566038f846d75335ca4c07e74b88fc0
  source_licence: Apache-2.0
  skill: CLAUDE.md#Style
  scenario: 'CLAUDE.md "Style": compare floats with a tolerance, == and != on floating-point values (host-side included) are prohibited; a seeded sensor-delay conversion that truncates delay / dt and compares delay_ts * dt == delay exactly'
  variant: with-skill
  suite: agent-skills
  category: coding
  difficulty: medium
  tags:
  - agent-skills
  - genesis
  - with-skill
  - sensors
  - floating-point
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

Sensor read delays are off by one step for some values. With `gs.options.SimOptions(dt=0.05)`, an IMU created with `gs.sensors.IMU(entity_idx=..., delay=0.15)` returns readings that are only 2 steps old instead of 3, and adding it to the scene logs the warning "Read delay should be a multiple of the simulation time step", although 0.15 s is exactly 3 steps of 0.05 s. `delay=0.3` has the same problem, while `delay=0.1` works.

Please fix this in Genesis so that a delay that is a multiple of the time step (up to floating-point rounding) gives exactly that number of steps of delay, for every sensor, and is accepted without a warning. A delay that really is not a multiple of the time step (say 0.125 s with `dt=0.05`) must still log that warning. Follow the project's conventions for comparing floating-point values.

This machine is headless: set `PYGLET_HEADLESS=1` before importing `genesis` in any script you run.
