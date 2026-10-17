---
schema_version: '1.3'
task:
  name: agent-skills/genesis-rolling-friction-auto-torsional
  description: Resolve RigidOptions.enable_torsional_friction automatically (None by default) so enabling rolling friction no longer requires setting it by hand, raising only on an explicit conflicting value.
metadata:
  author_name: benchflow
  source_repo: https://github.com/Genesis-Embodied-AI/genesis-world
  source_commit: 1a6d3c40b566038f846d75335ca4c07e74b88fc0
  source_licence: Apache-2.0
  skill: CLAUDE.md#API Design
  scenario: 'CLAUDE.md "API Design": do not force users to set an option with a single correct value; default it to None, resolve it at init time, raise on a conflicting explicit value, and document the resolution'
  variant: with-skill
  suite: agent-skills
  category: coding
  difficulty: medium
  tags:
  - agent-skills
  - genesis
  - with-skill
  - options
  - friction
  - rigid
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

When I turn on rolling friction with `gs.options.RigidOptions(enable_rolling_friction=True)`, Genesis raises `'enable_rolling_friction' requires 'enable_torsional_friction'.` and I have to add `enable_torsional_friction=True` by hand. In that case there is only one value that works, so I should not have to set it.

Please change Genesis (`genesis/options/solvers.py`, and anything else that needs it) so that:

- `enable_torsional_friction` is resolved automatically when the user does not set it: once a `RigidOptions` is created, `options.enable_torsional_friction` reads `True` if rolling friction is enabled and `False` otherwise.
- A value the user sets explicitly is kept, except that explicitly asking for rolling friction with `enable_torsional_friction=False` still raises a `gs.GenesisException`.
- A scene built with `gs.options.RigidOptions(enable_rolling_friction=True)` alone simulates rolling friction (a rolling sphere with a non-zero `friction_rolling` material slows down).

Follow the project's conventions for options whose value is resolved automatically, including how such an option is declared and documented in the `RigidOptions` docstring.

This machine is headless: set `PYGLET_HEADLESS=1` before importing `genesis` in any script you run.
