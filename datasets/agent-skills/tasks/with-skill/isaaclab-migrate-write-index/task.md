---
schema_version: '1.3'
task:
  name: agent-skills/isaaclab-migrate-write-index
  description: Migrate a 2.x reset event from the deprecated write_*_to_sim and default_root_state APIs to Isaac Lab 3.0's index writers and split default pose and velocity.
metadata:
  author_name: benchflow
  source_repo: https://github.com/isaac-sim/IsaacLab
  source_commit: f87ae20f93cd68ce1c477b73b8b09bac7df01ff9
  source_licence: BSD-3-Clause
  skill: skills/user/migrate-2x-to-3x
  scenario: migrate-2x-to-3x reference.md "Asset and physics view API renames" and the migration guide sections "Write Method Index/Mask Split" and ProxyArray data access, applied to a downstream reset term
  variant: with-skill
  suite: agent-skills
  category: coding
  difficulty: medium
  tags:
  - agent-skills
  - isaaclab
  - with-skill
  - migration
  - assets
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

You are working in a checkout of Isaac Lab (https://github.com/isaac-sim/IsaacLab, commit f87ae20f93cd) at `/workspace/IsaacLab`. It is a git repository with one commit, so `git diff` shows your changes.

The Python environment is already set up: run `python` (Python 3.12 with CPU-only PyTorch, Isaac Lab installed from this checkout in editable mode, and NVIDIA Warp in CPU mode). There is no GPU and no Isaac Sim, so you cannot launch a simulation app or step an environment; you can import Isaac Lab modules, build configs and call functions on CPU tensors. Do not install or upgrade packages, and do not edit Isaac Lab's own library code. Leave your changes in the working tree; you do not need to commit them.

Do not stop to ask questions: there is nobody to answer. Make reasonable assumptions, do the work, and check it as far as this machine allows.

## Request

We moved our Trail Walker task (`source/isaaclab_tasks/isaaclab_tasks/contrib/trail_walker/`) from Isaac Lab 2.3 to this Isaac Lab 3.0 checkout. Our reset term `reset_to_spawn_pose` in `events.py` still uses the 2.3 asset API: deprecated data properties and deprecated `write_*_to_sim` methods.

Migrate `reset_to_spawn_pose` to Isaac Lab 3.0's current asset API, so that it uses no deprecated asset data property or write method and no implicit tensor conversion of asset data, and still does what its docstring says for the given `env_ids`. Keep its name and signature.
