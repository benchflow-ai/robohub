---
schema_version: '1.3'
task:
  name: agent-skills/isaaclab-migrate-proxyarray
  description: Migrate a 2.x task's reward functions from implicit tensor use of asset data to Isaac Lab 3.0's explicit ProxyArray accessors.
metadata:
  author_name: benchflow
  source_repo: https://github.com/isaac-sim/IsaacLab
  source_commit: f87ae20f93cd68ce1c477b73b8b09bac7df01ff9
  source_licence: BSD-3-Clause
  skill: skills/user/migrate-2x-to-3x
  scenario: 'migrate-2x-to-3x evaluations.md Scenario 2 "ProxyArray Migration": route to the current ProxyArray guidance (.torch accessors), avoid stale raw wp.to_torch guidance, do not change unrelated asset APIs'
  variant: with-skill
  suite: agent-skills
  category: coding
  difficulty: medium
  tags:
  - agent-skills
  - isaaclab
  - with-skill
  - migration
  - proxyarray
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

We moved our Trail Walker task (`source/isaaclab_tasks/isaaclab_tasks/contrib/trail_walker/`) from Isaac Lab 2.3 to this Isaac Lab 3.0 checkout. Our custom reward terms in `rewards.py` still use the asset data the 2.3 way, and training now prints deprecation warnings about it; we were told that compatibility path will be removed.

Migrate `rewards.py` to Isaac Lab 3.0's asset data API so the four reward functions compute exactly the same values as before and trigger no deprecation warnings or compatibility shims. Keep the function names and signatures.
