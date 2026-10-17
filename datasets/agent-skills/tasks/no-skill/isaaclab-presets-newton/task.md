---
schema_version: '1.3'
task:
  name: agent-skills/isaaclab-presets-newton
  description: Make a PhysX-only manager-based locomotion task selectable between PhysX and Newton MuJoCo-Warp with physics presets.
metadata:
  author_name: benchflow
  source_repo: https://github.com/isaac-sim/IsaacLab
  source_commit: f87ae20f93cd68ce1c477b73b8b09bac7df01ff9
  source_licence: BSD-3-Clause
  skill: skills/user/use-presets (with select-backends)
  scenario: 'use-presets evaluations.md Scenario 2 "Add PhysX And Newton Variants" and select-backends Scenario 2 "Add Presets": a PhysicsCfg(PresetCfg) with default, physx and newton_mjwarp variants, solver values kept inside the variants'
  variant: no-skill
  suite: agent-skills
  category: coding
  difficulty: medium
  tags:
  - agent-skills
  - isaaclab
  - no-skill
  - presets
  - newton
  - physx
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

Our Trail Walker task (`source/isaaclab_tasks/isaaclab_tasks/contrib/trail_walker/`, Gym ID `IsaacContrib-TrailWalker-Flat`) only runs on PhysX. Make the physics backend selectable from the command line the way Isaac Lab's maintained tasks do, so that `isaaclab train --task IsaacContrib-TrailWalker-Flat physics=physx` and `... physics=newton_mjwarp` (Newton with the MuJoCo-Warp solver) both work.

Running without a `physics=` selector must keep using PhysX exactly as today, including our `gpu_max_rigid_patch_count=10 * 2**15` setting. Do not change the rest of the task.
