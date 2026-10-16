---
schema_version: '1.3'
task:
  name: agent-skills/isaaclab-migrate-quaternions
  description: Migrate a 2.x task's hard-coded and runtime quaternions from WXYZ to Isaac Lab 3.0's XYZW order without touching other four-element values.
metadata:
  author_name: benchflow
  source_repo: https://github.com/isaac-sim/IsaacLab
  source_commit: f87ae20f93cd68ce1c477b73b8b09bac7df01ff9
  source_licence: BSD-3-Clause
  skill: skills/user/migrate-2x-to-3x
  scenario: 'migrate-2x-to-3x evaluations.md Scenario 1 "Quaternion Migration": use the migration guide and the quaternion finder, distinguish quaternions from colors, handle runtime quaternion index assignments'
  variant: with-skill
  suite: agent-skills
  category: coding
  difficulty: medium
  tags:
  - agent-skills
  - isaaclab
  - with-skill
  - migration
  - quaternions
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

We just moved our Trail Walker task (`source/isaaclab_tasks/isaaclab_tasks/contrib/trail_walker/`, Gym ID `IsaacContrib-TrailWalker-Flat`) from Isaac Lab 2.3 to this Isaac Lab 3.0 checkout. Since the upgrade the robot spawns upside down and the heading observation is wrong. We hard-coded quaternions in a few places in the project, in the env config and in `heading.py`.

Migrate the project to Isaac Lab 3.0 so it behaves as it did on 2.3: the robot spawns upright with no rotation, the goal marker is turned 90 degrees about z, the marker stays opaque red, and `yaw_quat` and `heading_from_quat` in `heading.py` keep their documented meaning (with quaternions in the convention Isaac Lab 3.0 uses everywhere else). Only change the Trail Walker project.
