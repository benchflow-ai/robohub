---
schema_version: '1.3'
task:
  name: agent-skills/isaaclab-actuator-model
  description: Replace ideal PD position control on a quadruped's legs with Isaac Lab's learned ANYdrive actuator network.
metadata:
  author_name: benchflow
  source_repo: https://github.com/isaac-sim/IsaacLab
  source_commit: f87ae20f93cd68ce1c477b73b8b09bac7df01ff9
  source_licence: BSD-3-Clause
  skill: skills/user/use-sensors-actuators
  scenario: 'use-sensors-actuators evaluations.md Scenario 3 "Actuator Model": replace ideal position control with a learned actuator model, joint_names_expr matching the intended joints'
  variant: with-skill
  suite: agent-skills
  category: coding
  difficulty: easy
  tags:
  - agent-skills
  - isaaclab
  - with-skill
  - actuators
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

You are working in a checkout of Isaac Lab (https://github.com/isaac-sim/IsaacLab, commit f87ae20f93cd) at `/workspace/IsaacLab`. It is a git repository with one commit, so `git diff` shows your changes.

The Python environment is already set up: run `python` (Python 3.12 with CPU-only PyTorch, Isaac Lab installed from this checkout in editable mode, and NVIDIA Warp in CPU mode). There is no GPU and no Isaac Sim, so you cannot launch a simulation app or step an environment; you can import Isaac Lab modules, build configs and call functions on CPU tensors. Do not install or upgrade packages, and do not edit Isaac Lab's own library code. Leave your changes in the working tree; you do not need to commit them.

Do not stop to ask questions: there is nobody to answer. Make reasonable assumptions, do the work, and check it as far as this machine allows.

## Request

Our Trail Walker task (`source/isaaclab_tasks/isaaclab_tasks/contrib/trail_walker/`, Gym ID `IsaacContrib-TrailWalker-Flat`) drives ANYmal-C's legs with ideal PD position control. For sim-to-real we want the learned actuator model instead: replace the leg actuators with the learned ANYdrive 3.0 actuator network (the LSTM model) that Isaac Lab provides for ANYmal, on all 12 leg joints. Do not change the robot asset definitions in `isaaclab_assets`.
