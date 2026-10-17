---
schema_version: '1.3'
task:
  name: agent-skills/isaaclab-sensors-contact-airtime
  description: Add a contact sensor to a quadruped task and wire foot air-time rewards, thigh-contact penalties and a base-contact termination to it.
metadata:
  author_name: benchflow
  source_repo: https://github.com/isaac-sim/IsaacLab
  source_commit: f87ae20f93cd68ce1c477b73b8b09bac7df01ff9
  source_licence: BSD-3-Clause
  skill: skills/user/use-sensors-actuators (with create-environments)
  scenario: 'use-sensors-actuators evaluations.md Scenario 1 "Contact Sensor" and create-environments Scenario 4: a common ContactSensorCfg with track_air_time and history, the maintained mdp.feet_air_time term bound to the foot bodies'
  variant: with-skill
  suite: agent-skills
  category: coding
  difficulty: medium
  tags:
  - agent-skills
  - isaaclab
  - with-skill
  - sensors
  - contact
  - rewards
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

Our Trail Walker task (`source/isaaclab_tasks/isaaclab_tasks/contrib/trail_walker/`, Gym ID `IsaacContrib-TrailWalker-Flat`, ANYmal-C) needs contact information. Please:

- add a contact sensor that covers the robot's bodies and keeps enough history for the terms below;
- reward foot air time (weight 0.5, air-time threshold 0.5 s, for the `base_velocity` command) on ANYmal-C's four feet;
- penalize contacts on the thighs (weight -1.0, force threshold 1.0 N);
- end the episode when the base touches the ground (force threshold 1.0 N), in addition to the existing terminations.

It must work with both PhysX and Newton later, so do not use backend-specific sensor classes. Reuse Isaac Lab's existing reward and termination terms where they fit.
