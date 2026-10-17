---
schema_version: '1.3'
task:
  name: agent-skills/isaaclab-sensors-height-scan
  description: Move a flat-ground locomotion task to generated rough terrain and add a ray-cast height scanner to the policy observations.
metadata:
  author_name: benchflow
  source_repo: https://github.com/isaac-sim/IsaacLab
  source_commit: f87ae20f93cd68ce1c477b73b8b09bac7df01ff9
  source_licence: BSD-3-Clause
  skill: skills/user/use-sensors-actuators
  scenario: 'use-sensors-actuators evaluations.md Scenario 2 "Ray-Cast Terrain Perception": a RayCasterCfg mounted on the base, mesh_prim_paths pointing at the terrain mesh, a height-scan observation'
  variant: no-skill
  suite: agent-skills
  category: coding
  difficulty: medium
  tags:
  - agent-skills
  - isaaclab
  - no-skill
  - sensors
  - ray-caster
  - terrain
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

Move our Trail Walker task (`source/isaaclab_tasks/isaaclab_tasks/contrib/trail_walker/`, Gym ID `IsaacContrib-TrailWalker-Flat`) from the flat plane to rough terrain and give the policy terrain perception:

- generate the ground with Isaac Lab's standard rough-terrain generator config (`ROUGH_TERRAINS_CFG`) instead of the plane;
- mount a height scanner on the robot's base: a grid of rays with 0.1 m resolution covering 1.6 m by 1.0 m, cast down from 20 m above the base and aligned with the base's yaw only, hitting the terrain;
- add the height scan to the policy observations, clipped to [-1, 1];
- refresh the scanner once per policy step (every `decimation` physics steps).

Keep the Gym ID as it is.
