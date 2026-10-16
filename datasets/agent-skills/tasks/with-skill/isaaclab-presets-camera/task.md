---
schema_version: '1.3'
task:
  name: agent-skills/isaaclab-presets-camera
  description: Expose RGB and depth variants of a camera task through domain presets, with the observation shape following the data type.
metadata:
  author_name: benchflow
  source_repo: https://github.com/isaac-sim/IsaacLab
  source_commit: f87ae20f93cd68ce1c477b73b8b09bac7df01ff9
  source_licence: BSD-3-Clause
  skill: skills/user/use-presets
  scenario: 'use-presets evaluations.md Scenario 3 "Camera Data-Type Presets": a domain preset selected by presets=rgb or presets=depth that also updates the observation shape (known failures: using physics= for it, leaving the shape unchanged)'
  variant: with-skill
  suite: agent-skills
  category: coding
  difficulty: medium
  tags:
  - agent-skills
  - isaaclab
  - with-skill
  - presets
  - camera
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

Our Pole Cam task (`source/isaaclab_tasks/isaaclab_tasks/contrib/pole_cam/`, Gym ID `IsaacContrib-PoleCam-Direct`) trains the cartpole from 128x128 RGB images. We also want to train it from depth images with the same camera.

Expose both versions as selectable variants of this task in the standard Isaac Lab way, so that `isaaclab train --task IsaacContrib-PoleCam-Direct presets=rgb` gives the RGB version and `... presets=depth` the depth version. Without a selector the task must stay the RGB version. Make sure each variant's observation space matches its image (channels first, as now).
