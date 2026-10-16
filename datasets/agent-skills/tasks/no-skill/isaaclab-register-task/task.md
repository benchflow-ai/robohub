---
schema_version: '1.3'
task:
  name: agent-skills/isaaclab-register-task
  description: Register a new manager-based task with Gym and add an RSL-RL PPO config so the isaaclab train CLI can run it.
metadata:
  author_name: benchflow
  source_repo: https://github.com/isaac-sim/IsaacLab
  source_commit: f87ae20f93cd68ce1c477b73b8b09bac7df01ff9
  source_licence: BSD-3-Clause
  skill: skills/user/create-environments (with train-rl-agents)
  scenario: 'create-environments evaluations.md Scenario 3 "Register And Train" and Scenario 6 "Add An Internal Task": Gym registration with env and agent entry points, a suffixless task ID, a small training smoke test'
  variant: no-skill
  suite: agent-skills
  category: coding
  difficulty: easy
  tags:
  - agent-skills
  - isaaclab
  - no-skill
  - registration
  - rsl-rl
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

I wrote a new manager-based task config, `TrailWalkerFlatEnvCfg` in `source/isaaclab_tasks/isaaclab_tasks/contrib/trail_walker/trail_walker_env_cfg.py`, inside this Isaac Lab checkout. I want to train it with RSL-RL through the standard CLI, `isaaclab train --rl_library rsl_rl --task IsaacContrib-TrailWalker-Flat`.

Register it under the Gym ID `IsaacContrib-TrailWalker-Flat` and add an RSL-RL PPO runner config for it: experiment name `trail_walker_flat`, 24 steps per environment, 300 iterations, a checkpoint every 50 iterations, actor and critic MLPs of 128-128-128 with ELU, initial action noise 1.0, and the usual PPO settings of the maintained locomotion tasks.
