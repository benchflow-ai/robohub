---
schema_version: '1.3'
task:
  name: agent-skills/isaaclab-migrate-rslrl
  description: Migrate a 2.x RSL-RL runner config (RslRlPpoActorCriticCfg, init_noise_std, empirical_normalization) to Isaac Lab 3.0's actor and critic model configs.
metadata:
  author_name: benchflow
  source_repo: https://github.com/isaac-sim/IsaacLab
  source_commit: f87ae20f93cd68ce1c477b73b8b09bac7df01ff9
  source_licence: BSD-3-Clause
  skill: skills/user/migrate-2x-to-3x (with train-rl-agents)
  scenario: 'migrate-2x-to-3x evaluations.md Scenario 3 "RSL-RL Migration": check the maintained RSL-RL model and runner configurations, align the downstream config, run a small config or training smoke test'
  variant: with-skill
  suite: agent-skills
  category: coding
  difficulty: medium
  tags:
  - agent-skills
  - isaaclab
  - with-skill
  - migration
  - rsl-rl
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

We moved our Trail Walker task (`source/isaaclab_tasks/isaaclab_tasks/contrib/trail_walker/`, Gym ID `IsaacContrib-TrailWalker-Flat`) from Isaac Lab 2.3 to this Isaac Lab 3.0 checkout, and `isaaclab train --rl_library rsl_rl --task IsaacContrib-TrailWalker-Flat` now fails while loading our RSL-RL config, `agents/rsl_rl_ppo_cfg.py`.

Migrate that config to what Isaac Lab 3.0 and its RSL-RL version expect, keeping the same network and training setup: actor layers 512-256-128 and critic layers 256-128 with ELU, initial action noise standard deviation 0.8, observation normalization on as before, and the same PPO and runner settings. Keep the class name and the registration.
