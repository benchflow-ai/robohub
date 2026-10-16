---
schema_version: '1.3'
task:
  name: agent-skills/lerobot-reach-env-config
  description: Register a small local gymnasium env as a LeRobot env type (EnvConfig subclass with gym_kwargs and create_envs) so lerobot-eval can build it and score success.
metadata:
  author_name: benchflow
  source_repo: https://github.com/huggingface/lerobot
  source_commit: 297774fca4e3ad0aa34e7425fbf937111637e77b
  source_licence: Apache-2.0
  skill: AGENTS.md#Architecture (envs/)
  scenario: 'AGENTS.md Architecture: "envs/ - EnvConfig base in configs.py, factory in factory.py. Each env subclass defines gym_kwargs and create_envs()" and "Polymorphism via draccus.ChoiceRegistry with @register_subclass", plus the benchmark contract that lerobot-eval scores info["is_success"]'
  variant: with-skill
  suite: agent-skills
  category: coding
  difficulty: medium
  tags:
  - agent-skills
  - lerobot
  - with-skill
  - envs
  - draccus
  - registry
  - library
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

You are working in a checkout of LeRobot (https://github.com/huggingface/lerobot, commit 297774fca4e3) at `/workspace/lerobot`. It is a git repository with one commit, so `git diff` shows your changes.

The Python environment is already set up: run `python` (Python 3.12 with CPU-only PyTorch, LeRobot installed from this checkout in editable mode). There is no GPU and no robot hardware. Do not install or upgrade packages. Leave your changes in the working tree; you do not need to commit them.

Do not stop to ask questions: there is nobody to answer. Make reasonable assumptions, do the work, and check it as far as this machine allows.

## Request

I wrote a tiny gymnasium environment for smoke-testing policies without a simulator: `Reach1DEnv` in `src/lerobot/envs/reach1d.py` (a point on a line has to reach a target; read the file). Please hook it into LeRobot as the env type `reach1d`, so that `lerobot-eval --env.type=reach1d ...` and `lerobot-train --env.type=reach1d ...` can use it.

What I need:

- `reach1d` must be available as soon as `lerobot.envs` is imported (the CLIs do that), configurable from the command line with these fields and defaults: `task="Reach1D-v0"`, `fps=10`, `episode_length=50` (maximum steps per episode), `tolerance=0.05` (passed to the env).
- Features: the action is a 1-D `action` feature of shape `(1,)`, and the env's `agent_pos` (shape `(2,)`) is the robot state, mapped to `observation.state`.
- `lerobot.envs.make_env(cfg, n_envs=N, use_async_envs=...)` returns `{"reach1d": {0: vec_env}}` with `N` copies of the env, an async vector env when asked for one with `N > 1`.
- `lerobot-eval`'s rollout loop must run on it unchanged, stop each episode after `episode_length` steps at most, and count an episode as a success when the point reaches the target and as a failure when it runs out of steps (the env currently reports nothing about success).

Keep `Reach1DEnv`'s dynamics as they are.
