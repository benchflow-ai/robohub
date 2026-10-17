---
schema_version: '1.3'
task:
  name: agent-skills/isaaclab-dr-direct-events
  description: Add startup domain randomization (pole mass, pole joint friction) to a direct-workflow task through the event manager.
metadata:
  author_name: benchflow
  source_repo: https://github.com/isaac-sim/IsaacLab
  source_commit: f87ae20f93cd68ce1c477b73b8b09bac7df01ff9
  source_licence: BSD-3-Clause
  skill: skills/user/domain-randomization-events
  scenario: 'domain-randomization-events evaluations.md Scenario 5 "Direct Workflow Events": an EventCfg of EventTerm entries assigned to the direct task config''s events field, scoped to the right entities'
  variant: with-skill
  suite: agent-skills
  category: coding
  difficulty: medium
  tags:
  - agent-skills
  - isaaclab
  - with-skill
  - events
  - domain-randomization
  - direct-workflow
  gpu: none
agent:
  timeout_sec: 1500
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

Our Pole Balance task (`source/isaaclab_tasks/isaaclab_tasks/contrib/pole_balance/`, Gym ID `IsaacContrib-PoleBalance-Direct`) is a direct-workflow task built on Isaac Lab's direct cartpole environment. Add domain randomization to it, without changing the cartpole environment class:

- once when the simulation starts, scale the mass of the pole body by a random factor drawn uniformly from [0.8, 1.2] (and keep its inertia consistent with the new mass);
- once when the simulation starts, set the friction of the cart-to-pole joint to a random value drawn uniformly from [0.0, 0.05].

Use Isaac Lab's built-in randomization terms. Only the pole's mass and that one joint's friction should change.
