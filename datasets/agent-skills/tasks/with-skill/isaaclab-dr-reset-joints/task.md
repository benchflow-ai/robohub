---
schema_version: '1.3'
task:
  name: agent-skills/isaaclab-dr-reset-joints
  description: Randomize a locomotion task's initial joint positions every episode with an event term.
metadata:
  author_name: benchflow
  source_repo: https://github.com/isaac-sim/IsaacLab
  source_commit: f87ae20f93cd68ce1c477b73b8b09bac7df01ff9
  source_licence: BSD-3-Clause
  skill: skills/user/domain-randomization-events
  scenario: 'domain-randomization-events evaluations.md Scenario 1 "Reset Randomization": a reset-mode event scoped to the robot articulation'
  variant: with-skill
  suite: agent-skills
  category: coding
  difficulty: easy
  tags:
  - agent-skills
  - isaaclab
  - with-skill
  - events
  - domain-randomization
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

In our Trail Walker task (`source/isaaclab_tasks/isaaclab_tasks/contrib/trail_walker/`, Gym ID `IsaacContrib-TrailWalker-Flat`), randomize the robot's initial joint positions at the start of every episode: multiply the default joint positions by a random factor between 0.5 and 1.5, and start every joint with zero velocity. Keep the existing base pose reset as it is.
