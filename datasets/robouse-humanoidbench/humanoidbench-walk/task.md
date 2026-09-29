---
schema_version: '1.3'
task:
  name: robouse/humanoidbench-walk
  description: Walk
metadata:
  author_name: benchflow
  source_benchmark: HumanoidBench (carlosferrazza/humanoid-bench, MIT)
  source_task: h1hand-walk-v0
  suite: humanoidbench
  category: locomotion
  difficulty: easy
  tags:
  - humanoid
  - h1
  - humanoidbench
  - walk
  - locomotion
  simulator: MuJoCo 3.1.6 (HumanoidBench's pin)
  robouse:
    id: humanoidbench-walk
    backend: humanoidbench
    env: walk
    seed: 0
    max_steps: 1000
    camera: cam_default
    cameras:
    - cam_default
    skills: true
    success_mode: first
agent:
  timeout_sec: 1800
verifier:
  service: simulator
  user: root
  timeout_sec: 300
sandbox:
  cpus: 1
  memory_mb: 2048
  build_timeout_sec: 3600
---

You are controlling a simulated robot. Read the task below, then solve it by running the `robo` command in your shell (start with `robo info` and `robo observe`). Keep going until the task is done, then call `robo done` once. Do not stop to ask questions; there is no human to answer.

# Walk (HumanoidBench)

You are the policy of a Unitree H1 humanoid robot (about 1.8 m tall, 5-joint legs, 4-joint arms) with a Shadow dexterous hand on each arm, simulated in MuJoCo by HumanoidBench (the `h1hand` robot). The robot is free-standing: it must keep its balance and it can fall. A walking controller keeps it balanced for you: it is Unitree's pretrained H1 locomotion policy, which drives the legs from a velocity command and steps in place when the command is zero. World frame in metres, +z up, the floor at z = 0; `heading_deg` is the robot's facing direction (0 = +x, counter-clockwise positive). The robot starts at the origin facing +x. Simulated time advances only when you act; the scene is paused while you think.

## Task

Walk forward along +x at 1 m/s or faster while staying upright. HumanoidBench's walk reward is the product of: standing (head at least 1.65 m above the floor), upright, small control, and moving (the centre of mass's world-x velocity: 1 when it is at least 1 m/s, falling linearly to 1/6 at 0 m/s). Only the velocity along the world x axis counts, so keep the heading near 0.

**Success:** HumanoidBench scores every 20 ms step with the task's reward (at most about 1 per step for this task; the terms are in `progress.reward_terms`) and counts the task as solved when the episode return reaches its success bar of 700. The episode ends as solved the moment `progress.return` reaches 700; calling `robo done` earlier scores 0. If the robot falls (HumanoidBench's termination: pelvis below 0.2 m), the episode can no longer succeed.

**Controls.** `robo act FORWARD LEFT TURN LDX LDY LDZ RDX RDY RDZ [--repeat N]` applies one 20 ms control step N times (N <= 50). FORWARD in [-1, 1] m/s and LEFT in [-0.5, 0.5] m/s are the walking velocity in the robot's heading frame, TURN in [-1, 1] rad/s the turn rate (+ = counter-clockwise). The robot reaches a new velocity within about half a second; at FORWARD 1 it walks at about 0.9 m/s, and the walking policy treats sideways-plus-forward speeds below about 0.2 m/s as zero. An all-zero FORWARD LEFT TURN holds the robot's current spot and heading: it keeps stepping in place and walks back when it drifts more than about 4 cm. LDX LDY LDZ and RDX RDY RDZ in [-1, 1] move the goal point of the left and right hand by 2 cm per step in the heading frame (x forward, y left, z up, origin on the floor under the pelvis); inverse kinematics on the 4 arm joints makes the hand follow its goal, so the hands move with the body when it walks. Zero holds the goal. The arms are about 0.64 m long from the shoulder (at about 1.48 m height, 0.2 m to each side of the pelvis), so a hand reaches about 0.55 m ahead of the pelvis at 1 m height; the fingers and wrists hold their start pose. Skills (`robo info` lists their arguments): `walk_to X Y [HEADING_DEG] [TOL]` walks to a floor point (it does not avoid obstacles), `walk FORWARD LEFT TURN STEPS` holds a velocity command, `turn_to HEADING_DEG`, `stand STEPS` holds the spot, `reach left|right X Y Z [TOL]` moves a hand to a world point and keeps it there while the body sways, `hands_home`, and `sit_down` (one-way: stops the walking controller and sits down where the robot stands, the pelvis ending about 0.2 m further back).

**Observation.** `robo observe` reports `robot` (`pelvis` position, `heading_deg`, `head_height`, `torso_upright` (1 = upright), `com_velocity` (world x, y) and `com_velocity_body` (forward, left), the current velocity `command`, and per hand `pos` (world), `pos_body` and `goal_body` (heading frame) and `anchored_at` (the world point a `reach` holds), `fallen`), `scene` (the task's objects and goals) and `progress`: HumanoidBench's `return` so far, `last_reward` and its `reward_terms`, `steps` of `episode_steps`, the `success_bar` for return-based tasks and `solved`. Camera images: `robo observe --image` (HumanoidBench's view for the task, following the robot).

The step budget is 1000 steps (20 s of simulated time), HumanoidBench's episode length for this task.

## How to control the robot

You are the robot's policy. You act only through the `robo` command in your shell. There is no other way to move the robot, and you cannot read or change the simulator, the scoring, or other files to succeed; the episode server judges the final physical state itself.

```
robo info                          # the robot, its sensors, action groups, skills and step budget
robo observe                       # robot and scene state as numbers
robo observe --image [--camera C]  # also saves a camera image and prints its path (open it to look)
robo act V1 V2 ... [--repeat N]    # one low-level action (the action groups under Controls), applied N times (N <= 50)
robo skill NAME ARG ...            # run a skill listed by `robo info`; it runs until it finishes and reports the result
robo done "short summary"          # end the episode and ask for scoring
robo give-up "reason"              # end the episode without claiming success
```

- Positions are in metres in the world frame (+z up); angles are in degrees unless a field says otherwise.
- The episode has a fixed step budget (see `robo info`); every simulated control step counts, including the steps a skill runs.
- Skills are ordinary controllers: they can fail, stop early or be blocked by the scene. Read what they report and re-observe.
- The episode ends as solved the moment the task's success rule holds (see Success); `robo done` before that scores 0.
- Call `robo done` exactly once when finished.
