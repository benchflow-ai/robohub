---
schema_version: '1.3'
task:
  name: robouse/gymrobotics-fetch-push-s1
  description: Push the block across the table so it rests on the goal position.
metadata:
  author_name: benchflow
  source_benchmark: Gymnasium-Robotics (Farama)
  source_task: FetchPush-v4
  suite: gymrobotics
  category: manipulation
  difficulty: medium
  tags:
  - fetch
  - mujoco
  - single-arm
  - push
  robouse:
    id: gymrobotics-fetch-push-s1
    backend: gymrobotics
    env: FetchPush-v4
    seed: 1
    max_steps: 300
    camera: default
    skills: true
    success_mode: final
agent:
  timeout_sec: 900
verifier:
  service: simulator
  user: root
  timeout_sec: 300
sandbox:
  cpus: 1
  memory_mb: 2048
  build_timeout_sec: 3600
oracle:
  env:
    ROBOUSE_ORACLE_TOKEN: ${ROBOUSE_ORACLE_TOKEN:-}
---

You are controlling a simulated robot. Read the task below, then solve it by running the `robo` command in your shell (start with `robo info` and `robo observe`). Keep going until the task is done, then call `robo done` once. Do not stop to ask questions; there is no human to answer.

# Fetch Push (seed 1)

A Fetch robot arm with a two-finger parallel gripper stands at a table (adapted from Gymnasium-Robotics, `FetchPush-v4`, layout seed 1).

**Goal:** Push the block across the table so it rests on the goal position.

Success: the block centre (`obj1_pos`) is within 5 cm of `goal_pos`, judged after you call `robo done` and the robot has held still for about 10 steps.

In `robo observe`: `hand_pos`/`hand_vel` are the gripper position (m) and velocity (m/s), `gripper_open` is the finger opening (0 = closed, 1 = fully open) and `gripper_width` the gap in metres, `obj1_pos`, `obj1_vel` and `obj1_rot` (Euler angles, rad) describe the block, and `goal_pos` is the target (shown as a red sphere in camera images). The block is a 5 cm cube.

The action `robo act DX DY DZ GRIP` moves the gripper by up to 5 cm per step along world x/y/z (DX = 1 means 5 cm); GRIP +1 closes the fingers and -1 opens them. The gripper always points down.

## How to control the robot

You are the robot's policy. You act only through the `robo` command in your shell. There is no other way to move the robot, and you cannot read or change the simulator, the scoring, or other files to succeed; the episode server judges the final physical state itself.

```
robo info                         # action space, available skills, step budget
robo observe                      # robot and object state as numbers
robo observe --image              # also saves a camera image and prints its path (open it to look)
robo act DX DY DZ GRIP [--repeat N]   # low-level action, applied N times (N <= 50)
robo move-to X Y Z [--grip G]     # skill: move the gripper toward a point (if enabled for this task)
robo grip G [--steps N]           # skill: hold position and set the gripper (+1 close, -1 open)
robo done "short summary"         # end the episode and ask for scoring
robo give-up "reason"             # end the episode without claiming success
```

- Positions are in metres in the world frame (x, y on the table plane, z up).
- The episode has a fixed step budget (see `robo info`); every simulated step counts, including skills.
- Unless the task says otherwise, success is judged about 10 steps after you call `robo done`, with the robot holding still, so the goal must still be true when the robot stops.
- Work in small steps and re-observe after each motion. Call `robo done` exactly once when finished.
