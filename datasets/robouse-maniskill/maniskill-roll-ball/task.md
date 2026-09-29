---
schema_version: '1.3'
task:
  name: robouse/maniskill-roll-ball
  description: Roll the ball into the goal region (ManiSkill3 RollBall-v1, Franka Panda, CPU physics).
metadata:
  author_name: benchflow
  source_benchmark: ManiSkill3 (Hao Su Lab / Hillbot; mani-skill 3.0.1, SAPIEN 3.0.3)
  source_task: RollBall-v1
  suite: maniskill
  category: manipulation
  difficulty: medium
  tags:
  - sapien
  - maniskill
  - panda
  - tabletop
  - dynamic
  - non-prehensile
  success_rule: ManiSkill RollBall-v1 evaluate()['success']
  reference_solution: 'scripted end-effector controller: lower the closed fingers behind the ball on the line to the goal and push it through its centre at a steady speed'
  robouse:
    id: maniskill-roll-ball
    backend: maniskill
    env: maniskill-roll-ball
    seed: 1
    max_steps: 300
    camera: render_camera
    cameras:
    - render_camera
    - base_camera
    skills: true
    success_mode: first
    obs_mode: state
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

# Roll the ball into the goal region (ManiSkill3 RollBall-v1)

A Franka Emika Panda arm (7 joints, two-finger parallel gripper that opens to 8 cm) stands at a wooden table in ManiSkill3's `RollBall-v1` task, simulated with SAPIEN 3 (PhysX). In this task the robot's base is at x = -0.1, y = 1.0 and faces -y (toward the far end of the table), so the ball has to be sent in the -y direction, away from the robot. The table top is the plane z = 0.

**Goal:** Push the blue ball (3.5 cm radius) so that it rolls across the table into the red-and-white target region (radius 10 cm) centred at `goal_pos`, more than a metre away, far out of the arm's reach.

**Success:** ManiSkill's own RollBall check: the ball's centre is within 10 cm of the target centre in the table plane. This task uses success mode `first`: the episode ends as solved the moment the ball is over the target (it may roll on afterwards), matching ManiSkill's own success-once evaluation.

**Task fields in `robo observe`:** `ball_pos`, `ball_quat`, `ball_radius` (0.035 m), `ball_vel` (m/s), `goal_pos` and `goal_radius` (0.1 m).

**Controls.** The action has 7 numbers, not 4: `robo act DX DY DZ DROLL DPITCH DYAW GRIP`, each in [-1, 1], in the WORLD frame. DX/DY/DZ move the gripper's commanded position by 2 cm per unit per step. DROLL/DPITCH/DYAW turn its commanded orientation by 0.1 rad (5.7 degrees) per unit per step about the world x, y and z axes, around the point between the fingertips; leave them at 0 unless you need to turn the hand (for example DYAW to line the fingers up with an object's faces). GRIP: any positive value closes the fingers, a negative value -f opens them to fraction f (-1 fully open), 0 keeps them as they are (they take about 5 steps to move). One step is 50 ms (ManiSkill's 20 Hz control rate). The arm follows the commanded pose through inverse kinematics and ManiSkill's joint position controller; the commanded position stays within 5 cm of the measured hand (and the orientation within 0.35 rad), so pushing against something stalls instead of winding up. Some orientations are out of the arm's reach (the last wrist joint turns about 166 degrees each way): turning the fingers by 90 or 180 degrees about the vertical gives the same grasp on a cube, so prefer the smaller turn. `robo move-to X Y Z [--grip G]` and `robo grip G` also work; they move only the position and keep the hand's orientation, and `move-to` stops within 1 cm of the point unless you pass a tighter `--tol` (e.g. `--tol 0.002`).

**Observation.** `robo observe` reports `hand_pos` (the point between the fingertips, metres), `hand_quat` (hand orientation quaternion w, x, y, z), `hand_down_axis` (unit vector the fingers point along; [0, 0, -1] is straight down), `finger_axis` (unit vector along which the fingers close), `hand_target_pos` and `hand_target_quat` (the commanded pose the arm is moving to), `gripper_open` (0 closed .. 1 fully open) and `gripper_width` (distance between the fingers, metres), plus the task fields below. Every `<object>_quat` is (w, x, y, z); `<object>_yaw_deg` is the object's heading about the vertical (its local x axis, degrees from +x). `grasped`, where present, is ManiSkill's own grasp test (both fingers pressing on the object). `robo observe --image` saves a 384x384 picture from the `render_camera` view; `--camera base_camera` gives other views.

The step budget is 300 steps (15 s of simulated time).

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
