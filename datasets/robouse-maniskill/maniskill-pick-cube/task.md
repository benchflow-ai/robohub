---
schema_version: '1.3'
task:
  name: robouse/maniskill-pick-cube
  description: Pick up the cube and hold it at the goal (ManiSkill3 PickCube-v1, Franka Panda, CPU physics).
metadata:
  author_name: benchflow
  source_benchmark: ManiSkill3 (Hao Su Lab / Hillbot; mani-skill 3.0.1, SAPIEN 3.0.3)
  source_task: PickCube-v1
  suite: maniskill
  category: manipulation
  difficulty: easy
  tags:
  - sapien
  - maniskill
  - panda
  - tabletop
  - pick-place
  success_rule: ManiSkill PickCube-v1 evaluate()['success']
  reference_solution: 'scripted end-effector controller: grasp the cube across two faces, carry it to the goal and correct with the observed cube position'
  robouse:
    id: maniskill-pick-cube
    backend: maniskill
    env: maniskill-pick-cube
    seed: 1
    max_steps: 300
    camera: render_camera
    cameras:
    - render_camera
    - base_camera
    skills: true
    success_mode: final
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

# Pick up the cube and hold it at the goal (ManiSkill3 PickCube-v1)

A Franka Emika Panda arm (7 joints, two-finger parallel gripper that opens to 8 cm) stands at a wooden table in ManiSkill3's `PickCube-v1` task, simulated with SAPIEN 3 (PhysX). The robot's base is at x = -0.615, y = 0 and faces +x, so +x points away from the robot and +y to its left. The table top is the plane z = 0.

**Goal:** Pick up the red cube (4 cm) and hold it so that its centre is at the goal point `goal_pos`, which floats in the air above the table (the green sphere in the images).

**Success:** ManiSkill's own PickCube check: the cube's centre is within 2.5 cm of `goal_pos` and the robot is static (every arm joint slower than 0.2 rad/s). Keep holding the cube at the goal when you call `robo done`. Success is judged by the episode server after you call `robo done` and the robot has held still for about 10 steps, so the result must last.

**Task fields in `robo observe`:** `cube_pos`, `cube_quat`, `cube_yaw_deg`, `cube_half_size` (0.02 m), `goal_pos`, `goal_radius` (0.025 m) and `grasped`.

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
