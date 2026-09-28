---
schema_version: '1.3'
task:
  name: robouse/robosuite-pickplace-can-panda
  description: Move the can from the bin on the robot's right into its compartment of the divided bin on its left.
metadata:
  author_name: benchflow
  source_benchmark: robosuite (Zhu et al.)
  source_task: PickPlaceCan
  suite: robosuite
  category: manipulation
  difficulty: medium
  tags:
  - panda
  - mujoco
  - robosuite
  - single-arm
  robots:
  - Panda
  success_rule: robosuite PickPlaceCan._check_success()
  success_mode_reason: 'A placement must last: success is judged after `robo done` and a 10-step settle, with the robot holding still, so the object has to stay where it was put (and robosuite also requires the gripper to have let go and moved away).'
  reference_solution: 'scripted end-effector controller: grasp the can from above, carry it over the target compartment at a safe height, lower, release, retreat'
  robouse:
    id: robosuite-pickplace-can-panda
    backend: robosuite
    env: PickPlaceCan
    robots: Panda
    controller: BASIC
    seed: 0
    max_steps: 400
    camera: agentview
    cameras:
    - agentview
    - robot0_eye_in_hand
    image_size: 320
    skills: true
    success_mode: final
    obs_mode: state
agent:
  timeout_sec: 1200
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

# Put the can in its bin (Panda)

A Franka Emika Panda (7-DoF) with its parallel-jaw gripper works at a table; its base is at the -x end, facing +x. The scene is robosuite's `PickPlaceCan` environment (MuJoCo).

**Goal:** Pick up the soda can from the bin on the robot's right (bin 1, at -y) and put it into its target compartment of the divided bin on the robot's left (bin 2, at +y). robosuite's PickPlace check: the can's centre lies inside the target compartment (x inside `target_bin_x_range`, y inside `target_bin_y_range`, z between `bin_floor_z` and `bin_floor_z` + 0.1) and the gripper is at least about 4 cm away from it.

Call `robo done` when finished; the check is made after the robot then holds still for 10 steps, so the result must last. 

## This robot

- The action has 7 numbers, not 4: `robo act DX DY DZ DROLL DPITCH DYAW GRIP`, each in [-1, 1]: robosuite's OSC_POSE end-effector command plus the gripper, 20 steps per second, in the world frame. DX/DY/DZ = 1.0 asks for a 5 cm move (held at 1.0 the hand travels about 1.1 cm per step); DROLL/DPITCH/DYAW rotate the hand about the world x/y/z axes (1.0 asks for 0.5 rad; leave them at 0 unless you need to turn the hand, e.g. DYAW to line the fingers up with an object); GRIP +1 closes, -1 opens, 0 keeps the fingers as they are (they move over about 5 steps).
- `robo move-to X Y Z [--grip G]` and `robo grip G` are available; they move only the position and keep the hand's orientation (it can drift slowly under load; correct it with DROLL/DPITCH/DYAW).
- World frame, metres: +x points away from the robot's base, +y to the robot's left, +z up.
- `robo observe` fields: `hand_pos` (the gripper's grasp point between the fingertips, metres), `hand_quat` (hand orientation quaternion x, y, z, w; about [1, 0, 0, 0] when the gripper points straight down), `gripper_open` (distance between the two finger pads in metres: largest when open, smallest when closed on nothing, in between when holding something), `gripper_yaw_deg` (direction of the line through the two fingertips in the table plane, degrees from +x, in [-90, 90); the fingers close along this line); `can_pos`, `can_quat`, `can_yaw_deg` (the can; a cylinder about 5 cm wide and 12 cm tall); `target_bin_center`, `target_bin_x_range`, `target_bin_y_range` (the target compartment), `bin_floor_z` (z of the bins' floor). Every `<object>_quat` (x, y, z, w) comes with `<object>_yaw_deg`, the object's heading about the vertical axis (its local x axis, degrees from +x).
- `robo observe --image` saves the `agentview` camera; add `--camera robot0_eye_in_hand` for the wrist camera. Images are 320x320.
- The step budget is 400 steps.
- Lift the object well above the bin walls (z around 1.05 or higher) before carrying it over; release it low inside the compartment, then move the hand up and away before `robo done`.

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
