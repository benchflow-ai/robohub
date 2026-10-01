---
schema_version: '1.3'
task:
  name: robouse/robosuite-twoarm-lift-panda
  description: Two robots, facing each other across the table, lift a pot by its two handles.
metadata:
  author_name: benchflow
  source_benchmark: robosuite (Zhu et al.)
  source_task: TwoArmLift
  suite: robosuite
  category: bimanual
  difficulty: hard
  tags:
  - panda
  - mujoco
  - robosuite
  - two-arm
  robots:
  - Panda
  - Panda
  success_rule: robosuite TwoArmLift._check_success()
  success_mode_reason: 'Lifting is a transient act: robosuite''s evaluation (robomimic rollouts) ends an episode at the first step where the object is above the threshold, so the episode is scored the same way.'
  reference_solution: 'scripted end-effector controller: both arms grasp their handles from above, then rise together'
  robouse:
    id: robosuite-twoarm-lift-panda
    backend: robosuite
    env: TwoArmLift
    robots:
    - Panda
    - Panda
    env_configuration: opposed
    controller: BASIC
    seed: 0
    max_steps: 400
    camera: frontview
    cameras:
    - frontview
    - robot0_eye_in_hand
    image_size: 320
    skills: false
    success_mode: first
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
oracle:
  env:
    ROBOUSE_ORACLE_TOKEN: ${ROBOUSE_ORACLE_TOKEN:-}
---

You are controlling a simulated robot. Read the task below, then solve it by running the `robo` command in your shell (start with `robo info` and `robo observe`). Keep going until the task is done, then call `robo done` once. Do not stop to ask questions; there is no human to answer.

# Lift the pot with two arms (Panda)

Two Franka Emika Panda (7-DoF) with its parallel-jaw gripper arms face each other across a table: robot0 on the -y side, robot1 on the +y side. The scene is robosuite's `TwoArmLift` environment (MuJoCo).

**Goal:** Lift the pot together: robot0 holds `handle0`, robot1 holds `handle1`. robosuite's TwoArmLift check: the pot's bottom is more than 10 cm above the table. The episode ends by itself at that moment.

The episode ends as solved the moment the check first passes (you do not need to call `robo done` then). 

## This robot

- The action has 14 numbers: robot0's `DX DY DZ DROLL DPITCH DYAW GRIP`, then robot1's, each in [-1, 1]: robosuite's OSC_POSE end-effector commands plus the gripper, 20 steps per second, all in the world frame (the same frame for both robots). DX/DY/DZ = 1.0 asks for a 5 cm move (held at 1.0 a hand travels about 1.1 cm per step); DROLL/DPITCH/DYAW rotate that hand about the world x/y/z axes (1.0 asks for 0.5 rad); GRIP +1 closes, -1 opens, 0 keeps the fingers as they are (they move over about 5 steps).
- `robo move-to` and `robo grip` are not available (two robots); use `robo act`.
- World frame, metres: +x points away from the table edge between the robots, +y to the robot1 side, +z up.
- `robo observe` fields: for each robot i in 0, 1: `roboti_hand_pos`, `roboti_hand_quat` (x, y, z, w), and, if it has a gripper, `roboti_gripper_open` (finger-pad distance, metres) and `roboti_gripper_yaw_deg` (direction of the fingertip line in the table plane, degrees from +x, in [-90, 90)); `pot_pos`, `pot_quat`, `pot_yaw_deg`; `handle0_pos` (robot0's handle), `handle1_pos` (robot1's handle); `table_height`. Every `<object>_quat` (x, y, z, w) comes with `<object>_yaw_deg`, the object's heading about the vertical axis (its local x axis, degrees from +x).
- `robo observe --image` saves the `frontview` camera; add `--camera robot0_eye_in_hand` for the wrist camera. Images are 320x320.
- The step budget is 400 steps.
- Each handle is a bar; close the fingers across it (the fingertip line along the direction from the pot's centre to that handle). Lift both hands at the same rate, or the pot tips.

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
