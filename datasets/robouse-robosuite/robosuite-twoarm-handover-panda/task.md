---
schema_version: '1.3'
task:
  name: robouse/robosuite-twoarm-handover-panda
  description: Robot0 picks up a hammer and hands it to robot1.
metadata:
  author_name: benchflow
  source_benchmark: robosuite (Zhu et al.)
  source_task: TwoArmHandover
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
  success_rule: robosuite TwoArmHandover._check_success()
  success_mode_reason: 'The handover must be complete and stable: success is judged after `robo done` and a 10-step settle, so robot1 must really hold the hammer by its handle with robot0 clear of it.'
  reference_solution: 'scripted end-effector controller: robot0 grasps its end of the handle and brings the hammer to the middle, robot1 grasps the other end from above, robot0 releases and backs away'
  robouse:
    id: robosuite-twoarm-handover-panda
    backend: robosuite
    env: TwoArmHandover
    robots:
    - Panda
    - Panda
    env_configuration: opposed
    controller: BASIC
    seed: 2
    max_steps: 700
    camera: frontview
    cameras:
    - frontview
    - robot0_eye_in_hand
    image_size: 320
    skills: false
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

# Hand over the hammer (two arms)

Two Franka Emika Panda (7-DoF) with its parallel-jaw gripper arms face each other across a table: robot0 on the -y side, robot1 on the +y side. The scene is robosuite's `TwoArmHandover` environment (MuJoCo).

**Goal:** The hammer lies on the table on robot0's side. Robot0 picks it up and hands it to robot1. robosuite's TwoArmHandover check: robot1 grasps the hammer's handle, robot0 no longer touches the hammer, and the hammer is more than 10 cm above the table. This must still hold after `robo done`.

Call `robo done` when finished; the check is made after the robot then holds still for 10 steps, so the result must last. 

## This robot

- The action has 14 numbers: robot0's `DX DY DZ DROLL DPITCH DYAW GRIP`, then robot1's, each in [-1, 1]: robosuite's OSC_POSE end-effector commands plus the gripper, 20 steps per second, all in the world frame (the same frame for both robots). DX/DY/DZ = 1.0 asks for a 5 cm move (held at 1.0 a hand travels about 1.1 cm per step); DROLL/DPITCH/DYAW rotate that hand about the world x/y/z axes (1.0 asks for 0.5 rad); GRIP +1 closes, -1 opens, 0 keeps the fingers as they are (they move over about 5 steps).
- `robo move-to` and `robo grip` are not available (two robots); use `robo act`.
- World frame, metres: +x points away from the table edge between the robots, +y to the robot1 side, +z up.
- `robo observe` fields: for each robot i in 0, 1: `roboti_hand_pos`, `roboti_hand_quat` (x, y, z, w), and, if it has a gripper, `roboti_gripper_open` (finger-pad distance, metres) and `roboti_gripper_yaw_deg` (direction of the fingertip line in the table plane, degrees from +x, in [-90, 90)); `hammer_pos` (centre of the handle), `hammer_quat`, `hammer_yaw_deg`, `handle_pos` (same point), `hammer_handle_axis` (unit vector along the handle, pointing toward the head), `hammer_head_pos`, `hammer_handle_length` (m); `table_height`.
- `robo observe --image` saves the `frontview` camera; add `--camera robot0_eye_in_hand` for the wrist camera. Images are 320x320.
- The step budget is 700 steps.
- Robot0 stands at -y and robot1 at +y. Both robots must hold the handle at the same time for a moment, so leave room for two hands on it. Close the fingers across the handle. Move robot1 in from above, not through the hammer.

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
