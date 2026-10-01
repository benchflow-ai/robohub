---
schema_version: '1.3'
task:
  name: robouse/robosuite-twoarm-peginhole-panda
  description: One robot holds a peg, the other a plate with a hole; get the peg through the hole.
metadata:
  author_name: benchflow
  source_benchmark: robosuite (Zhu et al.)
  source_task: TwoArmPegInHole
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
  success_rule: robosuite TwoArmPegInHole._check_success()
  success_mode_reason: 'The peg must stay through the hole: success is judged after `robo done` and a 10-step settle with both arms holding still.'
  reference_solution: 'scripted end-effector controller: both arms turn about 45 degrees to a shared axis, then robot0 lines the peg up and pushes it through'
  robouse:
    id: robosuite-twoarm-peginhole-panda
    backend: robosuite
    env: TwoArmPegInHole
    robots:
    - Panda
    - Panda
    env_configuration: opposed
    controller: BASIC
    seed: 0
    max_steps: 500
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
oracle:
  env:
    ROBOUSE_ORACLE_TOKEN: ${ROBOUSE_ORACLE_TOKEN:-}
---

You are controlling a simulated robot. Read the task below, then solve it by running the `robo` command in your shell (start with `robo info` and `robo observe`). Keep going until the task is done, then call `robo done` once. Do not stop to ask questions; there is no human to answer.

# Put the peg through the hole (two arms)

Two Franka Emika Panda (7-DoF) with its parallel-jaw gripper arms face each other across a table: robot0 on the -y side, robot1 on the +y side. The scene is robosuite's `TwoArmPegInHole` environment (MuJoCo).

**Goal:** Robot0 holds a peg (a cylinder fixed to its hand) and robot1 holds a square plate with a hole (fixed to its hand). Bring them together so the peg passes through the hole. robosuite's TwoArmPegInHole check: the peg's axis is nearly parallel to the hole's axis (|cos| > 0.95), the peg's centre is within 6 cm of the hole's axis, and it sits between 12 cm in front of and 14 cm behind the hole. Both must still hold after `robo done`.

Call `robo done` when finished; the check is made after the robot then holds still for 10 steps, so the result must last. 

## This robot

- The action has 12 numbers: robot0's `DX DY DZ DROLL DPITCH DYAW`, then robot1's, each in [-1, 1]: robosuite's OSC_POSE end-effector commands, 20 steps per second, all in the world frame. DX/DY/DZ = 1.0 asks for a 5 cm move (held at 1.0 a hand travels about 1.1 cm per step); DROLL/DPITCH/DYAW rotate that hand about the world x/y/z axes (1.0 asks for 0.5 rad).
- `robo move-to` and `robo grip` are not available (two robots); use `robo act`.
- World frame, metres: +x points away from the table edge between the robots, +y to the robot1 side, +z up.
- `robo observe` fields: for each robot i in 0, 1: `roboti_hand_pos`, `roboti_hand_quat` (x, y, z, w), and, if it has a gripper, `roboti_gripper_open` (finger-pad distance, metres) and `roboti_gripper_yaw_deg` (direction of the fingertip line in the table plane, degrees from +x, in [-90, 90)) (neither robot has a gripper); `peg_pos` (centre of the peg), `peg_axis` (unit vector along the peg), `peg_quat`, `hole_pos`, `hole_quat`, `hole_center` (centre of the hole), `hole_axis` (unit normal of the plate through the hole), and robosuite's own alignment numbers: `peg_hole_cos` (|cos| of the angle between the two axes), `peg_hole_d` (distance of the peg's centre from the hole's axis, m), `peg_hole_t` (position of the hole's centre along the peg's axis relative to the peg's centre, m).
- `robo observe --image` saves the `frontview` camera; add `--camera robot0_eye_in_hand` for the wrist camera. Images are 320x320.
- The step budget is 500 steps.
- The action has 12 numbers: 6 for robot0, then 6 for robot1 (no grippers). At the start the peg points down and the plate stands upright, so the axes are about 90 degrees apart; turning one wrist the whole way is hard, so share the turn between the two arms.

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
