---
schema_version: '1.3'
task:
  name: robouse/libero-90-34
  description: Put the yellow and white mug to the front of the white mug.
metadata:
  author_name: benchflow
  source_benchmark: LIBERO (Liu et al. 2023)
  source_task: KITCHEN_SCENE6_put_the_yellow_and_white_mug_to_the_front_of_the_white_mug
  suite: libero-90
  libero_suite: libero_90
  libero_task_index: 34
  language_instruction: put the yellow and white mug to the front of the white mug
  category: manipulation
  difficulty: medium
  tags:
  - franka
  - mujoco
  - robosuite
  - single-arm
  - language
  reference_solution: LIBERO demonstration demo_0 of yifengzhu-hf/LIBERO-datasets/libero_90/KITCHEN_SCENE6_put_the_yellow_and_white_mug_to_the_front_of_the_white_mug_demo.hdf5 (replay reaches success at step 104)
  robouse:
    id: libero-90-34
    backend: libero
    suite: libero_90
    env: KITCHEN_SCENE6_put_the_yellow_and_white_mug_to_the_front_of_the_white_mug
    task_index: 34
    seed: 0
    max_steps: 400
    camera: agentview
    cameras:
    - agentview
    - robot0_eye_in_hand
    image_size: 256
    settle_steps: 50
    skills: true
    success_mode: first
    obs_mode: state
    init_source: yifengzhu-hf/LIBERO-datasets/libero_90/KITCHEN_SCENE6_put_the_yellow_and_white_mug_to_the_front_of_the_white_mug_demo.hdf5 demo_0 states[0]
    init_state:
    - 0.25000000000000017
    - -0.021377782372251682
    - -0.15937733893537748
    - -0.005209546377118643
    - -2.4362322820876066
    - -0.0017155427180420375
    - 2.2167963592292996
    - 0.7709933187769621
    - 0.03404840983247482
    - -0.03406329730173067
    - -0.09616363206702386
    - -0.2615669125392535
    - 0.8973797592155676
    - -0.7071070259802631
    - 1.365088186511185e-05
    - 8.792909845300809e-06
    - 0.7071065362063103
    - -0.02130478179450604
    - -0.0003640296430381788
    - 0.8998776320700327
    - -0.7071067048202595
    - -1.3530997068139127e-05
    - -1.4247341039606506e-05
    - 0.7071068572798315
    - -1.3695305012467125
    - 0.017949842065866462
    - 0.014328073567319525
    - -0.017322381015074606
    - 0.007666088305678341
    - -1.7110451030901335e-05
    - 0.018463099936254185
    - 0.04112620171128436
    - 0.057650480336088425
    - -0.057482469085641814
    - -1.3763431906658647e-07
    - -7.617843614924596e-07
    - 0.005974378125489711
    - -5.371704278640558e-05
    - -0.0002981087289698188
    - -1.2656768120411089e-09
    - -5.655470836659948e-10
    - 1.5760740630218567e-08
    - 0.005701178829296483
    - -8.117380028845263e-06
    - 0.000295127312358425
    - 2.2858341510241337e-10
    - 0.0
agent:
  timeout_sec: 1500
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

# Put the yellow and white mug to the front of the white mug

A Franka Panda arm with a parallel-jaw gripper works at a table in a LIBERO scene (MuJoCo / robosuite; LIBERO suite `libero_90`, task `KITCHEN_SCENE6_put_the_yellow_and_white_mug_to_the_front_of_the_white_mug`). LIBERO-90: 90 short-horizon tasks in 20 kitchen, living-room and study scenes (the task name starts with the scene); LIBERO uses them for pretraining.

**Goal (LIBERO's language instruction):** put the yellow and white mug to the front of the white mug.

The episode ends as solved the moment LIBERO's own success check (the task's goal predicates) passes; you do not need to call `robo done` after that. If you finish without the check passing, call `robo done` (or `robo give-up`).

## This robot

- The action has 7 numbers, not 4: `robo act DX DY DZ DROLL DPITCH DYAW GRIPPER`, each in [-1, 1]. It is LIBERO's delta end-effector command (robosuite's OSC_POSE controller, 20 steps per second). Holding DX/DY/DZ at 1.0 moves the hand about 1 cm per step; DROLL/DPITCH/DYAW rotate the hand (1.0 asks for 0.5 rad; usually leave them at 0). GRIPPER -1 opens, +1 closes.
- `robo move-to X Y Z [--grip G]` and `robo grip G` are available (they keep the hand's orientation).
- World frame, metres: +x points forward, away from the robot's base; +y is to the robot's left; +z is up. The table surface is at about z = 0.90.
- `robo observe` fields: `hand_pos` (end-effector position), `eef_quat` (orientation quaternion x, y, z, w) and `eef_axis_angle`, `gripper_qpos` (the two finger joints), `gripper_open` (finger gap in metres, about 0.08 when fully open), and `objects`: the position and quaternion of every object in the scene, under LIBERO's names (e.g. `akita_black_bowl_1`, `plate_1`; `..._1`, `..._2` number identical objects); `fixtures`: the position and quaternion of each piece of furniture (cabinet, stove, microwave, rack, shelf, ...); `articulated`: each movable part of a fixture (drawer slide, door hinge, stove knob), keyed by its joint name (e.g. `wooden_cabinet_1_top_level` is the cabinet's top drawer), with `qpos` (joint position), `range` (joint limits), LIBERO's thresholds for that fixture (`open_ranges` / `close_ranges`, or `turnon_ranges` / `turnoff_ranges` for a stove knob: the joint positions at which LIBERO counts the part as open / closed or on / off) and `box_min` / `box_max` (the world-frame bounding box of the moving part).
- `robo observe --image` saves the front camera (`agentview`); add `--camera robot0_eye_in_hand` for the wrist camera. Both are 256x256.
- The step budget is 400 steps (LeRobot's LIBERO evaluation budget for this suite).

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
