---
schema_version: '1.3'
task:
  name: robouse/libero-10-9
  description: Put the yellow and white mug in the microwave and close it.
metadata:
  author_name: benchflow
  source_benchmark: LIBERO (Liu et al. 2023)
  source_task: KITCHEN_SCENE6_put_the_yellow_and_white_mug_in_the_microwave_and_close_it
  suite: libero
  libero_suite: libero_10
  libero_task_index: 9
  language_instruction: put the yellow and white mug in the microwave and close it
  category: manipulation
  difficulty: hard
  tags:
  - franka
  - mujoco
  - robosuite
  - single-arm
  - language
  reference_solution: LIBERO demonstration demo_1 of yifengzhu-hf/LIBERO-datasets/libero_10/KITCHEN_SCENE6_put_the_yellow_and_white_mug_in_the_microwave_and_close_it_demo.hdf5 (replay reaches success at step 238)
  robouse:
    id: libero-10-9
    backend: libero
    suite: libero_10
    env: KITCHEN_SCENE6_put_the_yellow_and_white_mug_in_the_microwave_and_close_it
    task_index: 9
    seed: 0
    max_steps: 520
    camera: agentview
    cameras:
    - agentview
    - robot0_eye_in_hand
    image_size: 256
    settle_steps: 50
    skills: true
    success_mode: first
    obs_mode: state
    init_source: yifengzhu-hf/LIBERO-datasets/libero_10/KITCHEN_SCENE6_put_the_yellow_and_white_mug_in_the_microwave_and_close_it_demo.hdf5 demo_1 states[0]
    init_state:
    - 0.25000000000000017
    - -0.01511520224238547
    - -0.09361488357491556
    - -0.013511883324915036
    - -2.414743421488261
    - -0.0017283318770111315
    - 2.2532629217581657
    - 0.8380691028897778
    - 0.03404761749512105
    - -0.03406400933438885
    - -0.11515392842759886
    - -0.2674590841253641
    - 0.8973797592155676
    - -0.7071070259802631
    - 1.3650881865112714e-05
    - 8.792909845300346e-06
    - 0.7071065362063103
    - 0.008604505126512332
    - 0.010627211992811537
    - 0.8998776320700327
    - -0.7071067048202595
    - -1.3530997068143409e-05
    - -1.4247341039620825e-05
    - 0.7071068572798315
    - -1.6071842242048775
    - 0.007882921087943317
    - 0.2513448677974318
    - -0.004286993347401373
    - 0.21452305257218532
    - 0.024169248452446754
    - 0.12419289395574552
    - 0.256559185851704
    - 0.057655296026179643
    - -0.057478406290419
    - -1.3763431906922717e-07
    - -7.617843614962316e-07
    - 0.005974378125489708
    - -5.371704278645366e-05
    - -0.0002981087289698895
    - -1.2656768125147663e-09
    - -5.655470809481746e-10
    - 1.5760740635471362e-08
    - 0.005701178829296427
    - -8.117380031701304e-06
    - 0.00029512731236127975
    - 2.2858341564713763e-10
    - 0.0
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

# Put the yellow and white mug in the microwave and close it

A Franka Panda arm with a parallel-jaw gripper works at a table in a LIBERO scene (MuJoCo / robosuite; LIBERO suite `libero_10`, task `KITCHEN_SCENE6_put_the_yellow_and_white_mug_in_the_microwave_and_close_it`). LIBERO-Long (libero_10): long-horizon tasks, most with two sub-goals.

**Goal (LIBERO's language instruction):** put the yellow and white mug in the microwave and close it.

The episode ends as solved the moment LIBERO's own success check (the task's goal predicates) passes; you do not need to call `robo done` after that. If you finish without the check passing, call `robo done` (or `robo give-up`).

## This robot

- The action has 7 numbers, not 4: `robo act DX DY DZ DROLL DPITCH DYAW GRIPPER`, each in [-1, 1]. It is LIBERO's delta end-effector command (robosuite's OSC_POSE controller, 20 steps per second). Holding DX/DY/DZ at 1.0 moves the hand about 1 cm per step; DROLL/DPITCH/DYAW rotate the hand (1.0 asks for 0.5 rad; usually leave them at 0). GRIPPER -1 opens, +1 closes.
- `robo move-to X Y Z [--grip G]` and `robo grip G` are available (they keep the hand's orientation).
- World frame, metres: +x points forward, away from the robot's base; +y is to the robot's left; +z is up. The table surface is at about z = 0.90.
- `robo observe` fields: `hand_pos` (end-effector position), `eef_quat` (orientation quaternion x, y, z, w) and `eef_axis_angle`, `gripper_qpos` (the two finger joints), `gripper_open` (finger gap in metres, about 0.08 when fully open), and `objects`: the position and quaternion of every object in the scene, under LIBERO's names (e.g. `akita_black_bowl_1`, `plate_1`; `..._1`, `..._2` number identical objects); `fixtures`: the position and quaternion of each piece of furniture (cabinet, stove, microwave, rack, shelf, ...); `articulated`: each movable part of a fixture (drawer slide, door hinge, stove knob), keyed by its joint name (e.g. `wooden_cabinet_1_top_level` is the cabinet's top drawer), with `qpos` (joint position), `range` (joint limits), LIBERO's thresholds for that fixture (`open_ranges` / `close_ranges`, or `turnon_ranges` / `turnoff_ranges` for a stove knob: the joint positions at which LIBERO counts the part as open / closed or on / off) and `box_min` / `box_max` (the world-frame bounding box of the moving part).
- `robo observe --image` saves the front camera (`agentview`); add `--camera robot0_eye_in_hand` for the wrist camera. Both are 256x256.
- The step budget is 520 steps (LeRobot's LIBERO evaluation budget for this suite).

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
