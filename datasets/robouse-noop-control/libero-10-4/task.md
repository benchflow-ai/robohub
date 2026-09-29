---
schema_version: '1.3'
task:
  name: robouse/libero-10-4
  description: Put the white mug on the left plate and put the yellow and white mug on the right plate.
metadata:
  author_name: benchflow
  source_benchmark: LIBERO (Liu et al. 2023)
  source_task: LIVING_ROOM_SCENE5_put_the_white_mug_on_the_left_plate_and_put_the_yellow_and_white_mug_on_the_right_plate
  suite: libero
  libero_suite: libero_10
  libero_task_index: 4
  language_instruction: put the white mug on the left plate and put the yellow and white mug on the right plate
  category: manipulation
  difficulty: hard
  tags:
  - franka
  - mujoco
  - robosuite
  - single-arm
  - language
  reference_solution: LIBERO demonstration demo_0 of yifengzhu-hf/LIBERO-datasets/libero_10/LIVING_ROOM_SCENE5_put_the_white_mug_on_the_left_plate_and_put_the_yellow_and_white_mug_on_the_right_plate_demo.hdf5 (replay reaches success at step 226)
  robouse:
    id: libero-10-4
    backend: libero
    suite: libero_10
    env: LIVING_ROOM_SCENE5_put_the_white_mug_on_the_left_plate_and_put_the_yellow_and_white_mug_on_the_right_plate
    task_index: 4
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
    init_source: yifengzhu-hf/LIBERO-datasets/libero_10/LIVING_ROOM_SCENE5_put_the_white_mug_on_the_left_plate_and_put_the_yellow_and_white_mug_on_the_right_plate_demo.hdf5 demo_0 states[0]
    init_state:
    - 0.25000000000000017
    - -0.014962399443102205
    - -0.16285437361493457
    - -0.0068914139638271785
    - -2.4568427747985795
    - -0.009460602583113945
    - 2.228280764843927
    - 0.7736865932537708
    - 0.034027630380334226
    - -0.03408367045873427
    - -0.11082543193228861
    - -0.1259300523272762
    - 0.4343699488393418
    - -0.7071069013731585
    - 1.0172763262848549e-07
    - 6.675731368089582e-08
    - 0.7071066609999057
    - -0.22408579609035442
    - -0.014928788922846253
    - 0.4385989872647111
    - -0.7071068579899178
    - -5.528613175183194e-07
    - 3.83119974127542e-07
    - 0.707106704382849
    - -0.0709987100511367
    - 0.09507135180989548
    - 0.43685696889004044
    - -0.7071067450257308
    - -9.863721141872602e-07
    - 5.538107392888178e-08
    - 0.7071068173466724
    - -0.01796758302750207
    - -0.300587880951472
    - 0.4392727869250624
    - 0.7071068416649496
    - 1.2253611784922874e-06
    - -6.586728017297573e-07
    - 0.7071067207067718
    - 0.00010091268754508765
    - 0.31500000274230583
    - 0.4392727869250624
    - 0.7071068416649496
    - 1.2253611785020715e-06
    - -6.586728017133175e-07
    - 0.7071067207067718
    - 0.00487912977810694
    - -0.010200285941482371
    - -0.03524693663471774
    - -0.01943256427537588
    - 0.0007245411328826736
    - 0.020372598400933143
    - -0.02954721767186813
    - 0.05769816652193185
    - -0.05742580043172106
    - -7.114629464813605e-11
    - -4.5271501574383037e-10
    - 1.1368093065897208e-06
    - -2.8896237771275514e-08
    - -1.8466801290058093e-07
    - -1.0830534240632493e-12
    - 8.910641575931936e-12
    - -1.8765529134496854e-10
    - 1.8430481127017043e-06
    - -4.9171860601492894e-09
    - 1.0451616665163141e-07
    - -2.772296163430936e-13
    - 6.847141487852879e-13
    - -7.276785008265917e-12
    - 2.298245667213222e-06
    - -1.7375212798310834e-08
    - 1.291237281954364e-07
    - -3.260116823731078e-13
    - 1.8627973545444986e-10
    - 1.1309285628620214e-11
    - 2.7645383597014605e-06
    - 7.554087240711004e-08
    - 4.563006472135172e-09
    - 1.7231478663488665e-12
    - 1.862797321368044e-10
    - 1.1309284050694633e-11
    - 2.764538359678934e-06
    - 7.554087117756521e-08
    - 4.563005993246664e-09
    - 1.7231479977979188e-12
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

# Put the white mug on the left plate and put the yellow and white mug on the right plate

A Franka Panda arm with a parallel-jaw gripper works at a low table in a LIBERO scene (MuJoCo / robosuite; LIBERO suite `libero_10`, task `LIVING_ROOM_SCENE5_put_the_white_mug_on_the_left_plate_and_put_the_yellow_and_white_mug_on_the_right_plate`). LIBERO-Long (libero_10): long-horizon tasks, most with two sub-goals.

**Goal (LIBERO's language instruction):** put the white mug on the left plate and put the yellow and white mug on the right plate.

The episode ends as solved the moment LIBERO's own success check (the task's goal predicates) passes; you do not need to call `robo done` after that. If you finish without the check passing, call `robo done` (or `robo give-up`).

## This robot

- The action has 7 numbers, not 4: `robo act DX DY DZ DROLL DPITCH DYAW GRIPPER`, each in [-1, 1]. It is LIBERO's delta end-effector command (robosuite's OSC_POSE controller, 20 steps per second). Holding DX/DY/DZ at 1.0 moves the hand about 1 cm per step; DROLL/DPITCH/DYAW rotate the hand (1.0 asks for 0.5 rad; usually leave them at 0). GRIPPER -1 opens, +1 closes.
- `robo move-to X Y Z [--grip G]` and `robo grip G` are available (they keep the hand's orientation).
- World frame, metres: +x points forward, away from the robot's base; +y is to the robot's left; +z is up. The table surface is at about z = 0.43.
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
