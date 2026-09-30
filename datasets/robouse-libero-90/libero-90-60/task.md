---
schema_version: '1.3'
task:
  name: robouse/libero-90-60
  description: Pick up the black bowl on the left and put it in the tray.
metadata:
  author_name: benchflow
  source_benchmark: LIBERO (Liu et al. 2023)
  source_task: LIVING_ROOM_SCENE4_pick_up_the_black_bowl_on_the_left_and_put_it_in_the_tray
  suite: libero-90
  libero_suite: libero_90
  libero_task_index: 60
  language_instruction: pick up the black bowl on the left and put it in the tray
  category: manipulation
  difficulty: medium
  tags:
  - franka
  - mujoco
  - robosuite
  - single-arm
  - language
  reference_solution: LIBERO demonstration demo_0 of yifengzhu-hf/LIBERO-datasets/libero_90/LIVING_ROOM_SCENE4_pick_up_the_black_bowl_on_the_left_and_put_it_in_the_tray_demo.hdf5 (replay reaches success at step 98)
  robouse:
    id: libero-90-60
    backend: libero
    suite: libero_90
    env: LIVING_ROOM_SCENE4_pick_up_the_black_bowl_on_the_left_and_put_it_in_the_tray
    task_index: 60
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
    init_source: yifengzhu-hf/LIBERO-datasets/libero_90/LIVING_ROOM_SCENE4_pick_up_the_black_bowl_on_the_left_and_put_it_in_the_tray_demo.hdf5 demo_0 states[0]
    init_state:
    - 0.25000000000000017
    - -0.0017618605943955036
    - -0.14658138438554838
    - -0.07238097937873918
    - -2.4599653699340753
    - 0.027943519024663393
    - 2.2801167347003592
    - 0.6657945131447147
    - 0.03378264108400998
    - -0.03417028400640405
    - -0.12320722106056804
    - -0.15068668930531373
    - 0.4351699275556287
    - 0.7071067791707762
    - -6.826208510568352e-08
    - -1.2183615572298947e-08
    - 0.7071067832023155
    - -0.07945868218194271
    - 0.02624071617825649
    - 0.4351699275556287
    - 0.7071067791707762
    - -6.826208510618229e-08
    - -1.2183615573158606e-08
    - 0.7071067832023155
    - -0.22961689467643281
    - -0.11980903967173266
    - 0.4797074607342256
    - 0.5000000162947403
    - 0.4999999837052388
    - 0.5000000024348927
    - 0.4999999975651276
    - 0.11524202873099822
    - -0.222269855636634
    - 0.4498398389808574
    - 5.028867884109457e-09
    - -3.782457014429499e-09
    - 1.2586724035202943e-08
    - 1.0
    - -0.003373084068721057
    - 0.260164185826066
    - 0.43738012050757274
    - -7.451456348804263e-06
    - -0.00173263793260483
    - 0.0012644815872153276
    - 0.9999976994956469
    - -0.12161516271683576
    - 0.17959784124153175
    - -0.4147968777827316
    - 0.13422416433661533
    - 0.2039708035266409
    - 0.051339169603426826
    - -0.6610418680051374
    - 0.05762151951602395
    - -0.05707047699555799
    - 1.1457031482187244e-11
    - 6.364949011720703e-11
    - 2.9220132505701583e-06
    - -6.6124025128628165e-09
    - -3.66696203518308e-08
    - -9.683465755976751e-14
    - 1.1457031462135966e-11
    - 6.364948963040738e-11
    - 2.922013250569697e-06
    - -6.6124017861271726e-09
    - -3.666961860812503e-08
    - -9.68348142178079e-14
    - -1.5755571597202728e-09
    - 1.0020058942716604e-09
    - -1.204396816286533e-09
    - -2.594777565660934e-08
    - -3.7198219288948553e-13
    - -1.6504588429489712e-08
    - -6.061720485594502e-11
    - -1.4785294904209347e-11
    - 2.8959753960032525e-07
    - -1.162183166226917e-09
    - 4.6857720003395236e-09
    - 1.2428105282102256e-13
    - 6.9159029467570584e-09
    - 3.514609199127409e-09
    - 1.957734750534621e-06
    - 5.701211084439175e-06
    - -1.1170259724996162e-05
    - 4.8008153065123825e-08
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

# Pick up the black bowl on the left and put it in the tray

A Franka Panda arm with a parallel-jaw gripper works at a low table in a LIBERO scene (MuJoCo / robosuite; LIBERO suite `libero_90`, task `LIVING_ROOM_SCENE4_pick_up_the_black_bowl_on_the_left_and_put_it_in_the_tray`). LIBERO-90: 90 short-horizon tasks in 20 kitchen, living-room and study scenes (the task name starts with the scene); LIBERO uses them for pretraining.

**Goal (LIBERO's language instruction):** pick up the black bowl on the left and put it in the tray.

The episode ends as solved the moment LIBERO's own success check (the task's goal predicates) passes; you do not need to call `robo done` after that. If you finish without the check passing, call `robo done` (or `robo give-up`).

## This robot

- The action has 7 numbers, not 4: `robo act DX DY DZ DROLL DPITCH DYAW GRIPPER`, each in [-1, 1]. It is LIBERO's delta end-effector command (robosuite's OSC_POSE controller, 20 steps per second). Holding DX/DY/DZ at 1.0 moves the hand about 1 cm per step; DROLL/DPITCH/DYAW rotate the hand (1.0 asks for 0.5 rad; usually leave them at 0). GRIPPER -1 opens, +1 closes.
- `robo move-to X Y Z [--grip G]` and `robo grip G` are available (they keep the hand's orientation).
- World frame, metres: +x points forward, away from the robot's base; +y is to the robot's left; +z is up. The table surface is at about z = 0.43.
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
