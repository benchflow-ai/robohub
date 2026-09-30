---
schema_version: '1.3'
task:
  name: robouse/libero-spatial-9
  description: Pick up the black bowl on the wooden cabinet and place it on the plate.
metadata:
  author_name: benchflow
  source_benchmark: LIBERO (Liu et al. 2023)
  source_task: pick_up_the_black_bowl_on_the_wooden_cabinet_and_place_it_on_the_plate
  suite: libero
  libero_suite: libero_spatial
  libero_task_index: 9
  language_instruction: pick up the black bowl on the wooden cabinet and place it on the plate
  category: manipulation
  difficulty: easy
  tags:
  - franka
  - mujoco
  - robosuite
  - single-arm
  - language
  reference_solution: LIBERO demonstration demo_0 of yifengzhu-hf/LIBERO-datasets/libero_spatial/pick_up_the_black_bowl_on_the_wooden_cabinet_and_place_it_on_the_plate_demo.hdf5 (replay reaches success at step 126)
  robouse:
    id: libero-spatial-9
    backend: libero
    suite: libero_spatial
    env: pick_up_the_black_bowl_on_the_wooden_cabinet_and_place_it_on_the_plate
    task_index: 9
    seed: 0
    max_steps: 280
    camera: agentview
    cameras:
    - agentview
    - robot0_eye_in_hand
    image_size: 256
    settle_steps: 50
    skills: true
    success_mode: first
    obs_mode: state
    init_source: yifengzhu-hf/LIBERO-datasets/libero_spatial/pick_up_the_black_bowl_on_the_wooden_cabinet_and_place_it_on_the_plate_demo.hdf5 demo_0 states[0]
    init_state:
    - 0.25000000000000017
    - 0.018433586223996434
    - -0.13530116862197053
    - -0.01235431653305524
    - -2.4371607036258913
    - -0.014964586088184097
    - 2.229720618385646
    - 0.775016993548039
    - 0.0340469033667335
    - -0.03406531367308525
    - 0.014415743712169674
    - -0.29071893632380835
    - 1.1263997104596004
    - 0.707106776245139
    - -8.068387989710887e-08
    - 5.0936253984122745e-09
    - 0.7071067861279516
    - -0.25224343499904783
    - -0.15217525690014427
    - 0.9254584906895845
    - 0.7071006950454188
    - 0.004070064963756559
    - 8.648229704709714e-05
    - 0.7071011484634209
    - 0.06669794803788115
    - 0.018072927081672158
    - 0.9092097974085793
    - 0.7071067797562697
    - -6.936573033715682e-07
    - -2.0480328856664852e-06
    - 0.7071067826135193
    - -0.2004318812363835
    - 0.20843092269363836
    - 0.8991595343682753
    - 0.7071067709929622
    - -1.516567757215634e-06
    - 2.144557653631366e-06
    - 0.7071067913752543
    - 0.053120957588951766
    - 0.18596115027377502
    - 0.9023145976636144
    - 0.7071068944202988
    - -1.0137770013842766e-05
    - 2.0046027443774955e-05
    - 0.7071066675959596
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - -0.02036446166502678
    - -0.045300076941827076
    - 0.02224455578013241
    - -0.004660353297595439
    - 0.00163979968719538
    - -0.021808663668088094
    - 8.829272742180849e-05
    - 0.05752081989470961
    - -0.05760588044858684
    - 8.71847120318489e-10
    - 5.332297586217542e-09
    - 1.842413025737348e-05
    - -4.994358104485841e-07
    - -3.0571636826527053e-06
    - -7.623863334051703e-12
    - -2.5828010123318783e-05
    - 1.2989496769109127e-05
    - 0.00029768281212435964
    - 0.00532014302663159
    - -0.002620844217628544
    - -1.1941272955058026e-05
    - -3.11001747493892e-08
    - -2.9459104345325406e-08
    - 0.004317179528839081
    - -3.334410574218038e-06
    - -3.188347455411294e-06
    - 3.2513659395067094e-11
    - 1.6134108430425377e-09
    - -4.406652723766682e-08
    - 0.004850734336304699
    - -1.6344860315735023e-06
    - 5.026305041794414e-05
    - -4.2566036286585104e-10
    - 2.915644672401807e-07
    - 3.0526807631277506e-07
    - 0.004946092249603376
    - 0.00012342310665900774
    - 0.00012863603417459277
    - 5.282964696075451e-09
    - 0.0
    - 0.0
    - 0.0
    - 0.0
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

# Pick up the black bowl on the wooden cabinet and place it on the plate

A Franka Panda arm with a parallel-jaw gripper works at a table in a LIBERO scene (MuJoCo / robosuite; LIBERO suite `libero_spatial`, task `pick_up_the_black_bowl_on_the_wooden_cabinet_and_place_it_on_the_plate`). LIBERO-Spatial: the same objects in different layouts; the instruction says which of two identical bowls to move.

**Goal (LIBERO's language instruction):** pick up the black bowl on the wooden cabinet and place it on the plate.

The episode ends as solved the moment LIBERO's own success check (the task's goal predicates) passes; you do not need to call `robo done` after that. If you finish without the check passing, call `robo done` (or `robo give-up`).

## This robot

- The action has 7 numbers, not 4: `robo act DX DY DZ DROLL DPITCH DYAW GRIPPER`, each in [-1, 1]. It is LIBERO's delta end-effector command (robosuite's OSC_POSE controller, 20 steps per second). Holding DX/DY/DZ at 1.0 moves the hand about 1 cm per step; DROLL/DPITCH/DYAW rotate the hand (1.0 asks for 0.5 rad; usually leave them at 0). GRIPPER -1 opens, +1 closes.
- `robo move-to X Y Z [--grip G]` and `robo grip G` are available (they keep the hand's orientation).
- World frame, metres: +x points forward, away from the robot's base; +y is to the robot's left; +z is up. The table surface is at about z = 0.90.
- `robo observe` fields: `hand_pos` (end-effector position), `eef_quat` (orientation quaternion x, y, z, w) and `eef_axis_angle`, `gripper_qpos` (the two finger joints), `gripper_open` (finger gap in metres, about 0.08 when fully open), and `objects`: the position and quaternion of every object in the scene, under LIBERO's names (e.g. `akita_black_bowl_1`, `plate_1`; `..._1`, `..._2` number identical objects); `fixtures`: the position and quaternion of each piece of furniture (cabinet, stove, microwave, rack, shelf, ...); `articulated`: each movable part of a fixture (drawer slide, door hinge, stove knob), keyed by its joint name (e.g. `wooden_cabinet_1_top_level` is the cabinet's top drawer), with `qpos` (joint position), `range` (joint limits), LIBERO's thresholds for that fixture (`open_ranges` / `close_ranges`, or `turnon_ranges` / `turnoff_ranges` for a stove knob: the joint positions at which LIBERO counts the part as open / closed or on / off) and `box_min` / `box_max` (the world-frame bounding box of the moving part).
- `robo observe --image` saves the front camera (`agentview`); add `--camera robot0_eye_in_hand` for the wrist camera. Both are 256x256.
- The step budget is 280 steps (LeRobot's LIBERO evaluation budget for this suite).

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
