---
schema_version: '1.3'
task:
  name: robouse/libero-90-69
  description: Put the chocolate pudding to the left of the plate.
metadata:
  author_name: benchflow
  source_benchmark: LIBERO (Liu et al. 2023)
  source_task: LIVING_ROOM_SCENE6_put_the_chocolate_pudding_to_the_left_of_the_plate
  suite: libero-90
  libero_suite: libero_90
  libero_task_index: 69
  language_instruction: put the chocolate pudding to the left of the plate
  category: manipulation
  difficulty: medium
  tags:
  - franka
  - mujoco
  - robosuite
  - single-arm
  - language
  reference_solution: LIBERO demonstration demo_0 of yifengzhu-hf/LIBERO-datasets/libero_90/LIVING_ROOM_SCENE6_put_the_chocolate_pudding_to_the_left_of_the_plate_demo.hdf5 (replay reaches success at step 99)
  robouse:
    id: libero-90-69
    backend: libero
    suite: libero_90
    env: LIVING_ROOM_SCENE6_put_the_chocolate_pudding_to_the_left_of_the_plate
    task_index: 69
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
    init_source: yifengzhu-hf/LIBERO-datasets/libero_90/LIVING_ROOM_SCENE6_put_the_chocolate_pudding_to_the_left_of_the_plate_demo.hdf5 demo_0 states[0]
    init_state:
    - 0.25000000000000017
    - 0.00741055042579912
    - -0.1552935845219084
    - 0.030146743534562936
    - -2.4264482243128933
    - 0.00546749077910464
    - 2.229874242748572
    - 0.8018283047524095
    - 0.034092702903602654
    - -0.03400076759023839
    - -0.08564732792727099
    - -0.13125350053624815
    - 0.4343699488393418
    - -0.7071069013731585
    - 1.0172763262643605e-07
    - 6.675731368102593e-08
    - 0.7071066609999057
    - -0.19220334028116168
    - -0.011030252413686159
    - 0.4385989872647111
    - -0.7071068579899178
    - -5.528613175566909e-07
    - 3.8311997402921334e-07
    - 0.707106704382849
    - 0.16941507177355775
    - 0.014076058769369145
    - 0.43927280798879165
    - 0.7069512916236862
    - 1.0014928666119273e-06
    - -7.140386933115757e-07
    - 0.7072622365644082
    - -0.07180252869924494
    - 0.10376145694454475
    - 0.4498398389808574
    - 5.028867884102409e-09
    - -3.7824569906140615e-09
    - 1.258672403549197e-08
    - 1.0
    - 0.026585722428751055
    - 0.11330357577062629
    - 0.09271463671289469
    - 0.09127192481308721
    - 0.047992486590946314
    - -0.040659007118049616
    - 0.21223460297579536
    - 0.05727050028184114
    - -0.05774595285288178
    - -7.114629397127914e-11
    - -4.5271501724367694e-10
    - 1.1368093065892587e-06
    - -2.8896237759197605e-08
    - -1.8466801292737422e-07
    - -1.0830534962164128e-12
    - 8.910655218693584e-12
    - -1.8765526249242878e-10
    - 1.84304811270086e-06
    - -4.917191242713266e-09
    - 1.0451615559622477e-07
    - -2.7722737853193366e-13
    - 7.012109047801818e-08
    - -4.2022455418517054e-08
    - 1.2599599532538478e-06
    - 2.8482244649191272e-05
    - -1.7114577907967404e-05
    - 6.958290445621518e-10
    - -6.061720494090114e-11
    - -1.47852953666286e-11
    - 2.8959753960014446e-07
    - -1.1621831493877618e-09
    - 4.6857717240236764e-09
    - 1.2428035843230537e-13
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
---

You are controlling a simulated robot. Read the task below, then solve it by running the `robo` command in your shell (start with `robo info` and `robo observe`). Keep going until the task is done, then call `robo done` once. Do not stop to ask questions; there is no human to answer.

# Put the chocolate pudding to the left of the plate

A Franka Panda arm with a parallel-jaw gripper works at a low table in a LIBERO scene (MuJoCo / robosuite; LIBERO suite `libero_90`, task `LIVING_ROOM_SCENE6_put_the_chocolate_pudding_to_the_left_of_the_plate`). LIBERO-90: 90 short-horizon tasks in 20 kitchen, living-room and study scenes (the task name starts with the scene); LIBERO uses them for pretraining.

**Goal (LIBERO's language instruction):** put the chocolate pudding to the left of the plate.

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
