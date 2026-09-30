---
schema_version: '1.3'
task:
  name: robouse/libero-90-37
  description: Put the white bowl to the right of the plate.
metadata:
  author_name: benchflow
  source_benchmark: LIBERO (Liu et al. 2023)
  source_task: KITCHEN_SCENE7_put_the_white_bowl_to_the_right_of_the_plate
  suite: libero-90
  libero_suite: libero_90
  libero_task_index: 37
  language_instruction: put the white bowl to the right of the plate
  category: manipulation
  difficulty: medium
  tags:
  - franka
  - mujoco
  - robosuite
  - single-arm
  - language
  reference_solution: LIBERO demonstration demo_0 of yifengzhu-hf/LIBERO-datasets/libero_90/KITCHEN_SCENE7_put_the_white_bowl_to_the_right_of_the_plate_demo.hdf5 (replay reaches success at step 109)
  robouse:
    id: libero-90-37
    backend: libero
    suite: libero_90
    env: KITCHEN_SCENE7_put_the_white_bowl_to_the_right_of_the_plate
    task_index: 37
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
    init_source: yifengzhu-hf/LIBERO-datasets/libero_90/KITCHEN_SCENE7_put_the_white_bowl_to_the_right_of_the_plate_demo.hdf5 demo_0 states[0]
    init_state:
    - 0.25000000000000017
    - -0.0038585459825182075
    - -0.1519652913544647
    - -0.0005941700378102601
    - -2.428778154561166
    - -0.014240497424500335
    - 2.236395128693247
    - 0.7651096566590885
    - 0.03401435008861338
    - -0.034089436408007344
    - -0.03145091661068681
    - -0.2714770719897236
    - 1.1071967528211937
    - 0.707106741904556
    - -2.0220650935944672e-05
    - 3.2740751972114896e-06
    - 0.7071068201718389
    - 0.021277991814058844
    - -0.01991993170968003
    - 0.9023145976636144
    - 0.7071068944202988
    - -1.013777001384907e-05
    - 2.0046027443771093e-05
    - 0.7071066675959596
    - -0.0026469115909797175
    - -0.014186184428775813
    - 0.08744048079967863
    - -0.06701865260309861
    - 0.10872233914972661
    - -0.017358405345098228
    - -0.006295223123653071
    - -0.076498277109936
    - 0.057649340104784695
    - -0.05743225440182414
    - 5.245081240113987e-08
    - 1.6688952560533882e-07
    - 0.030601442460853882
    - -0.0001550919174875913
    - -0.0001861872461096642
    - -4.0974870443071446e-08
    - 2.915644672394398e-07
    - 3.0526807631254413e-07
    - 0.004946092249603383
    - 0.0001234231066588186
    - 0.00012863603417447245
    - 5.282964696228537e-09
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

# Put the white bowl to the right of the plate

A Franka Panda arm with a parallel-jaw gripper works at a table in a LIBERO scene (MuJoCo / robosuite; LIBERO suite `libero_90`, task `KITCHEN_SCENE7_put_the_white_bowl_to_the_right_of_the_plate`). LIBERO-90: 90 short-horizon tasks in 20 kitchen, living-room and study scenes (the task name starts with the scene); LIBERO uses them for pretraining.

**Goal (LIBERO's language instruction):** put the white bowl to the right of the plate.

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
