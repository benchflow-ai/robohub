---
schema_version: '1.3'
task:
  name: robouse/libero-10-6
  description: Put the white mug on the plate and put the chocolate pudding to the right of the plate.
metadata:
  author_name: benchflow
  source_benchmark: LIBERO (Liu et al. 2023)
  source_task: LIVING_ROOM_SCENE6_put_the_white_mug_on_the_plate_and_put_the_chocolate_pudding_to_the_right_of_the_plate
  suite: libero
  libero_suite: libero_10
  libero_task_index: 6
  language_instruction: put the white mug on the plate and put the chocolate pudding to the right of the plate
  category: manipulation
  difficulty: hard
  tags:
  - franka
  - mujoco
  - robosuite
  - single-arm
  - language
  reference_solution: LIBERO demonstration demo_0 of yifengzhu-hf/LIBERO-datasets/libero_10/LIVING_ROOM_SCENE6_put_the_white_mug_on_the_plate_and_put_the_chocolate_pudding_to_the_right_of_the_plate_demo.hdf5 (replay reaches success at step 193)
  robouse:
    id: libero-10-6
    backend: libero
    suite: libero_10
    env: LIVING_ROOM_SCENE6_put_the_white_mug_on_the_plate_and_put_the_chocolate_pudding_to_the_right_of_the_plate
    task_index: 6
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
    init_source: yifengzhu-hf/LIBERO-datasets/libero_10/LIVING_ROOM_SCENE6_put_the_white_mug_on_the_plate_and_put_the_chocolate_pudding_to_the_right_of_the_plate_demo.hdf5 demo_0 states[0]
    init_state:
    - 0.25000000000000017
    - -0.013739845410446807
    - -0.1614043737900772
    - -0.026704978861375298
    - -2.449877899297952
    - -0.0041661844093531545
    - 2.2226846832836284
    - 0.7834967127538618
    - 0.03400898139535012
    - -0.034091822419680534
    - -0.09339323309781303
    - -0.1263744432518141
    - 0.4343699488393418
    - -0.7071069013731585
    - 1.0172763262795456e-07
    - 6.675731367985108e-08
    - 0.7071066609999057
    - -0.20112409065118567
    - 0.01163155057776368
    - 0.4385989872647111
    - -0.7071068579899178
    - -5.52861317559078e-07
    - 3.831199740262128e-07
    - 0.707106704382849
    - 0.15424423505296672
    - 0.002605945166636063
    - 0.43927280868351787
    - 0.7070931570734764
    - 1.0116995903382692e-06
    - -7.421134261334427e-07
    - 0.7071204050360093
    - -0.03840002830969903
    - 0.1026465386865624
    - 0.4498398389808574
    - 5.028867884115466e-09
    - -3.7824570034363e-09
    - 1.2586724035519156e-08
    - 1.0
    - -0.01847544706052831
    - 0.0545997621749888
    - -0.0863465939634946
    - 0.05004511343473922
    - -0.01910723293964029
    - 0.0138083843103824
    - -0.002870193444947747
    - 0.05774938706409486
    - -0.05729939820228677
    - -7.114629335092314e-11
    - -4.527150188050038e-10
    - 1.1368093065863386e-06
    - -2.8896237748115244e-08
    - -1.8466801295529253e-07
    - -1.0830535787342767e-12
    - 8.910656296510827e-12
    - -1.8765526489922314e-10
    - 1.8430481127028034e-06
    - -4.917191225915508e-09
    - 1.0451615555855826e-07
    - -2.7722668423067205e-13
    - 7.448568057475378e-08
    - -3.250383044951199e-08
    - 1.2103266702466268e-06
    - 3.02626280189876e-05
    - -1.3229459162134865e-05
    - 7.472201459067481e-10
    - -6.06172049377172e-11
    - -1.4785295324116286e-11
    - 2.8959753960052054e-07
    - -1.1621830825139666e-09
    - 4.6857717435879885e-09
    - 1.242804617351489e-13
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

# Put the white mug on the plate and put the chocolate pudding to the right of the plate

A Franka Panda arm with a parallel-jaw gripper works at a low table in a LIBERO scene (MuJoCo / robosuite; LIBERO suite `libero_10`, task `LIVING_ROOM_SCENE6_put_the_white_mug_on_the_plate_and_put_the_chocolate_pudding_to_the_right_of_the_plate`). LIBERO-Long (libero_10): long-horizon tasks, most with two sub-goals.

**Goal (LIBERO's language instruction):** put the white mug on the plate and put the chocolate pudding to the right of the plate.

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
