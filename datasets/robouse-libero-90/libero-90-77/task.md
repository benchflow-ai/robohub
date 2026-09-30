---
schema_version: '1.3'
task:
  name: robouse/libero-90-77
  description: Pick up the book and place it in the back compartment of the caddy.
metadata:
  author_name: benchflow
  source_benchmark: LIBERO (Liu et al. 2023)
  source_task: STUDY_SCENE2_pick_up_the_book_and_place_it_in_the_back_compartment_of_the_caddy
  suite: libero-90
  libero_suite: libero_90
  libero_task_index: 77
  language_instruction: pick up the book and place it in the back compartment of the caddy
  category: manipulation
  difficulty: medium
  tags:
  - franka
  - mujoco
  - robosuite
  - single-arm
  - language
  reference_solution: LIBERO demonstration demo_0 of yifengzhu-hf/LIBERO-datasets/libero_90/STUDY_SCENE2_pick_up_the_book_and_place_it_in_the_back_compartment_of_the_caddy_demo.hdf5 (replay reaches success at step 181)
  robouse:
    id: libero-90-77
    backend: libero
    suite: libero_90
    env: STUDY_SCENE2_pick_up_the_book_and_place_it_in_the_back_compartment_of_the_caddy
    task_index: 77
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
    init_source: yifengzhu-hf/LIBERO-datasets/libero_90/STUDY_SCENE2_pick_up_the_book_and_place_it_in_the_back_compartment_of_the_caddy_demo.hdf5 demo_0 states[0]
    init_state:
    - 0.25000000000000017
    - -0.025506448679900838
    - -0.13401475224742904
    - -0.01951308275812024
    - -2.452861993816024
    - 0.00485651838544134
    - 2.2677006520221776
    - 0.7918485472180614
    - 0.03404395066409011
    - -0.03406600602896695
    - -0.20141076139344866
    - -0.05239321868199799
    - 0.8829597034738329
    - -0.47744465746631914
    - 1.4579719061452709e-08
    - -4.80836816055226e-08
    - 0.8786618229198687
    - -0.20047240328296817
    - 0.16976390927706594
    - 0.8869689131443201
    - -0.7071068831386871
    - -5.520412001714356e-07
    - 3.840203852401883e-07
    - 0.7071066792340737
    - 0.018063718901027176
    - 0.10454382707363113
    - -0.023462532400343645
    - 0.04605617197490356
    - 0.056273469686576996
    - 0.12302748152997264
    - 0.11152902689690017
    - 0.05771682641691963
    - -0.057436644072682924
    - -5.252827491900965e-10
    - 3.924370404818345e-10
    - 9.876915797294728e-06
    - -2.958763012997076e-07
    - 1.9336918637808312e-08
    - 2.554901413573079e-14
    - 2.36913220260579e-11
    - -4.97828028428938e-10
    - 7.136022452261281e-06
    - -1.2980796299922885e-08
    - 2.7621760131466633e-07
    - -6.029943850743938e-13
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

# Pick up the book and place it in the back compartment of the caddy

A Franka Panda arm with a parallel-jaw gripper works at a desk in a LIBERO scene (MuJoCo / robosuite; LIBERO suite `libero_90`, task `STUDY_SCENE2_pick_up_the_book_and_place_it_in_the_back_compartment_of_the_caddy`). LIBERO-90: 90 short-horizon tasks in 20 kitchen, living-room and study scenes (the task name starts with the scene); LIBERO uses them for pretraining.

**Goal (LIBERO's language instruction):** pick up the book and place it in the back compartment of the caddy.

The episode ends as solved the moment LIBERO's own success check (the task's goal predicates) passes; you do not need to call `robo done` after that. If you finish without the check passing, call `robo done` (or `robo give-up`).

## This robot

- The action has 7 numbers, not 4: `robo act DX DY DZ DROLL DPITCH DYAW GRIPPER`, each in [-1, 1]. It is LIBERO's delta end-effector command (robosuite's OSC_POSE controller, 20 steps per second). Holding DX/DY/DZ at 1.0 moves the hand about 1 cm per step; DROLL/DPITCH/DYAW rotate the hand (1.0 asks for 0.5 rad; usually leave them at 0). GRIPPER -1 opens, +1 closes.
- `robo move-to X Y Z [--grip G]` and `robo grip G` are available (they keep the hand's orientation).
- World frame, metres: +x points forward, away from the robot's base; +y is to the robot's left; +z is up. The desk surface is at about z = 0.88.
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
