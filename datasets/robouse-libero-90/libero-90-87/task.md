---
schema_version: '1.3'
task:
  name: robouse/libero-90-87
  description: Pick up the book on the left and place it on top of the shelf.
metadata:
  author_name: benchflow
  source_benchmark: LIBERO (Liu et al. 2023)
  source_task: STUDY_SCENE4_pick_up_the_book_on_the_left_and_place_it_on_top_of_the_shelf
  suite: libero-90
  libero_suite: libero_90
  libero_task_index: 87
  language_instruction: pick up the book on the left and place it on top of the shelf
  category: manipulation
  difficulty: medium
  tags:
  - franka
  - mujoco
  - robosuite
  - single-arm
  - language
  reference_solution: LIBERO demonstration demo_1 of yifengzhu-hf/LIBERO-datasets/libero_90/STUDY_SCENE4_pick_up_the_book_on_the_left_and_place_it_on_top_of_the_shelf_demo.hdf5 (replay reaches success at step 130)
  robouse:
    id: libero-90-87
    backend: libero
    suite: libero_90
    env: STUDY_SCENE4_pick_up_the_book_on_the_left_and_place_it_on_top_of_the_shelf
    task_index: 87
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
    init_source: yifengzhu-hf/LIBERO-datasets/libero_90/STUDY_SCENE4_pick_up_the_book_on_the_left_and_place_it_on_top_of_the_shelf_demo.hdf5 demo_1 states[0]
    init_state:
    - 0.25000000000000017
    - -0.02440855227338668
    - -0.13872199034178245
    - -0.011431691379183824
    - -2.4440192772194145
    - 0.0014403489096439278
    - 2.2340443994979684
    - 0.7876361482844071
    - 0.033976406069384284
    - -0.0341140212599107
    - -0.15427704683055068
    - -0.14601800527381753
    - 0.8829597034738329
    - -0.45087579798973304
    - 1.313096649934988e-08
    - -4.84993439485159e-08
    - 0.8925866987509499
    - -0.19452497569704133
    - -0.008310969870073206
    - 0.8831598332502683
    - -0.4756305646158247
    - -2.0484198676785697e-07
    - -1.6768776332644737e-07
    - 0.8796451364062452
    - -0.24883978137350243
    - -0.24594628557672607
    - 0.8831598332502683
    - -0.4642908027421692
    - -2.0697931239968315e-07
    - -1.6504238774018528e-07
    - 0.8856828159612006
    - -0.019868937357752168
    - 0.15577701715228248
    - -0.1560660070269114
    - 0.1399132241518722
    - -0.03712600315556521
    - 0.052263043984189114
    - 0.026331035198042335
    - 0.05754845072332493
    - -0.057292551307819996
    - -5.00807202870489e-10
    - 4.2322681864208495e-10
    - 9.876915797293881e-06
    - -2.9587630127586916e-07
    - 1.9336918645977342e-08
    - 2.5549021406026697e-14
    - -4.079214682425242e-11
    - -3.172638698072722e-10
    - 7.603300045123753e-07
    - 6.736790359136693e-08
    - -1.347892525516049e-07
    - 1.8405784845012116e-13
    - -4.892958086511062e-11
    - -3.161111491162984e-10
    - 7.603299988669301e-07
    - 6.736790357813047e-08
    - -1.3478925257999005e-07
    - 1.8405785908712628e-13
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

# Pick up the book on the left and place it on top of the shelf

A Franka Panda arm with a parallel-jaw gripper works at a desk in a LIBERO scene (MuJoCo / robosuite; LIBERO suite `libero_90`, task `STUDY_SCENE4_pick_up_the_book_on_the_left_and_place_it_on_top_of_the_shelf`). LIBERO-90: 90 short-horizon tasks in 20 kitchen, living-room and study scenes (the task name starts with the scene); LIBERO uses them for pretraining.

**Goal (LIBERO's language instruction):** pick up the book on the left and place it on top of the shelf.

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
