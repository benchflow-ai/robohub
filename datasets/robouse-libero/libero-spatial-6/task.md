---
schema_version: '1.3'
task:
  name: robouse/libero-spatial-6
  description: Pick up the black bowl next to the cookie box and place it on the plate.
metadata:
  author_name: benchflow
  source_benchmark: LIBERO (Liu et al. 2023)
  source_task: pick_up_the_black_bowl_next_to_the_cookie_box_and_place_it_on_the_plate
  suite: libero
  libero_suite: libero_spatial
  libero_task_index: 6
  language_instruction: pick up the black bowl next to the cookie box and place it on the plate
  category: manipulation
  difficulty: easy
  tags:
  - franka
  - mujoco
  - robosuite
  - single-arm
  - language
  reference_solution: LIBERO demonstration demo_1 of yifengzhu-hf/LIBERO-datasets/libero_spatial/pick_up_the_black_bowl_next_to_the_cookie_box_and_place_it_on_the_plate_demo.hdf5 (replay reaches success at step 125)
  robouse:
    id: libero-spatial-6
    backend: libero
    suite: libero_spatial
    env: pick_up_the_black_bowl_next_to_the_cookie_box_and_place_it_on_the_plate
    task_index: 6
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
    init_source: yifengzhu-hf/LIBERO-datasets/libero_spatial/pick_up_the_black_bowl_next_to_the_cookie_box_and_place_it_on_the_plate_demo.hdf5 demo_1 states[0]
    init_state:
    - 0.25000000000000017
    - -0.0003848683518945359
    - -0.14015558111982016
    - -0.007504957599459452
    - -2.4515486662167145
    - 0.02354148368420454
    - 2.249551546677342
    - 0.7820792044889268
    - 0.034064828326230395
    - -0.03404613377964944
    - 0.14238786038570425
    - -0.08302143478058285
    - 0.8981585151399347
    - 0.7071067776242912
    - -1.072453719171908e-05
    - -1.4310617004710896e-06
    - 0.7071067846660275
    - -0.26484375237636326
    - -0.13711930380303666
    - 0.9254584906895845
    - 0.7071006950454188
    - 0.0040700649637569
    - 8.648229704734369e-05
    - 0.7071011484634209
    - 0.07243759736454915
    - 0.017555611446545688
    - 0.9092097974085793
    - 0.7071067797562697
    - -6.936573033724791e-07
    - -2.0480328856651672e-06
    - 0.7071067826135193
    - -0.20380148663482395
    - 0.18651756414151482
    - 0.8991595343682753
    - 0.7071067709929622
    - -1.5165677572112996e-06
    - 2.144557653636346e-06
    - 0.7071067913752543
    - 0.05760934822023154
    - 0.20620227469200625
    - 0.9023145976636144
    - 0.7071068944202988
    - -1.0137770013839903e-05
    - 2.004602744377721e-05
    - 0.7071066675959596
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - -0.007517516806989986
    - -0.052675692732765615
    - -0.025477879842243468
    - -0.06536148598112065
    - 0.0032719236286269434
    - 0.09234432019925926
    - -0.034989985390839
    - 0.05762781910170688
    - -0.05749358224746037
    - 1.414696485235294e-07
    - -3.4906643731231485e-08
    - 0.006561141189068618
    - -7.860505322538243e-05
    - 2.0775069654618546e-05
    - -1.287626069967705e-09
    - -2.582801012434585e-05
    - 1.2989496766858818e-05
    - 0.0002976828121173041
    - 0.005320143026646709
    - -0.00262084421753412
    - -1.1941272955251942e-05
    - -3.110017478221241e-08
    - -2.9459104358475212e-08
    - 0.004317179528839087
    - -3.334410577392563e-06
    - -3.1883474567596363e-06
    - 3.251365922179733e-11
    - 1.6134108424632255e-09
    - -4.406652723853083e-08
    - 0.0048507343363047
    - -1.6344860315617417e-06
    - 5.02630504179608e-05
    - -4.2566036279982363e-10
    - 2.915644672405356e-07
    - 3.052680763126984e-07
    - 0.004946092249603385
    - 0.0001234231066592053
    - 0.0001286360341746184
    - 5.282964696155245e-09
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

# Pick up the black bowl next to the cookie box and place it on the plate

A Franka Panda arm with a parallel-jaw gripper works at a table in a LIBERO scene (MuJoCo / robosuite; LIBERO suite `libero_spatial`, task `pick_up_the_black_bowl_next_to_the_cookie_box_and_place_it_on_the_plate`). LIBERO-Spatial: the same objects in different layouts; the instruction says which of two identical bowls to move.

**Goal (LIBERO's language instruction):** pick up the black bowl next to the cookie box and place it on the plate.

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
