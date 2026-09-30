---
schema_version: '1.3'
task:
  name: robouse/libero-spatial-7
  description: Pick up the black bowl on the stove and place it on the plate.
metadata:
  author_name: benchflow
  source_benchmark: LIBERO (Liu et al. 2023)
  source_task: pick_up_the_black_bowl_on_the_stove_and_place_it_on_the_plate
  suite: libero
  libero_suite: libero_spatial
  libero_task_index: 7
  language_instruction: pick up the black bowl on the stove and place it on the plate
  category: manipulation
  difficulty: easy
  tags:
  - franka
  - mujoco
  - robosuite
  - single-arm
  - language
  reference_solution: LIBERO demonstration demo_1 of yifengzhu-hf/LIBERO-datasets/libero_spatial/pick_up_the_black_bowl_on_the_stove_and_place_it_on_the_plate_demo.hdf5 (replay reaches success at step 115)
  robouse:
    id: libero-spatial-7
    backend: libero
    suite: libero_spatial
    env: pick_up_the_black_bowl_on_the_stove_and_place_it_on_the_plate
    task_index: 7
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
    init_source: yifengzhu-hf/LIBERO-datasets/libero_spatial/pick_up_the_black_bowl_on_the_stove_and_place_it_on_the_plate_demo.hdf5 demo_1 states[0]
    init_state:
    - 0.25000000000000017
    - -0.0008195560994445703
    - -0.13981739407930205
    - 0.003305269229887255
    - -2.4697960354619095
    - -0.011959279277570583
    - 2.2555994363225533
    - 0.8034791955880828
    - 0.034044707338553644
    - -0.03406708582349023
    - -0.25059287385753254
    - -0.13324794624061773
    - 0.9254584906895844
    - 0.7071006950454188
    - 0.004070064963757124
    - 8.648229704688504e-05
    - 0.7071011484634209
    - 0.003285409003723842
    - -0.2962350599850481
    - 1.1263997104596004
    - 0.707106776245139
    - -8.068387989640784e-08
    - 5.093625398608787e-09
    - 0.7071067861279516
    - 0.05910036330241507
    - 0.03860533463052775
    - 0.9092097974085793
    - 0.7071067797562697
    - -6.93657303373686e-07
    - -2.048032885665394e-06
    - 0.7071067826135193
    - -0.20522712450094002
    - 0.1894163028300862
    - 0.8991595343682753
    - 0.7071067709929622
    - -1.5165677572094184e-06
    - 2.1445576536306963e-06
    - 0.7071067913752543
    - 0.05508411025096507
    - 0.18971300607033817
    - 0.9023145976636144
    - 0.7071068944202988
    - -1.0137770013845542e-05
    - 2.0046027443775592e-05
    - 0.7071066675959596
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - -0.0021832321212367305
    - 0.009410292111194108
    - -0.0013705681758809634
    - -1.860779545430358e-05
    - -0.0622412695174307
    - 0.005659598394703719
    - 0.04775467953554189
    - 0.05754489358010827
    - -0.057580318580096795
    - -2.5828010122188353e-05
    - 1.2989496769300196e-05
    - 0.00029768281211985033
    - 0.005320143026577522
    - -0.002620844217618652
    - -1.1941272954920261e-05
    - 8.718471189141882e-10
    - 5.332297585384836e-09
    - 1.8424130257332852e-05
    - -4.994358103513437e-07
    - -3.0571636826233234e-06
    - -7.623863139990936e-12
    - -3.1100174782194974e-08
    - -2.9459104358475728e-08
    - 0.004317179528839084
    - -3.3344105774360804e-06
    - -3.1883474566695726e-06
    - 3.251365903674639e-11
    - 1.6134108424203542e-09
    - -4.406652723666133e-08
    - 0.004850734336304693
    - -1.634486031562808e-06
    - 5.026305041782107e-05
    - -4.256603627000454e-10
    - 2.915644672405761e-07
    - 3.05268076312593e-07
    - 0.004946092249603386
    - 0.0001234231066589401
    - 0.00012863603417452978
    - 5.282964695769828e-09
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

# Pick up the black bowl on the stove and place it on the plate

A Franka Panda arm with a parallel-jaw gripper works at a table in a LIBERO scene (MuJoCo / robosuite; LIBERO suite `libero_spatial`, task `pick_up_the_black_bowl_on_the_stove_and_place_it_on_the_plate`). LIBERO-Spatial: the same objects in different layouts; the instruction says which of two identical bowls to move.

**Goal (LIBERO's language instruction):** pick up the black bowl on the stove and place it on the plate.

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
