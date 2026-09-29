---
schema_version: '1.3'
task:
  name: robouse/libero-spatial-8
  description: Pick up the black bowl next to the plate and place it on the plate.
metadata:
  author_name: benchflow
  source_benchmark: LIBERO (Liu et al. 2023)
  source_task: pick_up_the_black_bowl_next_to_the_plate_and_place_it_on_the_plate
  suite: libero
  libero_suite: libero_spatial
  libero_task_index: 8
  language_instruction: pick up the black bowl next to the plate and place it on the plate
  category: manipulation
  difficulty: easy
  tags:
  - franka
  - mujoco
  - robosuite
  - single-arm
  - language
  reference_solution: LIBERO demonstration demo_0 of yifengzhu-hf/LIBERO-datasets/libero_spatial/pick_up_the_black_bowl_next_to_the_plate_and_place_it_on_the_plate_demo.hdf5 (replay reaches success at step 101)
  robouse:
    id: libero-spatial-8
    backend: libero
    suite: libero_spatial
    env: pick_up_the_black_bowl_next_to_the_plate_and_place_it_on_the_plate
    task_index: 8
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
    init_source: yifengzhu-hf/LIBERO-datasets/libero_spatial/pick_up_the_black_bowl_next_to_the_plate_and_place_it_on_the_plate_demo.hdf5 demo_0 states[0]
    init_state:
    - 0.25000000000000017
    - 0.005153524820369571
    - -0.1505806167149877
    - 0.00516637096328087
    - -2.431203938253512
    - -0.0014165553670828952
    - 2.227339389482204
    - 0.7999054419654583
    - 0.03406287185355451
    - -0.03404751166419868
    - -0.0017402461551477252
    - 0.3128234184580242
    - 0.8981585151399347
    - 0.7071067776242912
    - -1.0724537191720632e-05
    - -1.4310617004713047e-06
    - 0.7071067846660275
    - -0.19388301959658796
    - 0.3192136234215791
    - 0.8981585151399347
    - 0.7071067776242912
    - -1.0724537191719875e-05
    - -1.4310617004704888e-06
    - 0.7071067846660275
    - 0.06199611478190825
    - 0.03189479987967049
    - 0.9092097974085793
    - 0.7071067797562697
    - -6.936573033801512e-07
    - -2.0480328856664962e-06
    - 0.7071067826135193
    - -0.2075891835197063
    - 0.2141816373025432
    - 0.8991595343682753
    - 0.7071067709929622
    - -1.5165677572059534e-06
    - 2.1445576536257907e-06
    - 0.7071067913752543
    - 0.0467863428479596
    - 0.19147123513365547
    - 0.9023145976636144
    - 0.7071068944202988
    - -1.0137770013842676e-05
    - 2.0046027443776632e-05
    - 0.7071066675959596
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.005175956595187907
    - -0.00477027908734592
    - 0.02830817422344967
    - 0.008898368521692478
    - 0.03828508519527885
    - -0.026756151003430366
    - -0.02173158625440472
    - 0.057537437758551066
    - -0.05757706527057251
    - 1.414696485257373e-07
    - -3.490664373247852e-08
    - 0.006561141189068617
    - -7.860505322546593e-05
    - 2.0775069654659038e-05
    - -1.2876260703190534e-09
    - 1.4146964852454175e-07
    - -3.490664373149082e-08
    - 0.006561141189068616
    - -7.860505322544757e-05
    - 2.0775069654550276e-05
    - -1.287626070121058e-09
    - -3.110017474934397e-08
    - -2.9459104344896933e-08
    - 0.004317179528839082
    - -3.3344105743677197e-06
    - -3.188347455223259e-06
    - 3.25136591188939e-11
    - 1.61341084234953e-09
    - -4.406652723477976e-08
    - 0.004850734336304676
    - -1.634486031576617e-06
    - 5.026305041716445e-05
    - -4.25660362608957e-10
    - 2.915644672403163e-07
    - 3.0526807631282376e-07
    - 0.004946092249603374
    - 0.0001234231066589758
    - 0.000128636034174639
    - 5.282964695962282e-09
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
---

You are controlling a simulated robot. Read the task below, then solve it by running the `robo` command in your shell (start with `robo info` and `robo observe`). Keep going until the task is done, then call `robo done` once. Do not stop to ask questions; there is no human to answer.

# Pick up the black bowl next to the plate and place it on the plate

A Franka Panda arm with a parallel-jaw gripper works at a table in a LIBERO scene (MuJoCo / robosuite; LIBERO suite `libero_spatial`, task `pick_up_the_black_bowl_next_to_the_plate_and_place_it_on_the_plate`). LIBERO-Spatial: the same objects in different layouts; the instruction says which of two identical bowls to move.

**Goal (LIBERO's language instruction):** pick up the black bowl next to the plate and place it on the plate.

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
