---
schema_version: '1.3'
task:
  name: robouse/libero-spatial-4
  description: Pick up the black bowl in the top drawer of the wooden cabinet and place it on the plate.
metadata:
  author_name: benchflow
  source_benchmark: LIBERO (Liu et al. 2023)
  source_task: pick_up_the_black_bowl_in_the_top_drawer_of_the_wooden_cabinet_and_place_it_on_the_plate
  suite: libero
  libero_suite: libero_spatial
  libero_task_index: 4
  language_instruction: pick up the black bowl in the top drawer of the wooden cabinet and place it on the plate
  category: manipulation
  difficulty: easy
  tags:
  - franka
  - mujoco
  - robosuite
  - single-arm
  - language
  reference_solution: LIBERO demonstration demo_0 of yifengzhu-hf/LIBERO-datasets/libero_spatial/pick_up_the_black_bowl_in_the_top_drawer_of_the_wooden_cabinet_and_place_it_on_the_plate_demo.hdf5 (replay reaches success at step 115)
  robouse:
    id: libero-spatial-4
    backend: libero
    suite: libero_spatial
    env: pick_up_the_black_bowl_in_the_top_drawer_of_the_wooden_cabinet_and_place_it_on_the_plate
    task_index: 4
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
    init_source: yifengzhu-hf/LIBERO-datasets/libero_spatial/pick_up_the_black_bowl_in_the_top_drawer_of_the_wooden_cabinet_and_place_it_on_the_plate_demo.hdf5 demo_0 states[0]
    init_state:
    - 0.25000000000000017
    - -0.005955293309896754
    - -0.1410435977465238
    - 0.012351360139143725
    - -2.440567235943337
    - -0.007674432699931779
    - 2.2451986228913263
    - 0.7793970060389396
    - 0.03404969202985844
    - -0.034062689934260384
    - 0.07949308582103895
    - -0.1463237286016539
    - 1.0625595843246074
    - 0.7071067770358898
    - -7.285515900314769e-08
    - -6.113574150480177e-09
    - 0.7071067853372016
    - 0.02122620175580832
    - -0.3077152473712605
    - 1.1263997104596004
    - 0.707106776245139
    - -8.068387989734569e-08
    - 5.093625397703738e-09
    - 0.7071067861279516
    - 0.06399846622053387
    - 0.03241931596274761
    - 0.9092097974085793
    - 0.7071067797562697
    - -6.93657303375886e-07
    - -2.0480328856631284e-06
    - 0.7071067826135193
    - -0.20348318729663165
    - 0.18729726077126846
    - 0.8991595343682753
    - 0.7071067709929622
    - -1.5165677572155225e-06
    - 2.1445576536313316e-06
    - 0.7071067913752543
    - 0.06217144270058862
    - 0.21286775680023248
    - 0.9023145976636144
    - 0.7071068944202988
    - -1.0137770013837841e-05
    - 2.0046027443780847e-05
    - 0.7071066675959596
    - -0.15502193700075612
    - 0.0
    - 0.0
    - 0.0
    - 0.0068025170969739914
    - 0.022931176991486966
    - -0.00706028568881074
    - 0.012629730614565658
    - -3.384295979041157e-05
    - -0.007937255428591647
    - 2.9876837495545834e-06
    - 0.057562177889493804
    - -0.05756277057537728
    - -8.80171731621374e-08
    - -1.8480239701836518e-07
    - 2.742320055753544e-05
    - -1.8384904216257264e-07
    - -1.1021586568699055e-06
    - -2.714351920962733e-12
    - 8.718471208409459e-10
    - 5.3322975861030646e-09
    - 1.8424130257396654e-05
    - -4.994358104676301e-07
    - -3.057163682648585e-06
    - -7.623863411945588e-12
    - -3.110017478221983e-08
    - -2.9459104358518556e-08
    - 0.004317179528839079
    - -3.3344105772758256e-06
    - -3.188347456629105e-06
    - 3.25136589438464e-11
    - 1.6134108431205657e-09
    - -4.406652723665533e-08
    - 0.0048507343363046905
    - -1.634486031574769e-06
    - 5.0263050417888976e-05
    - -4.2566036282177995e-10
    - 2.9156446724037e-07
    - 3.0526807631304383e-07
    - 0.004946092249603378
    - 0.00012342310665931016
    - 0.00012863603417490876
    - 5.282964696405577e-09
    - 2.0678940095664267e-07
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

# Pick up the black bowl in the top drawer of the wooden cabinet and place it on the plate

A Franka Panda arm with a parallel-jaw gripper works at a table in a LIBERO scene (MuJoCo / robosuite; LIBERO suite `libero_spatial`, task `pick_up_the_black_bowl_in_the_top_drawer_of_the_wooden_cabinet_and_place_it_on_the_plate`). LIBERO-Spatial: the same objects in different layouts; the instruction says which of two identical bowls to move.

**Goal (LIBERO's language instruction):** pick up the black bowl in the top drawer of the wooden cabinet and place it on the plate.

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
