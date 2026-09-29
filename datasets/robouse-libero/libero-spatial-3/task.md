---
schema_version: '1.3'
task:
  name: robouse/libero-spatial-3
  description: Pick up the black bowl on the cookie box and place it on the plate.
metadata:
  author_name: benchflow
  source_benchmark: LIBERO (Liu et al. 2023)
  source_task: pick_up_the_black_bowl_on_the_cookie_box_and_place_it_on_the_plate
  suite: libero
  libero_suite: libero_spatial
  libero_task_index: 3
  language_instruction: pick up the black bowl on the cookie box and place it on the plate
  category: manipulation
  difficulty: easy
  tags:
  - franka
  - mujoco
  - robosuite
  - single-arm
  - language
  reference_solution: LIBERO demonstration demo_0 of yifengzhu-hf/LIBERO-datasets/libero_spatial/pick_up_the_black_bowl_on_the_cookie_box_and_place_it_on_the_plate_demo.hdf5 (replay reaches success at step 88)
  robouse:
    id: libero-spatial-3
    backend: libero
    suite: libero_spatial
    env: pick_up_the_black_bowl_on_the_cookie_box_and_place_it_on_the_plate
    task_index: 3
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
    init_source: yifengzhu-hf/LIBERO-datasets/libero_spatial/pick_up_the_black_bowl_on_the_cookie_box_and_place_it_on_the_plate_demo.hdf5 demo_0 states[0]
    init_state:
    - 0.25000000000000017
    - 0.012674036215890963
    - -0.1633836252421253
    - -0.023460044695263558
    - -2.416028015831608
    - -0.026029288633547965
    - 2.2448164414129232
    - 0.7745907636060788
    - 0.03403332477676948
    - -0.03407857128015566
    - 0.07648157374470925
    - 0.02536123987158409
    - 0.9169196589889572
    - 0.7071067735325446
    - -1.766718870820565e-05
    - 1.8975237421085122e-05
    - 0.707106788365241
    - 0.044169446324389336
    - -0.27117313626006323
    - 1.1263997104596004
    - 0.707106776245139
    - -8.068387989870796e-08
    - 5.093625398727014e-09
    - 0.7071067861279516
    - 0.07648213479841488
    - 0.025376874594948813
    - 0.9091425019454634
    - 0.7071067799070927
    - -1.719820219462202e-05
    - 1.8569938775315878e-05
    - 0.7071067820130151
    - -0.2108592674544802
    - 0.19975632731882462
    - 0.8991595343682753
    - 0.7071067709929622
    - -1.516567757206454e-06
    - 2.1445576536246358e-06
    - 0.7071067913752543
    - 0.048190304442709844
    - 0.18944436795399341
    - 0.9023145976636144
    - 0.7071068944202988
    - -1.0137770013841214e-05
    - 2.004602744377643e-05
    - 0.7071066675959596
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - -0.017120036236636823
    - -3.615236063587279e-05
    - 0.017806570384011886
    - -2.024970241428178e-05
    - 0.00041986806898601254
    - 8.5185770581784e-05
    - -0.04633388502686714
    - 0.05760762301107051
    - -0.05752320191953566
    - -9.589849902777965e-07
    - -8.070421475236387e-06
    - 0.007260295550235101
    - -7.334992368924057e-05
    - -0.0005716541856519713
    - 2.0850678297803277e-08
    - 8.718471192970883e-10
    - 5.332297586807914e-09
    - 1.842413025738705e-05
    - -4.994358103653669e-07
    - -3.0571636826752643e-06
    - -7.623863181881642e-12
    - -5.37528661292596e-07
    - -4.501375951823919e-06
    - 0.004920867709260157
    - -5.900522241764696e-05
    - -0.0005019248577587589
    - 3.350946722250984e-08
    - 1.6134108430674042e-09
    - -4.406652723503166e-08
    - 0.004850734336304681
    - -1.6344860316152752e-06
    - 5.026305041718309e-05
    - -4.2566036274492137e-10
    - 2.9156446724077335e-07
    - 3.052680763123791e-07
    - 0.004946092249603384
    - 0.00012342310665916346
    - 0.0001286360341746424
    - 5.282964695989359e-09
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

# Pick up the black bowl on the cookie box and place it on the plate

A Franka Panda arm with a parallel-jaw gripper works at a table in a LIBERO scene (MuJoCo / robosuite; LIBERO suite `libero_spatial`, task `pick_up_the_black_bowl_on_the_cookie_box_and_place_it_on_the_plate`). LIBERO-Spatial: the same objects in different layouts; the instruction says which of two identical bowls to move.

**Goal (LIBERO's language instruction):** pick up the black bowl on the cookie box and place it on the plate.

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
