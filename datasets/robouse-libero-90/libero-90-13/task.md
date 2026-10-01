---
schema_version: '1.3'
task:
  name: robouse/libero-90-13
  description: Put the black bowl at the front on the plate.
metadata:
  author_name: benchflow
  source_benchmark: LIBERO (Liu et al. 2023)
  source_task: KITCHEN_SCENE2_put_the_black_bowl_at_the_front_on_the_plate
  suite: libero-90
  libero_suite: libero_90
  libero_task_index: 13
  language_instruction: put the black bowl at the front on the plate
  category: manipulation
  difficulty: medium
  tags:
  - franka
  - mujoco
  - robosuite
  - single-arm
  - language
  reference_solution: LIBERO demonstration demo_0 of yifengzhu-hf/LIBERO-datasets/libero_90/KITCHEN_SCENE2_put_the_black_bowl_at_the_front_on_the_plate_demo.hdf5 (replay reaches success at step 123)
  robouse:
    id: libero-90-13
    backend: libero
    suite: libero_90
    env: KITCHEN_SCENE2_put_the_black_bowl_at_the_front_on_the_plate
    task_index: 13
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
    init_source: yifengzhu-hf/LIBERO-datasets/libero_90/KITCHEN_SCENE2_put_the_black_bowl_at_the_front_on_the_plate_demo.hdf5 demo_0 states[0]
    init_state:
    - 0.25000000000000017
    - 0.022552593406312955
    - -0.1493824732933395
    - 0.002882618558840883
    - -2.423744694484115
    - -0.00522660811015221
    - 2.2671249713900687
    - 0.7862638084408258
    - 0.034057785076285076
    - -0.034049713587761
    - 0.11757002429624064
    - 0.15054762558918697
    - 0.8981585151399347
    - 0.7071067776242912
    - -1.0724537191719246e-05
    - -1.4310617004711641e-06
    - 0.7071067846660275
    - -0.05330575381146472
    - 0.1867920615161565
    - 0.8981585151399347
    - 0.7071067776242912
    - -1.0724537191718355e-05
    - -1.4310617004702813e-06
    - 0.7071067846660275
    - -0.14621757625023055
    - 0.07355312246779401
    - 0.8981585151399347
    - 0.7071067776242912
    - -1.0724537191719956e-05
    - -1.431061700470815e-06
    - 0.7071067846660275
    - 0.014110269038681802
    - 0.012817456818056086
    - 0.9023145976636144
    - 0.7071068944202988
    - -1.0137770013841817e-05
    - 2.0046027443786614e-05
    - 0.7071066675959596
    - 0.0
    - 0.0
    - 0.0
    - -0.014347757043729658
    - 0.04336716023524918
    - 0.04339317008692299
    - 0.049178258557455445
    - 3.8725614443094216e-05
    - 0.0026511597331108612
    - 0.05000204662456644
    - 0.057278180886696194
    - -0.05787720104060988
    - 1.4146964852688094e-07
    - -3.4906643731011574e-08
    - 0.00656114118906862
    - -7.860505322553319e-05
    - 2.077506965453004e-05
    - -1.2876260704725001e-09
    - 1.4146964852588713e-07
    - -3.4906643731327014e-08
    - 0.006561141189068619
    - -7.860505322550332e-05
    - 2.0775069654540826e-05
    - -1.2876260703221621e-09
    - 1.4146964852443442e-07
    - -3.490664373069186e-08
    - 0.0065611411890686136
    - -7.860505322541431e-05
    - 2.0775069654597174e-05
    - -1.2876260701015586e-09
    - 2.9156446724013564e-07
    - 3.052680763134686e-07
    - 0.004946092249603383
    - 0.00012342310665873526
    - 0.00012863603417487144
    - 5.2829646957298145e-09
    - 0.0
    - 0.0
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

# Put the black bowl at the front on the plate

A Franka Panda arm with a parallel-jaw gripper works at a table in a LIBERO scene (MuJoCo / robosuite; LIBERO suite `libero_90`, task `KITCHEN_SCENE2_put_the_black_bowl_at_the_front_on_the_plate`). LIBERO-90: 90 short-horizon tasks in 20 kitchen, living-room and study scenes (the task name starts with the scene); LIBERO uses them for pretraining.

**Goal (LIBERO's language instruction):** put the black bowl at the front on the plate.

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
