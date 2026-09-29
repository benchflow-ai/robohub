---
schema_version: '1.3'
task:
  name: robouse/libero-spatial-5
  description: Pick up the black bowl on the ramekin and place it on the plate.
metadata:
  author_name: benchflow
  source_benchmark: LIBERO (Liu et al. 2023)
  source_task: pick_up_the_black_bowl_on_the_ramekin_and_place_it_on_the_plate
  suite: libero
  libero_suite: libero_spatial
  libero_task_index: 5
  language_instruction: pick up the black bowl on the ramekin and place it on the plate
  category: manipulation
  difficulty: easy
  tags:
  - franka
  - mujoco
  - robosuite
  - single-arm
  - language
  reference_solution: LIBERO demonstration demo_0 of yifengzhu-hf/LIBERO-datasets/libero_spatial/pick_up_the_black_bowl_on_the_ramekin_and_place_it_on_the_plate_demo.hdf5 (replay reaches success at step 121)
  robouse:
    id: libero-spatial-5
    backend: libero
    suite: libero_spatial
    env: pick_up_the_black_bowl_on_the_ramekin_and_place_it_on_the_plate
    task_index: 5
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
    init_source: yifengzhu-hf/LIBERO-datasets/libero_spatial/pick_up_the_black_bowl_on_the_ramekin_and_place_it_on_the_plate_demo.hdf5 demo_0 states[0]
    init_state:
    - 0.25000000000000017
    - -0.006848000170091198
    - -0.15734975097364756
    - -0.00561581107807469
    - -2.449637437618366
    - -0.0066247474970927926
    - 2.2114887448023755
    - 0.7992141273076543
    - 0.0340697607071729
    - -0.034034361307377796
    - -0.2174529565331339
    - 0.20066078258248993
    - 0.9530954330504452
    - 0.7434372224599407
    - 0.043497433984756785
    - 0.1999557517026401
    - 0.6367313144952295
    - 0.056743217135020216
    - 0.016761333553431096
    - 0.9169196589889572
    - 0.7071067735325446
    - -1.7667188708212744e-05
    - 1.897523742108281e-05
    - 0.707106788365241
    - 0.05674377818872586
    - 0.016776968276795822
    - 0.9091425019454634
    - 0.7071067799070927
    - -1.7198202194634913e-05
    - 1.8569938775312317e-05
    - 0.7071067820130151
    - -0.2061899805744845
    - 0.2027554007263493
    - 0.8991970641687556
    - 0.7073019878838721
    - -9.901919942241797e-05
    - 0.0004024914225717924
    - 0.7069113990673624
    - 0.06942298948931576
    - 0.21428822618266194
    - 0.9023145976636144
    - 0.7071068944202988
    - -1.0137770013838664e-05
    - 2.0046027443779624e-05
    - 0.7071066675959596
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.02408684253971988
    - 0.001705267635433588
    - 0.06695054976352222
    - 0.006138856529974259
    - -0.004079524198256383
    - 0.0028339122111657074
    - 0.08771947523590498
    - 0.05754128153103366
    - -0.05754593933328104
    - -0.08607122755174491
    - 0.019603931801160712
    - 0.008541126835521749
    - 2.282990657014829
    - 1.340584235123291
    - -1.30725880489858
    - -9.589849902804588e-07
    - -8.070421475234788e-06
    - 0.007260295550235113
    - -7.334992368913937e-05
    - -0.0005716541856518726
    - 2.0850678297913044e-08
    - -5.375286612924285e-07
    - -4.5013759518241185e-06
    - 0.004920867709260174
    - -5.900522241749022e-05
    - -0.0005019248577587311
    - 3.350946722233157e-08
    - 1.0453533943114748e-05
    - 1.6997268406596113e-05
    - 0.003848544818582137
    - -0.011548348613941564
    - -0.01893429191421887
    - 6.386979396132496e-08
    - 2.915644672403256e-07
    - 3.052680763130938e-07
    - 0.004946092249603383
    - 0.00012342310665924527
    - 0.0001286360341747274
    - 5.282964696318613e-09
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

# Pick up the black bowl on the ramekin and place it on the plate

A Franka Panda arm with a parallel-jaw gripper works at a table in a LIBERO scene (MuJoCo / robosuite; LIBERO suite `libero_spatial`, task `pick_up_the_black_bowl_on_the_ramekin_and_place_it_on_the_plate`). LIBERO-Spatial: the same objects in different layouts; the instruction says which of two identical bowls to move.

**Goal (LIBERO's language instruction):** pick up the black bowl on the ramekin and place it on the plate.

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
