---
schema_version: '1.3'
task:
  name: robouse/libero-object-7
  description: Pick up the milk and place it in the basket.
metadata:
  author_name: benchflow
  source_benchmark: LIBERO (Liu et al. 2023)
  source_task: pick_up_the_milk_and_place_it_in_the_basket
  suite: libero
  libero_suite: libero_object
  libero_task_index: 7
  language_instruction: pick up the milk and place it in the basket
  category: manipulation
  difficulty: easy
  tags:
  - franka
  - mujoco
  - robosuite
  - single-arm
  - language
  reference_solution: LIBERO demonstration demo_0 of yifengzhu-hf/LIBERO-datasets/libero_object/pick_up_the_milk_and_place_it_in_the_basket_demo.hdf5 (replay reaches success at step 139)
  robouse:
    id: libero-object-7
    backend: libero
    suite: libero_object
    env: pick_up_the_milk_and_place_it_in_the_basket
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
    init_source: yifengzhu-hf/LIBERO-datasets/libero_object/pick_up_the_milk_and_place_it_in_the_basket_demo.hdf5 demo_0 states[0]
    init_state:
    - 0.25000000000000017
    - 0.00266595151613318
    - -0.16237007346594412
    - -0.024524947291115467
    - -2.4526359368492288
    - 0.018758913005413977
    - 2.1969596705148957
    - 0.8104755454949353
    - 0.034034967356673294
    - -0.03406494492567341
    - -0.12104206390872352
    - -0.2388144216908309
    - 0.0671277174592644
    - 0.4999856413017228
    - 0.5000143923132085
    - 0.4999906528035471
    - 0.500009312994109
    - 0.010602284244610436
    - 0.24572124149582975
    - -0.004582626752576174
    - 0.7071047520207787
    - -0.0016938434289765407
    - 0.001693718997033996
    - 0.7071047531165593
    - 0.0485320953598164
    - -0.09989390314337836
    - 0.008921476051766767
    - 1.604115147687428e-10
    - -3.1630094258426805e-07
    - -1.0697785454456034e-06
    - 0.9999999999993777
    - -0.14976104207526725
    - 0.060061653298964235
    - 0.03839258272994866
    - -0.0022609185008489103
    - 0.002261381952863357
    - 0.7076841208251468
    - 0.7065217332334052
    - 0.09668119125088818
    - -0.1957245974407596
    - 0.0086954309406588
    - 8.874955155719993e-10
    - -4.172922948829315e-07
    - -2.421619799126347e-06
    - 0.999999999996981
    - 0.14999998601054693
    - 0.030002843464304617
    - 0.06699943919188384
    - 0.5000115735518467
    - 0.4999884062357617
    - 0.5000165521882278
    - 0.4999834672084934
    - -0.2000000636415267
    - -0.0799996724403695
    - 0.013065439187715341
    - 3.943167459430426e-09
    - -3.527343612944518e-07
    - 1.1634988936076556e-06
    - 0.9999999999992609
    - -0.033957624827947605
    - 0.06341920028424791
    - -0.06524889695789134
    - 0.07384761348251934
    - -0.010175850824888493
    - -0.11190285820620445
    - 0.03880624016999625
    - 0.05780047912837511
    - -0.05728120045077164
    - -6.010090146003241e-07
    - 3.146628739704183e-05
    - -0.34723634376285417
    - -1.877976071460946e-05
    - 6.10998442485857e-08
    - -0.0005327449793995661
    - 8.81460748481855e-11
    - -7.43320061279101e-08
    - 1.86903243828811e-05
    - -1.9330059878944262e-08
    - 1.6251681115956666e-05
    - -8.346251694445844e-11
    - -3.411724303052364e-12
    - -3.2811240949584824e-12
    - 9.137297090000196e-08
    - -4.4859448594851686e-10
    - 4.0590201841543867e-10
    - 1.0457756754751441e-13
    - 2.5015803255645845e-10
    - 1.0071061907301437e-09
    - 4.5539734491084706e-08
    - 2.658769590694097e-08
    - -1.6202770380930597e-10
    - 6.6329789711600035e-09
    - -9.635929331912378e-11
    - -1.909838334124707e-11
    - 3.106862287446055e-06
    - -1.1063703704327967e-08
    - 1.2000058744220848e-08
    - 8.789747685251301e-12
    - -4.6664496612950745e-06
    - -4.8743187501062044e-05
    - -0.345264980765949
    - -6.387227688043289e-05
    - 1.2908106930045768e-07
    - 0.0007338109785445145
    - -1.8340841847224115e-10
    - 3.642415258625971e-10
    - 2.531897739365258e-06
    - 3.0224714596128536e-08
    - 1.4442912566133074e-08
    - 2.191510866557131e-12
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

# Pick up the milk and place it in the basket

A Franka Panda arm with a parallel-jaw gripper works over the floor in a LIBERO scene (MuJoCo / robosuite; LIBERO suite `libero_object`, task `pick_up_the_milk_and_place_it_in_the_basket`). LIBERO-Object: the same layout with different grocery items; pick the named item and put it in the basket.

**Goal (LIBERO's language instruction):** pick up the milk and place it in the basket.

The episode ends as solved the moment LIBERO's own success check (the task's goal predicates) passes; you do not need to call `robo done` after that. If you finish without the check passing, call `robo done` (or `robo give-up`).

## This robot

- The action has 7 numbers, not 4: `robo act DX DY DZ DROLL DPITCH DYAW GRIPPER`, each in [-1, 1]. It is LIBERO's delta end-effector command (robosuite's OSC_POSE controller, 20 steps per second). Holding DX/DY/DZ at 1.0 moves the hand about 1 cm per step; DROLL/DPITCH/DYAW rotate the hand (1.0 asks for 0.5 rad; usually leave them at 0). GRIPPER -1 opens, +1 closes.
- `robo move-to X Y Z [--grip G]` and `robo grip G` are available (they keep the hand's orientation).
- World frame, metres: +x points forward, away from the robot's base; +y is to the robot's left; +z is up. The objects stand on the floor, at about z = 0.0.
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
