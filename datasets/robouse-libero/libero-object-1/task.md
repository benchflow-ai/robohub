---
schema_version: '1.3'
task:
  name: robouse/libero-object-1
  description: Pick up the cream cheese and place it in the basket.
metadata:
  author_name: benchflow
  source_benchmark: LIBERO (Liu et al. 2023)
  source_task: pick_up_the_cream_cheese_and_place_it_in_the_basket
  suite: libero
  libero_suite: libero_object
  libero_task_index: 1
  language_instruction: pick up the cream cheese and place it in the basket
  category: manipulation
  difficulty: easy
  tags:
  - franka
  - mujoco
  - robosuite
  - single-arm
  - language
  reference_solution: LIBERO demonstration demo_0 of yifengzhu-hf/LIBERO-datasets/libero_object/pick_up_the_cream_cheese_and_place_it_in_the_basket_demo.hdf5 (replay reaches success at step 128)
  robouse:
    id: libero-object-1
    backend: libero
    suite: libero_object
    env: pick_up_the_cream_cheese_and_place_it_in_the_basket
    task_index: 1
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
    init_source: yifengzhu-hf/LIBERO-datasets/libero_object/pick_up_the_cream_cheese_and_place_it_in_the_basket_demo.hdf5 demo_0 states[0]
    init_state:
    - 0.25000000000000017
    - 0.0012473326900477079
    - -0.15650527393544125
    - 0.03715338880716834
    - -2.4360482761700974
    - 0.0075228456690515045
    - 2.2280652634930815
    - 0.7854274412901053
    - 0.03406611940618487
    - -0.034044898264240044
    - 0.05345949164380829
    - -0.1030376047164858
    - 0.008921476051766767
    - 1.6041151478287124e-10
    - -3.163009425904928e-07
    - -1.0697785454425784e-06
    - 0.9999999999993777
    - 0.010440216115074138
    - 0.26994856154108304
    - -0.0045826267525761735
    - 0.7071047520207788
    - -0.0016938434289765479
    - 0.001693718997034003
    - 0.7071047531165592
    - -0.11934115262789488
    - -0.23984468630968298
    - 0.03837934574298576
    - -0.0022602213040863587
    - 0.002275415725608412
    - 0.7076953195934438
    - 0.706510473035231
    - -0.15134748981317878
    - 0.062053200924753284
    - 0.06699836627139005
    - 0.499972463349426
    - 0.5000275949228675
    - 0.49998355575483333
    - 0.5000163839142806
    - 0.1002389579247328
    - -0.19993834670103577
    - 0.03839258272994866
    - -0.00226091850084891
    - 0.002261381952863358
    - 0.7076841208251468
    - 0.7065217332334052
    - 0.1544443351325269
    - 0.02935554219221972
    - 0.0086954309406588
    - 8.874955155611242e-10
    - -4.1729229489253747e-07
    - -2.4216197991293014e-06
    - 0.999999999996981
    - -0.2000000139894531
    - -0.07999715653569539
    - 0.06699943919188384
    - 0.5000115735518467
    - 0.4999884062357617
    - 0.5000165521882278
    - 0.4999834672084934
    - 0.004803713333376864
    - 0.09164783487309724
    - -0.012287894255125564
    - 0.07759417040116347
    - 0.0014620639746320413
    - 0.001023962537754922
    - 0.07907183781294855
    - 0.05747323138096013
    - -0.05766756210736432
    - -3.41173840543921e-12
    - -3.2811258759593952e-12
    - 9.137297089778216e-08
    - -4.4859450490601533e-10
    - 4.0590345192115114e-10
    - 1.0457719538578223e-13
    - 8.814607232698272e-11
    - -7.433200612464167e-08
    - 1.8690324382869302e-05
    - -1.9330059694124377e-08
    - 1.6251681114225293e-05
    - -8.346251580065699e-11
    - -3.198494682225076e-05
    - 3.2095756428224284e-06
    - 0.0009094897691254884
    - 8.990287645629871e-05
    - -2.150929619064235e-06
    - -0.0008431013064611819
    - -8.449806116258885e-06
    - 5.7023120689845344e-05
    - -0.34522973165777465
    - -0.00014174999605586551
    - 1.0196410233601133e-08
    - -0.0009340218091402745
    - 2.5015803261201956e-10
    - 1.007106190616275e-09
    - 4.5539734490944396e-08
    - 2.6587695863287984e-08
    - -1.6202770376965812e-10
    - 6.632978987094658e-09
    - -9.635931460583597e-11
    - -1.9098383296035777e-11
    - 3.106862287443906e-06
    - -1.1063703985559292e-08
    - 1.200006141751471e-08
    - 8.789747990633156e-12
    - -4.666449661435019e-06
    - -4.8743187500964866e-05
    - -0.34526498076594897
    - -6.387227688284643e-05
    - 1.2908106929767973e-07
    - 0.0007338109785432093
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

# Pick up the cream cheese and place it in the basket

A Franka Panda arm with a parallel-jaw gripper works over the floor in a LIBERO scene (MuJoCo / robosuite; LIBERO suite `libero_object`, task `pick_up_the_cream_cheese_and_place_it_in_the_basket`). LIBERO-Object: the same layout with different grocery items; pick the named item and put it in the basket.

**Goal (LIBERO's language instruction):** pick up the cream cheese and place it in the basket.

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
