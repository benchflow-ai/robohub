---
schema_version: '1.3'
task:
  name: robouse/libero-object-0
  description: Pick up the alphabet soup and place it in the basket.
metadata:
  author_name: benchflow
  source_benchmark: LIBERO (Liu et al. 2023)
  source_task: pick_up_the_alphabet_soup_and_place_it_in_the_basket
  suite: libero
  libero_suite: libero_object
  libero_task_index: 0
  language_instruction: pick up the alphabet soup and place it in the basket
  category: manipulation
  difficulty: easy
  tags:
  - franka
  - mujoco
  - robosuite
  - single-arm
  - language
  reference_solution: LIBERO demonstration demo_0 of yifengzhu-hf/LIBERO-datasets/libero_object/pick_up_the_alphabet_soup_and_place_it_in_the_basket_demo.hdf5 (replay reaches success at step 121)
  robouse:
    id: libero-object-0
    backend: libero
    suite: libero_object
    env: pick_up_the_alphabet_soup_and_place_it_in_the_basket
    task_index: 0
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
    init_source: yifengzhu-hf/LIBERO-datasets/libero_object/pick_up_the_alphabet_soup_and_place_it_in_the_basket_demo.hdf5 demo_0 states[0]
    init_state:
    - 0.25000000000000017
    - -0.013905872683010892
    - -0.1328952057535187
    - -0.03951907255554702
    - -2.4174816644984487
    - -0.015649966209845885
    - 2.2434208093632106
    - 0.770441485068051
    - 0.033941225377195405
    - -0.03412461578451979
    - -0.11934115262789488
    - -0.23984468630968295
    - 0.03837934574298576
    - -0.0022602213040863552
    - 0.002275415725608408
    - 0.7076953195934438
    - 0.706510473035231
    - -0.014371274648342883
    - 0.24592721728171635
    - -0.0045826267525761735
    - 0.7071047520207788
    - -0.0016938434289765466
    - 0.001693718997033999
    - 0.7071047531165592
    - 0.049986736067502695
    - -0.09997800008698375
    - 0.09144217291816688
    - 0.5001146704699069
    - 0.49988473331490013
    - 0.5000438075808332
    - 0.4999567584096952
    - -0.15198169028562752
    - 0.05506232109006241
    - 0.008921476051766767
    - 1.6041151477683685e-10
    - -3.163009425960641e-07
    - -1.0697785454441503e-06
    - 0.9999999999993777
    - 0.09555777693741475
    - -0.19812376224136655
    - 0.06699836627139005
    - 0.499972463349426
    - 0.5000275949228675
    - 0.49998355575483333
    - 0.5000163839142806
    - 0.15024227808724153
    - 0.03005554876583053
    - 0.03839986849134235
    - -0.0022909680042823327
    - 0.002291299998329956
    - 0.7076265223851385
    - 0.706579228556173
    - -0.19937919237624294
    - -0.07871577162706023
    - 0.0086954309406588
    - 8.874955155618174e-10
    - -4.1729229488417265e-07
    - -2.4216197991279674e-06
    - 0.999999999996981
    - -0.045048232597457154
    - 0.15501601371341342
    - -0.16564108678735487
    - 0.1618614551697931
    - -0.05955318640122909
    - -0.01203043902210241
    - -0.15613718711150051
    - 0.05764498625266167
    - -0.05718162727321916
    - -3.198494682225001e-05
    - 3.209575642820655e-06
    - 0.000909489769125492
    - 8.990287645626138e-05
    - -2.150929619063294e-06
    - -0.0008431013064610779
    - 8.814607143438503e-11
    - -7.433200612189256e-08
    - 1.8690324382868062e-05
    - -1.9330059625919374e-08
    - 1.6251681114238646e-05
    - -8.346251552124368e-11
    - -4.666910739172225e-05
    - 7.714755381933014e-05
    - -0.980411604248145
    - -0.0001913701024716004
    - 2.947579623944275e-06
    - -0.0003582907146141043
    - -3.4117384369509977e-12
    - -3.2811258913281077e-12
    - 9.13729708951697e-08
    - -4.485947119173626e-10
    - 4.0590351174281815e-10
    - 1.0457741964708946e-13
    - -8.449806116225133e-06
    - 5.7023120689933687e-05
    - -0.3452297316577747
    - -0.00014174999605561317
    - 1.019641022849361e-08
    - -0.000934021809140985
    - 1.533558786612869e-09
    - 3.0085265499401476e-09
    - 4.826518956931478e-08
    - 7.907978929667151e-08
    - -4.524252508963007e-10
    - 4.0437994690596145e-08
    - -9.635929329144576e-11
    - -1.9098383292269963e-11
    - 3.106862287450274e-06
    - -1.1063703994314658e-08
    - 1.2000058947011525e-08
    - 8.789747984118193e-12
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

# Pick up the alphabet soup and place it in the basket

A Franka Panda arm with a parallel-jaw gripper works over the floor in a LIBERO scene (MuJoCo / robosuite; LIBERO suite `libero_object`, task `pick_up_the_alphabet_soup_and_place_it_in_the_basket`). LIBERO-Object: the same layout with different grocery items; pick the named item and put it in the basket.

**Goal (LIBERO's language instruction):** pick up the alphabet soup and place it in the basket.

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
