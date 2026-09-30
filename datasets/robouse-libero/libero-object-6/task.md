---
schema_version: '1.3'
task:
  name: robouse/libero-object-6
  description: Pick up the butter and place it in the basket.
metadata:
  author_name: benchflow
  source_benchmark: LIBERO (Liu et al. 2023)
  source_task: pick_up_the_butter_and_place_it_in_the_basket
  suite: libero
  libero_suite: libero_object
  libero_task_index: 6
  language_instruction: pick up the butter and place it in the basket
  category: manipulation
  difficulty: easy
  tags:
  - franka
  - mujoco
  - robosuite
  - single-arm
  - language
  reference_solution: LIBERO demonstration demo_0 of yifengzhu-hf/LIBERO-datasets/libero_object/pick_up_the_butter_and_place_it_in_the_basket_demo.hdf5 (replay reaches success at step 158)
  robouse:
    id: libero-object-6
    backend: libero
    suite: libero_object
    env: pick_up_the_butter_and_place_it_in_the_basket
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
    init_source: yifengzhu-hf/LIBERO-datasets/libero_object/pick_up_the_butter_and_place_it_in_the_basket_demo.hdf5 demo_0 states[0]
    init_state:
    - 0.25000000000000017
    - -0.002073356818961984
    - -0.1524395652013989
    - 0.017717297341802375
    - -2.4390902891885573
    - 0.00686147565278338
    - 2.223530010070162
    - 0.8162096181959237
    - 0.03405372220944209
    - -0.034057943015913535
    - -0.11946720009558223
    - -0.2395408888030649
    - 0.0086954309406588
    - 8.87495515586755e-10
    - -4.1729229488213384e-07
    - -2.4216197991270975e-06
    - 0.999999999996981
    - -0.013240462554803503
    - 0.2582100232711622
    - -0.004582626752576174
    - 0.7071047520207787
    - -0.001693843428976539
    - 0.0016937189970339993
    - 0.7071047531165593
    - 0.05023895792473277
    - -0.09993834670103578
    - 0.03839258272994866
    - -0.0022609185008489116
    - 0.002261381952863357
    - 0.7076841208251468
    - 0.7065217332334052
    - -0.1500000139894531
    - 0.060002843464304595
    - 0.06699943919188384
    - 0.5000115735518467
    - 0.4999884062357617
    - 0.5000165521882278
    - 0.4999834672084934
    - 0.09999993635847314
    - -0.19999967244036956
    - 0.013065439187715341
    - 3.9431674593999495e-09
    - -3.527343612955697e-07
    - 1.1634988936070163e-06
    - 0.9999999999992609
    - 0.15658891544579487
    - 0.03597367796611047
    - 0.05462402814683191
    - 0.7071067808404041
    - 5.443398236748818e-08
    - 4.1950385953953075e-07
    - 0.7071067815325646
    - -0.20001836774182324
    - -0.08000519853222673
    - 0.08872548376508589
    - 0.5000380359614599
    - 0.4999620016247479
    - 0.499926020124606
    - 0.5000739284601361
    - -0.0003745080234250269
    - 0.05194601948705707
    - -0.022768453485981107
    - 0.04294629864744709
    - -0.0009514048329570254
    - 0.019761105960222874
    - 0.05706624269602109
    - 0.05749697560245333
    - -0.05762971622889606
    - -9.635929330921097e-11
    - -1.9098383304488994e-11
    - 3.106862287450156e-06
    - -1.1063703852258552e-08
    - 1.2000058771365719e-08
    - 8.789747821009174e-12
    - 8.81460709339637e-11
    - -7.433200612941177e-08
    - 1.869032438287996e-05
    - -1.9330059819926544e-08
    - 1.6251681115977476e-05
    - -8.346251663829148e-11
    - 2.501580328227275e-10
    - 1.0071061900806059e-09
    - 4.5539734491781066e-08
    - 2.658769589729946e-08
    - -1.6202770356423704e-10
    - 6.632979022906813e-09
    - -4.666449661321886e-06
    - -4.874318750110243e-05
    - -0.345264980765949
    - -6.387227688115295e-05
    - 1.290810692971022e-07
    - 0.0007338109785450136
    - -1.8340841861180493e-10
    - 3.6424152546542837e-10
    - 2.5318977393605805e-06
    - 3.0224714645764455e-08
    - 1.4442912189308281e-08
    - 2.1915099183738513e-12
    - 1.327497570491237e-07
    - 9.914017692615914e-08
    - 9.704703099458063e-05
    - 2.4701412148951127e-06
    - 1.8415272476247905e-06
    - -2.734968820390426e-10
    - -6.236917579375485e-05
    - -1.7353412856495513e-05
    - -0.9913011276643918
    - -0.00025108994971284214
    - -6.185912446322701e-07
    - 8.027456081417494e-05
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

# Pick up the butter and place it in the basket

A Franka Panda arm with a parallel-jaw gripper works over the floor in a LIBERO scene (MuJoCo / robosuite; LIBERO suite `libero_object`, task `pick_up_the_butter_and_place_it_in_the_basket`). LIBERO-Object: the same layout with different grocery items; pick the named item and put it in the basket.

**Goal (LIBERO's language instruction):** pick up the butter and place it in the basket.

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
