---
schema_version: '1.3'
task:
  name: robouse/libero-object-2
  description: Pick up the salad dressing and place it in the basket.
metadata:
  author_name: benchflow
  source_benchmark: LIBERO (Liu et al. 2023)
  source_task: pick_up_the_salad_dressing_and_place_it_in_the_basket
  suite: libero
  libero_suite: libero_object
  libero_task_index: 2
  language_instruction: pick up the salad dressing and place it in the basket
  category: manipulation
  difficulty: easy
  tags:
  - franka
  - mujoco
  - robosuite
  - single-arm
  - language
  reference_solution: LIBERO demonstration demo_1 of yifengzhu-hf/LIBERO-datasets/libero_object/pick_up_the_salad_dressing_and_place_it_in_the_basket_demo.hdf5 (replay reaches success at step 113)
  robouse:
    id: libero-object-2
    backend: libero
    suite: libero_object
    env: pick_up_the_salad_dressing_and_place_it_in_the_basket
    task_index: 2
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
    init_source: yifengzhu-hf/LIBERO-datasets/libero_object/pick_up_the_salad_dressing_and_place_it_in_the_basket_demo.hdf5 demo_1 states[0]
    init_state:
    - 0.25000000000000017
    - -0.009176087556614738
    - -0.1453169373679933
    - 0.0008662565613499055
    - -2.432617356757152
    - -0.01232802597989281
    - 2.2348847858935685
    - 0.7688854076674996
    - 0.034044247728298685
    - -0.03406791812082404
    - 0.049986736067502695
    - -0.09997800008698375
    - 0.09144217291816688
    - 0.5001146704699069
    - 0.4998847333149001
    - 0.5000438075808332
    - 0.4999567584096952
    - -0.009824902977237935
    - 0.2684704390979882
    - -0.0045826267525761735
    - 0.7071047520207788
    - -0.0016938434289765498
    - 0.001693718997034003
    - 0.7071047531165592
    - -0.12001836774182317
    - -0.24000519853222674
    - 0.08872548376508589
    - 0.5000380359614599
    - 0.49996200162474796
    - 0.499926020124606
    - 0.5000739284601361
    - -0.1493411526278948
    - 0.0601553136903171
    - 0.03837934574298575
    - -0.0022602213040863457
    - 0.0022754157256083974
    - 0.7076953195934438
    - 0.7065104730352308
    - 0.103255108165062
    - -0.1980225737852775
    - 0.008921476051766767
    - 1.604115147774741e-10
    - -3.1630094259562687e-07
    - -1.0697785454434125e-06
    - 0.9999999999993777
    - 0.14710668119461395
    - 0.028391353341198158
    - 0.06699836627139005
    - 0.4999724633494259
    - 0.5000275949228674
    - 0.49998355575483316
    - 0.5000163839142807
    - -0.19976104207526726
    - -0.0799383467010358
    - 0.03839258272994866
    - -0.002260918500848905
    - 0.002261381952863352
    - 0.7076841208251468
    - 0.7065217332334052
    - 0.0067377056535617985
    - -0.0185722796057513
    - -0.006497961192004376
    - -0.03136256061582585
    - -6.30523264621325e-05
    - 0.0447114730701227
    - 4.527564861753801e-05
    - 0.05759166663309558
    - -0.05753579629613188
    - -4.6669107391775746e-05
    - 7.714755381937541e-05
    - -0.980411604248145
    - -0.00019137010247182018
    - 2.9475796239467624e-06
    - -0.00035829071461426673
    - 8.81460699680788e-11
    - -7.433200612044487e-08
    - 1.86903243828637e-05
    - -1.933005958724876e-08
    - 1.6251681114228424e-05
    - -8.346251556255013e-11
    - -6.236917579377595e-05
    - -1.735341285655875e-05
    - -0.9913011276643918
    - -0.0002510899497129952
    - -6.185912446338268e-07
    - 8.027456081460236e-05
    - -3.198494682229785e-05
    - 3.209575642844984e-06
    - 0.0009094897691259768
    - 8.990287645699054e-05
    - -2.1509296190715086e-06
    - -0.0008431013064624393
    - -3.4117384041755476e-12
    - -3.281125869658369e-12
    - 9.13729708935193e-08
    - -4.4859475705923567e-10
    - 4.059034325762262e-10
    - 1.0457735505846966e-13
    - -8.449806115548618e-06
    - 5.7023120690356525e-05
    - -0.34522973165777465
    - -0.00014174999604582691
    - 1.0196410225759257e-08
    - -0.0009340218091473633
    - 2.5015801853248514e-10
    - 1.0071062597934509e-09
    - 4.5539734480314815e-08
    - 2.658769770946197e-08
    - -1.6202771601841067e-10
    - 6.632978698679006e-09
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

# Pick up the salad dressing and place it in the basket

A Franka Panda arm with a parallel-jaw gripper works over the floor in a LIBERO scene (MuJoCo / robosuite; LIBERO suite `libero_object`, task `pick_up_the_salad_dressing_and_place_it_in_the_basket`). LIBERO-Object: the same layout with different grocery items; pick the named item and put it in the basket.

**Goal (LIBERO's language instruction):** pick up the salad dressing and place it in the basket.

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
