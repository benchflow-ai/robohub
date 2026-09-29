---
schema_version: '1.3'
task:
  name: robouse/libero-object-4
  description: Pick up the ketchup and place it in the basket.
metadata:
  author_name: benchflow
  source_benchmark: LIBERO (Liu et al. 2023)
  source_task: pick_up_the_ketchup_and_place_it_in_the_basket
  suite: libero
  libero_suite: libero_object
  libero_task_index: 4
  language_instruction: pick up the ketchup and place it in the basket
  category: manipulation
  difficulty: easy
  tags:
  - franka
  - mujoco
  - robosuite
  - single-arm
  - language
  reference_solution: LIBERO demonstration demo_1 of yifengzhu-hf/LIBERO-datasets/libero_object/pick_up_the_ketchup_and_place_it_in_the_basket_demo.hdf5 (replay reaches success at step 145)
  robouse:
    id: libero-object-4
    backend: libero
    suite: libero_object
    env: pick_up_the_ketchup_and_place_it_in_the_basket
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
    init_source: yifengzhu-hf/LIBERO-datasets/libero_object/pick_up_the_ketchup_and_place_it_in_the_basket_demo.hdf5 demo_1 states[0]
    init_state:
    - 0.25000000000000017
    - -0.01465208886045434
    - -0.15806801736879666
    - 0.012066317944520176
    - -2.458774553214278
    - -0.012584708519145872
    - 2.22592317296613
    - 0.7983860672681151
    - 0.034039264419589245
    - -0.03407278906149884
    - -0.12001836774182317
    - -0.24000519853222674
    - 0.08872548376508589
    - 0.5000380359614599
    - 0.4999620016247479
    - 0.499926020124606
    - 0.5000739284601361
    - 0.0008640093916956368
    - 0.24659107560026927
    - -0.0045826267525761735
    - 0.7071047520207788
    - -0.0016938434289765507
    - 0.0016937189970340032
    - 0.7071047531165592
    - 0.058306054081078654
    - -0.10595319329474603
    - 0.05462337560407025
    - 0.7071067804491808
    - 5.615071646275904e-08
    - 4.3671530942387105e-07
    - 0.7071067819237771
    - -0.1500132639324975
    - 0.0600219999130162
    - 0.09144217291816688
    - 0.5001146704699069
    - 0.49988473331490013
    - 0.5000438075808331
    - 0.4999567584096953
    - 0.10065884737210512
    - -0.19984468630968297
    - 0.03837934574298576
    - -0.00226022130408636
    - 0.0022754157256084048
    - 0.7076953195934438
    - 0.706510473035231
    - 0.15396585609928565
    - 0.03336950161557881
    - 0.008921476051766767
    - 1.6041151474185644e-10
    - -3.163009425893913e-07
    - -1.06977854544284e-06
    - 0.9999999999993777
    - -0.1950175915589215
    - -0.07770084359416662
    - 0.06699836627139005
    - 0.49997246334942597
    - 0.5000275949228675
    - 0.4999835557548333
    - 0.5000163839142806
    - 0.01608596724947315
    - -0.001650298056602134
    - -0.012995539233364283
    - -0.002403668934086041
    - -0.02158364111393385
    - 0.0017464601945677352
    - -0.018838509073685648
    - 0.05764278403238615
    - -0.057485883113259834
    - -6.236917579377984e-05
    - -1.735341285654107e-05
    - -0.9913011276643918
    - -0.0002510899497129511
    - -6.185912446327525e-07
    - 8.027456081430631e-05
    - 8.814606996201046e-11
    - -7.433200612409795e-08
    - 1.869032438286444e-05
    - -1.933005958764399e-08
    - 1.6251681114287e-05
    - -8.346251530112395e-11
    - 1.9840300043280357e-08
    - 1.4275438398930002e-08
    - 0.00014051051384667438
    - 3.6789593472930614e-07
    - 2.642601221497781e-07
    - -3.073527109395208e-11
    - -4.6669107391817413e-05
    - 7.714755381919286e-05
    - -0.9804116042481448
    - -0.00019137010247195112
    - 2.947579623947373e-06
    - -0.00035829071461357745
    - -3.1984946822249676e-05
    - 3.2095756428203354e-06
    - 0.0009094897691254888
    - 8.990287645627687e-05
    - -2.1509296190635135e-06
    - -0.0008431013064610744
    - -3.4117384300060446e-12
    - -3.281125895644552e-12
    - 9.137297090027924e-08
    - -4.4859496258670603e-10
    - 4.059033812389936e-10
    - 1.0457740355345175e-13
    - -8.449806115782419e-06
    - 5.7023120690148616e-05
    - -0.345229731657774
    - -0.0001417499960498284
    - 1.0196410215821481e-08
    - -0.000934021809144095
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

# Pick up the ketchup and place it in the basket

A Franka Panda arm with a parallel-jaw gripper works over the floor in a LIBERO scene (MuJoCo / robosuite; LIBERO suite `libero_object`, task `pick_up_the_ketchup_and_place_it_in_the_basket`). LIBERO-Object: the same layout with different grocery items; pick the named item and put it in the basket.

**Goal (LIBERO's language instruction):** pick up the ketchup and place it in the basket.

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
