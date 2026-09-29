---
schema_version: '1.3'
task:
  name: robouse/libero-object-3
  description: Pick up the bbq sauce and place it in the basket.
metadata:
  author_name: benchflow
  source_benchmark: LIBERO (Liu et al. 2023)
  source_task: pick_up_the_bbq_sauce_and_place_it_in_the_basket
  suite: libero
  libero_suite: libero_object
  libero_task_index: 3
  language_instruction: pick up the bbq sauce and place it in the basket
  category: manipulation
  difficulty: easy
  tags:
  - franka
  - mujoco
  - robosuite
  - single-arm
  - language
  reference_solution: LIBERO demonstration demo_0 of yifengzhu-hf/LIBERO-datasets/libero_object/pick_up_the_bbq_sauce_and_place_it_in_the_basket_demo.hdf5 (replay reaches success at step 124)
  robouse:
    id: libero-object-3
    backend: libero
    suite: libero_object
    env: pick_up_the_bbq_sauce_and_place_it_in_the_basket
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
    init_source: yifengzhu-hf/LIBERO-datasets/libero_object/pick_up_the_bbq_sauce_and_place_it_in_the_basket_demo.hdf5 demo_0 states[0]
    init_state:
    - 0.25000000000000017
    - 0.02557826296641626
    - -0.1334631593737072
    - 0.010694757720116205
    - -2.4339195253419024
    - -0.015704675211090334
    - 2.2434413179142174
    - 0.7818612264501031
    - 0.03405043881990413
    - -0.034060984975197
    - 0.049913780244934584
    - -0.1099557742041471
    - 0.05462337560407025
    - 0.7071067804491808
    - 5.6150716459646955e-08
    - 4.367153094209687e-07
    - 0.7071067819237771
    - 0.006483161411494311
    - 0.2629610593425194
    - -0.0045826267525761735
    - 0.7071047520207788
    - -0.001693843428976547
    - 0.0016937189970340017
    - 0.7071047531165592
    - -0.12000006364152686
    - -0.23999967244036957
    - 0.013065439187715341
    - 3.943167459414393e-09
    - -3.5273436130471174e-07
    - 1.1634988936093172e-06
    - 0.9999999999992609
    - -0.15001836774182323
    - 0.05999480146777321
    - 0.08872548376508589
    - 0.5000380359614599
    - 0.49996200162474796
    - 0.49992602012460596
    - 0.5000739284601361
    - 0.09998673606750277
    - -0.19997800008698383
    - 0.09144217291816688
    - 0.5001146704699069
    - 0.4998847333149001
    - 0.5000438075808332
    - 0.4999567584096952
    - 0.15066441912012846
    - 0.030144497099278366
    - 0.038386402222627174
    - -0.002290376874245718
    - 0.002304894438700414
    - 0.7076290945911723
    - 0.7065766102295478
    - -0.20130777925543522
    - -0.08475738576108072
    - 0.008921476051766767
    - 1.6041151477647193e-10
    - -3.163009425892925e-07
    - -1.0697785454446583e-06
    - 0.9999999999993777
    - -0.023745836121849447
    - 0.045892109953313415
    - 0.01643373514544981
    - 0.03461760184963845
    - 0.005083987295863464
    - 0.03005590277981238
    - 0.020238976207638263
    - 0.05747889835217279
    - -0.05765630509281785
    - 1.9840300013279512e-08
    - 1.4275438380926507e-08
    - 0.00014051051384666988
    - 3.678959341814768e-07
    - 2.6426012175984004e-07
    - -3.073527076989312e-11
    - 8.814606869503244e-11
    - -7.433200612207373e-08
    - 1.869032438286314e-05
    - -1.93300596410764e-08
    - 1.6251681114184134e-05
    - -8.34625156627236e-11
    - -1.8340842134968164e-10
    - 3.6424152578700103e-10
    - 2.5318977393603175e-06
    - 3.022471460469575e-08
    - 1.4442912949471974e-08
    - 2.1915113142583466e-12
    - -6.236917579382435e-05
    - -1.7353412856552268e-05
    - -0.9913011276643918
    - -0.0002510899497131958
    - -6.185912446334743e-07
    - 8.027456081452698e-05
    - -4.666910739177722e-05
    - 7.714755381935245e-05
    - -0.980411604248145
    - -0.0001913701024718098
    - 2.9475796239465935e-06
    - -0.0003582907146142308
    - -1.970251654136506e-05
    - 7.817692165478403e-05
    - 0.0009233684592240263
    - 0.002048914911406433
    - -1.413759963323688e-05
    - -0.0005159852006272631
    - -3.4117384452812473e-12
    - -3.2811241141026645e-12
    - 9.137297089766851e-08
    - -4.4859442643398156e-10
    - 4.0590359418010804e-10
    - 1.0457747413524578e-13
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

# Pick up the bbq sauce and place it in the basket

A Franka Panda arm with a parallel-jaw gripper works over the floor in a LIBERO scene (MuJoCo / robosuite; LIBERO suite `libero_object`, task `pick_up_the_bbq_sauce_and_place_it_in_the_basket`). LIBERO-Object: the same layout with different grocery items; pick the named item and put it in the basket.

**Goal (LIBERO's language instruction):** pick up the bbq sauce and place it in the basket.

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
