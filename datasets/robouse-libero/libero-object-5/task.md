---
schema_version: '1.3'
task:
  name: robouse/libero-object-5
  description: Pick up the tomato sauce and place it in the basket.
metadata:
  author_name: benchflow
  source_benchmark: LIBERO (Liu et al. 2023)
  source_task: pick_up_the_tomato_sauce_and_place_it_in_the_basket
  suite: libero
  libero_suite: libero_object
  libero_task_index: 5
  language_instruction: pick up the tomato sauce and place it in the basket
  category: manipulation
  difficulty: easy
  tags:
  - franka
  - mujoco
  - robosuite
  - single-arm
  - language
  reference_solution: LIBERO demonstration demo_0 of yifengzhu-hf/LIBERO-datasets/libero_object/pick_up_the_tomato_sauce_and_place_it_in_the_basket_demo.hdf5 (replay reaches success at step 121)
  robouse:
    id: libero-object-5
    backend: libero
    suite: libero_object
    env: pick_up_the_tomato_sauce_and_place_it_in_the_basket
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
    init_source: yifengzhu-hf/LIBERO-datasets/libero_object/pick_up_the_tomato_sauce_and_place_it_in_the_basket_demo.hdf5 demo_0 states[0]
    init_state:
    - 0.25000000000000017
    - -0.013966906620156597
    - -0.13774223008882802
    - 0.0009259005637277104
    - -2.47477447607607
    - 0.009384658917927818
    - 2.2460197256699437
    - 0.8223672931417945
    - 0.03405692829501561
    - -0.03405528482983878
    - 0.05023895792473278
    - -0.0999383467010358
    - 0.03839258272994866
    - -0.0022609185008489103
    - 0.002261381952863357
    - 0.7076841208251468
    - 0.7065217332334052
    - -0.007889594473920056
    - 0.2701750283714878
    - -0.0045826267525761735
    - 0.7071047520207788
    - -0.0016938434289765437
    - 0.0016937189970340006
    - 0.7071047531165592
    - -0.12031578590863698
    - -0.24095916908689302
    - 0.06699836627139005
    - 0.4999724633494259
    - 0.5000275949228674
    - 0.49998355575483316
    - 0.5000163839142807
    - -0.14594724641823242
    - 0.06271886846971825
    - 0.0086954309406588
    - 8.874955155753507e-10
    - -4.1729229489102605e-07
    - -2.4216197991262466e-06
    - 0.999999999996981
    - 0.09999998601054684
    - -0.19999715653569525
    - 0.06699943919188382
    - 0.5000115735518464
    - 0.4999884062357617
    - 0.5000165521882279
    - 0.49998346720849346
    - 0.1499999363584733
    - 0.030000327559630557
    - 0.013065439187715341
    - 3.943167459410005e-09
    - -3.527343612948644e-07
    - 1.1634988936087773e-06
    - 0.9999999999992609
    - -0.19093014581913048
    - -0.07034025325048371
    - 0.05462337560407025
    - 0.7071067804491808
    - 5.615071645884187e-08
    - 4.367153094226672e-07
    - 0.7071067819237771
    - 0.012296346027358483
    - 0.020467639894546707
    - -0.008714514367282883
    - -0.00032132275100574633
    - -0.009220080354314342
    - 0.015377049944306997
    - 4.8280459168866956e-05
    - 0.05759465732502807
    - -0.057532236294399096
    - 2.501580327462288e-10
    - 1.0071061901457168e-09
    - 4.5539734490156404e-08
    - 2.658769590794217e-08
    - -1.6202770380991852e-10
    - 6.632978988131385e-09
    - 8.814607230743386e-11
    - -7.43320061189468e-08
    - 1.8690324382872538e-05
    - -1.9330059693339273e-08
    - 1.6251681114139143e-05
    - -8.346251605721908e-11
    - -8.449806115613695e-06
    - 5.702312069039548e-05
    - -0.34522973165777476
    - -0.00014174999604762477
    - 1.019641020526847e-08
    - -0.0009340218091484449
    - -9.63593146108546e-11
    - -1.9098382580657678e-11
    - 3.106862287447974e-06
    - -1.1063703650618051e-08
    - 1.2000061143613005e-08
    - 8.789747718185688e-12
    - -4.666449661932239e-06
    - -4.87431875007042e-05
    - -0.3452649807659492
    - -6.387227689093561e-05
    - 1.2908106929323942e-07
    - 0.0007338109785390797
    - -1.8340841703575636e-10
    - 3.6424152586743995e-10
    - 2.531897739360734e-06
    - 3.0224714340131026e-08
    - 1.4442912471312558e-08
    - 2.1915107920269926e-12
    - 1.9840300043984953e-08
    - 1.4275438398796922e-08
    - 0.00014051051384667603
    - 3.6789593474261287e-07
    - 2.6426012211260734e-07
    - -3.0735270917027464e-11
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

# Pick up the tomato sauce and place it in the basket

A Franka Panda arm with a parallel-jaw gripper works over the floor in a LIBERO scene (MuJoCo / robosuite; LIBERO suite `libero_object`, task `pick_up_the_tomato_sauce_and_place_it_in_the_basket`). LIBERO-Object: the same layout with different grocery items; pick the named item and put it in the basket.

**Goal (LIBERO's language instruction):** pick up the tomato sauce and place it in the basket.

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
