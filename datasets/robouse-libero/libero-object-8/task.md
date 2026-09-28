---
schema_version: '1.3'
task:
  name: robouse/libero-object-8
  description: Pick up the chocolate pudding and place it in the basket.
metadata:
  author_name: benchflow
  source_benchmark: LIBERO (Liu et al. 2023)
  source_task: pick_up_the_chocolate_pudding_and_place_it_in_the_basket
  suite: libero
  libero_suite: libero_object
  libero_task_index: 8
  language_instruction: pick up the chocolate pudding and place it in the basket
  category: manipulation
  difficulty: easy
  tags:
  - franka
  - mujoco
  - robosuite
  - single-arm
  - language
  reference_solution: LIBERO demonstration demo_0 of yifengzhu-hf/LIBERO-datasets/libero_object/pick_up_the_chocolate_pudding_and_place_it_in_the_basket_demo.hdf5 (replay reaches success at step 158)
  robouse:
    id: libero-object-8
    backend: libero
    suite: libero_object
    env: pick_up_the_chocolate_pudding_and_place_it_in_the_basket
    task_index: 8
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
    init_source: yifengzhu-hf/LIBERO-datasets/libero_object/pick_up_the_chocolate_pudding_and_place_it_in_the_basket_demo.hdf5 demo_0 states[0]
    init_state:
    - 0.25000000000000017
    - -0.01775126935233975
    - -0.15323808892764237
    - 0.013023216689692448
    - -2.4408350948524338
    - 0.00041223380016766387
    - 2.219977082661683
    - 0.7990154781673625
    - 0.034055076759877935
    - -0.034055229832223324
    - -0.12000006364152686
    - -0.23999967244036957
    - 0.013065439187715341
    - 3.9431674594183196e-09
    - -3.5273436129885313e-07
    - 1.1634988936068098e-06
    - 0.9999999999992609
    - 0.011411086201557367
    - 0.2678974868280785
    - -0.0045826267525761735
    - 0.7071047520207788
    - -0.0016938434289765483
    - 0.0016937189970340023
    - 0.7071047531165592
    - 0.049999986010546886
    - -0.09999715653569541
    - 0.06699943919188384
    - 0.5000115735518464
    - 0.49998840623576174
    - 0.5000165521882279
    - 0.49998346720849346
    - -0.14746343601549255
    - 0.050025109909042305
    - 0.05462337560407025
    - 0.7071067804491808
    - 5.615071645964921e-08
    - 4.367153094232371e-07
    - 0.7071067819237771
    - 0.09998163225817684
    - -0.20000519853222673
    - 0.08872548376508589
    - 0.5000380359614599
    - 0.4999620016247479
    - 0.49992602012460613
    - 0.5000739284601361
    - 0.14999369315747652
    - 0.030008202877128264
    - 0.09160163759163913
    - 0.5000581288407789
    - 0.49994160185741643
    - 0.5000245790390295
    - 0.4999756822779894
    - -0.1993411526278948
    - -0.07984468630968283
    - 0.03837934574298576
    - -0.0022602213040863626
    - 0.002275415725608405
    - 0.7076953195934438
    - 0.706510473035231
    - 0.014394386252789456
    - 0.023268559674118224
    - -0.006001301466552139
    - 0.017198494572097992
    - -0.1544878637101282
    - 0.01647082791838335
    - 0.1131967024441687
    - 0.05757866867465758
    - -0.05754994049740111
    - -1.8340841714261044e-10
    - 3.6424152557905134e-10
    - 2.5318977393607037e-06
    - 3.0224714523003606e-08
    - 1.444291214691149e-08
    - 2.191510042353385e-12
    - 8.814607051166923e-11
    - -7.433200612344807e-08
    - 1.869032438286548e-05
    - -1.9330059666137164e-08
    - 1.6251681114207004e-05
    - -8.346251583870069e-11
    - -4.666449662036904e-06
    - -4.874318750059981e-05
    - -0.3452649807659493
    - -6.387227689217417e-05
    - 1.290810692986025e-07
    - 0.0007338109785381586
    - 1.984030004396476e-08
    - 1.4275438398523868e-08
    - 0.00014051051384667435
    - 3.6789593474585436e-07
    - 2.6426012211612174e-07
    - -3.0735270909188434e-11
    - -6.236917579370685e-05
    - -1.7353412856496614e-05
    - -0.9913011276643918
    - -0.00025108994971265804
    - -6.185912446320432e-07
    - 8.027456081409942e-05
    - -2.186326021055811e-05
    - 2.7892826998525047e-05
    - -0.9798279838741575
    - -8.694172845636922e-05
    - 1.6374379036746992e-06
    - -0.0001392661481881449
    - -3.1984946822249676e-05
    - 3.2095756428199246e-06
    - 0.0009094897691254896
    - 8.990287645619845e-05
    - -2.1509296190634716e-06
    - -0.0008431013064611642
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

# Pick up the chocolate pudding and place it in the basket

A Franka Panda arm with a parallel-jaw gripper works at a table in a LIBERO scene (MuJoCo / robosuite; LIBERO suite `libero_object`, task `pick_up_the_chocolate_pudding_and_place_it_in_the_basket`). LIBERO-Object: the same layout with different grocery items; pick the named item and put it in the basket.

**Goal (LIBERO's language instruction):** pick up the chocolate pudding and place it in the basket.

The episode ends as solved the moment LIBERO's own success check (the task's goal predicates) passes; you do not need to call `robo done` after that. If you finish without the check passing, call `robo done` (or `robo give-up`).

## This robot

- The action has 7 numbers, not 4: `robo act DX DY DZ DROLL DPITCH DYAW GRIPPER`, each in [-1, 1]. It is LIBERO's delta end-effector command (robosuite's OSC_POSE controller, 20 steps per second). Holding DX/DY/DZ at 1.0 moves the hand about 1 cm per step; DROLL/DPITCH/DYAW rotate the hand (1.0 asks for 0.5 rad; usually leave them at 0). GRIPPER -1 opens, +1 closes.
- `robo move-to X Y Z [--grip G]` and `robo grip G` are available (they keep the hand's orientation).
- World frame, metres: +x points forward, away from the robot's base; +y is to the robot's left; +z is up. The table surface is at about z = 0.90.
- `robo observe` fields: `hand_pos` (end-effector position), `eef_quat` (orientation quaternion x, y, z, w) and `eef_axis_angle`, `gripper_qpos` (the two finger joints), `gripper_open` (finger gap in metres, about 0.08 when fully open), and `objects`: the position and quaternion of every object in the scene, under LIBERO's names (e.g. `akita_black_bowl_1`, `plate_1`; `..._1`, `..._2` number identical objects).
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
