---
schema_version: '1.3'
task:
  name: robouse/libero-object-9
  description: Pick up the orange juice and place it in the basket.
metadata:
  author_name: benchflow
  source_benchmark: LIBERO (Liu et al. 2023)
  source_task: pick_up_the_orange_juice_and_place_it_in_the_basket
  suite: libero
  libero_suite: libero_object
  libero_task_index: 9
  language_instruction: pick up the orange juice and place it in the basket
  category: manipulation
  difficulty: easy
  tags:
  - franka
  - mujoco
  - robosuite
  - single-arm
  - language
  reference_solution: LIBERO demonstration demo_0 of yifengzhu-hf/LIBERO-datasets/libero_object/pick_up_the_orange_juice_and_place_it_in_the_basket_demo.hdf5 (replay reaches success at step 115)
  robouse:
    id: libero-object-9
    backend: libero
    suite: libero_object
    env: pick_up_the_orange_juice_and_place_it_in_the_basket
    task_index: 9
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
    init_source: yifengzhu-hf/LIBERO-datasets/libero_object/pick_up_the_orange_juice_and_place_it_in_the_basket_demo.hdf5 demo_0 states[0]
    init_state:
    - 0.25000000000000017
    - 0.0057999304363970435
    - -0.1703609772610122
    - 0.01133691934787833
    - -2.4254966464070877
    - -0.02118709246545375
    - 2.225453517552912
    - 0.8006565799675152
    - 0.03403896705724837
    - -0.03407328707525761
    - 0.04999998601054686
    - -0.09999715653569538
    - 0.06699943919188384
    - 0.5000115735518467
    - 0.4999884062357616
    - 0.5000165521882278
    - 0.49998346720849335
    - -0.006843268307277593
    - 0.2577352592654049
    - -0.0045826267525761735
    - 0.7071047520207788
    - -0.0016938434289765474
    - 0.0016937189970340028
    - 0.7071047531165592
    - -0.1198795049888331
    - -0.24094883921585963
    - 0.0086954309406588
    - 8.874955155493025e-10
    - -4.1729229488578127e-07
    - -2.4216197991267464e-06
    - 0.999999999996981
    - -0.15000006364152668
    - 0.06000032755963053
    - 0.013065439187715341
    - 3.943167459429957e-09
    - -3.5273436129243034e-07
    - 1.163498893607174e-06
    - 0.9999999999992609
    - 0.10007826663016589
    - -0.20407414373069177
    - 0.05462337560407025
    - 0.7071067804491808
    - 5.615071646088668e-08
    - 4.3671530942273904e-07
    - 0.7071067819237771
    - 0.14999068698573895
    - 0.029997394020632458
    - 0.08888381695967436
    - 0.5000199168212283
    - 0.49998010104537893
    - 0.4999597186329283
    - 0.5000402594644034
    - -0.20001326393249746
    - -0.07997800008698375
    - 0.09144217291816688
    - 0.5001146704699068
    - 0.4998847333149002
    - 0.5000438075808332
    - 0.4999567584096952
    - -0.002277774853432017
    - 0.08882928430547227
    - 9.119911461299651e-05
    - 0.09118609444980114
    - 0.005201605484043583
    - -0.1515471341541142
    - -0.0073638485747343696
    - 0.05752834008237923
    - -0.05759806138343637
    - -4.66644966134501e-06
    - -4.87431875015467e-05
    - -0.34526498076594964
    - -6.387227688277567e-05
    - 1.2908106928838375e-07
    - 0.000733810978551581
    - 8.814606579014187e-11
    - -7.433200611829407e-08
    - 1.869032438285422e-05
    - -1.933005959617208e-08
    - 1.6251681114129623e-05
    - -8.34625160099536e-11
    - -9.635931099458908e-11
    - -1.9098383291128082e-11
    - 3.1068622874467006e-06
    - -1.1063703773127227e-08
    - 1.2000060697514385e-08
    - 8.789747717550706e-12
    - -1.8340841853547893e-10
    - 3.642415257629485e-10
    - 2.5318977393617506e-06
    - 3.022471451236821e-08
    - 1.444291244331665e-08
    - 2.1915105199175815e-12
    - 1.984030004333669e-08
    - 1.4275438398080305e-08
    - 0.0001405105138466752
    - 3.6789593474142406e-07
    - 2.64260122093793e-07
    - -3.073527088109979e-11
    - -3.0406346984008915e-05
    - -8.364789778928567e-06
    - -0.9907215719182421
    - -9.259397781992186e-05
    - -4.9534948086801e-07
    - 3.107276800813949e-05
    - -4.6669107391673994e-05
    - 7.714755381927664e-05
    - -0.9804116042481448
    - -0.00019137010247139108
    - 2.94757962394187e-06
    - -0.000358290714613883
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

# Pick up the orange juice and place it in the basket

A Franka Panda arm with a parallel-jaw gripper works over the floor in a LIBERO scene (MuJoCo / robosuite; LIBERO suite `libero_object`, task `pick_up_the_orange_juice_and_place_it_in_the_basket`). LIBERO-Object: the same layout with different grocery items; pick the named item and put it in the basket.

**Goal (LIBERO's language instruction):** pick up the orange juice and place it in the basket.

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
