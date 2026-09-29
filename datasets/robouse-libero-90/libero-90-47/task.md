---
schema_version: '1.3'
task:
  name: robouse/libero-90-47
  description: Pick up the cream cheese box and put it in the basket.
metadata:
  author_name: benchflow
  source_benchmark: LIBERO (Liu et al. 2023)
  source_task: LIVING_ROOM_SCENE1_pick_up_the_cream_cheese_box_and_put_it_in_the_basket
  suite: libero-90
  libero_suite: libero_90
  libero_task_index: 47
  language_instruction: pick up the cream cheese box and put it in the basket
  category: manipulation
  difficulty: medium
  tags:
  - franka
  - mujoco
  - robosuite
  - single-arm
  - language
  reference_solution: LIBERO demonstration demo_0 of yifengzhu-hf/LIBERO-datasets/libero_90/LIVING_ROOM_SCENE1_pick_up_the_cream_cheese_box_and_put_it_in_the_basket_demo.hdf5 (replay reaches success at step 150)
  robouse:
    id: libero-90-47
    backend: libero
    suite: libero_90
    env: LIVING_ROOM_SCENE1_pick_up_the_cream_cheese_box_and_put_it_in_the_basket
    task_index: 47
    seed: 0
    max_steps: 400
    camera: agentview
    cameras:
    - agentview
    - robot0_eye_in_hand
    image_size: 256
    settle_steps: 50
    skills: true
    success_mode: first
    obs_mode: state
    init_source: yifengzhu-hf/LIBERO-datasets/libero_90/LIVING_ROOM_SCENE1_pick_up_the_cream_cheese_box_and_put_it_in_the_basket_demo.hdf5 demo_0 states[0]
    init_state:
    - 0.25000000000000017
    - -0.0018724177355549983
    - -0.17336068569782168
    - -0.010438826434310565
    - -2.416513373277018
    - 0.0033509861572319903
    - 2.2055420157500847
    - 0.7789097266881193
    - 0.03407212300570838
    - -0.034039981993651595
    - 0.05443406838488953
    - -0.0874016348542197
    - 0.4751648626212499
    - -0.0024825034538953745
    - 0.002158032535247014
    - 0.7074504440013328
    - 0.706755296658253
    - -0.12717786743161374
    - 0.057638029624956205
    - 0.44569584300064613
    - -3.487862285560789e-17
    - -3.419913640135803e-09
    - -1.1564254660568251e-08
    - 1.0
    - 0.08762551196855926
    - -0.19160746550031857
    - 0.47516914741370325
    - -0.0023299931301753373
    - 0.002344732969113108
    - 0.7074532159718667
    - 0.7067524464551789
    - -0.21011409155037453
    - -0.15077721968323146
    - 0.6312454499278823
    - 0.5000266167646797
    - 0.49997340082875863
    - 0.499943591026722
    - 0.5000563836027886
    - -0.009516880673064183
    - 0.25793045558036876
    - 0.4321919610888971
    - 0.7071047490794425
    - -0.0016949793370027828
    - 0.0016949779842838804
    - 0.7071047503192003
    - 0.0052021081532730796
    - 0.04066243147626841
    - 0.029530527654948343
    - 0.062375482139415635
    - -0.0433309957924928
    - -0.1347321123959313
    - 5.428566432567133e-05
    - 0.057536556078547685
    - -0.05758348371123913
    - 0.00021731256211199621
    - 4.089412436924756e-05
    - 0.0003760446939819734
    - 0.0010661410261223118
    - -1.3522369246823073e-06
    - 0.005679699385978948
    - -7.359369980741251e-16
    - -1.6365150042087154e-15
    - 2.810350444718283e-09
    - -2.0906581371661743e-13
    - 8.664059349522564e-14
    - 2.7399538190274834e-17
    - 1.1674503788112148e-08
    - 5.5494677581596735e-09
    - 1.8531305090605288e-08
    - 1.4502740665765952e-07
    - -6.511204579609482e-10
    - 3.0507396627894315e-07
    - -5.833401812287138e-05
    - -1.5534314233528257e-05
    - -0.6124323020550401
    - -0.0003036949667745701
    - -5.072348092438877e-07
    - 0.00011628228721874606
    - -9.7742886919583e-13
    - -7.846063078813719e-09
    - 4.6803922573934115e-06
    - 2.1184007327024334e-10
    - 1.7156703158140486e-06
    - 1.5481247363098046e-12
agent:
  timeout_sec: 1500
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

# Pick up the cream cheese box and put it in the basket

A Franka Panda arm with a parallel-jaw gripper works at a low table in a LIBERO scene (MuJoCo / robosuite; LIBERO suite `libero_90`, task `LIVING_ROOM_SCENE1_pick_up_the_cream_cheese_box_and_put_it_in_the_basket`). LIBERO-90: 90 short-horizon tasks in 20 kitchen, living-room and study scenes (the task name starts with the scene); LIBERO uses them for pretraining.

**Goal (LIBERO's language instruction):** pick up the cream cheese box and put it in the basket.

The episode ends as solved the moment LIBERO's own success check (the task's goal predicates) passes; you do not need to call `robo done` after that. If you finish without the check passing, call `robo done` (or `robo give-up`).

## This robot

- The action has 7 numbers, not 4: `robo act DX DY DZ DROLL DPITCH DYAW GRIPPER`, each in [-1, 1]. It is LIBERO's delta end-effector command (robosuite's OSC_POSE controller, 20 steps per second). Holding DX/DY/DZ at 1.0 moves the hand about 1 cm per step; DROLL/DPITCH/DYAW rotate the hand (1.0 asks for 0.5 rad; usually leave them at 0). GRIPPER -1 opens, +1 closes.
- `robo move-to X Y Z [--grip G]` and `robo grip G` are available (they keep the hand's orientation).
- World frame, metres: +x points forward, away from the robot's base; +y is to the robot's left; +z is up. The table surface is at about z = 0.43.
- `robo observe` fields: `hand_pos` (end-effector position), `eef_quat` (orientation quaternion x, y, z, w) and `eef_axis_angle`, `gripper_qpos` (the two finger joints), `gripper_open` (finger gap in metres, about 0.08 when fully open), and `objects`: the position and quaternion of every object in the scene, under LIBERO's names (e.g. `akita_black_bowl_1`, `plate_1`; `..._1`, `..._2` number identical objects); `fixtures`: the position and quaternion of each piece of furniture (cabinet, stove, microwave, rack, shelf, ...); `articulated`: each movable part of a fixture (drawer slide, door hinge, stove knob), keyed by its joint name (e.g. `wooden_cabinet_1_top_level` is the cabinet's top drawer), with `qpos` (joint position), `range` (joint limits), LIBERO's thresholds for that fixture (`open_ranges` / `close_ranges`, or `turnon_ranges` / `turnoff_ranges` for a stove knob: the joint positions at which LIBERO counts the part as open / closed or on / off) and `box_min` / `box_max` (the world-frame bounding box of the moving part).
- `robo observe --image` saves the front camera (`agentview`); add `--camera robot0_eye_in_hand` for the wrist camera. Both are 256x256.
- The step budget is 400 steps (LeRobot's LIBERO evaluation budget for this suite).

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
