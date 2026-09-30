---
schema_version: '1.3'
task:
  name: robouse/libero-90-57
  description: Pick up the cream cheese and put it in the tray.
metadata:
  author_name: benchflow
  source_benchmark: LIBERO (Liu et al. 2023)
  source_task: LIVING_ROOM_SCENE3_pick_up_the_cream_cheese_and_put_it_in_the_tray
  suite: libero-90
  libero_suite: libero_90
  libero_task_index: 57
  language_instruction: pick up the cream cheese and put it in the tray
  category: manipulation
  difficulty: medium
  tags:
  - franka
  - mujoco
  - robosuite
  - single-arm
  - language
  reference_solution: LIBERO demonstration demo_0 of yifengzhu-hf/LIBERO-datasets/libero_90/LIVING_ROOM_SCENE3_pick_up_the_cream_cheese_and_put_it_in_the_tray_demo.hdf5 (replay reaches success at step 147)
  robouse:
    id: libero-90-57
    backend: libero
    suite: libero_90
    env: LIVING_ROOM_SCENE3_pick_up_the_cream_cheese_and_put_it_in_the_tray
    task_index: 57
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
    init_source: yifengzhu-hf/LIBERO-datasets/libero_90/LIVING_ROOM_SCENE3_pick_up_the_cream_cheese_and_put_it_in_the_tray_demo.hdf5 demo_0 states[0]
    init_state:
    - 0.25000000000000017
    - 0.0077871792738422275
    - -0.15320118323496779
    - 0.006157656262369356
    - -2.4650395444637687
    - 0.0019437532882747028
    - 2.2423569486508645
    - 0.7727646859777774
    - 0.03405400549083801
    - -0.03405633786346074
    - -0.084197143017187
    - -0.16974183856097597
    - 0.4751648626212499
    - -0.0024825034538958415
    - 0.002158032535246587
    - 0.7074504440013328
    - 0.706755296658253
    - 0.08587139614693835
    - -0.1818782773055415
    - 0.44569584300064613
    - -3.4878673487869e-17
    - -3.4199136400917783e-09
    - -1.156425466036822e-08
    - 1.0
    - -0.11869273158314106
    - 0.02966152905521506
    - 0.47516914741370325
    - -0.00232999313017535
    - 0.002344732969113104
    - 0.7074532159718667
    - 0.7067524464551789
    - -0.2298845606643431
    - -0.1359827018774594
    - 0.6312454499278823
    - 0.5000266167646797
    - 0.49997340082875874
    - 0.4999435910267222
    - 0.5000563836027885
    - 0.042416022966493605
    - 0.030093596013709707
    - 0.44546984280974117
    - 3.335838800358965e-10
    - -4.512586033779688e-09
    - -2.6176911301749636e-08
    - 0.9999999999999998
    - 0.006647449898035862
    - 0.25509602618471905
    - 0.43738012050757274
    - -7.451456349303273e-06
    - -0.0017326379326048271
    - 0.0012644815872153248
    - 0.9999976994956469
    - -0.012562113864332453
    - 0.0003681484783219429
    - -0.015943126034674873
    - 0.00014474023403862382
    - -0.0038013572399266567
    - 0.02916428321441841
    - -0.022401047012407766
    - 0.057590847685013026
    - -0.057540444479176465
    - 0.00021731256211227634
    - 4.089412436931114e-05
    - 0.0003760446939787366
    - 0.001066141026123914
    - -1.3522369246886448e-06
    - 0.00567969938598577
    - -7.359381495612264e-16
    - -1.6368285845484608e-15
    - 2.8103504415510474e-09
    - -2.0914013721011137e-13
    - 8.662882164931744e-14
    - 2.74154525492133e-17
    - 1.1674503787777918e-08
    - 5.549467760845118e-09
    - 1.8531305090856056e-08
    - 1.4502740674105066e-07
    - -6.511204581622834e-10
    - 3.0507396636258236e-07
    - -5.833401812249096e-05
    - -1.5534314233511496e-05
    - -0.6124323020550405
    - -0.0003036949667729034
    - -5.072348092433494e-07
    - 0.00011628228721844595
    - -2.531262782869859e-13
    - -1.3150332248495368e-13
    - 1.6443301243653696e-08
    - -1.7037964457678943e-11
    - 2.994308800520092e-11
    - 1.9112336597752534e-15
    - 6.915902946767955e-09
    - 3.5146091995897208e-09
    - 1.9577347505288704e-06
    - 5.7012110844229885e-06
    - -1.1170259724901127e-05
    - 4.800815306490448e-08
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
oracle:
  env:
    ROBOUSE_ORACLE_TOKEN: ${ROBOUSE_ORACLE_TOKEN:-}
---

You are controlling a simulated robot. Read the task below, then solve it by running the `robo` command in your shell (start with `robo info` and `robo observe`). Keep going until the task is done, then call `robo done` once. Do not stop to ask questions; there is no human to answer.

# Pick up the cream cheese and put it in the tray

A Franka Panda arm with a parallel-jaw gripper works at a low table in a LIBERO scene (MuJoCo / robosuite; LIBERO suite `libero_90`, task `LIVING_ROOM_SCENE3_pick_up_the_cream_cheese_and_put_it_in_the_tray`). LIBERO-90: 90 short-horizon tasks in 20 kitchen, living-room and study scenes (the task name starts with the scene); LIBERO uses them for pretraining.

**Goal (LIBERO's language instruction):** pick up the cream cheese and put it in the tray.

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
