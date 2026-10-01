---
schema_version: '1.3'
task:
  name: robouse/libero-90-58
  description: Pick up the ketchup and put it in the tray.
metadata:
  author_name: benchflow
  source_benchmark: LIBERO (Liu et al. 2023)
  source_task: LIVING_ROOM_SCENE3_pick_up_the_ketchup_and_put_it_in_the_tray
  suite: libero-90
  libero_suite: libero_90
  libero_task_index: 58
  language_instruction: pick up the ketchup and put it in the tray
  category: manipulation
  difficulty: medium
  tags:
  - franka
  - mujoco
  - robosuite
  - single-arm
  - language
  reference_solution: LIBERO demonstration demo_0 of yifengzhu-hf/LIBERO-datasets/libero_90/LIVING_ROOM_SCENE3_pick_up_the_ketchup_and_put_it_in_the_tray_demo.hdf5 (replay reaches success at step 132)
  robouse:
    id: libero-90-58
    backend: libero
    suite: libero_90
    env: LIVING_ROOM_SCENE3_pick_up_the_ketchup_and_put_it_in_the_tray
    task_index: 58
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
    init_source: yifengzhu-hf/LIBERO-datasets/libero_90/LIVING_ROOM_SCENE3_pick_up_the_ketchup_and_put_it_in_the_tray_demo.hdf5 demo_0 states[0]
    init_state:
    - 0.25000000000000017
    - -0.01171715002076093
    - -0.12678956817352421
    - -0.02133433822588772
    - -2.4453403016441078
    - -0.003863733897705096
    - 2.248948099797689
    - 0.7785946203332256
    - 0.034047535605957496
    - -0.03406403903308268
    - -0.09989856566316363
    - -0.15177458493817256
    - 0.4751648626212499
    - -0.002482503453895824
    - 0.0021580325352465314
    - 0.7074504440013328
    - 0.706755296658253
    - 0.11365507710097054
    - -0.1765761402680327
    - 0.44569584300064613
    - -3.487855985118558e-17
    - -3.419913640879161e-09
    - -1.1564254660265554e-08
    - 1.0
    - -0.08591713383056807
    - 0.04009564668846426
    - 0.47516914741370325
    - -0.002329993130175362
    - 0.002344732969113115
    - 0.7074532159718667
    - 0.7067524464551789
    - -0.25556486050235505
    - -0.16606487850011356
    - 0.6312454499278823
    - 0.5000266167646797
    - 0.4999734008287588
    - 0.49994359102672215
    - 0.5000563836027886
    - 0.06270570360246058
    - 0.03518449739172452
    - 0.44546984280974117
    - 3.335838800318037e-10
    - -4.512586033546577e-09
    - -2.6176911301544988e-08
    - 0.9999999999999998
    - 0.002289970821781789
    - 0.2697496089602091
    - 0.43738012050757274
    - -7.451456349303599e-06
    - -0.0017326379326048232
    - 0.0012644815872153272
    - 0.9999976994956469
    - 0.005725953664888812
    - -0.025098738636112835
    - 0.0030155182064927626
    - -0.03850007210989856
    - -0.05006050695232641
    - 0.08101658457616709
    - 0.040988426156406806
    - 0.05766380999404913
    - -0.05747044515481229
    - 0.00021731256211227884
    - 4.089412436929496e-05
    - 0.00037604469397873843
    - 0.0010661410261235058
    - -1.3522369246855716e-06
    - 0.005679699385985872
    - -7.359349664077837e-16
    - -1.6368927695587327e-15
    - 2.810350441440227e-09
    - -2.090973401942686e-13
    - 8.651714557832207e-14
    - 2.7218653030334544e-17
    - 1.1674503787174635e-08
    - 5.549467765562208e-09
    - 1.8531305091904547e-08
    - 1.4502740686628154e-07
    - -6.511204587405771e-10
    - 3.050739663488738e-07
    - -5.833401812251077e-05
    - -1.553431423352937e-05
    - -0.6124323020550405
    - -0.0003036949667730444
    - -5.072348092442176e-07
    - 0.00011628228721869015
    - -2.5312627770686386e-13
    - -1.3150336876552026e-13
    - 1.6443301245100822e-08
    - -1.7037803913055966e-11
    - 2.9943046092542876e-11
    - 1.9110900934175e-15
    - 6.915902946746654e-09
    - 3.514609200755765e-09
    - 1.957734750525195e-06
    - 5.701211084382394e-06
    - -1.1170259724926771e-05
    - 4.800815306491772e-08
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

# Pick up the ketchup and put it in the tray

A Franka Panda arm with a parallel-jaw gripper works at a low table in a LIBERO scene (MuJoCo / robosuite; LIBERO suite `libero_90`, task `LIVING_ROOM_SCENE3_pick_up_the_ketchup_and_put_it_in_the_tray`). LIBERO-90: 90 short-horizon tasks in 20 kitchen, living-room and study scenes (the task name starts with the scene); LIBERO uses them for pretraining.

**Goal (LIBERO's language instruction):** pick up the ketchup and put it in the tray.

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
