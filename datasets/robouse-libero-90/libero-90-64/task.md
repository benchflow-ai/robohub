---
schema_version: '1.3'
task:
  name: robouse/libero-90-64
  description: Stack the right bowl on the left bowl and place them in the tray.
metadata:
  author_name: benchflow
  source_benchmark: LIBERO (Liu et al. 2023)
  source_task: LIVING_ROOM_SCENE4_stack_the_right_bowl_on_the_left_bowl_and_place_them_in_the_tray
  suite: libero-90
  libero_suite: libero_90
  libero_task_index: 64
  language_instruction: stack the right bowl on the left bowl and place them in the tray
  category: manipulation
  difficulty: medium
  tags:
  - franka
  - mujoco
  - robosuite
  - single-arm
  - language
  reference_solution: LIBERO demonstration demo_0 of yifengzhu-hf/LIBERO-datasets/libero_90/LIVING_ROOM_SCENE4_stack_the_right_bowl_on_the_left_bowl_and_place_them_in_the_tray_demo.hdf5 (replay reaches success at step 221)
  robouse:
    id: libero-90-64
    backend: libero
    suite: libero_90
    env: LIVING_ROOM_SCENE4_stack_the_right_bowl_on_the_left_bowl_and_place_them_in_the_tray
    task_index: 64
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
    init_source: yifengzhu-hf/LIBERO-datasets/libero_90/LIVING_ROOM_SCENE4_stack_the_right_bowl_on_the_left_bowl_and_place_them_in_the_tray_demo.hdf5 demo_0 states[0]
    init_state:
    - 0.25000000000000017
    - 0.007312443502169586
    - -0.16290707997114132
    - -0.008779359167063497
    - -2.429508384703159
    - 0.023308105613950494
    - 2.240263679296243
    - 0.7976129129978526
    - 0.03406923991664729
    - -0.03404312942446873
    - -0.07765537222829598
    - -0.16510312162689508
    - 0.4351699275556287
    - 0.7071067791707762
    - -6.826208510369258e-08
    - -1.218361557317245e-08
    - 0.7071067832023155
    - -0.07500292524786355
    - 0.032535628889818646
    - 0.4351699275556287
    - 0.7071067791707762
    - -6.8262085105447e-08
    - -1.2183615573879674e-08
    - 0.7071067832023155
    - -0.27105285495087755
    - -0.10457992430119277
    - 0.47970746073422504
    - 0.5000000162947373
    - 0.49999998370524124
    - 0.5000000024348976
    - 0.49999999756512326
    - 0.07698093196085222
    - -0.18413792269453733
    - 0.4498398389808574
    - 5.028867884098796e-09
    - -3.782456993032329e-09
    - 1.2586724035407301e-08
    - 1.0
    - 0.00014871368436695885
    - 0.2686669305200829
    - 0.43738012050757274
    - -7.4514563488043435e-06
    - -0.001732637932604828
    - 0.0012644815872153248
    - 0.9999976994956469
    - -0.010527001182989664
    - 0.04102955814619706
    - -0.014999238282463715
    - 0.0408494799661273
    - 0.08040744506335563
    - -0.05433924709695818
    - -0.07047090455867151
    - 0.05754315098382412
    - -0.05758212605465948
    - 1.145703102812061e-11
    - 6.364949033420733e-11
    - 2.9220132505714954e-06
    - -6.6124017702840235e-09
    - -3.66696186338323e-08
    - -9.683474062408597e-14
    - 1.1457030745444715e-11
    - 6.364948972508547e-11
    - 2.9220132505706246e-06
    - -6.6124017599750965e-09
    - -3.666961861158091e-08
    - -9.683470486635699e-14
    - -1.5755759128872336e-09
    - 1.0020051139151741e-09
    - -1.204437486546977e-09
    - -2.5948981719324043e-08
    - -4.2981809702080234e-13
    - -1.6504778822147394e-08
    - -6.061720487427655e-11
    - -1.478529536536854e-11
    - 2.895975395992494e-07
    - -1.162183256475763e-09
    - 4.685771944085589e-09
    - 1.2428087606250994e-13
    - 6.915902946873608e-09
    - 3.5146091993644253e-09
    - 1.9577347505371675e-06
    - 5.7012110844309235e-06
    - -1.1170259724967175e-05
    - 4.8008153064969414e-08
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

# Stack the right bowl on the left bowl and place them in the tray

A Franka Panda arm with a parallel-jaw gripper works at a low table in a LIBERO scene (MuJoCo / robosuite; LIBERO suite `libero_90`, task `LIVING_ROOM_SCENE4_stack_the_right_bowl_on_the_left_bowl_and_place_them_in_the_tray`). LIBERO-90: 90 short-horizon tasks in 20 kitchen, living-room and study scenes (the task name starts with the scene); LIBERO uses them for pretraining.

**Goal (LIBERO's language instruction):** stack the right bowl on the left bowl and place them in the tray.

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
