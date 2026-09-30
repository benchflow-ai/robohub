---
schema_version: '1.3'
task:
  name: robouse/libero-90-56
  description: Pick up the butter and put it in the tray.
metadata:
  author_name: benchflow
  source_benchmark: LIBERO (Liu et al. 2023)
  source_task: LIVING_ROOM_SCENE3_pick_up_the_butter_and_put_it_in_the_tray
  suite: libero-90
  libero_suite: libero_90
  libero_task_index: 56
  language_instruction: pick up the butter and put it in the tray
  category: manipulation
  difficulty: medium
  tags:
  - franka
  - mujoco
  - robosuite
  - single-arm
  - language
  reference_solution: LIBERO demonstration demo_0 of yifengzhu-hf/LIBERO-datasets/libero_90/LIVING_ROOM_SCENE3_pick_up_the_butter_and_put_it_in_the_tray_demo.hdf5 (replay reaches success at step 92)
  robouse:
    id: libero-90-56
    backend: libero
    suite: libero_90
    env: LIVING_ROOM_SCENE3_pick_up_the_butter_and_put_it_in_the_tray
    task_index: 56
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
    init_source: yifengzhu-hf/LIBERO-datasets/libero_90/LIVING_ROOM_SCENE3_pick_up_the_butter_and_put_it_in_the_tray_demo.hdf5 demo_0 states[0]
    init_state:
    - 0.25000000000000017
    - 0.015231087993116947
    - -0.13700228754223012
    - -0.009632430808985567
    - -2.4549697508240724
    - -0.009463621606030911
    - 2.254303646978925
    - 0.7692290728048146
    - 0.034052981852945696
    - -0.03405925217573799
    - -0.088881315837842
    - -0.14257214000017596
    - 0.4751648626212499
    - -0.0024825034538961074
    - 0.002158032535246514
    - 0.7074504440013328
    - 0.7067552966582531
    - 0.08993267080463375
    - -0.2125499983523325
    - 0.44569584300064613
    - -3.4880180283772907e-17
    - -3.4199136410378103e-09
    - -1.1564254660047764e-08
    - 1.0
    - -0.08739743374162712
    - 0.05087420904127179
    - 0.47516914741370325
    - -0.0023299931301753877
    - 0.0023447329691130797
    - 0.7074532159718667
    - 0.7067524464551789
    - -0.23391114012148245
    - -0.1340076794533684
    - 0.6312454499278823
    - 0.5000266167646797
    - 0.4999734008287587
    - 0.499943591026722
    - 0.5000563836027886
    - 0.044166792696541564
    - 0.025978445968825144
    - 0.44546984280974117
    - 3.3358388003408684e-10
    - -4.5125860310965895e-09
    - -2.6176911301143372e-08
    - 0.9999999999999998
    - 0.002171576533368443
    - 0.2522726030147618
    - 0.43738012050757274
    - -7.451456349303269e-06
    - -0.0017326379326048252
    - 0.0012644815872153274
    - 0.9999976994956469
    - -0.016981304455228406
    - 0.049676103817891525
    - 0.016807972470798394
    - 0.044025015685311725
    - -0.01183027083862689
    - 0.01711289148881646
    - 0.004601651729004572
    - 0.05754695788375905
    - -0.05757926423674528
    - 0.00021731256210991964
    - 4.089412436956051e-05
    - 0.0003760446939799012
    - 0.0010661410261302688
    - -1.3522369247926342e-06
    - 0.005679699385924334
    - -7.359344253523732e-16
    - -1.6371303917247025e-15
    - 2.8103504406530635e-09
    - -2.0908875312446732e-13
    - 8.653352776838903e-14
    - 2.721692579244695e-17
    - 1.167450378735268e-08
    - 5.549467764060055e-09
    - 1.8531305090151907e-08
    - 1.4502740682774445e-07
    - -6.511204586513028e-10
    - 3.0507396634535466e-07
    - -5.8334018122811554e-05
    - -1.553431423358555e-05
    - -0.6124323020550405
    - -0.00030369496677461443
    - -5.07234809244942e-07
    - 0.00011628228721904432
    - -2.5312627829773143e-13
    - -1.3150386574791615e-13
    - 1.644330124489191e-08
    - -1.7037857799034943e-11
    - 2.994313076973605e-11
    - 1.911098389650234e-15
    - 6.915902947138157e-09
    - 3.5146092001331065e-09
    - 1.957734750522803e-06
    - 5.701211084404093e-06
    - -1.1170259724913053e-05
    - 4.800815306486381e-08
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

# Pick up the butter and put it in the tray

A Franka Panda arm with a parallel-jaw gripper works at a low table in a LIBERO scene (MuJoCo / robosuite; LIBERO suite `libero_90`, task `LIVING_ROOM_SCENE3_pick_up_the_butter_and_put_it_in_the_tray`). LIBERO-90: 90 short-horizon tasks in 20 kitchen, living-room and study scenes (the task name starts with the scene); LIBERO uses them for pretraining.

**Goal (LIBERO's language instruction):** pick up the butter and put it in the tray.

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
