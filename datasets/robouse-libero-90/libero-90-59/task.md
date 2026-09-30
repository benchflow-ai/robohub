---
schema_version: '1.3'
task:
  name: robouse/libero-90-59
  description: Pick up the tomato sauce and put it in the tray.
metadata:
  author_name: benchflow
  source_benchmark: LIBERO (Liu et al. 2023)
  source_task: LIVING_ROOM_SCENE3_pick_up_the_tomato_sauce_and_put_it_in_the_tray
  suite: libero-90
  libero_suite: libero_90
  libero_task_index: 59
  language_instruction: pick up the tomato sauce and put it in the tray
  category: manipulation
  difficulty: medium
  tags:
  - franka
  - mujoco
  - robosuite
  - single-arm
  - language
  reference_solution: LIBERO demonstration demo_0 of yifengzhu-hf/LIBERO-datasets/libero_90/LIVING_ROOM_SCENE3_pick_up_the_tomato_sauce_and_put_it_in_the_tray_demo.hdf5 (replay reaches success at step 117)
  robouse:
    id: libero-90-59
    backend: libero
    suite: libero_90
    env: LIVING_ROOM_SCENE3_pick_up_the_tomato_sauce_and_put_it_in_the_tray
    task_index: 59
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
    init_source: yifengzhu-hf/LIBERO-datasets/libero_90/LIVING_ROOM_SCENE3_pick_up_the_tomato_sauce_and_put_it_in_the_tray_demo.hdf5 demo_0 states[0]
    init_state:
    - 0.25000000000000017
    - -0.009847544453742444
    - -0.15792692271496084
    - -0.018096210754638284
    - -2.4426630455741587
    - 0.02984653887165781
    - 2.182472843987393
    - 0.7702264487822392
    - 0.03403158167052552
    - -0.03406506070291233
    - -0.11453926239983783
    - -0.14054176342164054
    - 0.4751648626212499
    - -0.0024825034538957084
    - 0.002158032535246612
    - 0.7074504440013327
    - 0.7067552966582531
    - 0.11561025631259095
    - -0.21736330950480515
    - 0.44569584300064613
    - -3.4879724976433353e-17
    - -3.419913643351725e-09
    - -1.1564254660765912e-08
    - 1.0
    - -0.10039155882412384
    - 0.03891518756452237
    - 0.47516914741370325
    - -0.002329993130175414
    - 0.0023447329691130597
    - 0.7074532159718667
    - 0.7067524464551789
    - -0.24575405640443218
    - -0.1511190004377317
    - 0.6312454499278823
    - 0.5000266167646799
    - 0.4999734008287587
    - 0.49994359102672215
    - 0.5000563836027887
    - 0.047414626356141513
    - 0.07139247007793258
    - 0.44546984280974117
    - 3.335838800310803e-10
    - -4.512586034467569e-09
    - -2.6176911301834372e-08
    - 0.9999999999999998
    - 0.003871519219187887
    - 0.25818183243957754
    - 0.43738012050757274
    - -7.451456349303127e-06
    - -0.0017326379326048234
    - 0.001264481587215319
    - 0.9999976994956469
    - -0.022254137583006758
    - 0.1427327159335352
    - -0.12254834951047654
    - 0.1491305070087434
    - 0.07768731157103197
    - -0.13529885594519184
    - -0.056987372867147626
    - 0.057801574104493735
    - -0.057255236437551954
    - 0.00021731256211159346
    - 4.089412436999186e-05
    - 0.00037604469397782374
    - 0.001066141026141597
    - -1.3522369248255734e-06
    - 0.0056796993859674705
    - -7.359360767682952e-16
    - -1.6363524008543433e-15
    - 2.810350442490392e-09
    - -2.0911031756220904e-13
    - 8.657703618571487e-14
    - 2.7358899410763678e-17
    - 1.167450378700178e-08
    - 5.5494677670960605e-09
    - 1.85313050900957e-08
    - 1.4502740691389157e-07
    - -6.511204591355733e-10
    - 3.0507396636193496e-07
    - -5.8334018122783596e-05
    - -1.5534314233580397e-05
    - -0.6124323020550405
    - -0.00030369496677422943
    - -5.072348092427771e-07
    - 0.00011628228721839345
    - -2.531262781656492e-13
    - -1.315030162108861e-13
    - 1.644330124474631e-08
    - -1.703777050731113e-11
    - 2.994306418366437e-11
    - 1.9111191527305165e-15
    - 6.915902946699489e-09
    - 3.5146091998221614e-09
    - 1.957734750532622e-06
    - 5.701211084414916e-06
    - -1.1170259724853659e-05
    - 4.800815306466055e-08
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

# Pick up the tomato sauce and put it in the tray

A Franka Panda arm with a parallel-jaw gripper works at a low table in a LIBERO scene (MuJoCo / robosuite; LIBERO suite `libero_90`, task `LIVING_ROOM_SCENE3_pick_up_the_tomato_sauce_and_put_it_in_the_tray`). LIBERO-90: 90 short-horizon tasks in 20 kitchen, living-room and study scenes (the task name starts with the scene); LIBERO uses them for pretraining.

**Goal (LIBERO's language instruction):** pick up the tomato sauce and put it in the tray.

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
