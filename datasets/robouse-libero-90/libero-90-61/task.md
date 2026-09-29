---
schema_version: '1.3'
task:
  name: robouse/libero-90-61
  description: Pick up the chocolate pudding and put it in the tray.
metadata:
  author_name: benchflow
  source_benchmark: LIBERO (Liu et al. 2023)
  source_task: LIVING_ROOM_SCENE4_pick_up_the_chocolate_pudding_and_put_it_in_the_tray
  suite: libero-90
  libero_suite: libero_90
  libero_task_index: 61
  language_instruction: pick up the chocolate pudding and put it in the tray
  category: manipulation
  difficulty: medium
  tags:
  - franka
  - mujoco
  - robosuite
  - single-arm
  - language
  reference_solution: LIBERO demonstration demo_0 of yifengzhu-hf/LIBERO-datasets/libero_90/LIVING_ROOM_SCENE4_pick_up_the_chocolate_pudding_and_put_it_in_the_tray_demo.hdf5 (replay reaches success at step 140)
  robouse:
    id: libero-90-61
    backend: libero
    suite: libero_90
    env: LIVING_ROOM_SCENE4_pick_up_the_chocolate_pudding_and_put_it_in_the_tray
    task_index: 61
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
    init_source: yifengzhu-hf/LIBERO-datasets/libero_90/LIVING_ROOM_SCENE4_pick_up_the_chocolate_pudding_and_put_it_in_the_tray_demo.hdf5 demo_0 states[0]
    init_state:
    - 0.25000000000000017
    - -0.016319968503671373
    - -0.15594338084703485
    - -0.0007896675594607367
    - -2.4566245842576255
    - -0.0063136991856350706
    - 2.2329648791496823
    - 0.7753709085100399
    - 0.034040148524120926
    - -0.034071361174708936
    - -0.10075016395717043
    - -0.17220399754619534
    - 0.4351699275556287
    - 0.7071067791707762
    - -6.82620851067554e-08
    - -1.2183615573911625e-08
    - 0.7071067832023155
    - -0.08096148165219276
    - 0.05680033993781138
    - 0.4351699275556287
    - 0.7071067791707762
    - -6.826208510622477e-08
    - -1.2183615573518966e-08
    - 0.7071067832023155
    - -0.2581383738572999
    - -0.10849453684004538
    - 0.47970746073422543
    - 0.5000000162947423
    - 0.49999998370523757
    - 0.5000000024348855
    - 0.499999997565134
    - 0.11323664941837285
    - -0.1798428765973445
    - 0.4498398389808574
    - 5.028867884106875e-09
    - -3.782457013457278e-09
    - 1.2586724035317816e-08
    - 1.0
    - 0.005448170874287543
    - 0.2699213960382478
    - 0.43738012050757274
    - -7.451456348804754e-06
    - -0.0017326379326048293
    - 0.001264481587215322
    - 0.9999976994956469
    - 0.00858953637597148
    - -0.01609569664431266
    - -0.03206846734903904
    - -0.021638014633871043
    - -0.006444804590956071
    - 0.04611895753028503
    - -0.013571248871029994
    - 0.05764994686472116
    - -0.05747430490194004
    - 1.1457031252052166e-11
    - 6.364949054122578e-11
    - 2.922013250563159e-06
    - -6.612401778466792e-09
    - -3.666961864137895e-08
    - -9.683477459942767e-14
    - 1.145702948948348e-11
    - 6.364949010398635e-11
    - 2.922013250567295e-06
    - -6.6124017141753195e-09
    - -3.666961862540888e-08
    - -9.683451190103976e-14
    - -1.575556631626665e-09
    - 1.0020380517538427e-09
    - -1.2046010329685514e-09
    - -2.594867030747808e-08
    - -4.4423712770165714e-13
    - -1.650710109100301e-08
    - -6.061720488203197e-11
    - -1.478529515565182e-11
    - 2.8959753960004896e-07
    - -1.16218321172648e-09
    - 4.685771912871122e-09
    - 1.242808232452795e-13
    - 6.915902946485914e-09
    - 3.5146091990850273e-09
    - 1.9577347505389336e-06
    - 5.701211084440647e-06
    - -1.1170259724943336e-05
    - 4.800815306491487e-08
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

# Pick up the chocolate pudding and put it in the tray

A Franka Panda arm with a parallel-jaw gripper works at a low table in a LIBERO scene (MuJoCo / robosuite; LIBERO suite `libero_90`, task `LIVING_ROOM_SCENE4_pick_up_the_chocolate_pudding_and_put_it_in_the_tray`). LIBERO-90: 90 short-horizon tasks in 20 kitchen, living-room and study scenes (the task name starts with the scene); LIBERO uses them for pretraining.

**Goal (LIBERO's language instruction):** pick up the chocolate pudding and put it in the tray.

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
