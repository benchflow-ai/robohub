---
schema_version: '1.3'
task:
  name: robouse/libero-90-62
  description: Pick up the salad dressing and put it in the tray.
metadata:
  author_name: benchflow
  source_benchmark: LIBERO (Liu et al. 2023)
  source_task: LIVING_ROOM_SCENE4_pick_up_the_salad_dressing_and_put_it_in_the_tray
  suite: libero-90
  libero_suite: libero_90
  libero_task_index: 62
  language_instruction: pick up the salad dressing and put it in the tray
  category: manipulation
  difficulty: medium
  tags:
  - franka
  - mujoco
  - robosuite
  - single-arm
  - language
  reference_solution: LIBERO demonstration demo_0 of yifengzhu-hf/LIBERO-datasets/libero_90/LIVING_ROOM_SCENE4_pick_up_the_salad_dressing_and_put_it_in_the_tray_demo.hdf5 (replay reaches success at step 103)
  robouse:
    id: libero-90-62
    backend: libero
    suite: libero_90
    env: LIVING_ROOM_SCENE4_pick_up_the_salad_dressing_and_put_it_in_the_tray
    task_index: 62
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
    init_source: yifengzhu-hf/LIBERO-datasets/libero_90/LIVING_ROOM_SCENE4_pick_up_the_salad_dressing_and_put_it_in_the_tray_demo.hdf5 demo_0 states[0]
    init_state:
    - 0.25000000000000017
    - -0.014724800645920435
    - -0.13985482097023597
    - 0.003455556375722668
    - -2.4457143165639392
    - 0.0031146662115411785
    - 2.2441180030066645
    - 0.7631807507242706
    - 0.03401163638502102
    - -0.03408608066722971
    - -0.08067287943407259
    - -0.1329113006574268
    - 0.4351699275556287
    - 0.7071067791707762
    - -6.826208510501431e-08
    - -1.2183615573868527e-08
    - 0.7071067832023155
    - -0.10031626972345176
    - 0.046207967850688286
    - 0.4351699275556287
    - 0.7071067791707762
    - -6.826208510391154e-08
    - -1.2183615572926872e-08
    - 0.7071067832023155
    - -0.25149077182376856
    - -0.11333413928360572
    - 0.4594798420460004
    - 0.5000000790770832
    - 0.4999999177035869
    - 0.5000000280150096
    - 0.4999999752043058
    - 0.10525941547120084
    - -0.18206396478743578
    - 0.4498398389808574
    - 5.028867884121352e-09
    - -3.782457012247385e-09
    - 1.2586724035276105e-08
    - 1.0
    - -0.002573949173085646
    - 0.26250999674009573
    - 0.43738012050757274
    - -7.451456348803627e-06
    - -0.00173263793260483
    - 0.0012644815872153252
    - 0.9999976994956469
    - -0.009622923638511468
    - 0.06534074241954477
    - -0.10958097445516812
    - 0.06934442415692106
    - -0.03564272306779728
    - -0.08859638796691476
    - -0.0972842215588324
    - 0.05759981498558903
    - -0.057414603115260573
    - 1.1457032169389811e-11
    - 6.364949054756592e-11
    - 2.9220132505656733e-06
    - -6.612402537912137e-09
    - -3.666962036750958e-08
    - -9.683475272988978e-14
    - 1.1457032140751193e-11
    - 6.364949032079817e-11
    - 2.9220132505716703e-06
    - -6.6124018108310796e-09
    - -3.666961863330318e-08
    - -9.683490799428371e-14
    - 1.3950294180902946e-12
    - 6.056722536245152e-12
    - 5.4516089139172094e-08
    - 6.176558003701387e-11
    - 4.881717470062529e-15
    - -2.6780971233410825e-10
    - -6.06172048699094e-11
    - -1.4785294954678367e-11
    - 2.895975396007132e-07
    - -1.1621831122100618e-09
    - 4.685771957510199e-09
    - 1.2428095452307755e-13
    - 6.915902947117912e-09
    - 3.5146092002447738e-09
    - 1.9577347505357966e-06
    - 5.701211084400194e-06
    - -1.1170259724958611e-05
    - 4.8008153064966515e-08
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

# Pick up the salad dressing and put it in the tray

A Franka Panda arm with a parallel-jaw gripper works at a low table in a LIBERO scene (MuJoCo / robosuite; LIBERO suite `libero_90`, task `LIVING_ROOM_SCENE4_pick_up_the_salad_dressing_and_put_it_in_the_tray`). LIBERO-90: 90 short-horizon tasks in 20 kitchen, living-room and study scenes (the task name starts with the scene); LIBERO uses them for pretraining.

**Goal (LIBERO's language instruction):** pick up the salad dressing and put it in the tray.

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
