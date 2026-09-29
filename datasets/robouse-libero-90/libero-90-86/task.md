---
schema_version: '1.3'
task:
  name: robouse/libero-90-86
  description: Pick up the book in the middle and place it on the cabinet shelf.
metadata:
  author_name: benchflow
  source_benchmark: LIBERO (Liu et al. 2023)
  source_task: STUDY_SCENE4_pick_up_the_book_in_the_middle_and_place_it_on_the_cabinet_shelf
  suite: libero-90
  libero_suite: libero_90
  libero_task_index: 86
  language_instruction: pick up the book in the middle and place it on the cabinet shelf
  category: manipulation
  difficulty: medium
  tags:
  - franka
  - mujoco
  - robosuite
  - single-arm
  - language
  reference_solution: LIBERO demonstration demo_0 of yifengzhu-hf/LIBERO-datasets/libero_90/STUDY_SCENE4_pick_up_the_book_in_the_middle_and_place_it_on_the_cabinet_shelf_demo.hdf5 (replay reaches success at step 156)
  robouse:
    id: libero-90-86
    backend: libero
    suite: libero_90
    env: STUDY_SCENE4_pick_up_the_book_in_the_middle_and_place_it_on_the_cabinet_shelf
    task_index: 86
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
    init_source: yifengzhu-hf/LIBERO-datasets/libero_90/STUDY_SCENE4_pick_up_the_book_in_the_middle_and_place_it_on_the_cabinet_shelf_demo.hdf5 demo_0 states[0]
    init_state:
    - 0.25000000000000017
    - 0.012771073156772207
    - -0.14151525613818172
    - -0.025322672442292977
    - -2.416717910128907
    - -0.0017442026669177572
    - 2.281875266078535
    - 0.7571470733001451
    - 0.03395335638670352
    - -0.03409454011713721
    - -0.1407209245440968
    - -0.1545815401364662
    - 0.8829597034738329
    - -0.5744895655479066
    - 1.996564894387558e-08
    - -4.610836699451131e-08
    - 0.818511905274795
    - -0.20587449497629928
    - -0.006031682310789978
    - 0.8831598332502683
    - -0.6396373641273576
    - -1.6778334462362988e-07
    - -2.0476370474119804e-07
    - 0.7686768127191922
    - -0.25046198987150353
    - -0.25158902988495857
    - 0.8831598332502683
    - -0.41917355985626215
    - -2.150154837456215e-07
    - -1.5442722318966664e-07
    - 0.9079061221940066
    - -0.06718826340497977
    - 0.25542384310143423
    - -0.1548027176879285
    - 0.2197154356747543
    - 0.013594900327133851
    - 0.12499517003403629
    - -0.22838062318333335
    - 0.05779372367205765
    - -0.05719023438376992
    - -6.005155251482118e-10
    - 2.632677369203427e-10
    - 9.876915797290427e-06
    - -2.9587630130184776e-07
    - 1.933691862963897e-08
    - 2.5549010096917624e-14
    - 8.495643554380991e-11
    - -3.0838736391679655e-10
    - 7.603300049479994e-07
    - 6.736790359162153e-08
    - -1.3478925254641766e-07
    - 1.840578452787989e-13
    - -8.042865009545405e-11
    - -3.0959908725078027e-10
    - 7.603300058167275e-07
    - 6.736790362845519e-08
    - -1.3478925253979937e-07
    - 1.8405783525503272e-13
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

# Pick up the book in the middle and place it on the cabinet shelf

A Franka Panda arm with a parallel-jaw gripper works at a desk in a LIBERO scene (MuJoCo / robosuite; LIBERO suite `libero_90`, task `STUDY_SCENE4_pick_up_the_book_in_the_middle_and_place_it_on_the_cabinet_shelf`). LIBERO-90: 90 short-horizon tasks in 20 kitchen, living-room and study scenes (the task name starts with the scene); LIBERO uses them for pretraining.

**Goal (LIBERO's language instruction):** pick up the book in the middle and place it on the cabinet shelf.

The episode ends as solved the moment LIBERO's own success check (the task's goal predicates) passes; you do not need to call `robo done` after that. If you finish without the check passing, call `robo done` (or `robo give-up`).

## This robot

- The action has 7 numbers, not 4: `robo act DX DY DZ DROLL DPITCH DYAW GRIPPER`, each in [-1, 1]. It is LIBERO's delta end-effector command (robosuite's OSC_POSE controller, 20 steps per second). Holding DX/DY/DZ at 1.0 moves the hand about 1 cm per step; DROLL/DPITCH/DYAW rotate the hand (1.0 asks for 0.5 rad; usually leave them at 0). GRIPPER -1 opens, +1 closes.
- `robo move-to X Y Z [--grip G]` and `robo grip G` are available (they keep the hand's orientation).
- World frame, metres: +x points forward, away from the robot's base; +y is to the robot's left; +z is up. The desk surface is at about z = 0.88.
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
