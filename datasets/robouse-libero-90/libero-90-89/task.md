---
schema_version: '1.3'
task:
  name: robouse/libero-90-89
  description: Pick up the book on the right and place it under the cabinet shelf.
metadata:
  author_name: benchflow
  source_benchmark: LIBERO (Liu et al. 2023)
  source_task: STUDY_SCENE4_pick_up_the_book_on_the_right_and_place_it_under_the_cabinet_shelf
  suite: libero-90
  libero_suite: libero_90
  libero_task_index: 89
  language_instruction: pick up the book on the right and place it under the cabinet shelf
  category: manipulation
  difficulty: medium
  tags:
  - franka
  - mujoco
  - robosuite
  - single-arm
  - language
  reference_solution: LIBERO demonstration demo_0 of yifengzhu-hf/LIBERO-datasets/libero_90/STUDY_SCENE4_pick_up_the_book_on_the_right_and_place_it_under_the_cabinet_shelf_demo.hdf5 (replay reaches success at step 112)
  robouse:
    id: libero-90-89
    backend: libero
    suite: libero_90
    env: STUDY_SCENE4_pick_up_the_book_on_the_right_and_place_it_under_the_cabinet_shelf
    task_index: 89
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
    init_source: yifengzhu-hf/LIBERO-datasets/libero_90/STUDY_SCENE4_pick_up_the_book_on_the_right_and_place_it_under_the_cabinet_shelf_demo.hdf5 demo_0 states[0]
    init_state:
    - 0.25000000000000017
    - -0.03576419132896837
    - -0.1383802825995672
    - 0.008640201693296263
    - -2.448384816792069
    - -0.015296039829199316
    - 2.231631727298537
    - 0.81651200614322
    - 0.034024105796192396
    - -0.03408715263713647
    - -0.1592303856353488
    - -0.15837436255843884
    - 0.8829597034738329
    - -0.5686911804632628
    - 1.9639324156807268e-08
    - -4.6248303658815767e-08
    - 0.8225511177205331
    - -0.19946370090170046
    - -0.008626955753745266
    - 0.8831598332502683
    - -0.4778088423068481
    - -2.044258319122783e-07
    - -1.6819484165524413e-07
    - 0.8784638354612669
    - -0.24634528117506965
    - -0.2429978674625388
    - 0.8831598332502683
    - -0.6482407298267718
    - -1.6547015820213986e-07
    - -2.0663748996012672e-07
    - 0.7614354576676767
    - 0.03138194037574097
    - 0.0948924318034424
    - -0.05338202196864095
    - 0.08972808159920241
    - -0.013888264164236544
    - -0.03152502573361773
    - -0.008137812931048802
    - 0.05770364134592805
    - -0.05742464619698931
    - -5.967348636324022e-10
    - 2.7172835761736786e-10
    - 9.876915799030141e-06
    - -2.9587630130357995e-07
    - 1.9336918637708157e-08
    - 2.5549015633230532e-14
    - -3.9219308824191126e-11
    - -3.174621350960639e-10
    - 7.60329998873221e-07
    - 6.736790357060485e-08
    - -1.3478925255502405e-07
    - 1.8405784978445323e-13
    - 9.187017187436532e-11
    - -3.0639881404588334e-10
    - 7.603300058181334e-07
    - 6.7367903573755e-08
    - -1.3478925254661505e-07
    - 1.840578490896678e-13
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

# Pick up the book on the right and place it under the cabinet shelf

A Franka Panda arm with a parallel-jaw gripper works at a desk in a LIBERO scene (MuJoCo / robosuite; LIBERO suite `libero_90`, task `STUDY_SCENE4_pick_up_the_book_on_the_right_and_place_it_under_the_cabinet_shelf`). LIBERO-90: 90 short-horizon tasks in 20 kitchen, living-room and study scenes (the task name starts with the scene); LIBERO uses them for pretraining.

**Goal (LIBERO's language instruction):** pick up the book on the right and place it under the cabinet shelf.

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
