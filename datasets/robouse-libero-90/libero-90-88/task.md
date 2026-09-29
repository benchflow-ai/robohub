---
schema_version: '1.3'
task:
  name: robouse/libero-90-88
  description: Pick up the book on the right and place it on the cabinet shelf.
metadata:
  author_name: benchflow
  source_benchmark: LIBERO (Liu et al. 2023)
  source_task: STUDY_SCENE4_pick_up_the_book_on_the_right_and_place_it_on_the_cabinet_shelf
  suite: libero-90
  libero_suite: libero_90
  libero_task_index: 88
  language_instruction: pick up the book on the right and place it on the cabinet shelf
  category: manipulation
  difficulty: medium
  tags:
  - franka
  - mujoco
  - robosuite
  - single-arm
  - language
  reference_solution: LIBERO demonstration demo_0 of yifengzhu-hf/LIBERO-datasets/libero_90/STUDY_SCENE4_pick_up_the_book_on_the_right_and_place_it_on_the_cabinet_shelf_demo.hdf5 (replay reaches success at step 82)
  robouse:
    id: libero-90-88
    backend: libero
    suite: libero_90
    env: STUDY_SCENE4_pick_up_the_book_on_the_right_and_place_it_on_the_cabinet_shelf
    task_index: 88
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
    init_source: yifengzhu-hf/LIBERO-datasets/libero_90/STUDY_SCENE4_pick_up_the_book_on_the_right_and_place_it_on_the_cabinet_shelf_demo.hdf5 demo_0 states[0]
    init_state:
    - 0.25000000000000017
    - -0.00453249593823742
    - -0.15382391757012642
    - 0.005195428523398626
    - -2.4236509873348426
    - 0.017223722177721026
    - 2.2502265396654306
    - 0.7731945277035819
    - 0.03407111450879812
    - -0.034041180879472256
    - -0.15433778222932232
    - -0.1584350963712574
    - 0.8829597034738329
    - -0.4642489679009558
    - 1.385889306358117e-08
    - -4.829637385674757e-08
    - 0.8857047452751368
    - -0.1988255804586412
    - 0.008385758534529006
    - 0.8831598332502683
    - -0.7054599891005762
    - -1.4890938829111067e-07
    - -2.1887306730437513e-07
    - 0.7087497469333904
    - -0.2585927670481497
    - -0.2574076027868921
    - 0.8831598332502683
    - -0.5576848205926213
    - -1.8784167202399273e-07
    - -1.865339962678767e-07
    - 0.8300527940321061
    - 0.004424576292893714
    - 0.02326593875194026
    - -0.0030366573750143784
    - -0.02932834946209028
    - 3.2049629002120732e-06
    - 0.09518271542988826
    - 3.0931414583338006e-08
    - 0.057545841217599196
    - -0.057580118711209924
    - -5.133095523022995e-10
    - 4.079731651503277e-10
    - 9.876915798161787e-06
    - -2.958763013237429e-07
    - 1.9336918643315042e-08
    - 2.55490211041096e-14
    - 1.382439168800754e-10
    - -2.8845966873849604e-10
    - 7.603300053828407e-07
    - 6.736790356157664e-08
    - -1.34789252540494e-07
    - 1.840578504144579e-13
    - 2.0443135768491854e-11
    - -3.192216176335072e-10
    - 7.603300036456509e-07
    - 6.736790362300338e-08
    - -1.3478925256832892e-07
    - 1.8405784906007866e-13
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

# Pick up the book on the right and place it on the cabinet shelf

A Franka Panda arm with a parallel-jaw gripper works at a desk in a LIBERO scene (MuJoCo / robosuite; LIBERO suite `libero_90`, task `STUDY_SCENE4_pick_up_the_book_on_the_right_and_place_it_on_the_cabinet_shelf`). LIBERO-90: 90 short-horizon tasks in 20 kitchen, living-room and study scenes (the task name starts with the scene); LIBERO uses them for pretraining.

**Goal (LIBERO's language instruction):** pick up the book on the right and place it on the cabinet shelf.

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
