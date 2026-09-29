---
schema_version: '1.3'
task:
  name: robouse/libero-90-84
  description: Pick up the red mug and place it to the right of the caddy.
metadata:
  author_name: benchflow
  source_benchmark: LIBERO (Liu et al. 2023)
  source_task: STUDY_SCENE3_pick_up_the_red_mug_and_place_it_to_the_right_of_the_caddy
  suite: libero-90
  libero_suite: libero_90
  libero_task_index: 84
  language_instruction: pick up the red mug and place it to the right of the caddy
  category: manipulation
  difficulty: medium
  tags:
  - franka
  - mujoco
  - robosuite
  - single-arm
  - language
  reference_solution: LIBERO demonstration demo_0 of yifengzhu-hf/LIBERO-datasets/libero_90/STUDY_SCENE3_pick_up_the_red_mug_and_place_it_to_the_right_of_the_caddy_demo.hdf5 (replay reaches success at step 109)
  robouse:
    id: libero-90-84
    backend: libero
    suite: libero_90
    env: STUDY_SCENE3_pick_up_the_red_mug_and_place_it_to_the_right_of_the_caddy
    task_index: 84
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
    init_source: yifengzhu-hf/LIBERO-datasets/libero_90/STUDY_SCENE3_pick_up_the_red_mug_and_place_it_to_the_right_of_the_caddy_demo.hdf5 demo_0 states[0]
    init_state:
    - 0.25000000000000017
    - -0.005028777235880748
    - -0.1469859046573421
    - -0.01459557271925312
    - -2.4481958291087498
    - -0.008777454617326071
    - 2.250707378573272
    - 0.757539143855193
    - 0.03405753389124898
    - -0.03405096336362364
    - -0.2050801603663958
    - -0.17010279342099066
    - 0.8829597034738329
    - -0.4037644987193496
    - 1.058657563589867e-08
    - -4.911754331148979e-08
    - 0.9148629567175129
    - 0.019885786193797744
    - 0.1270377834249141
    - 0.8869689131443201
    - -0.7071068831386871
    - -5.520412001387726e-07
    - 3.840203851940674e-07
    - 0.7071066792340737
    - -0.1755747582714151
    - 0.012662717004272456
    - 0.8827398431074825
    - -0.7071069385103981
    - 1.0000672108130468e-07
    - 6.550415589302066e-08
    - 0.707106623862652
    - 0.012345018139472025
    - 0.09532903287700695
    - 0.034361628887914664
    - 0.07834593656848722
    - 0.006732882239653154
    - 0.06040883155992879
    - 0.00030014045806669266
    - 0.05756914165889968
    - -0.05756305854940483
    - -4.540530494319539e-10
    - 4.730376556307046e-10
    - 9.876915799033413e-06
    - -2.9587630127796853e-07
    - 1.9336918638664222e-08
    - 2.5549016964253717e-14
    - 2.3691322055111857e-11
    - -4.978280297043795e-10
    - 7.136022452259794e-06
    - -1.298079629947993e-08
    - 2.762176012947722e-07
    - -6.029943499997805e-13
    - -1.863422235608016e-10
    - -1.1825924070486619e-09
    - 8.686770062491068e-06
    - -7.600160696322147e-08
    - -4.843970478498797e-07
    - -2.379470710161688e-12
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

# Pick up the red mug and place it to the right of the caddy

A Franka Panda arm with a parallel-jaw gripper works at a desk in a LIBERO scene (MuJoCo / robosuite; LIBERO suite `libero_90`, task `STUDY_SCENE3_pick_up_the_red_mug_and_place_it_to_the_right_of_the_caddy`). LIBERO-90: 90 short-horizon tasks in 20 kitchen, living-room and study scenes (the task name starts with the scene); LIBERO uses them for pretraining.

**Goal (LIBERO's language instruction):** pick up the red mug and place it to the right of the caddy.

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
