---
schema_version: '1.3'
task:
  name: robouse/libero-90-3
  description: Put the butter at the back in the top drawer of the cabinet and close it.
metadata:
  author_name: benchflow
  source_benchmark: LIBERO (Liu et al. 2023)
  source_task: KITCHEN_SCENE10_put_the_butter_at_the_back_in_the_top_drawer_of_the_cabinet_and_close_it
  suite: libero-90
  libero_suite: libero_90
  libero_task_index: 3
  language_instruction: put the butter at the back in the top drawer of the cabinet and close it
  category: manipulation
  difficulty: medium
  tags:
  - franka
  - mujoco
  - robosuite
  - single-arm
  - language
  reference_solution: LIBERO demonstration demo_0 of yifengzhu-hf/LIBERO-datasets/libero_90/KITCHEN_SCENE10_put_the_butter_at_the_back_in_the_top_drawer_of_the_cabinet_and_close_it_demo.hdf5 (replay reaches success at step 172)
  robouse:
    id: libero-90-3
    backend: libero
    suite: libero_90
    env: KITCHEN_SCENE10_put_the_butter_at_the_back_in_the_top_drawer_of_the_cabinet_and_close_it
    task_index: 3
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
    init_source: yifengzhu-hf/LIBERO-datasets/libero_90/KITCHEN_SCENE10_put_the_butter_at_the_back_in_the_top_drawer_of_the_cabinet_and_close_it_demo.hdf5 demo_0 states[0]
    init_state:
    - 0.25000000000000017
    - -0.01445491073425263
    - -0.16628349712560056
    - 0.032245865065639485
    - -2.4480513059607296
    - -0.006467654764252295
    - 2.1969930026013196
    - 0.7694102839719372
    - 0.03405429513102321
    - -0.0340578326407721
    - -0.08811462851584104
    - -0.015471955629778064
    - 0.8981585151399347
    - 0.7071067776242912
    - -1.0724537191720858e-05
    - -1.4310617004710426e-06
    - 0.7071067846660275
    - 0.014602628149641786
    - 0.2024719170981249
    - 0.9086548762142962
    - -4.0888836866729235e-10
    - -8.196927805366552e-07
    - -4.739534225494788e-06
    - 0.9999999999884325
    - -0.12073155134541562
    - 0.22048218801323516
    - 0.9086548762142962
    - -4.088883686658835e-10
    - -8.196927805411688e-07
    - -4.739534225495651e-06
    - 0.9999999999884325
    - 0.0024431476936828426
    - 0.04982929838809613
    - 0.9129232581896339
    - 9.570210259681475e-09
    - -4.332321868490455e-07
    - 2.14144773650023e-06
    - 0.9999999999976134
    - -0.14947628615369046
    - 0.0
    - 0.0
    - 0.02337564651536498
    - 0.054038047084981194
    - -0.017433421369236486
    - 0.061636320590721154
    - 0.0009147945930741633
    - -0.11364626370481587
    - 0.021710675788031774
    - 0.057526476474056794
    - -0.057600654296434343
    - 1.4146964852413033e-07
    - -3.4906643730623725e-08
    - 0.00656114118906861
    - -7.860505322540447e-05
    - 2.0775069654597133e-05
    - -1.2876260700539015e-09
    - -5.728458374138965e-09
    - -4.142549956178234e-08
    - 0.0006971084279367778
    - -5.214947459266456e-06
    - 6.757763810724203e-07
    - 3.81550372676877e-10
    - -5.7284583741207155e-09
    - -4.142549956143113e-08
    - 0.00069710842793678
    - -5.21494745923465e-06
    - 6.757763810286382e-07
    - 3.8155037268707697e-10
    - -1.7911519963159028e-07
    - 9.463563512629403e-08
    - 0.003418716150299916
    - 7.383681980263579e-06
    - 1.3920238412915158e-05
    - 3.5569574171372846e-10
    - 0.0
    - 0.0
    - 0.0
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

# Put the butter at the back in the top drawer of the cabinet and close it

A Franka Panda arm with a parallel-jaw gripper works at a table in a LIBERO scene (MuJoCo / robosuite; LIBERO suite `libero_90`, task `KITCHEN_SCENE10_put_the_butter_at_the_back_in_the_top_drawer_of_the_cabinet_and_close_it`). LIBERO-90: 90 short-horizon tasks in 20 kitchen, living-room and study scenes (the task name starts with the scene); LIBERO uses them for pretraining.

**Goal (LIBERO's language instruction):** put the butter at the back in the top drawer of the cabinet and close it.

The episode ends as solved the moment LIBERO's own success check (the task's goal predicates) passes; you do not need to call `robo done` after that. If you finish without the check passing, call `robo done` (or `robo give-up`).

## This robot

- The action has 7 numbers, not 4: `robo act DX DY DZ DROLL DPITCH DYAW GRIPPER`, each in [-1, 1]. It is LIBERO's delta end-effector command (robosuite's OSC_POSE controller, 20 steps per second). Holding DX/DY/DZ at 1.0 moves the hand about 1 cm per step; DROLL/DPITCH/DYAW rotate the hand (1.0 asks for 0.5 rad; usually leave them at 0). GRIPPER -1 opens, +1 closes.
- `robo move-to X Y Z [--grip G]` and `robo grip G` are available (they keep the hand's orientation).
- World frame, metres: +x points forward, away from the robot's base; +y is to the robot's left; +z is up. The table surface is at about z = 0.90.
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
