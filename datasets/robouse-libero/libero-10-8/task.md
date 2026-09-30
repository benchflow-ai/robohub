---
schema_version: '1.3'
task:
  name: robouse/libero-10-8
  description: Put both moka pots on the stove.
metadata:
  author_name: benchflow
  source_benchmark: LIBERO (Liu et al. 2023)
  source_task: KITCHEN_SCENE8_put_both_moka_pots_on_the_stove
  suite: libero
  libero_suite: libero_10
  libero_task_index: 8
  language_instruction: put both moka pots on the stove
  category: manipulation
  difficulty: hard
  tags:
  - franka
  - mujoco
  - robosuite
  - single-arm
  - language
  reference_solution: LIBERO demonstration demo_0 of yifengzhu-hf/LIBERO-datasets/libero_10/KITCHEN_SCENE8_put_both_moka_pots_on_the_stove_demo.hdf5 (replay reaches success at step 448)
  robouse:
    id: libero-10-8
    backend: libero
    suite: libero_10
    env: KITCHEN_SCENE8_put_both_moka_pots_on_the_stove
    task_index: 8
    seed: 0
    max_steps: 520
    camera: agentview
    cameras:
    - agentview
    - robot0_eye_in_hand
    image_size: 256
    settle_steps: 50
    skills: true
    success_mode: first
    obs_mode: state
    init_source: yifengzhu-hf/LIBERO-datasets/libero_10/KITCHEN_SCENE8_put_both_moka_pots_on_the_stove_demo.hdf5 demo_0 states[0]
    init_state:
    - 0.25000000000000017
    - -0.005609445748428146
    - -0.1606211313907391
    - 0.011923697310598662
    - -2.432210966671236
    - -0.03075810964567672
    - 2.259301149399121
    - 0.8413771154641262
    - 0.034061560149464945
    - -0.0340369234790744
    - -0.040857561356264706
    - 0.26302719851148093
    - 0.9661017023863033
    - -2.080146185111457e-10
    - 2.456996438852588e-07
    - -8.51425562984016e-06
    - 0.9999999999637237
    - 0.04886982148825019
    - 0.02915367544907101
    - 0.9661017023863033
    - -2.0801461852570146e-10
    - 2.456996438974504e-07
    - -8.514255629833372e-06
    - 0.9999999999637237
    - 1.9362136578799516
    - 0.026511992034915378
    - 0.22972491090328895
    - 0.10005120379953768
    - 0.2067869766840718
    - -0.06878576337494163
    - 0.01527749349433856
    - 0.36086700064008026
    - 0.05740903068659979
    - -0.05763409703320965
    - 3.1826483515579064e-09
    - -1.0740826576162245e-07
    - 0.00011075076689354214
    - -1.625815054778875e-06
    - -4.8169288379027006e-08
    - -2.2704200802945674e-12
    - 3.18264839197967e-09
    - -1.0740826573815519e-07
    - 0.00011075076689354912
    - -1.6258150544371244e-06
    - -4.8169288995957445e-08
    - -2.2704205652165943e-12
    - 0.0
agent:
  timeout_sec: 1800
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

# Put both moka pots on the stove

A Franka Panda arm with a parallel-jaw gripper works at a table in a LIBERO scene (MuJoCo / robosuite; LIBERO suite `libero_10`, task `KITCHEN_SCENE8_put_both_moka_pots_on_the_stove`). LIBERO-Long (libero_10): long-horizon tasks, most with two sub-goals.

**Goal (LIBERO's language instruction):** put both moka pots on the stove.

The episode ends as solved the moment LIBERO's own success check (the task's goal predicates) passes; you do not need to call `robo done` after that. If you finish without the check passing, call `robo done` (or `robo give-up`).

## This robot

- The action has 7 numbers, not 4: `robo act DX DY DZ DROLL DPITCH DYAW GRIPPER`, each in [-1, 1]. It is LIBERO's delta end-effector command (robosuite's OSC_POSE controller, 20 steps per second). Holding DX/DY/DZ at 1.0 moves the hand about 1 cm per step; DROLL/DPITCH/DYAW rotate the hand (1.0 asks for 0.5 rad; usually leave them at 0). GRIPPER -1 opens, +1 closes.
- `robo move-to X Y Z [--grip G]` and `robo grip G` are available (they keep the hand's orientation).
- World frame, metres: +x points forward, away from the robot's base; +y is to the robot's left; +z is up. The table surface is at about z = 0.90.
- `robo observe` fields: `hand_pos` (end-effector position), `eef_quat` (orientation quaternion x, y, z, w) and `eef_axis_angle`, `gripper_qpos` (the two finger joints), `gripper_open` (finger gap in metres, about 0.08 when fully open), and `objects`: the position and quaternion of every object in the scene, under LIBERO's names (e.g. `akita_black_bowl_1`, `plate_1`; `..._1`, `..._2` number identical objects); `fixtures`: the position and quaternion of each piece of furniture (cabinet, stove, microwave, rack, shelf, ...); `articulated`: each movable part of a fixture (drawer slide, door hinge, stove knob), keyed by its joint name (e.g. `wooden_cabinet_1_top_level` is the cabinet's top drawer), with `qpos` (joint position), `range` (joint limits), LIBERO's thresholds for that fixture (`open_ranges` / `close_ranges`, or `turnon_ranges` / `turnoff_ranges` for a stove knob: the joint positions at which LIBERO counts the part as open / closed or on / off) and `box_min` / `box_max` (the world-frame bounding box of the moving part).
- `robo observe --image` saves the front camera (`agentview`); add `--camera robot0_eye_in_hand` for the wrist camera. Both are 256x256.
- The step budget is 520 steps (LeRobot's LIBERO evaluation budget for this suite).

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
