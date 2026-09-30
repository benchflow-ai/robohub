---
schema_version: '1.3'
task:
  name: robouse/libero-10-2
  description: Turn on the stove and put the moka pot on it.
metadata:
  author_name: benchflow
  source_benchmark: LIBERO (Liu et al. 2023)
  source_task: KITCHEN_SCENE3_turn_on_the_stove_and_put_the_moka_pot_on_it
  suite: libero
  libero_suite: libero_10
  libero_task_index: 2
  language_instruction: turn on the stove and put the moka pot on it
  category: manipulation
  difficulty: hard
  tags:
  - franka
  - mujoco
  - robosuite
  - single-arm
  - language
  reference_solution: LIBERO demonstration demo_0 of yifengzhu-hf/LIBERO-datasets/libero_10/KITCHEN_SCENE3_turn_on_the_stove_and_put_the_moka_pot_on_it_demo.hdf5 (replay reaches success at step 260)
  robouse:
    id: libero-10-2
    backend: libero
    suite: libero_10
    env: KITCHEN_SCENE3_turn_on_the_stove_and_put_the_moka_pot_on_it
    task_index: 2
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
    init_source: yifengzhu-hf/LIBERO-datasets/libero_10/KITCHEN_SCENE3_turn_on_the_stove_and_put_the_moka_pot_on_it_demo.hdf5 demo_0 states[0]
    init_state:
    - 0.25000000000000017
    - 0.007857194399318718
    - -0.17516132301883525
    - -0.01962210292082796
    - -2.446684456379108
    - -0.017853408048352164
    - 2.2695960005632725
    - 0.7831199075307728
    - 0.0340482438941441
    - -0.03406361599498122
    - -0.0521650663967373
    - -0.23942200458170934
    - 0.8989956691834108
    - 0.7071067559272116
    - -6.078223204305158e-05
    - 6.142891735541552e-05
    - 0.7071068011652152
    - 0.06137868752456883
    - 0.00046593739053894817
    - 0.9661017023863033
    - -2.0801461850881157e-10
    - 2.456996438854567e-07
    - -8.514255629840177e-06
    - 0.9999999999637237
    - 0.0
    - -0.007661637712994945
    - 0.0091472768594524
    - 0.02603325954996398
    - -0.0007704544066932933
    - 0.007805813870523511
    - 0.012406456133262004
    - 0.006554244869377798
    - 0.057642446944092283
    - -0.057482822221022536
    - -1.5747682799746512e-09
    - -3.5827654940356274e-07
    - 0.004697247236681479
    - 1.7128964218978994e-06
    - 0.0003906167432709722
    - 4.370251436351809e-12
    - 3.182648351338092e-09
    - -1.07408265761704e-07
    - 0.00011075076689355746
    - -1.6258150547963915e-06
    - -4.816928837630954e-08
    - -2.270420140048356e-12
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

# Turn on the stove and put the moka pot on it

A Franka Panda arm with a parallel-jaw gripper works at a table in a LIBERO scene (MuJoCo / robosuite; LIBERO suite `libero_10`, task `KITCHEN_SCENE3_turn_on_the_stove_and_put_the_moka_pot_on_it`). LIBERO-Long (libero_10): long-horizon tasks, most with two sub-goals.

**Goal (LIBERO's language instruction):** turn on the stove and put the moka pot on it.

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
