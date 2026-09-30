---
schema_version: '1.3'
task:
  name: robouse/libero-90-76
  description: Pick up the yellow and white mug and place it to the right of the caddy.
metadata:
  author_name: benchflow
  source_benchmark: LIBERO (Liu et al. 2023)
  source_task: STUDY_SCENE1_pick_up_the_yellow_and_white_mug_and_place_it_to_the_right_of_the_caddy
  suite: libero-90
  libero_suite: libero_90
  libero_task_index: 76
  language_instruction: pick up the yellow and white mug and place it to the right of the caddy
  category: manipulation
  difficulty: medium
  tags:
  - franka
  - mujoco
  - robosuite
  - single-arm
  - language
  reference_solution: LIBERO demonstration demo_0 of yifengzhu-hf/LIBERO-datasets/libero_90/STUDY_SCENE1_pick_up_the_yellow_and_white_mug_and_place_it_to_the_right_of_the_caddy_demo.hdf5 (replay reaches success at step 118)
  robouse:
    id: libero-90-76
    backend: libero
    suite: libero_90
    env: STUDY_SCENE1_pick_up_the_yellow_and_white_mug_and_place_it_to_the_right_of_the_caddy
    task_index: 76
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
    init_source: yifengzhu-hf/LIBERO-datasets/libero_90/STUDY_SCENE1_pick_up_the_yellow_and_white_mug_and_place_it_to_the_right_of_the_caddy_demo.hdf5 demo_0 states[0]
    init_state:
    - 0.25000000000000017
    - -0.01652504391336879
    - -0.14579736106218874
    - -0.006686847062238663
    - -2.429192251700047
    - -0.012002235766816118
    - 2.2400227373383714
    - 0.7696774212912698
    - 0.03402963862572767
    - -0.034081481783072826
    - -0.21623330152598652
    - 0.17026971682041045
    - 0.8829597034738329
    - -0.571269355598536
    - 1.9784343930067826e-08
    - -4.6186452341714756e-08
    - 0.820762647392065
    - -0.11943016143088069
    - -0.019241685274700198
    - 0.8852269911115938
    - -0.7071067358182576
    - -9.854906034459919e-07
    - 5.615288457641819e-08
    - 0.7071068265541456
    - 0.012071624213767668
    - 0.12416772895072722
    - -0.02966185125754237
    - 0.11857987750484002
    - 0.06854085690806452
    - -0.0293090340491713
    - -0.05352581370245072
    - 0.057733023902857394
    - -0.057406991282166336
    - -5.98428347008441e-10
    - 2.6797819371279836e-10
    - 9.876915799027165e-06
    - -2.958763013127881e-07
    - 1.9336918627363697e-08
    - 2.5549006289107612e-14
    - 1.9870444724087512e-13
    - -1.347138937073308e-11
    - 7.114519732267977e-07
    - -7.594825915118152e-09
    - 2.9554051563486313e-07
    - -3.0818217279516866e-13
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

# Pick up the yellow and white mug and place it to the right of the caddy

A Franka Panda arm with a parallel-jaw gripper works at a desk in a LIBERO scene (MuJoCo / robosuite; LIBERO suite `libero_90`, task `STUDY_SCENE1_pick_up_the_yellow_and_white_mug_and_place_it_to_the_right_of_the_caddy`). LIBERO-90: 90 short-horizon tasks in 20 kitchen, living-room and study scenes (the task name starts with the scene); LIBERO uses them for pretraining.

**Goal (LIBERO's language instruction):** pick up the yellow and white mug and place it to the right of the caddy.

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
