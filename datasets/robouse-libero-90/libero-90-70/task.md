---
schema_version: '1.3'
task:
  name: robouse/libero-90-70
  description: Put the chocolate pudding to the right of the plate.
metadata:
  author_name: benchflow
  source_benchmark: LIBERO (Liu et al. 2023)
  source_task: LIVING_ROOM_SCENE6_put_the_chocolate_pudding_to_the_right_of_the_plate
  suite: libero-90
  libero_suite: libero_90
  libero_task_index: 70
  language_instruction: put the chocolate pudding to the right of the plate
  category: manipulation
  difficulty: medium
  tags:
  - franka
  - mujoco
  - robosuite
  - single-arm
  - language
  reference_solution: LIBERO demonstration demo_0 of yifengzhu-hf/LIBERO-datasets/libero_90/LIVING_ROOM_SCENE6_put_the_chocolate_pudding_to_the_right_of_the_plate_demo.hdf5 (replay reaches success at step 66)
  robouse:
    id: libero-90-70
    backend: libero
    suite: libero_90
    env: LIVING_ROOM_SCENE6_put_the_chocolate_pudding_to_the_right_of_the_plate
    task_index: 70
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
    init_source: yifengzhu-hf/LIBERO-datasets/libero_90/LIVING_ROOM_SCENE6_put_the_chocolate_pudding_to_the_right_of_the_plate_demo.hdf5 demo_0 states[0]
    init_state:
    - 0.25000000000000017
    - 0.005568196619437356
    - -0.14762050528021498
    - 0.056486278441554946
    - -2.4154982418410262
    - 0.02860351961610315
    - 2.2314220645123584
    - 0.8110310912419163
    - 0.03411980569711213
    - -0.03396238663809723
    - -0.12190893888823778
    - -0.12925994786447123
    - 0.4343699488393418
    - -0.7071069013731585
    - 1.0172763262551607e-07
    - 6.67573136829248e-08
    - 0.7071066609999057
    - -0.20830181866168587
    - -0.017645505363269885
    - 0.4385989872647111
    - -0.7071068579899178
    - -5.528613175755097e-07
    - 3.831199740644679e-07
    - 0.707106704382849
    - 0.13156220643359168
    - -0.010334771052242234
    - 0.4392727869250624
    - 0.7071068416649496
    - 1.225361178497375e-06
    - -6.586728017049151e-07
    - 0.7071067207067718
    - -0.043289688031433286
    - 0.11858127108498406
    - 0.4498398389808574
    - 5.028867884109708e-09
    - -3.782457011981781e-09
    - 1.2586724035189807e-08
    - 1.0
    - 0.04500045086874612
    - 0.10333081425309902
    - 0.10767855057741556
    - 0.02698207846314656
    - 0.020403455412122164
    - 0.08429602995440483
    - 0.13443594421574928
    - 0.057063743845844436
    - -0.057926956113870366
    - -7.114629380827706e-11
    - -4.5271501629394363e-10
    - 1.1368093065875907e-06
    - -2.8896237756265938e-08
    - -1.846680129104147e-07
    - -1.0830534101938268e-12
    - 8.910669326048496e-12
    - -1.8765529730827677e-10
    - 1.843048112628574e-06
    - -4.9171977738658014e-09
    - 1.0451617048827544e-07
    - -2.772260615727529e-13
    - 1.862797331659158e-10
    - 1.1309283170951093e-11
    - 2.7645383596763985e-06
    - 7.554087104670504e-08
    - 4.563006108274491e-09
    - 1.7231473276840695e-12
    - -6.061720484885474e-11
    - -1.4785294797521834e-11
    - 2.895975396013325e-07
    - -1.1621831026462105e-09
    - 4.685772011086612e-09
    - 1.2428106148801627e-13
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

# Put the chocolate pudding to the right of the plate

A Franka Panda arm with a parallel-jaw gripper works at a low table in a LIBERO scene (MuJoCo / robosuite; LIBERO suite `libero_90`, task `LIVING_ROOM_SCENE6_put_the_chocolate_pudding_to_the_right_of_the_plate`). LIBERO-90: 90 short-horizon tasks in 20 kitchen, living-room and study scenes (the task name starts with the scene); LIBERO uses them for pretraining.

**Goal (LIBERO's language instruction):** put the chocolate pudding to the right of the plate.

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
