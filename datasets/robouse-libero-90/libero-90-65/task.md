---
schema_version: '1.3'
task:
  name: robouse/libero-90-65
  description: Put the red mug on the left plate.
metadata:
  author_name: benchflow
  source_benchmark: LIBERO (Liu et al. 2023)
  source_task: LIVING_ROOM_SCENE5_put_the_red_mug_on_the_left_plate
  suite: libero-90
  libero_suite: libero_90
  libero_task_index: 65
  language_instruction: put the red mug on the left plate
  category: manipulation
  difficulty: medium
  tags:
  - franka
  - mujoco
  - robosuite
  - single-arm
  - language
  reference_solution: LIBERO demonstration demo_1 of yifengzhu-hf/LIBERO-datasets/libero_90/LIVING_ROOM_SCENE5_put_the_red_mug_on_the_left_plate_demo.hdf5 (replay reaches success at step 133)
  robouse:
    id: libero-90-65
    backend: libero
    suite: libero_90
    env: LIVING_ROOM_SCENE5_put_the_red_mug_on_the_left_plate
    task_index: 65
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
    init_source: yifengzhu-hf/LIBERO-datasets/libero_90/LIVING_ROOM_SCENE5_put_the_red_mug_on_the_left_plate_demo.hdf5 demo_1 states[0]
    init_state:
    - 0.25000000000000017
    - 0.005878614894827846
    - -0.1215895589039497
    - -0.01772637356008112
    - -2.432299288364982
    - 0.006433371519215181
    - 2.222656562472049
    - 0.7730404248931924
    - 0.03406286027860234
    - -0.034047453434779815
    - -0.11305457178465786
    - -0.16766422963949118
    - 0.4343699488393418
    - -0.7071069013731585
    - 1.0172763262334166e-07
    - 6.675731368116476e-08
    - 0.7071066609999057
    - -0.18596974795902207
    - 0.02251310904114388
    - 0.4385989872647111
    - -0.7071068579899178
    - -5.528613175582436e-07
    - 3.831199740284785e-07
    - 0.707106704382849
    - -0.058554360972786205
    - 0.07672480523686413
    - 0.43685696889004044
    - -0.7071067450257308
    - -9.86372114196494e-07
    - 5.5381073928696914e-08
    - 0.7071068173466724
    - -0.0041664617079330305
    - -0.32385566903265267
    - 0.4392727869250624
    - 0.7071068416649496
    - 1.2253611784968053e-06
    - -6.586728017369789e-07
    - 0.7071067207067718
    - 0.022288296070483767
    - 0.292510405935384
    - 0.4392727869250624
    - 0.7071068416649496
    - 1.2253611785016175e-06
    - -6.586728017141223e-07
    - 0.7071067207067718
    - -0.0035280247242005084
    - 0.0045809807828961005
    - 0.012034050907384677
    - -0.011573678116266858
    - 0.09037861880432614
    - 0.017886759511120124
    - -0.049010661411194714
    - 0.05759547860827368
    - -0.05753405293068625
    - -7.114629352147956e-11
    - -4.5271501544920404e-10
    - 1.1368093065912169e-06
    - -2.8896237751084306e-08
    - -1.8466801289528245e-07
    - -1.083053323733061e-12
    - 8.910656715422772e-12
    - -1.8765526716088902e-10
    - 1.8430481127075004e-06
    - -4.917191219380821e-09
    - 1.0451615552291104e-07
    - -2.7722639510466614e-13
    - 6.847139957055419e-13
    - -7.276784681548578e-12
    - 2.2982456672028636e-06
    - -1.7375213031537518e-08
    - 1.2912372845396125e-07
    - -3.2601170020959664e-13
    - 1.8627973934688572e-10
    - 1.1309285899594346e-11
    - 2.764538359662985e-06
    - 7.554087379595741e-08
    - 4.563006865515375e-09
    - 1.7231476768648382e-12
    - 1.8627973215093586e-10
    - 1.1309283901275342e-11
    - 2.764538359676878e-06
    - 7.554087117557414e-08
    - 4.563006013401434e-09
    - 1.7231479963510618e-12
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

# Put the red mug on the left plate

A Franka Panda arm with a parallel-jaw gripper works at a low table in a LIBERO scene (MuJoCo / robosuite; LIBERO suite `libero_90`, task `LIVING_ROOM_SCENE5_put_the_red_mug_on_the_left_plate`). LIBERO-90: 90 short-horizon tasks in 20 kitchen, living-room and study scenes (the task name starts with the scene); LIBERO uses them for pretraining.

**Goal (LIBERO's language instruction):** put the red mug on the left plate.

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
