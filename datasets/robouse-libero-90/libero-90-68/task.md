---
schema_version: '1.3'
task:
  name: robouse/libero-90-68
  description: Put the yellow and white mug on the right plate.
metadata:
  author_name: benchflow
  source_benchmark: LIBERO (Liu et al. 2023)
  source_task: LIVING_ROOM_SCENE5_put_the_yellow_and_white_mug_on_the_right_plate
  suite: libero-90
  libero_suite: libero_90
  libero_task_index: 68
  language_instruction: put the yellow and white mug on the right plate
  category: manipulation
  difficulty: medium
  tags:
  - franka
  - mujoco
  - robosuite
  - single-arm
  - language
  reference_solution: LIBERO demonstration demo_0 of yifengzhu-hf/LIBERO-datasets/libero_90/LIVING_ROOM_SCENE5_put_the_yellow_and_white_mug_on_the_right_plate_demo.hdf5 (replay reaches success at step 110)
  robouse:
    id: libero-90-68
    backend: libero
    suite: libero_90
    env: LIVING_ROOM_SCENE5_put_the_yellow_and_white_mug_on_the_right_plate
    task_index: 68
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
    init_source: yifengzhu-hf/LIBERO-datasets/libero_90/LIVING_ROOM_SCENE5_put_the_yellow_and_white_mug_on_the_right_plate_demo.hdf5 demo_0 states[0]
    init_state:
    - 0.25000000000000017
    - 0.005798361923733645
    - -0.17351871588263706
    - 0.01885320075260342
    - -2.4416938572202915
    - 0.004406817312617995
    - 2.2233564327039965
    - 0.7965737140938295
    - 0.034071053357466914
    - -0.0340404876502333
    - -0.08273108085368429
    - -0.16618344414292682
    - 0.4343699488393418
    - -0.7071069013731585
    - 1.0172763262329209e-07
    - 6.675731368064017e-08
    - 0.7071066609999057
    - -0.20786149419257927
    - -0.021972494839066006
    - 0.4385989872647111
    - -0.7071068579899178
    - -5.528613175471845e-07
    - 3.8311997405314106e-07
    - 0.707106704382849
    - -0.05961964678315957
    - 0.09743425669978754
    - 0.43685696889004044
    - -0.7071067450257308
    - -9.863721141827464e-07
    - 5.538107393046743e-08
    - 0.7071068173466724
    - 0.005952467773838534
    - -0.3009247232001067
    - 0.4392727869250624
    - 0.7071068416649496
    - 1.22536117849771e-06
    - -6.586728017148901e-07
    - 0.7071067207067718
    - 0.007695726830548016
    - 0.28881155706027833
    - 0.4392727869250624
    - 0.7071068416649496
    - 1.2253611784964604e-06
    - -6.5867280171337e-07
    - 0.7071067207067718
    - 0.0032910811366556225
    - 0.0500215783180121
    - 0.01599538368552491
    - 0.05941010061567419
    - 0.016953076650757386
    - -0.07215944835219178
    - 0.08298161539446691
    - 0.05747468748162863
    - -0.057650884151347954
    - -7.11462918734567e-11
    - -4.5271501834646214e-10
    - 1.1368093065907948e-06
    - -2.8896237721669387e-08
    - -1.846680129470086e-07
    - -1.0830534401869866e-12
    - 8.910655961979004e-12
    - -1.8765528085485468e-10
    - 1.8430481125895578e-06
    - -4.9171927396552905e-09
    - 1.0451616238144095e-07
    - -2.7722845463172093e-13
    - 6.84714436342402e-13
    - -7.276786467126971e-12
    - 2.2982456672070276e-06
    - -1.737521395493368e-08
    - 1.2912372764390733e-07
    - -3.2601171582884713e-13
    - 1.8627973189127923e-10
    - 1.1309283914603e-11
    - 2.7645383596792127e-06
    - 7.554087121067328e-08
    - 4.563006011729477e-09
    - 1.7231481799859512e-12
    - 1.862797319147202e-10
    - 1.1309283658301917e-11
    - 2.7645383596816513e-06
    - 7.55408712076456e-08
    - 4.563006046456941e-09
    - 1.7231481798050073e-12
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

# Put the yellow and white mug on the right plate

A Franka Panda arm with a parallel-jaw gripper works at a low table in a LIBERO scene (MuJoCo / robosuite; LIBERO suite `libero_90`, task `LIVING_ROOM_SCENE5_put_the_yellow_and_white_mug_on_the_right_plate`). LIBERO-90: 90 short-horizon tasks in 20 kitchen, living-room and study scenes (the task name starts with the scene); LIBERO uses them for pretraining.

**Goal (LIBERO's language instruction):** put the yellow and white mug on the right plate.

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
