---
schema_version: '1.3'
task:
  name: robouse/libero-spatial-0
  description: Pick up the black bowl between the plate and the ramekin and place it on the plate.
metadata:
  author_name: benchflow
  source_benchmark: LIBERO (Liu et al. 2023)
  source_task: pick_up_the_black_bowl_between_the_plate_and_the_ramekin_and_place_it_on_the_plate
  suite: libero
  libero_suite: libero_spatial
  libero_task_index: 0
  language_instruction: pick up the black bowl between the plate and the ramekin and place it on the plate
  category: manipulation
  difficulty: easy
  tags:
  - franka
  - mujoco
  - robosuite
  - single-arm
  - language
  reference_solution: LIBERO demonstration demo_0 of yifengzhu-hf/LIBERO-datasets/libero_spatial/pick_up_the_black_bowl_between_the_plate_and_the_ramekin_and_place_it_on_the_plate_demo.hdf5 (replay reaches success at step 80)
  robouse:
    id: libero-spatial-0
    backend: libero
    suite: libero_spatial
    env: pick_up_the_black_bowl_between_the_plate_and_the_ramekin_and_place_it_on_the_plate
    task_index: 0
    seed: 0
    max_steps: 280
    camera: agentview
    cameras:
    - agentview
    - robot0_eye_in_hand
    image_size: 256
    settle_steps: 50
    skills: true
    success_mode: first
    obs_mode: state
    init_source: yifengzhu-hf/LIBERO-datasets/libero_spatial/pick_up_the_black_bowl_between_the_plate_and_the_ramekin_and_place_it_on_the_plate_demo.hdf5 demo_0 states[0]
    init_state:
    - 0.25000000000000017
    - 0.009019898563086272
    - -0.177987441920143
    - -0.011762278221066853
    - -2.456470582555746
    - -0.010007706580357911
    - 2.2217566503965314
    - 0.7968477981629051
    - 0.03404727272255755
    - -0.03406500245258034
    - -0.0508518704165359
    - 0.20632919230309657
    - 0.8982497088884221
    - 0.7005747485364545
    - -0.0005446092009487705
    - -0.0009369690281430307
    - 0.7135781997811746
    - -0.18368891273205193
    - 0.3334826502030125
    - 0.8981585151399347
    - 0.7071067776242912
    - -1.0724537191721354e-05
    - -1.431061700470947e-06
    - 0.7071067846660275
    - 0.05507679066941424
    - 0.016311004296964728
    - 0.9092097974085793
    - 0.7071067797562697
    - -6.936573033754899e-07
    - -2.0480328856650525e-06
    - 0.7071067826135193
    - -0.1874222727543386
    - 0.2058558447026561
    - 0.8991595343682753
    - 0.7071067709929622
    - -1.51656775721273e-06
    - 2.144557653633392e-06
    - 0.7071067913752543
    - 0.08441362086504602
    - 0.19687207103884172
    - 0.9024714419263586
    - 0.7191058121649646
    - 0.0003934993357592161
    - 0.00017112105169062852
    - 0.6949004581855053
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - -0.010008046359049484
    - 0.03444800756876818
    - 0.010674512664988763
    - 0.0828625002650164
    - 0.0013687891783600002
    - -0.1380849820458782
    - -8.571108537405935e-05
    - 0.0575863150425562
    - -0.05753988202499773
    - -0.00010301646119612872
    - -2.9225113728833584e-05
    - 0.004121716305164118
    - 0.05600167947620913
    - 0.014804422378334677
    - 2.662853129535255e-06
    - 1.4146964852483345e-07
    - -3.4906643732016135e-08
    - 0.006561141189068619
    - -7.860505322542519e-05
    - 2.0775069654644625e-05
    - -1.287626070179986e-09
    - -3.110017478220929e-08
    - -2.9459104358471847e-08
    - 0.004317179528839084
    - -3.3344105771948264e-06
    - -3.1883474568318285e-06
    - 3.251365935691835e-11
    - 1.613410841819685e-09
    - -4.406652723811371e-08
    - 0.004850734336304697
    - -1.6344860315287347e-06
    - 5.0263050417952456e-05
    - -4.25660362654411e-10
    - -5.049789799441472e-05
    - 2.459038235104405e-05
    - 0.0007533381176057234
    - -0.02109519711541489
    - 0.009418029580854498
    - -1.0503919626205427e-06
    - 0.0
    - 0.0
    - 0.0
    - 0.0
agent:
  timeout_sec: 1200
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

# Pick up the black bowl between the plate and the ramekin and place it on the plate

A Franka Panda arm with a parallel-jaw gripper works at a table in a LIBERO scene (MuJoCo / robosuite; LIBERO suite `libero_spatial`, task `pick_up_the_black_bowl_between_the_plate_and_the_ramekin_and_place_it_on_the_plate`). LIBERO-Spatial: the same objects in different layouts; the instruction says which of two identical bowls to move.

**Goal (LIBERO's language instruction):** pick up the black bowl between the plate and the ramekin and place it on the plate.

The episode ends as solved the moment LIBERO's own success check (the task's goal predicates) passes; you do not need to call `robo done` after that. If you finish without the check passing, call `robo done` (or `robo give-up`).

## This robot

- The action has 7 numbers, not 4: `robo act DX DY DZ DROLL DPITCH DYAW GRIPPER`, each in [-1, 1]. It is LIBERO's delta end-effector command (robosuite's OSC_POSE controller, 20 steps per second). Holding DX/DY/DZ at 1.0 moves the hand about 1 cm per step; DROLL/DPITCH/DYAW rotate the hand (1.0 asks for 0.5 rad; usually leave them at 0). GRIPPER -1 opens, +1 closes.
- `robo move-to X Y Z [--grip G]` and `robo grip G` are available (they keep the hand's orientation).
- World frame, metres: +x points forward, away from the robot's base; +y is to the robot's left; +z is up. The table surface is at about z = 0.90.
- `robo observe` fields: `hand_pos` (end-effector position), `eef_quat` (orientation quaternion x, y, z, w) and `eef_axis_angle`, `gripper_qpos` (the two finger joints), `gripper_open` (finger gap in metres, about 0.08 when fully open), and `objects`: the position and quaternion of every object in the scene, under LIBERO's names (e.g. `akita_black_bowl_1`, `plate_1`; `..._1`, `..._2` number identical objects).
- `robo observe --image` saves the front camera (`agentview`); add `--camera robot0_eye_in_hand` for the wrist camera. Both are 256x256.
- The step budget is 280 steps (LeRobot's LIBERO evaluation budget for this suite).

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
