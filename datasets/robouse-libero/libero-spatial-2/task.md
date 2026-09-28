---
schema_version: '1.3'
task:
  name: robouse/libero-spatial-2
  description: Pick up the black bowl from table center and place it on the plate.
metadata:
  author_name: benchflow
  source_benchmark: LIBERO (Liu et al. 2023)
  source_task: pick_up_the_black_bowl_from_table_center_and_place_it_on_the_plate
  suite: libero
  libero_suite: libero_spatial
  libero_task_index: 2
  language_instruction: pick up the black bowl from table center and place it on the plate
  category: manipulation
  difficulty: easy
  tags:
  - franka
  - mujoco
  - robosuite
  - single-arm
  - language
  reference_solution: LIBERO demonstration demo_0 of yifengzhu-hf/LIBERO-datasets/libero_spatial/pick_up_the_black_bowl_from_table_center_and_place_it_on_the_plate_demo.hdf5 (replay reaches success at step 92)
  robouse:
    id: libero-spatial-2
    backend: libero
    suite: libero_spatial
    env: pick_up_the_black_bowl_from_table_center_and_place_it_on_the_plate
    task_index: 2
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
    init_source: yifengzhu-hf/LIBERO-datasets/libero_spatial/pick_up_the_black_bowl_from_table_center_and_place_it_on_the_plate_demo.hdf5 demo_0 states[0]
    init_state:
    - 0.25000000000000017
    - -0.0023087237368767726
    - -0.1406615344200603
    - -0.0020515233590529805
    - -2.4222600857240613
    - 0.0038576568891306724
    - 2.2267584994062517
    - 0.7885900840061469
    - 0.03405809172109161
    - -0.03405427949937426
    - -0.07500029343279396
    - 0.0006212310877957377
    - 0.8981585151399347
    - 0.7071067776242912
    - -1.072453719171965e-05
    - -1.4310617004716717e-06
    - 0.7071067846660275
    - 0.021524935065942052
    - 0.31638759814812395
    - 0.8981585151399347
    - 0.7071067776242912
    - -1.0724537191719446e-05
    - -1.4310617004708876e-06
    - 0.7071067846660275
    - 0.08291863731336635
    - 0.0340180518469707
    - 0.9092097974085793
    - 0.7071067797562697
    - -6.936573033714278e-07
    - -2.0480328856674407e-06
    - 0.7071067826135193
    - -0.19579504326276503
    - 0.19863886482047233
    - 0.8991595343682753
    - 0.7071067709929622
    - -1.5165677572040736e-06
    - 2.1445576536282954e-06
    - 0.7071067913752543
    - 0.06315814527370193
    - 0.1854957106144051
    - 0.9023145976636144
    - 0.7071068944202988
    - -1.0137770013839591e-05
    - 2.004602744377948e-05
    - 0.7071066675959596
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 4.87157663874668e-07
    - -0.00013327609550427475
    - -0.0004335335060655541
    - -0.0006350020268388989
    - 0.0005771901515165513
    - -1.6224633977146746e-05
    - 0.00011098422190222275
    - 0.05757783063406051
    - -0.05754724591264901
    - 1.4146964852491693e-07
    - -3.490664372991237e-08
    - 0.006561141189068621
    - -7.860505322545612e-05
    - 2.077506965450074e-05
    - -1.2876260701639227e-09
    - 1.414696485260873e-07
    - -3.4906643731391316e-08
    - 0.006561141189068625
    - -7.860505322550977e-05
    - 2.0775069654547436e-05
    - -1.2876260703527939e-09
    - -3.110017478218119e-08
    - -2.9459104332139826e-08
    - 0.004317179528839083
    - -3.334410577158539e-06
    - -3.1883474540587625e-06
    - 3.2513659460989796e-11
    - 1.6134108451472188e-09
    - -4.4066527236806104e-08
    - 0.004850734336304688
    - -1.6344860317317175e-06
    - 5.026305041720244e-05
    - -4.2566036323133387e-10
    - 2.915644672408717e-07
    - 3.0526807631288374e-07
    - 0.004946092249603377
    - 0.00012342310665915848
    - 0.00012863603417474887
    - 5.2829646959111995e-09
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

# Pick up the black bowl from table center and place it on the plate

A Franka Panda arm with a parallel-jaw gripper works at a table in a LIBERO scene (MuJoCo / robosuite; LIBERO suite `libero_spatial`, task `pick_up_the_black_bowl_from_table_center_and_place_it_on_the_plate`). LIBERO-Spatial: the same objects in different layouts; the instruction says which of two identical bowls to move.

**Goal (LIBERO's language instruction):** pick up the black bowl from table center and place it on the plate.

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
