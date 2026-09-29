---
schema_version: '1.3'
task:
  name: robouse/libero-90-5
  description: Put the chocolate pudding in the top drawer of the cabinet and close it.
metadata:
  author_name: benchflow
  source_benchmark: LIBERO (Liu et al. 2023)
  source_task: KITCHEN_SCENE10_put_the_chocolate_pudding_in_the_top_drawer_of_the_cabinet_and_close_it
  suite: libero-90
  libero_suite: libero_90
  libero_task_index: 5
  language_instruction: put the chocolate pudding in the top drawer of the cabinet and close it
  category: manipulation
  difficulty: medium
  tags:
  - franka
  - mujoco
  - robosuite
  - single-arm
  - language
  reference_solution: LIBERO demonstration demo_0 of yifengzhu-hf/LIBERO-datasets/libero_90/KITCHEN_SCENE10_put_the_chocolate_pudding_in_the_top_drawer_of_the_cabinet_and_close_it_demo.hdf5 (replay reaches success at step 157)
  robouse:
    id: libero-90-5
    backend: libero
    suite: libero_90
    env: KITCHEN_SCENE10_put_the_chocolate_pudding_in_the_top_drawer_of_the_cabinet_and_close_it
    task_index: 5
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
    init_source: yifengzhu-hf/LIBERO-datasets/libero_90/KITCHEN_SCENE10_put_the_chocolate_pudding_in_the_top_drawer_of_the_cabinet_and_close_it_demo.hdf5 demo_0 states[0]
    init_state:
    - 0.25000000000000017
    - -0.028621301507279977
    - -0.10981944206900994
    - -0.0037900301455446265
    - -2.406112539408007
    - 0.009812147799650768
    - 2.2353320232420812
    - 0.7925355657936124
    - 0.03405779801522629
    - -0.03405348428768428
    - -0.08956376214175281
    - -0.010858974244433559
    - 0.8981585151399347
    - 0.7071067776242912
    - -1.0724537191720522e-05
    - -1.431061700471133e-06
    - 0.7071067846660275
    - 0.023141845318917394
    - 0.20819306199544405
    - 0.9086548762142962
    - -4.088883686537806e-10
    - -8.196927805403796e-07
    - -4.739534225495164e-06
    - 0.9999999999884325
    - -0.09428187479674706
    - 0.1999579463091269
    - 0.9086548762142962
    - -4.088883686701197e-10
    - -8.196927805371039e-07
    - -4.739534225494881e-06
    - 0.9999999999884325
    - 0.01890876970841343
    - 0.06454196587555705
    - 0.9129232581896339
    - 9.570210259714474e-09
    - -4.3323218685025666e-07
    - 2.1414477365012354e-06
    - 0.9999999999976134
    - -0.1509133947032639
    - 0.0
    - 0.0
    - 0.028609123740458615
    - 0.33739672026795686
    - -0.01790310836974341
    - 0.33841838184636286
    - -0.007685766181102886
    - -0.10423483641987008
    - 0.039727103891763094
    - 0.05765218691120584
    - -0.05748300308365179
    - 1.4146964852394076e-07
    - -3.49066437294007e-08
    - 0.006561141189068615
    - -7.860505322539723e-05
    - 2.0775069654547582e-05
    - -1.2876260700115947e-09
    - -5.7284583741391575e-09
    - -4.14254995619275e-08
    - 0.0006971084279367759
    - -5.214947459474845e-06
    - 6.757763810999784e-07
    - 3.8155037284798547e-10
    - -5.728458374157081e-09
    - -4.1425499561777065e-08
    - 0.0006971084279367825
    - -5.2149474593049735e-06
    - 6.757763810474319e-07
    - 3.8155037270857664e-10
    - -1.791151996316199e-07
    - 9.463563512572024e-08
    - 0.003418716150299917
    - 7.3836819804068236e-06
    - 1.3920238412862756e-05
    - 3.556957416717269e-10
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
---

You are controlling a simulated robot. Read the task below, then solve it by running the `robo` command in your shell (start with `robo info` and `robo observe`). Keep going until the task is done, then call `robo done` once. Do not stop to ask questions; there is no human to answer.

# Put the chocolate pudding in the top drawer of the cabinet and close it

A Franka Panda arm with a parallel-jaw gripper works at a table in a LIBERO scene (MuJoCo / robosuite; LIBERO suite `libero_90`, task `KITCHEN_SCENE10_put_the_chocolate_pudding_in_the_top_drawer_of_the_cabinet_and_close_it`). LIBERO-90: 90 short-horizon tasks in 20 kitchen, living-room and study scenes (the task name starts with the scene); LIBERO uses them for pretraining.

**Goal (LIBERO's language instruction):** put the chocolate pudding in the top drawer of the cabinet and close it.

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
