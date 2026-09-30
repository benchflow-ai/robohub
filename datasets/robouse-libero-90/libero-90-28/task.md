---
schema_version: '1.3'
task:
  name: robouse/libero-90-28
  description: Close the top drawer of the cabinet.
metadata:
  author_name: benchflow
  source_benchmark: LIBERO (Liu et al. 2023)
  source_task: KITCHEN_SCENE5_close_the_top_drawer_of_the_cabinet
  suite: libero-90
  libero_suite: libero_90
  libero_task_index: 28
  language_instruction: close the top drawer of the cabinet
  category: manipulation
  difficulty: medium
  tags:
  - franka
  - mujoco
  - robosuite
  - single-arm
  - language
  reference_solution: LIBERO demonstration demo_0 of yifengzhu-hf/LIBERO-datasets/libero_90/KITCHEN_SCENE5_close_the_top_drawer_of_the_cabinet_demo.hdf5 (replay reaches success at step 55)
  robouse:
    id: libero-90-28
    backend: libero
    suite: libero_90
    env: KITCHEN_SCENE5_close_the_top_drawer_of_the_cabinet
    task_index: 28
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
    init_source: yifengzhu-hf/LIBERO-datasets/libero_90/KITCHEN_SCENE5_close_the_top_drawer_of_the_cabinet_demo.hdf5 demo_0 states[0]
    init_state:
    - 0.25000000000000017
    - 0.005689934252073042
    - -0.14135360627902113
    - -0.03415145151096381
    - -2.4335882397627304
    - 0.013567382466998578
    - 2.250188336441611
    - 0.7791280955942809
    - 0.034015620079908535
    - -0.034070607670318206
    - 0.038242225104535024
    - -0.06606962767884034
    - 0.8981585151399347
    - 0.7071067776242912
    - -1.0724537191720586e-05
    - -1.4310617004719423e-06
    - 0.7071067846660275
    - -0.06577612191488898
    - -0.24225623182345737
    - 0.9023145976636144
    - 0.7071068944202988
    - -1.0137770013843626e-05
    - 2.004602744377765e-05
    - 0.7071066675959596
    - -0.08428444942991574
    - -0.12323496055393644
    - 0.9723871721331563
    - 0.5000044823789854
    - 0.49999551753798316
    - 0.499991192026421
    - 0.5000088078612678
    - -0.15933479496981529
    - 0.0
    - 0.0
    - -0.04444090592773066
    - 0.18565139032069702
    - -0.08888331724494951
    - 0.15137981111460924
    - -0.012270418730590382
    - 0.1382238198537698
    - -0.12311097010090728
    - 0.057799005721859205
    - -0.05725415783326211
    - 1.4146964852532724e-07
    - -3.490664373049653e-08
    - 0.006561141189068617
    - -7.860505322544401e-05
    - 2.0775069654589222e-05
    - -1.2876260702418774e-09
    - 2.9156446724036455e-07
    - 3.052680763125381e-07
    - 0.004946092249603383
    - 0.00012342310665896337
    - 0.00012863603417454693
    - 5.282964695928136e-09
    - -2.8245030313597577e-07
    - -9.573701859851379e-08
    - 0.0001004166502093618
    - -3.906536751442272e-06
    - 1.3117100124080512e-11
    - 1.323974729841683e-06
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

# Close the top drawer of the cabinet

A Franka Panda arm with a parallel-jaw gripper works at a table in a LIBERO scene (MuJoCo / robosuite; LIBERO suite `libero_90`, task `KITCHEN_SCENE5_close_the_top_drawer_of_the_cabinet`). LIBERO-90: 90 short-horizon tasks in 20 kitchen, living-room and study scenes (the task name starts with the scene); LIBERO uses them for pretraining.

**Goal (LIBERO's language instruction):** close the top drawer of the cabinet.

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
