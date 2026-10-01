---
schema_version: '1.3'
task:
  name: robouse/libero-goal-9
  description: Put the wine bottle on the rack.
metadata:
  author_name: benchflow
  source_benchmark: LIBERO (Liu et al. 2023)
  source_task: put_the_wine_bottle_on_the_rack
  suite: libero
  libero_suite: libero_goal
  libero_task_index: 9
  language_instruction: put the wine bottle on the rack
  category: manipulation
  difficulty: medium
  tags:
  - franka
  - mujoco
  - robosuite
  - single-arm
  - language
  reference_solution: LIBERO demonstration demo_0 of yifengzhu-hf/LIBERO-datasets/libero_goal/put_the_wine_bottle_on_the_rack_demo.hdf5 (replay reaches success at step 140)
  robouse:
    id: libero-goal-9
    backend: libero
    suite: libero_goal
    env: put_the_wine_bottle_on_the_rack
    task_index: 9
    seed: 0
    max_steps: 300
    camera: agentview
    cameras:
    - agentview
    - robot0_eye_in_hand
    image_size: 256
    settle_steps: 50
    skills: true
    success_mode: first
    obs_mode: state
    init_source: yifengzhu-hf/LIBERO-datasets/libero_goal/put_the_wine_bottle_on_the_rack_demo.hdf5 demo_0 states[0]
    init_state:
    - 0.25000000000000017
    - -0.010827719834829845
    - -0.1754744346541302
    - 0.011732014781710433
    - -2.501597672044429
    - 0.010762696242995055
    - 2.2317431362280207
    - 0.7704934606608178
    - 0.03407103817991253
    - -0.03404111370148584
    - -0.09897637641664667
    - 0.0008544889286013318
    - 0.8981585151399347
    - 0.7071067776242912
    - -1.0724537191736548e-05
    - -1.4310617004846218e-06
    - 0.7071067846660275
    - -0.053922783084225635
    - 0.14696515269281307
    - 0.908880836284738
    - -6.358198043620697e-10
    - -6.263004995719992e-07
    - -2.1042245484118463e-06
    - 0.9999999999975901
    - -0.21117885547265683
    - -0.055151276498195845
    - 0.8986907289178425
    - -6.994649915867175e-09
    - -6.204090740395307e-06
    - 3.770094123284308e-05
    - 0.9999999992700742
    - 0.042419269159851745
    - -0.010342349721347208
    - 0.9023145976636144
    - 0.7071068944202988
    - -1.0137770013838664e-05
    - 2.0046027443781898e-05
    - 0.7071066675959596
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.015257196814925201
    - -0.025418652118732444
    - -0.0074279973941765425
    - -0.03416109420812287
    - -0.0035155220249573987
    - 0.06637849934023839
    - 0.013697648798109438
    - 0.05743569053023922
    - -0.0576905700512957
    - 1.414696485228001e-07
    - -3.4906643730643624e-08
    - 0.006561141189068615
    - -7.86050532247813e-05
    - 2.0775069654706546e-05
    - -1.2876260700081017e-09
    - -2.009935516227019e-09
    - -1.3682347461221475e-08
    - 0.0006981745727182191
    - -1.622482109803687e-06
    - 2.2843318712244614e-07
    - 7.091916064927857e-11
    - 3.625042518283948e-08
    - -2.20503795560335e-07
    - 0.004560894627160446
    - 0.00017154492857831993
    - 2.819024454838574e-05
    - 7.435971168768489e-10
    - 2.9156446724081126e-07
    - 3.0526807631300254e-07
    - 0.004946092249603376
    - 0.00012342310665910708
    - 0.00012863603417478776
    - 5.282964695876003e-09
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
oracle:
  env:
    ROBOUSE_ORACLE_TOKEN: ${ROBOUSE_ORACLE_TOKEN:-}
---

You are controlling a simulated robot. Read the task below, then solve it by running the `robo` command in your shell (start with `robo info` and `robo observe`). Keep going until the task is done, then call `robo done` once. Do not stop to ask questions; there is no human to answer.

# Put the wine bottle on the rack

A Franka Panda arm with a parallel-jaw gripper works at a table in a LIBERO scene (MuJoCo / robosuite; LIBERO suite `libero_goal`, task `put_the_wine_bottle_on_the_rack`). LIBERO-Goal: one fixed kitchen scene; the instruction sets the goal.

**Goal (LIBERO's language instruction):** put the wine bottle on the rack.

The episode ends as solved the moment LIBERO's own success check (the task's goal predicates) passes; you do not need to call `robo done` after that. If you finish without the check passing, call `robo done` (or `robo give-up`).

## This robot

- The action has 7 numbers, not 4: `robo act DX DY DZ DROLL DPITCH DYAW GRIPPER`, each in [-1, 1]. It is LIBERO's delta end-effector command (robosuite's OSC_POSE controller, 20 steps per second). Holding DX/DY/DZ at 1.0 moves the hand about 1 cm per step; DROLL/DPITCH/DYAW rotate the hand (1.0 asks for 0.5 rad; usually leave them at 0). GRIPPER -1 opens, +1 closes.
- `robo move-to X Y Z [--grip G]` and `robo grip G` are available (they keep the hand's orientation).
- World frame, metres: +x points forward, away from the robot's base; +y is to the robot's left; +z is up. The table surface is at about z = 0.90.
- `robo observe` fields: `hand_pos` (end-effector position), `eef_quat` (orientation quaternion x, y, z, w) and `eef_axis_angle`, `gripper_qpos` (the two finger joints), `gripper_open` (finger gap in metres, about 0.08 when fully open), and `objects`: the position and quaternion of every object in the scene, under LIBERO's names (e.g. `akita_black_bowl_1`, `plate_1`; `..._1`, `..._2` number identical objects); `fixtures`: the position and quaternion of each piece of furniture (cabinet, stove, microwave, rack, shelf, ...); `articulated`: each movable part of a fixture (drawer slide, door hinge, stove knob), keyed by its joint name (e.g. `wooden_cabinet_1_top_level` is the cabinet's top drawer), with `qpos` (joint position), `range` (joint limits), LIBERO's thresholds for that fixture (`open_ranges` / `close_ranges`, or `turnon_ranges` / `turnoff_ranges` for a stove knob: the joint positions at which LIBERO counts the part as open / closed or on / off) and `box_min` / `box_max` (the world-frame bounding box of the moving part).
- `robo observe --image` saves the front camera (`agentview`); add `--camera robot0_eye_in_hand` for the wrist camera. Both are 256x256.
- The step budget is 300 steps (LeRobot's LIBERO evaluation budget for this suite).

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
