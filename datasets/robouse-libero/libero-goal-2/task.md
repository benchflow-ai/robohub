---
schema_version: '1.3'
task:
  name: robouse/libero-goal-2
  description: Put the wine bottle on top of the cabinet.
metadata:
  author_name: benchflow
  source_benchmark: LIBERO (Liu et al. 2023)
  source_task: put_the_wine_bottle_on_top_of_the_cabinet
  suite: libero
  libero_suite: libero_goal
  libero_task_index: 2
  language_instruction: put the wine bottle on top of the cabinet
  category: manipulation
  difficulty: medium
  tags:
  - franka
  - mujoco
  - robosuite
  - single-arm
  - language
  reference_solution: LIBERO demonstration demo_0 of yifengzhu-hf/LIBERO-datasets/libero_goal/put_the_wine_bottle_on_top_of_the_cabinet_demo.hdf5 (replay reaches success at step 82)
  robouse:
    id: libero-goal-2
    backend: libero
    suite: libero_goal
    env: put_the_wine_bottle_on_top_of_the_cabinet
    task_index: 2
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
    init_source: yifengzhu-hf/LIBERO-datasets/libero_goal/put_the_wine_bottle_on_top_of_the_cabinet_demo.hdf5 demo_0 states[0]
    init_state:
    - 0.25000000000000017
    - -0.02934305025630378
    - -0.17014486203954607
    - 0.007648333094754443
    - -2.452564102598677
    - -0.002925982325495644
    - 2.227628578663948
    - 0.7524878016797752
    - 0.03405020725647786
    - -0.034061845617642576
    - -0.08569821721892197
    - 0.009967112757146864
    - 0.8981585151399347
    - 0.7071067776242912
    - -1.0724537191719951e-05
    - -1.4310617004712029e-06
    - 0.7071067846660275
    - -0.03683291600406698
    - 0.1437833701744601
    - 0.908880836284738
    - -6.358198043644164e-10
    - -6.263004995717166e-07
    - -2.1042245484149303e-06
    - 0.9999999999975901
    - -0.19440499653871998
    - -0.06302196225333061
    - 0.8986907289178425
    - -6.994649915883616e-09
    - -6.204090740395598e-06
    - 3.770094123284273e-05
    - 0.9999999992700742
    - 0.04643156773992635
    - -0.03308630159362711
    - 0.9023145976636144
    - 0.7071068944202988
    - -1.0137770013842993e-05
    - 2.004602744377598e-05
    - 0.7071066675959596
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.03068853320037696
    - 0.023225943121841917
    - -0.02858383255064006
    - 0.004711092193942367
    - 0.00872400097179736
    - 5.050364322107076e-05
    - 0.09886534826000874
    - 0.05760345111774951
    - -0.0575257566368278
    - 1.4146964852462667e-07
    - -3.490664373052078e-08
    - 0.006561141189068616
    - -7.86050532254567e-05
    - 2.077506965451695e-05
    - -1.2876260701245084e-09
    - -2.009935516256655e-09
    - -1.3682347459892427e-08
    - 0.0006981745727182207
    - -1.62248210977278e-06
    - 2.2843318727783664e-07
    - 7.091916097264052e-11
    - 3.625042518231041e-08
    - -2.2050379555947566e-07
    - 0.00456089462716045
    - 0.00017154492857832533
    - 2.8190244548352952e-05
    - 7.435971164952407e-10
    - 2.915644672402955e-07
    - 3.0526807631248627e-07
    - 0.004946092249603379
    - 0.0001234231066589861
    - 0.00012863603417464326
    - 5.282964696009602e-09
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

# Put the wine bottle on top of the cabinet

A Franka Panda arm with a parallel-jaw gripper works at a table in a LIBERO scene (MuJoCo / robosuite; LIBERO suite `libero_goal`, task `put_the_wine_bottle_on_top_of_the_cabinet`). LIBERO-Goal: one fixed kitchen scene; the instruction sets the goal.

**Goal (LIBERO's language instruction):** put the wine bottle on top of the cabinet.

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
