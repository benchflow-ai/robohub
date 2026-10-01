---
schema_version: '1.3'
task:
  name: robouse/libero-90-16
  description: Stack the black bowl at the front on the black bowl in the middle.
metadata:
  author_name: benchflow
  source_benchmark: LIBERO (Liu et al. 2023)
  source_task: KITCHEN_SCENE2_stack_the_black_bowl_at_the_front_on_the_black_bowl_in_the_middle
  suite: libero-90
  libero_suite: libero_90
  libero_task_index: 16
  language_instruction: stack the black bowl at the front on the black bowl in the middle
  category: manipulation
  difficulty: medium
  tags:
  - franka
  - mujoco
  - robosuite
  - single-arm
  - language
  reference_solution: LIBERO demonstration demo_0 of yifengzhu-hf/LIBERO-datasets/libero_90/KITCHEN_SCENE2_stack_the_black_bowl_at_the_front_on_the_black_bowl_in_the_middle_demo.hdf5 (replay reaches success at step 99)
  robouse:
    id: libero-90-16
    backend: libero
    suite: libero_90
    env: KITCHEN_SCENE2_stack_the_black_bowl_at_the_front_on_the_black_bowl_in_the_middle
    task_index: 16
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
    init_source: yifengzhu-hf/LIBERO-datasets/libero_90/KITCHEN_SCENE2_stack_the_black_bowl_at_the_front_on_the_black_bowl_in_the_middle_demo.hdf5 demo_0 states[0]
    init_state:
    - 0.25000000000000017
    - 0.028305811309242715
    - -0.11903480230955271
    - 0.028891988737897254
    - -2.3977674678583134
    - -0.021534904923584598
    - 2.2955627070927287
    - 0.798329988526867
    - 0.03406755736706781
    - -0.03404284391623455
    - 0.08609235956369661
    - 0.17479528020351795
    - 0.8981585151399347
    - 0.7071067776242912
    - -1.0724537191719004e-05
    - -1.4310617004719794e-06
    - 0.7071067846660275
    - -0.05060560338973764
    - 0.20576441252928104
    - 0.8981585151399347
    - 0.7071067776242912
    - -1.0724537191718572e-05
    - -1.431061700471805e-06
    - 0.7071067846660275
    - -0.1450874500081417
    - 0.06616381318748242
    - 0.8981585151399347
    - 0.7071067776242912
    - -1.0724537191718467e-05
    - -1.4310617004706955e-06
    - 0.7071067846660275
    - 0.016705038176381748
    - -0.022629035785091768
    - 0.9023145976636144
    - 0.7071068944202988
    - -1.0137770013838881e-05
    - 2.0046027443780627e-05
    - 0.7071066675959596
    - 0.0
    - 0.0
    - 0.0
    - -0.014322437977747965
    - 0.4012561457992063
    - 0.06769843804920672
    - 0.3396032907229758
    - -0.0694210570642635
    - 0.28469735539000945
    - 0.23853926284604174
    - 0.05740405911134395
    - -0.057712327654370024
    - 1.4146964852477934e-07
    - -3.490664373017911e-08
    - 0.006561141189068615
    - -7.860505322546036e-05
    - 2.077506965450493e-05
    - -1.2876260701423918e-09
    - 1.4146964852583406e-07
    - -3.4906643730176254e-08
    - 0.006561141189068615
    - -7.860505322549668e-05
    - 2.077506965451562e-05
    - -1.2876260703041455e-09
    - 1.4146964852500532e-07
    - -3.49066437304457e-08
    - 0.006561141189068609
    - -7.860505322546753e-05
    - 2.077506965451891e-05
    - -1.2876260701792396e-09
    - 2.915644672404459e-07
    - 3.052680763129615e-07
    - 0.004946092249603378
    - 0.00012342310665921887
    - 0.00012863603417476586
    - 5.282964696223606e-09
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

# Stack the black bowl at the front on the black bowl in the middle

A Franka Panda arm with a parallel-jaw gripper works at a table in a LIBERO scene (MuJoCo / robosuite; LIBERO suite `libero_90`, task `KITCHEN_SCENE2_stack_the_black_bowl_at_the_front_on_the_black_bowl_in_the_middle`). LIBERO-90: 90 short-horizon tasks in 20 kitchen, living-room and study scenes (the task name starts with the scene); LIBERO uses them for pretraining.

**Goal (LIBERO's language instruction):** stack the black bowl at the front on the black bowl in the middle.

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
