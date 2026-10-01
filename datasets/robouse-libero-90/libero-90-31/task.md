---
schema_version: '1.3'
task:
  name: robouse/libero-90-31
  description: Put the black bowl on top of the cabinet.
metadata:
  author_name: benchflow
  source_benchmark: LIBERO (Liu et al. 2023)
  source_task: KITCHEN_SCENE5_put_the_black_bowl_on_top_of_the_cabinet
  suite: libero-90
  libero_suite: libero_90
  libero_task_index: 31
  language_instruction: put the black bowl on top of the cabinet
  category: manipulation
  difficulty: medium
  tags:
  - franka
  - mujoco
  - robosuite
  - single-arm
  - language
  reference_solution: LIBERO demonstration demo_0 of yifengzhu-hf/LIBERO-datasets/libero_90/KITCHEN_SCENE5_put_the_black_bowl_on_top_of_the_cabinet_demo.hdf5 (replay reaches success at step 137)
  robouse:
    id: libero-90-31
    backend: libero
    suite: libero_90
    env: KITCHEN_SCENE5_put_the_black_bowl_on_top_of_the_cabinet
    task_index: 31
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
    init_source: yifengzhu-hf/LIBERO-datasets/libero_90/KITCHEN_SCENE5_put_the_black_bowl_on_top_of_the_cabinet_demo.hdf5 demo_0 states[0]
    init_state:
    - 0.25000000000000017
    - 0.03670113612450117
    - -0.1251689031814266
    - -0.02034911351250735
    - -2.394751998552735
    - 0.014491965100279997
    - 2.172416791720127
    - 0.7872862129364753
    - 0.033998205332924795
    - -0.03407037693972752
    - 0.023132067195684516
    - -0.07258895602051257
    - 0.8981585151399347
    - 0.7071067776242912
    - -1.0724537191720768e-05
    - -1.4310617004717621e-06
    - 0.7071067846660275
    - -0.04944302792993883
    - -0.271393224506368
    - 0.9023145976636144
    - 0.7071068944202988
    - -1.0137770013840357e-05
    - 2.0046027443778127e-05
    - 0.7071066675959596
    - -0.1008040688676755
    - -0.11051448059169365
    - 0.9723871721331563
    - 0.5000044823789854
    - 0.49999551753798316
    - 0.499991192026421
    - 0.5000088078612678
    - -0.1465747353849811
    - 0.0
    - 0.0
    - -0.08540088723613867
    - 0.3441701309535515
    - -0.11601439995742006
    - 0.4327738106320866
    - 0.06441380946668118
    - -0.29236041206449165
    - -0.23521222497805572
    - 0.057641861916241655
    - -0.05740742695608344
    - 1.414696485249917e-07
    - -3.490664373055657e-08
    - 0.0065611411890686
    - -7.860505322543263e-05
    - 2.0775069654595748e-05
    - -1.2876260701886537e-09
    - 2.9156446724059463e-07
    - 3.052680763125384e-07
    - 0.004946092249603386
    - 0.00012342310665918298
    - 0.00012863603417463868
    - 5.282964696102626e-09
    - -2.824503031360855e-07
    - -9.573701859860567e-08
    - 0.0001004166502093669
    - -3.9065367514511e-06
    - 1.3117100179670859e-11
    - 1.3239747298464566e-06
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

# Put the black bowl on top of the cabinet

A Franka Panda arm with a parallel-jaw gripper works at a table in a LIBERO scene (MuJoCo / robosuite; LIBERO suite `libero_90`, task `KITCHEN_SCENE5_put_the_black_bowl_on_top_of_the_cabinet`). LIBERO-90: 90 short-horizon tasks in 20 kitchen, living-room and study scenes (the task name starts with the scene); LIBERO uses them for pretraining.

**Goal (LIBERO's language instruction):** put the black bowl on top of the cabinet.

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
