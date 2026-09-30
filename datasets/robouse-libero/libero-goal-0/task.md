---
schema_version: '1.3'
task:
  name: robouse/libero-goal-0
  description: Open the middle drawer of the cabinet.
metadata:
  author_name: benchflow
  source_benchmark: LIBERO (Liu et al. 2023)
  source_task: open_the_middle_drawer_of_the_cabinet
  suite: libero
  libero_suite: libero_goal
  libero_task_index: 0
  language_instruction: open the middle drawer of the cabinet
  category: manipulation
  difficulty: medium
  tags:
  - franka
  - mujoco
  - robosuite
  - single-arm
  - language
  reference_solution: LIBERO demonstration demo_0 of yifengzhu-hf/LIBERO-datasets/libero_goal/open_the_middle_drawer_of_the_cabinet_demo.hdf5 (replay reaches success at step 127)
  robouse:
    id: libero-goal-0
    backend: libero
    suite: libero_goal
    env: open_the_middle_drawer_of_the_cabinet
    task_index: 0
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
    init_source: yifengzhu-hf/LIBERO-datasets/libero_goal/open_the_middle_drawer_of_the_cabinet_demo.hdf5 demo_0 states[0]
    init_state:
    - 0.25000000000000017
    - -0.01388070354278223
    - -0.1690074186255694
    - 0.005121813407845808
    - -2.4096506078664413
    - 0.012104268919278596
    - 2.202243916042183
    - 0.788210109577686
    - 0.03407504396929852
    - -0.034037111500462666
    - -0.07694334009457428
    - 0.006931435879949965
    - 0.8981585151399347
    - 0.7071067776242912
    - -1.0724537191720891e-05
    - -1.4310617004709704e-06
    - 0.7071067846660275
    - -0.054215399436762686
    - 0.12424902167166958
    - 0.908880836284738
    - -6.35819804380123e-10
    - -6.263004995716526e-07
    - -2.1042245484131524e-06
    - 0.9999999999975901
    - -0.2001502791384002
    - -0.061828050182927245
    - 0.8986907289178425
    - -6.99464991586925e-09
    - -6.204090740395049e-06
    - 3.7700941232842034e-05
    - 0.9999999992700742
    - 0.0630165995215829
    - -0.01829537038509236
    - 0.9023145976636144
    - 0.7071068944202988
    - -1.013777001383861e-05
    - 2.00460274437805e-05
    - 0.7071066675959596
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.02114442188272401
    - 0.04901994248170547
    - 0.002975342630204747
    - 0.05114368543768923
    - -0.016252571488138454
    - -0.10862430273055775
    - 0.02664083938752081
    - 0.05750370780153733
    - -0.05761611133097043
    - 1.4146964852391177e-07
    - -3.490664373049514e-08
    - 0.0065611411890686136
    - -7.860505322539787e-05
    - 2.077506965459318e-05
    - -1.2876260700221253e-09
    - -2.0099355162142608e-09
    - -1.3682347460375735e-08
    - 0.0006981745727182202
    - -1.6224821095972145e-06
    - 2.2843318716825363e-07
    - 7.09191606370487e-11
    - 3.625042518163624e-08
    - -2.205037955584591e-07
    - 0.004560894627160448
    - 0.00017154492857833026
    - 2.819024454836171e-05
    - 7.435971160136439e-10
    - 2.9156446724111725e-07
    - 3.0526807631300333e-07
    - 0.004946092249603386
    - 0.0001234231066591252
    - 0.00012863603417476199
    - 5.282964695726748e-09
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

# Open the middle drawer of the cabinet

A Franka Panda arm with a parallel-jaw gripper works at a table in a LIBERO scene (MuJoCo / robosuite; LIBERO suite `libero_goal`, task `open_the_middle_drawer_of_the_cabinet`). LIBERO-Goal: one fixed kitchen scene; the instruction sets the goal.

**Goal (LIBERO's language instruction):** open the middle drawer of the cabinet.

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
