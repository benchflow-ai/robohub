---
schema_version: '1.3'
task:
  name: robouse/libero-goal-1
  description: Put the bowl on the stove.
metadata:
  author_name: benchflow
  source_benchmark: LIBERO (Liu et al. 2023)
  source_task: put_the_bowl_on_the_stove
  suite: libero
  libero_suite: libero_goal
  libero_task_index: 1
  language_instruction: put the bowl on the stove
  category: manipulation
  difficulty: medium
  tags:
  - franka
  - mujoco
  - robosuite
  - single-arm
  - language
  reference_solution: LIBERO demonstration demo_0 of yifengzhu-hf/LIBERO-datasets/libero_goal/put_the_bowl_on_the_stove_demo.hdf5 (replay reaches success at step 83)
  robouse:
    id: libero-goal-1
    backend: libero
    suite: libero_goal
    env: put_the_bowl_on_the_stove
    task_index: 1
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
    init_source: yifengzhu-hf/LIBERO-datasets/libero_goal/put_the_bowl_on_the_stove_demo.hdf5 demo_0 states[0]
    init_state:
    - 0.25000000000000017
    - -0.011941282479744299
    - -0.14863805413981254
    - 0.03344582903989591
    - -2.442386985545223
    - -0.014288006683300504
    - 2.2589516075263583
    - 0.774727568945516
    - 0.03404541340707785
    - -0.034066551117634354
    - -0.1028593402800959
    - 0.014367431167607947
    - 0.8981585151399347
    - 0.7071067776242912
    - -1.0724537191718811e-05
    - -1.431061700470688e-06
    - 0.7071067846660275
    - -0.06564055312149446
    - 0.12089330890635891
    - 0.908880836284738
    - -6.358198043671046e-10
    - -6.263004995718932e-07
    - -2.1042245484140134e-06
    - 0.9999999999975901
    - -0.1968252002908192
    - -0.05109302597592776
    - 0.8986907289178425
    - -6.99464991579463e-09
    - -6.204090740395527e-06
    - 3.770094123284277e-05
    - 0.9999999992700742
    - 0.05084345161439483
    - -0.01464461446544418
    - 0.9023145976636144
    - 0.7071068944202988
    - -1.013777001385233e-05
    - 2.004602744377557e-05
    - 0.7071066675959596
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.018505216302422748
    - 0.09401045371127903
    - -0.02067802720712727
    - 0.09345784599338161
    - -0.005074651221336379
    - -0.00956072606940161
    - 8.242940713902758e-05
    - 0.05750073305625907
    - -0.05762891988907718
    - 1.4146964852593994e-07
    - -3.490664372990594e-08
    - 0.00656114118906862
    - -7.860505322550328e-05
    - 2.077506965449612e-05
    - -1.2876260703173355e-09
    - -2.0099355162435368e-09
    - -1.3682347459848906e-08
    - 0.0006981745727182181
    - -1.6224821094178127e-06
    - 2.284331871840767e-07
    - 7.091916056193144e-11
    - 3.6250425182058115e-08
    - -2.2050379555827137e-07
    - 0.004560894627160448
    - 0.0001715449285783037
    - 2.8190244548352552e-05
    - 7.435971164528824e-10
    - 2.915644672396835e-07
    - 3.0526807631229913e-07
    - 0.004946092249603378
    - 0.00012342310665882197
    - 0.0001286360341745153
    - 5.282964696117345e-09
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

# Put the bowl on the stove

A Franka Panda arm with a parallel-jaw gripper works at a table in a LIBERO scene (MuJoCo / robosuite; LIBERO suite `libero_goal`, task `put_the_bowl_on_the_stove`). LIBERO-Goal: one fixed kitchen scene; the instruction sets the goal.

**Goal (LIBERO's language instruction):** put the bowl on the stove.

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
