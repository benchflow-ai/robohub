---
schema_version: '1.3'
task:
  name: robouse/libero-goal-3
  description: Open the top drawer and put the bowl inside.
metadata:
  author_name: benchflow
  source_benchmark: LIBERO (Liu et al. 2023)
  source_task: open_the_top_drawer_and_put_the_bowl_inside
  suite: libero
  libero_suite: libero_goal
  libero_task_index: 3
  language_instruction: open the top drawer and put the bowl inside
  category: manipulation
  difficulty: medium
  tags:
  - franka
  - mujoco
  - robosuite
  - single-arm
  - language
  reference_solution: LIBERO demonstration demo_0 of yifengzhu-hf/LIBERO-datasets/libero_goal/open_the_top_drawer_and_put_the_bowl_inside_demo.hdf5 (replay reaches success at step 160)
  robouse:
    id: libero-goal-3
    backend: libero
    suite: libero_goal
    env: open_the_top_drawer_and_put_the_bowl_inside
    task_index: 3
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
    init_source: yifengzhu-hf/LIBERO-datasets/libero_goal/open_the_top_drawer_and_put_the_bowl_inside_demo.hdf5 demo_0 states[0]
    init_state:
    - 0.25000000000000017
    - -0.024405037260148754
    - -0.13702878438983238
    - -0.0008479156617000504
    - -2.402687084880886
    - -0.010292798906667469
    - 2.2525277895843785
    - 0.7851859465998293
    - 0.03404616639528421
    - -0.03406572658969437
    - -0.08676931736262916
    - 0.012451524750014783
    - 0.8981585151399347
    - 0.7071067776242912
    - -1.0724537191718901e-05
    - -1.4310617004714036e-06
    - 0.7071067846660275
    - -0.06517459466296836
    - 0.12071575747978859
    - 0.908880836284738
    - -6.358198043670517e-10
    - -6.263004995768119e-07
    - -2.104224548412041e-06
    - 0.9999999999975901
    - -0.19094664749768808
    - -0.05160372719105922
    - 0.8986907289178425
    - -6.994649915753735e-09
    - -6.204090740395158e-06
    - 3.7700941232842936e-05
    - 0.9999999992700742
    - 0.04565249133051329
    - -0.00961868522627699
    - 0.9023145976636144
    - 0.7071068944202988
    - -1.013777001383935e-05
    - 2.0046027443785608e-05
    - 0.7071066675959596
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.025941279043593935
    - 0.26860228172468065
    - 0.0006725925083450711
    - 0.2528526112699437
    - -0.11422128894636546
    - -0.07309683043928603
    - 0.0958439961859376
    - 0.057634070856155635
    - -0.05749715256561751
    - 1.414696485246313e-07
    - -3.4906643731386915e-08
    - 0.006561141189068618
    - -7.860505322545303e-05
    - 2.077506965455289e-05
    - -1.2876260701311856e-09
    - -2.009935516244079e-09
    - -1.3682347460739033e-08
    - 0.0006981745727182228
    - -1.6224821094967877e-06
    - 2.284331873454903e-07
    - 7.091916077559453e-11
    - 3.625042518167741e-08
    - -2.205037955572154e-07
    - 0.004560894627160446
    - 0.00017154492857826483
    - 2.8190244548360362e-05
    - 7.435971161676634e-10
    - 2.915644672404854e-07
    - 3.0526807631299163e-07
    - 0.004946092249603383
    - 0.0001234231066591139
    - 0.00012863603417489976
    - 5.2829646960778904e-09
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

# Open the top drawer and put the bowl inside

A Franka Panda arm with a parallel-jaw gripper works at a table in a LIBERO scene (MuJoCo / robosuite; LIBERO suite `libero_goal`, task `open_the_top_drawer_and_put_the_bowl_inside`). LIBERO-Goal: one fixed kitchen scene; the instruction sets the goal.

**Goal (LIBERO's language instruction):** open the top drawer and put the bowl inside.

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
