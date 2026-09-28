---
schema_version: '1.3'
task:
  name: robouse/libero-goal-4
  description: Put the bowl on top of the cabinet.
metadata:
  author_name: benchflow
  source_benchmark: LIBERO (Liu et al. 2023)
  source_task: put_the_bowl_on_top_of_the_cabinet
  suite: libero
  libero_suite: libero_goal
  libero_task_index: 4
  language_instruction: put the bowl on top of the cabinet
  category: manipulation
  difficulty: medium
  tags:
  - franka
  - mujoco
  - robosuite
  - single-arm
  - language
  reference_solution: LIBERO demonstration demo_0 of yifengzhu-hf/LIBERO-datasets/libero_goal/put_the_bowl_on_top_of_the_cabinet_demo.hdf5 (replay reaches success at step 81)
  robouse:
    id: libero-goal-4
    backend: libero
    suite: libero_goal
    env: put_the_bowl_on_top_of_the_cabinet
    task_index: 4
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
    init_source: yifengzhu-hf/LIBERO-datasets/libero_goal/put_the_bowl_on_top_of_the_cabinet_demo.hdf5 demo_0 states[0]
    init_state:
    - 0.25000000000000017
    - 0.01861177347535524
    - -0.16220580999545245
    - 0.04357253539958475
    - -2.4571043343349843
    - -0.005751408419520538
    - 2.2106438129056993
    - 0.805947266437644
    - 0.034076949187233294
    - -0.034032509969767234
    - -0.08102351605868957
    - 0.011407195892274428
    - 0.8981585151399347
    - 0.7071067776242912
    - -1.0724537191720991e-05
    - -1.4310617004712776e-06
    - 0.7071067846660275
    - -0.03218215037581079
    - 0.11579980450383488
    - 0.908880836284738
    - -6.358198043621913e-10
    - -6.263004995763831e-07
    - -2.1042245484129982e-06
    - 0.9999999999975901
    - -0.20764088072264386
    - -0.05557682663540382
    - 0.8986907289178425
    - -6.9946499157843796e-09
    - -6.204090740395215e-06
    - 3.770094123284271e-05
    - 0.9999999992700742
    - 0.049089951091151864
    - -0.02216395790074304
    - 0.9023145976636144
    - 0.7071068944202988
    - -1.013777001383855e-05
    - 2.004602744378038e-05
    - 0.7071066675959596
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.004604179290986471
    - 0.03369140070237331
    - 0.060160203231859666
    - 0.0017967048075690484
    - -0.038136791652995404
    - 0.034380617600813755
    - 0.11853201112979969
    - 0.05732966541197919
    - -0.05778150538222596
    - 1.414696485246201e-07
    - -3.490664373047879e-08
    - 0.006561141189068624
    - -7.860505322542238e-05
    - 2.077506965459695e-05
    - -1.2876260701273828e-09
    - -2.009935516237224e-09
    - -1.3682347460469718e-08
    - 0.0006981745727182173
    - -1.6224821095238208e-06
    - 2.2843318732276043e-07
    - 7.091916081736271e-11
    - 3.625042518207413e-08
    - -2.205037955584584e-07
    - 0.004560894627160446
    - 0.00017154492857831405
    - 2.819024454836672e-05
    - 7.435971164274792e-10
    - 2.915644672404097e-07
    - 3.052680763127613e-07
    - 0.00494609224960338
    - 0.00012342310665922537
    - 0.00012863603417477242
    - 5.282964696265796e-09
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

# Put the bowl on top of the cabinet

A Franka Panda arm with a parallel-jaw gripper works at a table in a LIBERO scene (MuJoCo / robosuite; LIBERO suite `libero_goal`, task `put_the_bowl_on_top_of_the_cabinet`). LIBERO-Goal: one fixed kitchen scene; the instruction sets the goal.

**Goal (LIBERO's language instruction):** put the bowl on top of the cabinet.

The episode ends as solved the moment LIBERO's own success check (the task's goal predicates) passes; you do not need to call `robo done` after that. If you finish without the check passing, call `robo done` (or `robo give-up`).

## This robot

- The action has 7 numbers, not 4: `robo act DX DY DZ DROLL DPITCH DYAW GRIPPER`, each in [-1, 1]. It is LIBERO's delta end-effector command (robosuite's OSC_POSE controller, 20 steps per second). Holding DX/DY/DZ at 1.0 moves the hand about 1 cm per step; DROLL/DPITCH/DYAW rotate the hand (1.0 asks for 0.5 rad; usually leave them at 0). GRIPPER -1 opens, +1 closes.
- `robo move-to X Y Z [--grip G]` and `robo grip G` are available (they keep the hand's orientation).
- World frame, metres: +x points forward, away from the robot's base; +y is to the robot's left; +z is up. The table surface is at about z = 0.90.
- `robo observe` fields: `hand_pos` (end-effector position), `eef_quat` (orientation quaternion x, y, z, w) and `eef_axis_angle`, `gripper_qpos` (the two finger joints), `gripper_open` (finger gap in metres, about 0.08 when fully open), and `objects`: the position and quaternion of every object in the scene, under LIBERO's names (e.g. `akita_black_bowl_1`, `plate_1`; `..._1`, `..._2` number identical objects).
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
