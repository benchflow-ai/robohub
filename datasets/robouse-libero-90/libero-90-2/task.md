---
schema_version: '1.3'
task:
  name: robouse/libero-90-2
  description: Put the black bowl in the top drawer of the cabinet.
metadata:
  author_name: benchflow
  source_benchmark: LIBERO (Liu et al. 2023)
  source_task: KITCHEN_SCENE10_put_the_black_bowl_in_the_top_drawer_of_the_cabinet
  suite: libero-90
  libero_suite: libero_90
  libero_task_index: 2
  language_instruction: put the black bowl in the top drawer of the cabinet
  category: manipulation
  difficulty: medium
  tags:
  - franka
  - mujoco
  - robosuite
  - single-arm
  - language
  reference_solution: LIBERO demonstration demo_0 of yifengzhu-hf/LIBERO-datasets/libero_90/KITCHEN_SCENE10_put_the_black_bowl_in_the_top_drawer_of_the_cabinet_demo.hdf5 (replay reaches success at step 105)
  robouse:
    id: libero-90-2
    backend: libero
    suite: libero_90
    env: KITCHEN_SCENE10_put_the_black_bowl_in_the_top_drawer_of_the_cabinet
    task_index: 2
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
    init_source: yifengzhu-hf/LIBERO-datasets/libero_90/KITCHEN_SCENE10_put_the_black_bowl_in_the_top_drawer_of_the_cabinet_demo.hdf5 demo_0 states[0]
    init_state:
    - 0.25000000000000017
    - 0.001917619786907908
    - -0.14621830096504773
    - 0.036964394056270965
    - -2.425434259093881
    - -0.008403609144877366
    - 2.2342370588090117
    - 0.7934297698607489
    - 0.03409670055933724
    - -0.03400418516056008
    - -0.10871136019657951
    - 0.005306450696057914
    - 0.8981585151399347
    - 0.7071067776242912
    - -1.0724537191719943e-05
    - -1.4310617004713318e-06
    - 0.7071067846660275
    - -0.013112171543664088
    - 0.19920385452505016
    - 0.9086548762142962
    - -4.0888836866302063e-10
    - -8.196927805376553e-07
    - -4.739534225495691e-06
    - 0.9999999999884325
    - -0.09830019035724909
    - 0.21317767425483825
    - 0.9086548762142962
    - -4.088883686674369e-10
    - -8.196927805364078e-07
    - -4.7395342254940346e-06
    - 0.9999999999884325
    - 0.019370392297886327
    - 0.06587403201132122
    - 0.9129232581896339
    - 9.57021025970714e-09
    - -4.33232186849634e-07
    - 2.1414477364997404e-06
    - 0.9999999999976134
    - -0.1583881776360222
    - 0.0
    - 0.0
    - 0.033570650150149844
    - 0.24677522037161873
    - 0.11871150805972744
    - 0.2315019633697034
    - -0.15156299464153025
    - -0.03312089424854298
    - 0.2432264330418418
    - 0.05724789227792516
    - -0.05777355352927992
    - 1.414696485248458e-07
    - -3.4906643730609074e-08
    - 0.0065611411890686136
    - -7.86050532254274e-05
    - 2.077506965459525e-05
    - -1.2876260701652532e-09
    - -5.7284583741423405e-09
    - -4.142549956140709e-08
    - 0.000697108427936781
    - -5.2149474590511914e-06
    - 6.757763808665872e-07
    - 3.8155037251158037e-10
    - -5.728458374147098e-09
    - -4.1425499561982484e-08
    - 0.0006971084279367763
    - -5.214947459242619e-06
    - 6.757763810448194e-07
    - 3.8155037262639536e-10
    - -1.7911519963166453e-07
    - 9.463563512653518e-08
    - 0.003418716150299918
    - 7.383681980240284e-06
    - 1.3920238412925665e-05
    - 3.55695741712029e-10
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

# Put the black bowl in the top drawer of the cabinet

A Franka Panda arm with a parallel-jaw gripper works at a table in a LIBERO scene (MuJoCo / robosuite; LIBERO suite `libero_90`, task `KITCHEN_SCENE10_put_the_black_bowl_in_the_top_drawer_of_the_cabinet`). LIBERO-90: 90 short-horizon tasks in 20 kitchen, living-room and study scenes (the task name starts with the scene); LIBERO uses them for pretraining.

**Goal (LIBERO's language instruction):** put the black bowl in the top drawer of the cabinet.

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
