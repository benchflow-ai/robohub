---
schema_version: '1.3'
task:
  name: robouse/libero-90-0
  description: Close the top drawer of the cabinet.
metadata:
  author_name: benchflow
  source_benchmark: LIBERO (Liu et al. 2023)
  source_task: KITCHEN_SCENE10_close_the_top_drawer_of_the_cabinet
  suite: libero-90
  libero_suite: libero_90
  libero_task_index: 0
  language_instruction: close the top drawer of the cabinet
  category: manipulation
  difficulty: medium
  tags:
  - franka
  - mujoco
  - robosuite
  - single-arm
  - language
  reference_solution: LIBERO demonstration demo_0 of yifengzhu-hf/LIBERO-datasets/libero_90/KITCHEN_SCENE10_close_the_top_drawer_of_the_cabinet_demo.hdf5 (replay reaches success at step 59)
  robouse:
    id: libero-90-0
    backend: libero
    suite: libero_90
    env: KITCHEN_SCENE10_close_the_top_drawer_of_the_cabinet
    task_index: 0
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
    init_source: yifengzhu-hf/LIBERO-datasets/libero_90/KITCHEN_SCENE10_close_the_top_drawer_of_the_cabinet_demo.hdf5 demo_0 states[0]
    init_state:
    - 0.25000000000000017
    - 0.0069742010048686135
    - -0.16917112481992663
    - -0.010507486792187954
    - -2.4166129022319027
    - 0.011713560596895742
    - 2.2138947899331933
    - 0.7897656539646127
    - 0.03406964673556971
    - -0.03404273177971899
    - -0.10444386672550951
    - 0.003005636195647636
    - 0.8981585151399347
    - 0.7071067776242912
    - -1.0724537191720104e-05
    - -1.4310617004709202e-06
    - 0.7071067846660275
    - 0.003284507087261328
    - 0.18685184994838644
    - 0.9086548762142962
    - -4.088883686637598e-10
    - -8.196927805403148e-07
    - -4.73953422549423e-06
    - 0.9999999999884325
    - -0.10291965505376711
    - 0.20344974342157707
    - 0.9086548762142962
    - -4.0888836866404913e-10
    - -8.196927805340942e-07
    - -4.739534225494213e-06
    - 0.9999999999884325
    - -0.006714311061582776
    - 0.03626014363380154
    - 0.9129232581896339
    - 9.57021025968358e-09
    - -4.332321868522383e-07
    - 2.141447736499979e-06
    - 0.9999999999976134
    - -0.14248104142442267
    - 0.0
    - 0.0
    - -0.00509941720972637
    - 0.0011030405301713936
    - 0.010520035934694756
    - 0.04446710692113364
    - 0.003126739701084362
    - -0.02434742710795308
    - 0.005042177687212161
    - 0.05760616804033308
    - -0.057518270300183756
    - 1.4146964852425665e-07
    - -3.4906643731254116e-08
    - 0.00656114118906862
    - -7.860505322544006e-05
    - 2.0775069654548233e-05
    - -1.2876260700743576e-09
    - -5.728458374122735e-09
    - -4.142549956196076e-08
    - 0.0006971084279367765
    - -5.214947459177883e-06
    - 6.757763811994059e-07
    - 3.8155037263452567e-10
    - -5.728458374166221e-09
    - -4.1425499561952235e-08
    - 0.0006971084279367782
    - -5.2149474591800455e-06
    - 6.7577638113174e-07
    - 3.81550372611545e-10
    - -1.7911519963154052e-07
    - 9.463563512648072e-08
    - 0.0034187161502999184
    - 7.383681980388706e-06
    - 1.3920238413016186e-05
    - 3.5569574204649314e-10
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

# Close the top drawer of the cabinet

A Franka Panda arm with a parallel-jaw gripper works at a table in a LIBERO scene (MuJoCo / robosuite; LIBERO suite `libero_90`, task `KITCHEN_SCENE10_close_the_top_drawer_of_the_cabinet`). LIBERO-90: 90 short-horizon tasks in 20 kitchen, living-room and study scenes (the task name starts with the scene); LIBERO uses them for pretraining.

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
