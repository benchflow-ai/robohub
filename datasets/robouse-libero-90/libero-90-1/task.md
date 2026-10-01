---
schema_version: '1.3'
task:
  name: robouse/libero-90-1
  description: Close the top drawer of the cabinet and put the black bowl on top of it.
metadata:
  author_name: benchflow
  source_benchmark: LIBERO (Liu et al. 2023)
  source_task: KITCHEN_SCENE10_close_the_top_drawer_of_the_cabinet_and_put_the_black_bowl_on_top_of_it
  suite: libero-90
  libero_suite: libero_90
  libero_task_index: 1
  language_instruction: close the top drawer of the cabinet and put the black bowl on top of it
  category: manipulation
  difficulty: medium
  tags:
  - franka
  - mujoco
  - robosuite
  - single-arm
  - language
  reference_solution: LIBERO demonstration demo_0 of yifengzhu-hf/LIBERO-datasets/libero_90/KITCHEN_SCENE10_close_the_top_drawer_of_the_cabinet_and_put_the_black_bowl_on_top_of_it_demo.hdf5 (replay reaches success at step 187)
  robouse:
    id: libero-90-1
    backend: libero
    suite: libero_90
    env: KITCHEN_SCENE10_close_the_top_drawer_of_the_cabinet_and_put_the_black_bowl_on_top_of_it
    task_index: 1
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
    init_source: yifengzhu-hf/LIBERO-datasets/libero_90/KITCHEN_SCENE10_close_the_top_drawer_of_the_cabinet_and_put_the_black_bowl_on_top_of_it_demo.hdf5 demo_0 states[0]
    init_state:
    - 0.25000000000000017
    - 0.017706237930857436
    - -0.16120469115816774
    - 0.009437511754511576
    - -2.457317001958437
    - -0.008978696177186636
    - 2.199025792285983
    - 0.7850933577623468
    - 0.03406048172577298
    - -0.03405132039018929
    - -0.10640376372941801
    - 0.00911966053142711
    - 0.8981585151399347
    - 0.7071067776242912
    - -1.072453719171846e-05
    - -1.4310617004708903e-06
    - 0.7071067846660275
    - 0.007906587110981115
    - 0.18687245400450894
    - 0.9086548762142962
    - -4.088883686657089e-10
    - -8.196927805367626e-07
    - -4.739534225496838e-06
    - 0.9999999999884325
    - -0.10957533873785376
    - 0.20976391828132668
    - 0.9086548762142962
    - -4.088883686620414e-10
    - -8.196927805343819e-07
    - -4.739534225493512e-06
    - 0.9999999999884325
    - 0.015716382220515794
    - 0.02545678051661995
    - 0.9129232581896339
    - 9.570210259669874e-09
    - -4.332321868470562e-07
    - 2.1414477365015527e-06
    - 0.9999999999976134
    - -0.15102388021060897
    - 0.0
    - 0.0
    - -0.009156217646220299
    - -0.02247767380291243
    - 0.023870357650472747
    - -0.01692818394631668
    - -5.2348764012129774e-05
    - -0.014716899501679178
    - -0.06951722284469525
    - 0.057521817201471474
    - -0.057608024817597386
    - 1.4146964852366335e-07
    - -3.490664372942795e-08
    - 0.006561141189068619
    - -7.860505322542049e-05
    - 2.0775069654478766e-05
    - -1.2876260699649773e-09
    - -5.728458374139007e-09
    - -4.1425499561110086e-08
    - 0.0006971084279367789
    - -5.214947459263266e-06
    - 6.757763810982988e-07
    - 3.8155037276050166e-10
    - -5.728458374153179e-09
    - -4.142549956229074e-08
    - 0.0006971084279367769
    - -5.214947459199228e-06
    - 6.75776381003852e-07
    - 3.8155037257228917e-10
    - -1.7911519963153906e-07
    - 9.463563512545925e-08
    - 0.0034187161502999214
    - 7.38368198039715e-06
    - 1.392023841307865e-05
    - 3.556957422510364e-10
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

# Close the top drawer of the cabinet and put the black bowl on top of it

A Franka Panda arm with a parallel-jaw gripper works at a table in a LIBERO scene (MuJoCo / robosuite; LIBERO suite `libero_90`, task `KITCHEN_SCENE10_close_the_top_drawer_of_the_cabinet_and_put_the_black_bowl_on_top_of_it`). LIBERO-90: 90 short-horizon tasks in 20 kitchen, living-room and study scenes (the task name starts with the scene); LIBERO uses them for pretraining.

**Goal (LIBERO's language instruction):** close the top drawer of the cabinet and put the black bowl on top of it.

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
