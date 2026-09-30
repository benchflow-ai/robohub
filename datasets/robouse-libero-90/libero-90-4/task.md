---
schema_version: '1.3'
task:
  name: robouse/libero-90-4
  description: Put the butter at the front in the top drawer of the cabinet and close it.
metadata:
  author_name: benchflow
  source_benchmark: LIBERO (Liu et al. 2023)
  source_task: KITCHEN_SCENE10_put_the_butter_at_the_front_in_the_top_drawer_of_the_cabinet_and_close_it
  suite: libero-90
  libero_suite: libero_90
  libero_task_index: 4
  language_instruction: put the butter at the front in the top drawer of the cabinet and close it
  category: manipulation
  difficulty: medium
  tags:
  - franka
  - mujoco
  - robosuite
  - single-arm
  - language
  reference_solution: LIBERO demonstration demo_0 of yifengzhu-hf/LIBERO-datasets/libero_90/KITCHEN_SCENE10_put_the_butter_at_the_front_in_the_top_drawer_of_the_cabinet_and_close_it_demo.hdf5 (replay reaches success at step 162)
  robouse:
    id: libero-90-4
    backend: libero
    suite: libero_90
    env: KITCHEN_SCENE10_put_the_butter_at_the_front_in_the_top_drawer_of_the_cabinet_and_close_it
    task_index: 4
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
    init_source: yifengzhu-hf/LIBERO-datasets/libero_90/KITCHEN_SCENE10_put_the_butter_at_the_front_in_the_top_drawer_of_the_cabinet_and_close_it_demo.hdf5 demo_0 states[0]
    init_state:
    - 0.25000000000000017
    - 0.02407770131960027
    - -0.16837228637822158
    - -0.011371397168303933
    - -2.43470779473215
    - 0.015705674822343068
    - 2.2605904743370857
    - 0.7803675731174428
    - 0.03409177852329121
    - -0.03401547113178257
    - -0.10204415579360501
    - 0.014991464655605758
    - 0.8981585151399347
    - 0.7071067776242912
    - -1.0724537191719252e-05
    - -1.4310617004716133e-06
    - 0.7071067846660275
    - -0.0062224202353239715
    - 0.21556374002802328
    - 0.9086548762142962
    - -4.0888836866108715e-10
    - -8.196927805321229e-07
    - -4.739534225495043e-06
    - 0.9999999999884325
    - -0.0854949224426073
    - 0.19222768057948225
    - 0.9086548762142962
    - -4.088883686769168e-10
    - -8.196927805351765e-07
    - -4.739534225497387e-06
    - 0.9999999999884325
    - 0.01640964947921182
    - 0.060368586537878965
    - 0.9129232581896339
    - 9.57021025969767e-09
    - -4.332321868519263e-07
    - 2.14144773650096e-06
    - 0.9999999999976134
    - -0.14650242137344557
    - 0.0
    - 0.0
    - -0.011800268895750492
    - 0.03168371541042834
    - 0.07526574431846483
    - 0.028305869010496275
    - 0.0018973845110839406
    - 0.008007457948921316
    - 0.06042111641023768
    - 0.05739180949476835
    - -0.05771571257163661
    - 1.414696485259592e-07
    - -3.4906643730188655e-08
    - 0.006561141189068611
    - -7.860505322550465e-05
    - 2.077506965451262e-05
    - -1.287626070322757e-09
    - -5.728458374140766e-09
    - -4.1425499561703466e-08
    - 0.000697108427936779
    - -5.21494745920749e-06
    - 6.757763812141825e-07
    - 3.81550372674853e-10
    - -5.7284583741596285e-09
    - -4.1425499560607954e-08
    - 0.0006971084279367773
    - -5.214947458934913e-06
    - 6.757763810877552e-07
    - 3.815503725607999e-10
    - -1.7911519963157427e-07
    - 9.463563512587979e-08
    - 0.0034187161502999236
    - 7.383681980283306e-06
    - 1.392023841291846e-05
    - 3.5569574176692393e-10
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

# Put the butter at the front in the top drawer of the cabinet and close it

A Franka Panda arm with a parallel-jaw gripper works at a table in a LIBERO scene (MuJoCo / robosuite; LIBERO suite `libero_90`, task `KITCHEN_SCENE10_put_the_butter_at_the_front_in_the_top_drawer_of_the_cabinet_and_close_it`). LIBERO-90: 90 short-horizon tasks in 20 kitchen, living-room and study scenes (the task name starts with the scene); LIBERO uses them for pretraining.

**Goal (LIBERO's language instruction):** put the butter at the front in the top drawer of the cabinet and close it.

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
