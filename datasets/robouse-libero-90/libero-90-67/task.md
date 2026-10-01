---
schema_version: '1.3'
task:
  name: robouse/libero-90-67
  description: Put the white mug on the left plate.
metadata:
  author_name: benchflow
  source_benchmark: LIBERO (Liu et al. 2023)
  source_task: LIVING_ROOM_SCENE5_put_the_white_mug_on_the_left_plate
  suite: libero-90
  libero_suite: libero_90
  libero_task_index: 67
  language_instruction: put the white mug on the left plate
  category: manipulation
  difficulty: medium
  tags:
  - franka
  - mujoco
  - robosuite
  - single-arm
  - language
  reference_solution: LIBERO demonstration demo_0 of yifengzhu-hf/LIBERO-datasets/libero_90/LIVING_ROOM_SCENE5_put_the_white_mug_on_the_left_plate_demo.hdf5 (replay reaches success at step 77)
  robouse:
    id: libero-90-67
    backend: libero
    suite: libero_90
    env: LIVING_ROOM_SCENE5_put_the_white_mug_on_the_left_plate
    task_index: 67
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
    init_source: yifengzhu-hf/LIBERO-datasets/libero_90/LIVING_ROOM_SCENE5_put_the_white_mug_on_the_left_plate_demo.hdf5 demo_0 states[0]
    init_state:
    - 0.25000000000000017
    - -0.008997270577406204
    - -0.1665761870772708
    - 0.014226302303868926
    - -2.459223856332544
    - -0.026101499390385592
    - 2.2219002467802778
    - 0.744607541083131
    - 0.03392305583478103
    - -0.03411512092144885
    - -0.11040495986375347
    - -0.1560950385422387
    - 0.4343699488393418
    - -0.7071069013731585
    - 1.017276326239446e-07
    - 6.67573136793773e-08
    - 0.7071066609999057
    - -0.19889041411325842
    - 0.015513663447847137
    - 0.4385989872647111
    - -0.7071068579899178
    - -5.528613175357645e-07
    - 3.831199740146775e-07
    - 0.707106704382849
    - -0.06622415249088912
    - 0.0998728252629402
    - 0.43685696889004044
    - -0.7071067450257308
    - -9.863721141907176e-07
    - 5.538107391833171e-08
    - 0.7071068173466724
    - 0.008747390711216335
    - -0.28761349911152034
    - 0.4392727869250624
    - 0.7071068416649496
    - 1.2253611784980608e-06
    - -6.586728017143114e-07
    - 0.7071067207067718
    - 0.00491951217652663
    - 0.3057648481863407
    - 0.4392727869250624
    - 0.7071068416649496
    - 1.2253611784957827e-06
    - -6.586728016971608e-07
    - 0.7071067207067718
    - -0.04343015550433128
    - 0.019298198650662315
    - -0.21080477137182604
    - -0.004470932064125954
    - -0.06427391355505738
    - 0.015343806047903986
    - -0.17804233554878074
    - 0.05769318846842347
    - -0.057276839226938124
    - -7.114629580342985e-11
    - -4.5271502098841136e-10
    - 1.136809306584407e-06
    - -2.8896237791978458e-08
    - -1.8466801299426187e-07
    - -1.0830539098710901e-12
    - 8.910693014268639e-12
    - -1.876552749057363e-10
    - 1.843048112256913e-06
    - -4.917207466867071e-09
    - 1.0451616069864199e-07
    - -2.772227386033205e-13
    - 6.847124387077431e-13
    - -7.276784032275253e-12
    - 2.298245667203742e-06
    - -1.7375212981299124e-08
    - 1.291237283557019e-07
    - -3.2601190039341236e-13
    - 1.8627973200514592e-10
    - 1.1309283951978703e-11
    - 2.764538359678707e-06
    - 7.554087119535871e-08
    - 4.563006006665779e-09
    - 1.7231480958199685e-12
    - 1.8627973313780177e-10
    - 1.130928190632891e-11
    - 2.764538359674953e-06
    - 7.554087105286785e-08
    - 4.56300627417942e-09
    - 1.723147421066888e-12
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

# Put the white mug on the left plate

A Franka Panda arm with a parallel-jaw gripper works at a low table in a LIBERO scene (MuJoCo / robosuite; LIBERO suite `libero_90`, task `LIVING_ROOM_SCENE5_put_the_white_mug_on_the_left_plate`). LIBERO-90: 90 short-horizon tasks in 20 kitchen, living-room and study scenes (the task name starts with the scene); LIBERO uses them for pretraining.

**Goal (LIBERO's language instruction):** put the white mug on the left plate.

The episode ends as solved the moment LIBERO's own success check (the task's goal predicates) passes; you do not need to call `robo done` after that. If you finish without the check passing, call `robo done` (or `robo give-up`).

## This robot

- The action has 7 numbers, not 4: `robo act DX DY DZ DROLL DPITCH DYAW GRIPPER`, each in [-1, 1]. It is LIBERO's delta end-effector command (robosuite's OSC_POSE controller, 20 steps per second). Holding DX/DY/DZ at 1.0 moves the hand about 1 cm per step; DROLL/DPITCH/DYAW rotate the hand (1.0 asks for 0.5 rad; usually leave them at 0). GRIPPER -1 opens, +1 closes.
- `robo move-to X Y Z [--grip G]` and `robo grip G` are available (they keep the hand's orientation).
- World frame, metres: +x points forward, away from the robot's base; +y is to the robot's left; +z is up. The table surface is at about z = 0.43.
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
