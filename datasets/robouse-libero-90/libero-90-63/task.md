---
schema_version: '1.3'
task:
  name: robouse/libero-90-63
  description: Stack the left bowl on the right bowl and place them in the tray.
metadata:
  author_name: benchflow
  source_benchmark: LIBERO (Liu et al. 2023)
  source_task: LIVING_ROOM_SCENE4_stack_the_left_bowl_on_the_right_bowl_and_place_them_in_the_tray
  suite: libero-90
  libero_suite: libero_90
  libero_task_index: 63
  language_instruction: stack the left bowl on the right bowl and place them in the tray
  category: manipulation
  difficulty: medium
  tags:
  - franka
  - mujoco
  - robosuite
  - single-arm
  - language
  reference_solution: LIBERO demonstration demo_0 of yifengzhu-hf/LIBERO-datasets/libero_90/LIVING_ROOM_SCENE4_stack_the_left_bowl_on_the_right_bowl_and_place_them_in_the_tray_demo.hdf5 (replay reaches success at step 187)
  robouse:
    id: libero-90-63
    backend: libero
    suite: libero_90
    env: LIVING_ROOM_SCENE4_stack_the_left_bowl_on_the_right_bowl_and_place_them_in_the_tray
    task_index: 63
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
    init_source: yifengzhu-hf/LIBERO-datasets/libero_90/LIVING_ROOM_SCENE4_stack_the_left_bowl_on_the_right_bowl_and_place_them_in_the_tray_demo.hdf5 demo_0 states[0]
    init_state:
    - 0.25000000000000017
    - 0.01527680793806062
    - -0.13271063999877505
    - -0.0433219525576525
    - -2.447389823518829
    - 0.008258020058673135
    - 2.226298667253129
    - 0.7784943493566027
    - 0.03401411004664741
    - -0.034078969994066684
    - -0.09823409350369676
    - -0.16365933781252498
    - 0.4351699275556287
    - 0.7071067791707762
    - -6.826208510655842e-08
    - -1.2183615572759937e-08
    - 0.7071067832023155
    - -0.09142515513277338
    - 0.07153261476207576
    - 0.4351699275556287
    - 0.7071067791707762
    - -6.82620851055102e-08
    - -1.2183615574812132e-08
    - 0.7071067832023155
    - -0.2574938799701522
    - -0.09929516355843368
    - 0.4797074607342248
    - 0.5000000162947377
    - 0.49999998370524373
    - 0.5000000024348765
    - 0.49999999756514146
    - 0.1171835961821278
    - -0.2195117000838723
    - 0.4498398389808574
    - 5.028867884105122e-09
    - -3.782457004191979e-09
    - 1.2586724035043727e-08
    - 1.0
    - 0.005060321034001996
    - 0.25738211493916574
    - 0.43738012050757274
    - -7.451456348804086e-06
    - -0.0017326379326048306
    - 0.0012644815872153235
    - 0.9999976994956469
    - -0.05896171217654339
    - 0.11902975589056436
    - -0.0792461022153215
    - 0.10689331311766147
    - 0.025951956104532395
    - -0.026027240070672978
    - -0.040756321965188046
    - 0.05773601558224425
    - -0.05729689050145038
    - 1.1457031639058558e-11
    - 6.364948954593763e-11
    - 2.922013250571395e-06
    - -6.612401792573866e-09
    - -3.666961860507721e-08
    - -9.683484432393912e-14
    - 1.1457031426378369e-11
    - 6.364948985854639e-11
    - 2.922013250571356e-06
    - -6.612401784797171e-09
    - -3.6669618616433595e-08
    - -9.683480688635077e-14
    - -1.5755547242188324e-09
    - 1.00198663435447e-09
    - -1.2043941904921022e-09
    - -2.5948728590493864e-08
    - -4.011292214905959e-13
    - -1.6499847041694177e-08
    - -6.061720490092739e-11
    - -1.4785294530977306e-11
    - 2.895975395989454e-07
    - -1.1621830944150783e-09
    - 4.685771854634609e-09
    - 1.2428069374207241e-13
    - 6.9159029485807824e-09
    - 3.5146092012007237e-09
    - 1.9577347505396633e-06
    - 5.701211084366628e-06
    - -1.1170259724895153e-05
    - 4.800815306478237e-08
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

# Stack the left bowl on the right bowl and place them in the tray

A Franka Panda arm with a parallel-jaw gripper works at a low table in a LIBERO scene (MuJoCo / robosuite; LIBERO suite `libero_90`, task `LIVING_ROOM_SCENE4_stack_the_left_bowl_on_the_right_bowl_and_place_them_in_the_tray`). LIBERO-90: 90 short-horizon tasks in 20 kitchen, living-room and study scenes (the task name starts with the scene); LIBERO uses them for pretraining.

**Goal (LIBERO's language instruction):** stack the left bowl on the right bowl and place them in the tray.

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
