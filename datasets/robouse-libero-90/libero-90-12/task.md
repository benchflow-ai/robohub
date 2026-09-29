---
schema_version: '1.3'
task:
  name: robouse/libero-90-12
  description: Put the black bowl at the back on the plate.
metadata:
  author_name: benchflow
  source_benchmark: LIBERO (Liu et al. 2023)
  source_task: KITCHEN_SCENE2_put_the_black_bowl_at_the_back_on_the_plate
  suite: libero-90
  libero_suite: libero_90
  libero_task_index: 12
  language_instruction: put the black bowl at the back on the plate
  category: manipulation
  difficulty: medium
  tags:
  - franka
  - mujoco
  - robosuite
  - single-arm
  - language
  reference_solution: LIBERO demonstration demo_0 of yifengzhu-hf/LIBERO-datasets/libero_90/KITCHEN_SCENE2_put_the_black_bowl_at_the_back_on_the_plate_demo.hdf5 (replay reaches success at step 87)
  robouse:
    id: libero-90-12
    backend: libero
    suite: libero_90
    env: KITCHEN_SCENE2_put_the_black_bowl_at_the_back_on_the_plate
    task_index: 12
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
    init_source: yifengzhu-hf/LIBERO-datasets/libero_90/KITCHEN_SCENE2_put_the_black_bowl_at_the_back_on_the_plate_demo.hdf5 demo_0 states[0]
    init_state:
    - 0.25000000000000017
    - -0.007085030952800984
    - -0.17927097086515895
    - 0.004373229135942012
    - -2.457345187734109
    - 0.012006720906810719
    - 2.2293149143777593
    - 0.786166462219797
    - 0.03408329692186354
    - -0.03402749649392035
    - 0.0928763825183225
    - 0.1642530449152202
    - 0.8981585151399347
    - 0.7071067776242912
    - -1.0724537191719018e-05
    - -1.4310617004701888e-06
    - 0.7071067846660275
    - -0.07029182406009392
    - 0.20454308970725815
    - 0.8981585151399347
    - 0.7071067776242912
    - -1.0724537191718689e-05
    - -1.4310617004717952e-06
    - 0.7071067846660275
    - -0.17288572457002832
    - 0.05234652513651784
    - 0.8981585151399347
    - 0.7071067776242912
    - -1.0724537191720373e-05
    - -1.4310617004719635e-06
    - 0.7071067846660275
    - 0.0022248017872249404
    - -0.002598038293600276
    - 0.9023145976636144
    - 0.7071068944202988
    - -1.0137770013849125e-05
    - 2.0046027443775348e-05
    - 0.7071066675959596
    - 0.0
    - 0.0
    - 0.0
    - 0.01988652707429315
    - 0.025467106088772475
    - 0.04262498372947191
    - -0.07875101753059136
    - -0.07954504099283392
    - -0.007413933692009347
    - 0.01941535871792061
    - 0.057413375534101356
    - -0.057702967241689954
    - 1.4146964852362261e-07
    - -3.4906643731341956e-08
    - 0.00656114118906862
    - -7.860505322541458e-05
    - 2.0775069654544316e-05
    - -1.2876260699801426e-09
    - 1.4146964852387013e-07
    - -3.490664373032364e-08
    - 0.00656114118906862
    - -7.8605053225425e-05
    - 2.0775069654510817e-05
    - -1.2876260700061458e-09
    - 1.4146964852609198e-07
    - -3.490664372971795e-08
    - 0.006561141189068612
    - -7.860505322550577e-05
    - 2.077506965448945e-05
    - -1.2876260703402197e-09
    - 2.9156446724008297e-07
    - 3.052680763125226e-07
    - 0.004946092249603383
    - 0.00012342310665880692
    - 0.00012863603417459561
    - 5.282964695872725e-09
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

# Put the black bowl at the back on the plate

A Franka Panda arm with a parallel-jaw gripper works at a table in a LIBERO scene (MuJoCo / robosuite; LIBERO suite `libero_90`, task `KITCHEN_SCENE2_put_the_black_bowl_at_the_back_on_the_plate`). LIBERO-90: 90 short-horizon tasks in 20 kitchen, living-room and study scenes (the task name starts with the scene); LIBERO uses them for pretraining.

**Goal (LIBERO's language instruction):** put the black bowl at the back on the plate.

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
