---
schema_version: '1.3'
task:
  name: robouse/libero-90-49
  description: Pick up the tomato sauce and put it in the basket.
metadata:
  author_name: benchflow
  source_benchmark: LIBERO (Liu et al. 2023)
  source_task: LIVING_ROOM_SCENE1_pick_up_the_tomato_sauce_and_put_it_in_the_basket
  suite: libero-90
  libero_suite: libero_90
  libero_task_index: 49
  language_instruction: pick up the tomato sauce and put it in the basket
  category: manipulation
  difficulty: medium
  tags:
  - franka
  - mujoco
  - robosuite
  - single-arm
  - language
  reference_solution: LIBERO demonstration demo_0 of yifengzhu-hf/LIBERO-datasets/libero_90/LIVING_ROOM_SCENE1_pick_up_the_tomato_sauce_and_put_it_in_the_basket_demo.hdf5 (replay reaches success at step 151)
  robouse:
    id: libero-90-49
    backend: libero
    suite: libero_90
    env: LIVING_ROOM_SCENE1_pick_up_the_tomato_sauce_and_put_it_in_the_basket
    task_index: 49
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
    init_source: yifengzhu-hf/LIBERO-datasets/libero_90/LIVING_ROOM_SCENE1_pick_up_the_tomato_sauce_and_put_it_in_the_basket_demo.hdf5 demo_0 states[0]
    init_state:
    - 0.25000000000000017
    - 0.033995750029624426
    - -0.15708931136113066
    - -0.007262622081839966
    - -2.450106433787366
    - -0.005024153311639033
    - 2.231577209774904
    - 0.805810304785771
    - 0.03405334159355227
    - -0.03405853286530976
    - 0.04552311605641752
    - -0.08958624421899716
    - 0.4751648626212499
    - -0.0024825034538958207
    - 0.0021580325352464937
    - 0.7074504440013328
    - 0.706755296658253
    - -0.128534670662006
    - 0.07988463811629738
    - 0.44569584300064613
    - -3.488291916307461e-17
    - -3.419913640818801e-09
    - -1.1564254660659067e-08
    - 1.0
    - 0.10567824275816073
    - -0.19395107241807602
    - 0.47516914741370325
    - -0.002329993130175374
    - 0.002344732969113113
    - 0.7074532159718667
    - 0.7067524464551789
    - -0.1885430488682957
    - -0.16512734487661826
    - 0.6312454499278823
    - 0.5000266167646802
    - 0.4999734008287584
    - 0.49994359102672187
    - 0.5000563836027885
    - -0.0003106240192837273
    - 0.26971579145426566
    - 0.4321919610888971
    - 0.7071047490794425
    - -0.0016949793370027767
    - 0.001694977984283883
    - 0.7071047503192003
    - -0.03972693099951623
    - -0.0033683846997729344
    - 0.021835162070197343
    - -0.01228847932088926
    - 0.054063851224701476
    - 0.031157478073953613
    - 0.01594373502282738
    - 0.05754965865154277
    - -0.05758137866330776
    - 0.0002173125621119972
    - 4.0894124369242795e-05
    - 0.0003760446939785026
    - 0.0010661410261221882
    - -1.3522369246832276e-06
    - 0.00567969938597882
    - -7.359386775482846e-16
    - -1.6365166602438768e-15
    - 2.8103504382703122e-09
    - -2.091540563517442e-13
    - 8.666427146392476e-14
    - 2.752232963206462e-17
    - 1.1674503786934546e-08
    - 5.5494677680336615e-09
    - 1.8531305092589585e-08
    - 1.4502740691957378e-07
    - -6.511204592562624e-10
    - 3.0507396635578804e-07
    - -5.8334018122948306e-05
    - -1.5534314233552885e-05
    - -0.6124323020550405
    - -0.0003036949667750183
    - -5.072348092423958e-07
    - 0.00011628228721838552
    - -9.774292801458637e-13
    - -7.846063078893353e-09
    - 4.680392257396672e-06
    - 2.1184007946813867e-10
    - 1.7156703158152552e-06
    - 1.5481247674311912e-12
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

# Pick up the tomato sauce and put it in the basket

A Franka Panda arm with a parallel-jaw gripper works at a low table in a LIBERO scene (MuJoCo / robosuite; LIBERO suite `libero_90`, task `LIVING_ROOM_SCENE1_pick_up_the_tomato_sauce_and_put_it_in_the_basket`). LIBERO-90: 90 short-horizon tasks in 20 kitchen, living-room and study scenes (the task name starts with the scene); LIBERO uses them for pretraining.

**Goal (LIBERO's language instruction):** pick up the tomato sauce and put it in the basket.

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
