---
schema_version: '1.3'
task:
  name: robouse/libero-90-54
  description: Pick up the tomato sauce and put it in the basket.
metadata:
  author_name: benchflow
  source_benchmark: LIBERO (Liu et al. 2023)
  source_task: LIVING_ROOM_SCENE2_pick_up_the_tomato_sauce_and_put_it_in_the_basket
  suite: libero-90
  libero_suite: libero_90
  libero_task_index: 54
  language_instruction: pick up the tomato sauce and put it in the basket
  category: manipulation
  difficulty: medium
  tags:
  - franka
  - mujoco
  - robosuite
  - single-arm
  - language
  reference_solution: LIBERO demonstration demo_0 of yifengzhu-hf/LIBERO-datasets/libero_90/LIVING_ROOM_SCENE2_pick_up_the_tomato_sauce_and_put_it_in_the_basket_demo.hdf5 (replay reaches success at step 93)
  robouse:
    id: libero-90-54
    backend: libero
    suite: libero_90
    env: LIVING_ROOM_SCENE2_pick_up_the_tomato_sauce_and_put_it_in_the_basket
    task_index: 54
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
    init_source: yifengzhu-hf/LIBERO-datasets/libero_90/LIVING_ROOM_SCENE2_pick_up_the_tomato_sauce_and_put_it_in_the_basket_demo.hdf5 demo_0 states[0]
    init_state:
    - 0.25000000000000017
    - -0.026907517902646894
    - -0.14067442779577125
    - 0.010681149150689897
    - -2.4643111992290265
    - 0.019670858667598313
    - 2.2357455575586016
    - 0.7907858668992576
    - 0.0340646092175925
    - -0.03404754052742762
    - -0.07780833614693364
    - -0.15050641833524492
    - 0.4751648626212499
    - -0.002482503453895693
    - 0.002158032535246585
    - 0.7074504440013327
    - 0.7067552966582531
    - 0.1153542627144459
    - -0.17582027243876383
    - 0.44569584300064613
    - -3.488181013317382e-17
    - -3.4199136420290713e-09
    - -1.1564254660000464e-08
    - 1.0
    - -0.10862656387263536
    - 0.02747529130530627
    - 0.47516914741370325
    - -0.0023299931301753937
    - 0.0023447329691130715
    - 0.7074532159718667
    - 0.7067524464551789
    - -0.2602235896780006
    - -0.12773883789052665
    - 0.6312454499278823
    - 0.5000266167646797
    - 0.49997340082875863
    - 0.49994359102672203
    - 0.5000563836027886
    - -0.012374378883763403
    - -0.26361518371467585
    - 0.5897167696579936
    - 0.5000106862844934
    - 0.49998930857605123
    - 0.5000124215800898
    - 0.4999875830223854
    - 0.05225578734697916
    - -0.10587621221496935
    - 0.5897144010070282
    - 0.4999838508126262
    - 0.5000162063786704
    - 0.49998762299598365
    - 0.5000123189843294
    - 0.0650715128687693
    - 0.04991394925407715
    - 0.44546984280974117
    - 3.3358388003145537e-10
    - -4.5125860314504715e-09
    - -2.6176911301684368e-08
    - 0.9999999999999998
    - 0.007661723151087582
    - 0.2605398642684736
    - 0.4321919610888971
    - 0.7071047490794425
    - -0.0016949793370027767
    - 0.0016949779842838826
    - 0.7071047503192003
    - 0.029403136051427487
    - -3.9669466003142395e-05
    - -0.028828810103679093
    - 2.662073836074009e-05
    - -0.002979022595046399
    - 5.6347937835938216e-05
    - 0.001376608560992658
    - 0.05760923112746718
    - -0.05751840320067524
    - 0.00021731256211159446
    - 4.089412436998246e-05
    - 0.00037604469398129395
    - 0.0010661410261413298
    - -1.352236924824682e-06
    - 0.005679699385967407
    - -7.359417715484613e-16
    - -1.6371836828229226e-15
    - 2.8103504421614835e-09
    - -2.090989472490251e-13
    - 8.663322454438789e-14
    - 2.7355843703478893e-17
    - 1.1674503787392647e-08
    - 5.549467763914079e-09
    - 1.8531305089152822e-08
    - 1.4502740682156053e-07
    - -6.511204587181268e-10
    - 3.050739663170025e-07
    - -5.833401812268594e-05
    - -1.5534314233449412e-05
    - -0.6124323020550405
    - -0.00030369496677415316
    - -5.07234809243946e-07
    - 0.00011628228721873158
    - -7.034107957834145e-06
    - 5.286007836457793e-07
    - -0.7779263010058409
    - 8.180346963200915e-06
    - 3.436915621860973e-07
    - -8.330731903468323e-05
    - 6.3907424571192206e-06
    - -5.1677615701414725e-05
    - -0.7779361556897684
    - 1.811738169779047e-05
    - 7.863613779222185e-08
    - 0.00010802355310646215
    - -2.5312627723407536e-13
    - -1.3150319043306804e-13
    - 1.6443301246574957e-08
    - -1.7037778952154475e-11
    - 2.994313078252752e-11
    - 1.9111082343448064e-15
    - -9.77424595137493e-13
    - -7.846063079989798e-09
    - 4.680392257394574e-06
    - 2.1184000924265696e-10
    - 1.7156703158316857e-06
    - 1.5481246369785054e-12
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

A Franka Panda arm with a parallel-jaw gripper works at a low table in a LIBERO scene (MuJoCo / robosuite; LIBERO suite `libero_90`, task `LIVING_ROOM_SCENE2_pick_up_the_tomato_sauce_and_put_it_in_the_basket`). LIBERO-90: 90 short-horizon tasks in 20 kitchen, living-room and study scenes (the task name starts with the scene); LIBERO uses them for pretraining.

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
