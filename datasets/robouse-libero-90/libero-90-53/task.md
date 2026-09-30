---
schema_version: '1.3'
task:
  name: robouse/libero-90-53
  description: Pick up the orange juice and put it in the basket.
metadata:
  author_name: benchflow
  source_benchmark: LIBERO (Liu et al. 2023)
  source_task: LIVING_ROOM_SCENE2_pick_up_the_orange_juice_and_put_it_in_the_basket
  suite: libero-90
  libero_suite: libero_90
  libero_task_index: 53
  language_instruction: pick up the orange juice and put it in the basket
  category: manipulation
  difficulty: medium
  tags:
  - franka
  - mujoco
  - robosuite
  - single-arm
  - language
  reference_solution: LIBERO demonstration demo_0 of yifengzhu-hf/LIBERO-datasets/libero_90/LIVING_ROOM_SCENE2_pick_up_the_orange_juice_and_put_it_in_the_basket_demo.hdf5 (replay reaches success at step 137)
  robouse:
    id: libero-90-53
    backend: libero
    suite: libero_90
    env: LIVING_ROOM_SCENE2_pick_up_the_orange_juice_and_put_it_in_the_basket
    task_index: 53
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
    init_source: yifengzhu-hf/LIBERO-datasets/libero_90/LIVING_ROOM_SCENE2_pick_up_the_orange_juice_and_put_it_in_the_basket_demo.hdf5 demo_0 states[0]
    init_state:
    - 0.25000000000000017
    - 0.002941147135262874
    - -0.1496301650084952
    - -0.033066801668641586
    - -2.4118611673090258
    - 0.00784305235695817
    - 2.2425723301920284
    - 0.7839788247872086
    - 0.03391860958268406
    - -0.034097841700043804
    - -0.07596476354884761
    - -0.17230536151374667
    - 0.4751648626212499
    - -0.002482503453895301
    - 0.0021580325352469755
    - 0.707450444001333
    - 0.706755296658253
    - 0.10414940720506455
    - -0.1909939269365252
    - 0.44569584300064613
    - -3.488028869293225e-17
    - -3.4199136422612112e-09
    - -1.1564254659767954e-08
    - 1.0
    - -0.11272274210838412
    - 0.04959960173619207
    - 0.47516914741370325
    - -0.0023299931301754566
    - 0.002344732969113009
    - 0.7074532159718667
    - 0.7067524464551789
    - -0.2626657693201642
    - -0.13269351855090813
    - 0.6312454499278823
    - 0.50002661676468
    - 0.4999734008287585
    - 0.49994359102672215
    - 0.5000563836027886
    - 0.02018475624926275
    - -0.24213262596996948
    - 0.5897167696579936
    - 0.5000106862844934
    - 0.4999893085760513
    - 0.5000124215800898
    - 0.49998758302238544
    - 0.041952697142872565
    - -0.10883950935037062
    - 0.5897144010070282
    - 0.499983850812626
    - 0.5000162063786705
    - 0.49998762299598365
    - 0.5000123189843293
    - 0.060549303853375876
    - 0.03900780061358504
    - 0.44546984280974117
    - 3.335838800305941e-10
    - -4.512586031525944e-09
    - -2.6176911301876724e-08
    - 0.9999999999999998
    - 0.004069816071824491
    - 0.2637986531371621
    - 0.4321919610888971
    - 0.7071047490794425
    - -0.0016949793370027821
    - 0.0016949779842838808
    - 0.7071047503192003
    - -0.07841877262573863
    - 0.1367038322692553
    - -0.206774713288909
    - 0.11773641684139678
    - -0.03853118640609499
    - 0.05467806970620267
    - -0.05491628865610971
    - 0.05790131532209808
    - -0.057065268450767405
    - 0.00021731256211175444
    - 4.089412436892186e-05
    - 0.00037604469398174405
    - 0.0010661410261137776
    - -1.3522369246337745e-06
    - 0.005679699385972183
    - -7.359421346733536e-16
    - -1.6375191845239757e-15
    - 2.8103504416276654e-09
    - -2.0916561432532737e-13
    - 8.668961497586359e-14
    - 2.7453669225338052e-17
    - 1.1674503787168812e-08
    - 5.549467765625685e-09
    - 1.8531305090699736e-08
    - 1.4502740686478573e-07
    - -6.511204586794591e-10
    - 3.0507396637944546e-07
    - -5.833401812276438e-05
    - -1.553431423342003e-05
    - -0.6124323020550405
    - -0.0003036949667748104
    - -5.072348092406826e-07
    - 0.00011628228721788837
    - -7.034107957640157e-06
    - 5.286007835741802e-07
    - -0.7779263010058416
    - 8.180346963049384e-06
    - 3.4369156218541496e-07
    - -8.330731903444935e-05
    - 6.390742457255517e-06
    - -5.16776157014707e-05
    - -0.7779361556897684
    - 1.8117381699569514e-05
    - 7.863613777747661e-08
    - 0.00010802355310705613
    - -2.5312627828515787e-13
    - -1.3150316524320789e-13
    - 1.6443301246427623e-08
    - -1.7037946869111164e-11
    - 2.994312495128755e-11
    - 1.9112468022633475e-15
    - -9.774269005680588e-13
    - -7.846063079830943e-09
    - 4.680392257391583e-06
    - 2.1184004370581013e-10
    - 1.7156703158292941e-06
    - 1.5481247232466493e-12
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

# Pick up the orange juice and put it in the basket

A Franka Panda arm with a parallel-jaw gripper works at a low table in a LIBERO scene (MuJoCo / robosuite; LIBERO suite `libero_90`, task `LIVING_ROOM_SCENE2_pick_up_the_orange_juice_and_put_it_in_the_basket`). LIBERO-90: 90 short-horizon tasks in 20 kitchen, living-room and study scenes (the task name starts with the scene); LIBERO uses them for pretraining.

**Goal (LIBERO's language instruction):** pick up the orange juice and put it in the basket.

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
