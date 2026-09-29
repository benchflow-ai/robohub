---
schema_version: '1.3'
task:
  name: robouse/libero-90-52
  description: Pick up the milk and put it in the basket.
metadata:
  author_name: benchflow
  source_benchmark: LIBERO (Liu et al. 2023)
  source_task: LIVING_ROOM_SCENE2_pick_up_the_milk_and_put_it_in_the_basket
  suite: libero-90
  libero_suite: libero_90
  libero_task_index: 52
  language_instruction: pick up the milk and put it in the basket
  category: manipulation
  difficulty: medium
  tags:
  - franka
  - mujoco
  - robosuite
  - single-arm
  - language
  reference_solution: LIBERO demonstration demo_0 of yifengzhu-hf/LIBERO-datasets/libero_90/LIVING_ROOM_SCENE2_pick_up_the_milk_and_put_it_in_the_basket_demo.hdf5 (replay reaches success at step 93)
  robouse:
    id: libero-90-52
    backend: libero
    suite: libero_90
    env: LIVING_ROOM_SCENE2_pick_up_the_milk_and_put_it_in_the_basket
    task_index: 52
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
    init_source: yifengzhu-hf/LIBERO-datasets/libero_90/LIVING_ROOM_SCENE2_pick_up_the_milk_and_put_it_in_the_basket_demo.hdf5 demo_0 states[0]
    init_state:
    - 0.25000000000000017
    - -0.0003846729789729846
    - -0.12744453857876056
    - 0.011734618333253446
    - -2.40042934253665
    - 0.006586739508754623
    - 2.2443072706473184
    - 0.7957707005388892
    - 0.03403657896191054
    - -0.03406409290050117
    - -0.08384307140923464
    - -0.13645273691454984
    - 0.4751648626212499
    - -0.0024825034538957964
    - 0.0021580325352465214
    - 0.7074504440013328
    - 0.706755296658253
    - 0.09478635655489855
    - -0.17658647706456046
    - 0.44569584300064613
    - -3.4878724069074966e-17
    - -3.4199136409657554e-09
    - -1.156425466047652e-08
    - 1.0
    - -0.09351415711615285
    - 0.06951811178554151
    - 0.47516914741370325
    - -0.002329993130175435
    - 0.002344732969113048
    - 0.7074532159718667
    - 0.7067524464551789
    - -0.2386227407542961
    - -0.1468159566904033
    - 0.6312454499278823
    - 0.5000266167646797
    - 0.4999734008287587
    - 0.499943591026722
    - 0.5000563836027886
    - 0.009369706010153927
    - -0.24097085447937344
    - 0.5897167696579936
    - 0.5000106862844933
    - 0.4999893085760514
    - 0.5000124215800898
    - 0.49998758302238544
    - 0.03016325298776463
    - -0.0873368176023979
    - 0.5897144010070282
    - 0.49998385081262614
    - 0.5000162063786705
    - 0.4999876229959837
    - 0.5000123189843294
    - 0.025304861932978585
    - 0.02587743237324283
    - 0.44546984280974117
    - 3.3358388003587055e-10
    - -4.512586034673074e-09
    - -2.617691130189231e-08
    - 0.9999999999999998
    - 0.00015728997447443695
    - 0.2565381729528939
    - 0.4321919610888971
    - 0.7071047490794425
    - -0.0016949793370027826
    - 0.001694977984283875
    - 0.7071047503192003
    - -0.014474802787126187
    - 0.20227606824255623
    - -0.06312738629043171
    - 0.2029877416198762
    - -0.05066797332591631
    - -0.09250485086326306
    - -0.06319482923822035
    - 0.057572189639676044
    - -0.05753651295432751
    - 0.00021731256211199686
    - 4.0894124369243194e-05
    - 0.0003760446939785059
    - 0.0010661410261221995
    - -1.3522369246830553e-06
    - 0.005679699385978819
    - -7.359448079403791e-16
    - -1.6365909743024115e-15
    - 2.8103504402856654e-09
    - -2.0904377362971478e-13
    - 8.672759622253736e-14
    - 2.7503334735712917e-17
    - 1.1674503786945036e-08
    - 5.549467767412765e-09
    - 1.8531305090122225e-08
    - 1.450274069173828e-07
    - -6.511204590348857e-10
    - 3.0507396636958224e-07
    - -5.8334018122860025e-05
    - -1.5534314233476737e-05
    - -0.6124323020550401
    - -0.00030369496677457925
    - -5.072348092439551e-07
    - 0.00011628228721877534
    - -7.0341079577303974e-06
    - 5.286007835124537e-07
    - -0.7779263010058409
    - 8.18034696312494e-06
    - 3.436915621846329e-07
    - -8.330731903414363e-05
    - 6.390742457093595e-06
    - -5.167761570146056e-05
    - -0.7779361556897684
    - 1.8117381698889343e-05
    - 7.863613778382848e-08
    - 0.0001080235531067374
    - -2.5312627857113603e-13
    - -1.3150291558408085e-13
    - 1.6443301247595467e-08
    - -1.7037734894695673e-11
    - 2.994303918424274e-11
    - 1.9110875202844925e-15
    - -9.7742958192134e-13
    - -7.846063080935374e-09
    - 4.680392257392689e-06
    - 2.1184008396808126e-10
    - 1.7156703158458345e-06
    - 1.5481248686950735e-12
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

# Pick up the milk and put it in the basket

A Franka Panda arm with a parallel-jaw gripper works at a low table in a LIBERO scene (MuJoCo / robosuite; LIBERO suite `libero_90`, task `LIVING_ROOM_SCENE2_pick_up_the_milk_and_put_it_in_the_basket`). LIBERO-90: 90 short-horizon tasks in 20 kitchen, living-room and study scenes (the task name starts with the scene); LIBERO uses them for pretraining.

**Goal (LIBERO's language instruction):** pick up the milk and put it in the basket.

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
