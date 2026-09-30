---
schema_version: '1.3'
task:
  name: robouse/libero-90-51
  description: Pick up the butter and put it in the basket.
metadata:
  author_name: benchflow
  source_benchmark: LIBERO (Liu et al. 2023)
  source_task: LIVING_ROOM_SCENE2_pick_up_the_butter_and_put_it_in_the_basket
  suite: libero-90
  libero_suite: libero_90
  libero_task_index: 51
  language_instruction: pick up the butter and put it in the basket
  category: manipulation
  difficulty: medium
  tags:
  - franka
  - mujoco
  - robosuite
  - single-arm
  - language
  reference_solution: end-effector path of LIBERO demonstration demo_0 of yifengzhu-hf/LIBERO-datasets/libero_90/LIVING_ROOM_SCENE2_pick_up_the_butter_and_put_it_in_the_basket_demo.hdf5, tracked closed-loop (the recorded actions do not replay in the released scene); success at step 149
  robouse:
    id: libero-90-51
    backend: libero
    suite: libero_90
    env: LIVING_ROOM_SCENE2_pick_up_the_butter_and_put_it_in_the_basket
    task_index: 51
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
    init_source: yifengzhu-hf/LIBERO-datasets/libero_90/LIVING_ROOM_SCENE2_pick_up_the_butter_and_put_it_in_the_basket_demo.hdf5 demo_0 states[0]
    init_state:
    - 0.25000000000000017
    - -0.0066837623476519005
    - -0.16345776977642745
    - 0.0029795660663314784
    - -2.4335462098615572
    - 0.008921935669731712
    - 2.221020760206953
    - 0.77968550867138
    - 0.0340726677376334
    - -0.034039700561978234
    - -0.09500776469560922
    - -0.15487513930812688
    - 0.4751648626212499
    - -0.002482503453895992
    - 0.002158032535246618
    - 0.7074504440013328
    - 0.7067552966582531
    - 0.08984063134926244
    - -0.18693355913523188
    - 0.44569584300064613
    - -3.4878249915383494e-17
    - -3.4199136406400187e-09
    - -1.1564254660316029e-08
    - 1.0
    - -0.10292332404661528
    - 0.037687878483881165
    - 0.47516914741370325
    - -0.0023299931301754124
    - 0.0023447329691130684
    - 0.7074532159718667
    - 0.7067524464551789
    - -0.27332970387588496
    - -0.14728112647734065
    - 0.6312454499278823
    - 0.5000266167646797
    - 0.49997340082875885
    - 0.49994359102672215
    - 0.5000563836027886
    - -0.023134074596067006
    - -0.26633732882657135
    - 0.5897167696579936
    - 0.5000106862844936
    - 0.4999893085760512
    - 0.5000124215800896
    - 0.49998758302238544
    - 0.027804874759698364
    - -0.11591329183885737
    - 0.5897144010070282
    - 0.49998385081262614
    - 0.5000162063786706
    - 0.49998762299598376
    - 0.5000123189843292
    - 0.052750436194895105
    - 0.07125408623213954
    - 0.44546984280974117
    - 3.335838800324531e-10
    - -4.5125860305126985e-09
    - -2.6176911301674604e-08
    - 0.9999999999999998
    - -0.005781874063815654
    - 0.252676871752229
    - 0.4321919610888971
    - 0.7071047490794425
    - -0.0016949793370027782
    - 0.001694977984283885
    - 0.7071047503192003
    - 0.010209878671671736
    - 0.02744216940886635
    - 0.01175225502860834
    - 0.030730831320934793
    - -0.07024769123154229
    - -0.039938659369034964
    - 0.033983918034021655
    - 0.057446607320662514
    - -0.057678603851203436
    - 0.00021731256210992024
    - 4.089412436955611e-05
    - 0.0003760446939799022
    - 0.0010661410261301211
    - -1.3522369247920868e-06
    - 0.005679699385924303
    - -7.359433378564865e-16
    - -1.6370293521616118e-15
    - 2.810350442843286e-09
    - -2.0926734038322472e-13
    - 8.667322105032052e-14
    - 2.7535290077022703e-17
    - 1.1674503786994285e-08
    - 5.549467767047376e-09
    - 1.8531305089249053e-08
    - 1.4502740690017885e-07
    - -6.511204593602586e-10
    - 3.0507396628827486e-07
    - -5.8334018122486606e-05
    - -1.5534314233553075e-05
    - -0.6124323020550401
    - -0.00030369496677301887
    - -5.072348092454254e-07
    - 0.00011628228721902277
    - -7.0341079579031125e-06
    - 5.286007836672125e-07
    - -0.7779263010058411
    - 8.18034696228788e-06
    - 3.4369156218972736e-07
    - -8.330731903528695e-05
    - 6.390742457205954e-06
    - -5.167761570142392e-05
    - -0.7779361556897684
    - 1.811738169910598e-05
    - 7.863613778123296e-08
    - 0.00010802355310613768
    - -2.531262787463924e-13
    - -1.3150342809189789e-13
    - 1.644330124745944e-08
    - -1.703798777391308e-11
    - 2.994310379962595e-11
    - 1.9112441642675457e-15
    - -9.774266428910047e-13
    - -7.84606308251628e-09
    - 4.680392257394454e-06
    - 2.1184003992336742e-10
    - 1.7156703158695294e-06
    - 1.5481249001088608e-12
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

# Pick up the butter and put it in the basket

A Franka Panda arm with a parallel-jaw gripper works at a low table in a LIBERO scene (MuJoCo / robosuite; LIBERO suite `libero_90`, task `LIVING_ROOM_SCENE2_pick_up_the_butter_and_put_it_in_the_basket`). LIBERO-90: 90 short-horizon tasks in 20 kitchen, living-room and study scenes (the task name starts with the scene); LIBERO uses them for pretraining.

**Goal (LIBERO's language instruction):** pick up the butter and put it in the basket.

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
