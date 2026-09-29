---
schema_version: '1.3'
task:
  name: robouse/libero-10-1
  description: Put both the cream cheese box and the butter in the basket.
metadata:
  author_name: benchflow
  source_benchmark: LIBERO (Liu et al. 2023)
  source_task: LIVING_ROOM_SCENE2_put_both_the_cream_cheese_box_and_the_butter_in_the_basket
  suite: libero
  libero_suite: libero_10
  libero_task_index: 1
  language_instruction: put both the cream cheese box and the butter in the basket
  category: manipulation
  difficulty: hard
  tags:
  - franka
  - mujoco
  - robosuite
  - single-arm
  - language
  reference_solution: LIBERO demonstration demo_0 of yifengzhu-hf/LIBERO-datasets/libero_10/LIVING_ROOM_SCENE2_put_both_the_cream_cheese_box_and_the_butter_in_the_basket_demo.hdf5 (replay reaches success at step 247)
  robouse:
    id: libero-10-1
    backend: libero
    suite: libero_10
    env: LIVING_ROOM_SCENE2_put_both_the_cream_cheese_box_and_the_butter_in_the_basket
    task_index: 1
    seed: 0
    max_steps: 520
    camera: agentview
    cameras:
    - agentview
    - robot0_eye_in_hand
    image_size: 256
    settle_steps: 50
    skills: true
    success_mode: first
    obs_mode: state
    init_source: yifengzhu-hf/LIBERO-datasets/libero_10/LIVING_ROOM_SCENE2_put_both_the_cream_cheese_box_and_the_butter_in_the_basket_demo.hdf5 demo_0 states[0]
    init_state:
    - 0.25000000000000017
    - 0.024567142970520418
    - -0.14867598525823902
    - -0.026042812152996527
    - -2.4014763363550884
    - 0.012306660255242732
    - 2.2066771901505198
    - 0.7744819643369567
    - 0.03402569907078076
    - -0.034069430775358574
    - -0.09350763932761874
    - -0.13101259477602922
    - 0.4751648626212499
    - -0.0024825034538957192
    - 0.002158032535247059
    - 0.707450444001333
    - 0.706755296658253
    - 0.09610786244643488
    - -0.20463387848170933
    - 0.44569584300064613
    - -3.488271607868909e-17
    - -3.4199136409147895e-09
    - -1.1564254660559655e-08
    - 1.0
    - -0.11842259447982136
    - 0.07399853831723198
    - 0.47516914741370325
    - -0.0023299931301753304
    - 0.002344732969113141
    - 0.7074532159718667
    - 0.7067524464551789
    - -0.24711176692464276
    - -0.1631608998984518
    - 0.6312454499278823
    - 0.5000266167646797
    - 0.49997340082875863
    - 0.4999435910267223
    - 0.5000563836027883
    - 0.023629997401506196
    - -0.23890362819775637
    - 0.5897167696579936
    - 0.5000106862844936
    - 0.49998930857605134
    - 0.5000124215800895
    - 0.4999875830223856
    - 0.025683465163524648
    - -0.08741874997304591
    - 0.5897144010070282
    - 0.49998385081262614
    - 0.5000162063786705
    - 0.4999876229959837
    - 0.5000123189843294
    - 0.058596026175406137
    - 0.028345163693170607
    - 0.44546984280974117
    - 3.3358388003517406e-10
    - -4.512586036125772e-09
    - -2.6176911301758037e-08
    - 0.9999999999999998
    - -0.005220631974437123
    - 0.26204534478634856
    - 0.4321919610888971
    - 0.7071047490794425
    - -0.0016949793370027821
    - 0.0016949779842838804
    - 0.7071047503192003
    - -0.061722457122778095
    - 0.21094858310835105
    - -0.058658016260513784
    - 0.3363068574583297
    - -0.02042895093196207
    - -0.3305290140754816
    - -0.009269345717830198
    - 0.05760539031467163
    - -0.057421971146970154
    - 0.00021731256210974668
    - 4.089412436877278e-05
    - 0.0003760446939835833
    - 0.0010661410261099654
    - -1.3522369246599022e-06
    - 0.005679699385919968
    - -7.359374140108259e-16
    - -1.6364677712752508e-15
    - 2.8103504419049686e-09
    - -2.0901252644877966e-13
    - 8.657236073468061e-14
    - 2.727386613439709e-17
    - 1.1674503787465793e-08
    - 5.5494677629980175e-09
    - 1.8531305092354213e-08
    - 1.4502740677686472e-07
    - -6.511204585034609e-10
    - 3.050739663627055e-07
    - -5.833401812256509e-05
    - -1.5534314233453498e-05
    - -0.6124323020550405
    - -0.0003036949667734646
    - -5.072348092399141e-07
    - 0.00011628228721752923
    - -7.034107957798178e-06
    - 5.286007835733393e-07
    - -0.7779263010058409
    - 8.180346961890767e-06
    - 3.436915621876212e-07
    - -8.330731903408282e-05
    - 6.390742457140715e-06
    - -5.167761570145741e-05
    - -0.7779361556897684
    - 1.8117381698847703e-05
    - 7.863613778389409e-08
    - 0.00010802355310706204
    - -2.531262802035009e-13
    - -1.3150309766647643e-13
    - 1.644330124313071e-08
    - -1.703776726281913e-11
    - 2.9943143297031395e-11
    - 1.9111358665039567e-15
    - -9.774283052654905e-13
    - -7.846063077277115e-09
    - 4.680392257389436e-06
    - 2.1184006482073496e-10
    - 1.71567031579102e-06
    - 1.5481246163629049e-12
agent:
  timeout_sec: 1800
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

# Put both the cream cheese box and the butter in the basket

A Franka Panda arm with a parallel-jaw gripper works at a low table in a LIBERO scene (MuJoCo / robosuite; LIBERO suite `libero_10`, task `LIVING_ROOM_SCENE2_put_both_the_cream_cheese_box_and_the_butter_in_the_basket`). LIBERO-Long (libero_10): long-horizon tasks, most with two sub-goals.

**Goal (LIBERO's language instruction):** put both the cream cheese box and the butter in the basket.

The episode ends as solved the moment LIBERO's own success check (the task's goal predicates) passes; you do not need to call `robo done` after that. If you finish without the check passing, call `robo done` (or `robo give-up`).

## This robot

- The action has 7 numbers, not 4: `robo act DX DY DZ DROLL DPITCH DYAW GRIPPER`, each in [-1, 1]. It is LIBERO's delta end-effector command (robosuite's OSC_POSE controller, 20 steps per second). Holding DX/DY/DZ at 1.0 moves the hand about 1 cm per step; DROLL/DPITCH/DYAW rotate the hand (1.0 asks for 0.5 rad; usually leave them at 0). GRIPPER -1 opens, +1 closes.
- `robo move-to X Y Z [--grip G]` and `robo grip G` are available (they keep the hand's orientation).
- World frame, metres: +x points forward, away from the robot's base; +y is to the robot's left; +z is up. The table surface is at about z = 0.43.
- `robo observe` fields: `hand_pos` (end-effector position), `eef_quat` (orientation quaternion x, y, z, w) and `eef_axis_angle`, `gripper_qpos` (the two finger joints), `gripper_open` (finger gap in metres, about 0.08 when fully open), and `objects`: the position and quaternion of every object in the scene, under LIBERO's names (e.g. `akita_black_bowl_1`, `plate_1`; `..._1`, `..._2` number identical objects); `fixtures`: the position and quaternion of each piece of furniture (cabinet, stove, microwave, rack, shelf, ...); `articulated`: each movable part of a fixture (drawer slide, door hinge, stove knob), keyed by its joint name (e.g. `wooden_cabinet_1_top_level` is the cabinet's top drawer), with `qpos` (joint position), `range` (joint limits), LIBERO's thresholds for that fixture (`open_ranges` / `close_ranges`, or `turnon_ranges` / `turnoff_ranges` for a stove knob: the joint positions at which LIBERO counts the part as open / closed or on / off) and `box_min` / `box_max` (the world-frame bounding box of the moving part).
- `robo observe --image` saves the front camera (`agentview`); add `--camera robot0_eye_in_hand` for the wrist camera. Both are 256x256.
- The step budget is 520 steps (LeRobot's LIBERO evaluation budget for this suite).

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
