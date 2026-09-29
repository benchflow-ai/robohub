---
schema_version: '1.3'
task:
  name: robouse/libero-10-0
  description: Put both the alphabet soup and the tomato sauce in the basket.
metadata:
  author_name: benchflow
  source_benchmark: LIBERO (Liu et al. 2023)
  source_task: LIVING_ROOM_SCENE2_put_both_the_alphabet_soup_and_the_tomato_sauce_in_the_basket
  suite: libero
  libero_suite: libero_10
  libero_task_index: 0
  language_instruction: put both the alphabet soup and the tomato sauce in the basket
  category: manipulation
  difficulty: hard
  tags:
  - franka
  - mujoco
  - robosuite
  - single-arm
  - language
  reference_solution: LIBERO demonstration demo_1 of yifengzhu-hf/LIBERO-datasets/libero_10/LIVING_ROOM_SCENE2_put_both_the_alphabet_soup_and_the_tomato_sauce_in_the_basket_demo.hdf5 (replay reaches success at step 293)
  robouse:
    id: libero-10-0
    backend: libero
    suite: libero_10
    env: LIVING_ROOM_SCENE2_put_both_the_alphabet_soup_and_the_tomato_sauce_in_the_basket
    task_index: 0
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
    init_source: yifengzhu-hf/LIBERO-datasets/libero_10/LIVING_ROOM_SCENE2_put_both_the_alphabet_soup_and_the_tomato_sauce_in_the_basket_demo.hdf5 demo_1 states[0]
    init_state:
    - 0.25000000000000017
    - 0.003861717025620707
    - -0.18428420752921346
    - -0.009491442702759466
    - -2.406835631951526
    - 0.018566491302623382
    - 2.2131238771454247
    - 0.7812443976296659
    - 0.03407317733266582
    - -0.034039177673829225
    - -0.10104740486561892
    - -0.16797251801000024
    - 0.4751648626212499
    - -0.002482503453895712
    - 0.0021580325352465735
    - 0.7074504440013328
    - 0.706755296658253
    - 0.09506601127194622
    - -0.2042838802559229
    - 0.44569584300064613
    - -3.48784457206702e-17
    - -3.4199136411015975e-09
    - -1.156425466035014e-08
    - 1.0
    - -0.1071186523931849
    - 0.04694644543206232
    - 0.47516914741370325
    - -0.002329993130175422
    - 0.00234473296911305
    - 0.7074532159718667
    - 0.7067524464551789
    - -0.26855853403345825
    - -0.13480523198819885
    - 0.6312454499278821
    - 0.5000266167646803
    - 0.4999734008287584
    - 0.49994359102672203
    - 0.5000563836027885
    - 0.004107920542990908
    - -0.2444287715032899
    - 0.5897167696579936
    - 0.5000106862844936
    - 0.49998930857605123
    - 0.5000124215800898
    - 0.4999875830223854
    - 0.054394747738254735
    - -0.10843306853012766
    - 0.5897144010070282
    - 0.49998385081262614
    - 0.5000162063786705
    - 0.49998762299598376
    - 0.5000123189843294
    - 0.03621372699061898
    - 0.057726506004534064
    - 0.44546984280974117
    - 3.335838800230721e-10
    - -4.512586032345516e-09
    - -2.6176911301861772e-08
    - 0.9999999999999998
    - 0.008515835549066073
    - 0.25799887285019274
    - 0.4321919610888971
    - 0.7071047490794425
    - -0.0016949793370027771
    - 0.0016949779842838786
    - 0.7071047503192003
    - -0.002711510372087918
    - 0.02704762288421254
    - -1.6296844160853183e-05
    - 0.07427494240401693
    - 0.018248136052324625
    - -0.11448317672638583
    - -0.008127148322936276
    - 0.05756401945838283
    - -0.05756102523261275
    - 0.00021731256211199833
    - 4.0894124369233714e-05
    - 0.00037604469397850543
    - 0.0010661410261219204
    - -1.3522369246812792e-06
    - 0.005679699385978784
    - -7.35942487729282e-16
    - -1.636803661275791e-15
    - 2.8103504423885306e-09
    - -2.091063264359601e-13
    - 8.66392479537044e-14
    - 2.741223781360853e-17
    - 1.1674503787276547e-08
    - 5.549467764846389e-09
    - 1.8531305097026896e-08
    - 1.4502740683703307e-07
    - -6.511204585567308e-10
    - 3.0507396638832475e-07
    - -5.833401812293021e-05
    - -1.5534314233522572e-05
    - -0.612432302055041
    - -0.0003036949667750561
    - -5.072348092413166e-07
    - 0.0001162822872181056
    - -7.03410795787079e-06
    - 5.286007836938589e-07
    - -0.7779263010058411
    - 8.180346962599696e-06
    - 3.436915621890065e-07
    - -8.330731903528839e-05
    - 6.390742457160627e-06
    - -5.1677615701422355e-05
    - -0.7779361556897684
    - 1.811738169870872e-05
    - 7.863613778462762e-08
    - 0.00010802355310613726
    - -2.5312627775573113e-13
    - -1.3150311935464477e-13
    - 1.6443301242358384e-08
    - -1.70378911692918e-11
    - 2.994300103521695e-11
    - 1.9111837074856677e-15
    - -9.774270822162143e-13
    - -7.846063079750408e-09
    - 4.680392257394383e-06
    - 2.118400465075511e-10
    - 1.715670315828078e-06
    - 1.5481246927767824e-12
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

# Put both the alphabet soup and the tomato sauce in the basket

A Franka Panda arm with a parallel-jaw gripper works at a low table in a LIBERO scene (MuJoCo / robosuite; LIBERO suite `libero_10`, task `LIVING_ROOM_SCENE2_put_both_the_alphabet_soup_and_the_tomato_sauce_in_the_basket`). LIBERO-Long (libero_10): long-horizon tasks, most with two sub-goals.

**Goal (LIBERO's language instruction):** put both the alphabet soup and the tomato sauce in the basket.

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
