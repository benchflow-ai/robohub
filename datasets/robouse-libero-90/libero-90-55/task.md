---
schema_version: '1.3'
task:
  name: robouse/libero-90-55
  description: Pick up the alphabet soup and put it in the tray.
metadata:
  author_name: benchflow
  source_benchmark: LIBERO (Liu et al. 2023)
  source_task: LIVING_ROOM_SCENE3_pick_up_the_alphabet_soup_and_put_it_in_the_tray
  suite: libero-90
  libero_suite: libero_90
  libero_task_index: 55
  language_instruction: pick up the alphabet soup and put it in the tray
  category: manipulation
  difficulty: medium
  tags:
  - franka
  - mujoco
  - robosuite
  - single-arm
  - language
  reference_solution: LIBERO demonstration demo_0 of yifengzhu-hf/LIBERO-datasets/libero_90/LIVING_ROOM_SCENE3_pick_up_the_alphabet_soup_and_put_it_in_the_tray_demo.hdf5 (replay reaches success at step 100)
  robouse:
    id: libero-90-55
    backend: libero
    suite: libero_90
    env: LIVING_ROOM_SCENE3_pick_up_the_alphabet_soup_and_put_it_in_the_tray
    task_index: 55
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
    init_source: yifengzhu-hf/LIBERO-datasets/libero_90/LIVING_ROOM_SCENE3_pick_up_the_alphabet_soup_and_put_it_in_the_tray_demo.hdf5 demo_0 states[0]
    init_state:
    - 0.25000000000000017
    - 0.01896007321502311
    - -0.15159998813836628
    - -0.004955442729926925
    - -2.4460066015999486
    - 0.0064624908105164495
    - 2.238482624675052
    - 0.7991480238646886
    - 0.03407118602803457
    - -0.034040595717342075
    - -0.07553603306958452
    - -0.138782866250204
    - 0.4751648626212499
    - -0.0024825034538958554
    - 0.002158032535246476
    - 0.7074504440013327
    - 0.7067552966582531
    - 0.08070424765090438
    - -0.20761007814839233
    - 0.44569584300064613
    - -3.4880345699576376e-17
    - -3.419913643744672e-09
    - -1.1564254660674585e-08
    - 1.0
    - -0.11314177219800822
    - 0.06150481928660696
    - 0.47516914741370325
    - -0.0023299931301753265
    - 0.002344732969113123
    - 0.7074532159718667
    - 0.7067524464551789
    - -0.2592146021315504
    - -0.1422542675456046
    - 0.6312454499278823
    - 0.5000266167646799
    - 0.4999734008287587
    - 0.4999435910267221
    - 0.5000563836027887
    - 0.0746682686876301
    - 0.06877980223879189
    - 0.44546984280974117
    - 3.3358388003023874e-10
    - -4.512586034137587e-09
    - -2.6176911301580772e-08
    - 0.9999999999999998
    - 0.008028660860051759
    - 0.25707272797217745
    - 0.43738012050757274
    - -7.451456349304145e-06
    - -0.0017326379326048236
    - 0.0012644815872153217
    - 0.9999976994956469
    - -0.018239253136536567
    - 0.027606190410489777
    - 0.027964189419049445
    - 0.01855785717001221
    - -0.04402282233686164
    - 0.010844177999483364
    - 0.0007822693592531803
    - 0.057483800104470205
    - -0.0576482653622799
    - 0.0002173125621113086
    - 4.0894124369957196e-05
    - 0.00037604469397758847
    - 0.0010661410261407352
    - -1.3522369248267826e-06
    - 0.00567969938596029
    - -7.359435504900522e-16
    - -1.6363160627994148e-15
    - 2.8103504390118974e-09
    - -2.0899150204560858e-13
    - 8.670260017851879e-14
    - 2.7451549678701208e-17
    - 1.1674503788110353e-08
    - 5.549467757888299e-09
    - 1.853130508705883e-08
    - 1.4502740667370384e-07
    - -6.511204578399739e-10
    - 3.050739663385006e-07
    - -5.8334018122787804e-05
    - -1.5534314233591453e-05
    - -0.6124323020550405
    - -0.00030369496677464284
    - -5.072348092428532e-07
    - 0.000116282287218451
    - -2.5312627632982423e-13
    - -1.3150329437058388e-13
    - 1.64433012422712e-08
    - -1.7037772340413612e-11
    - 2.994291088895445e-11
    - 1.9110443222197848e-15
    - 6.915902947078938e-09
    - 3.514609199637883e-09
    - 1.957734750530074e-06
    - 5.7012110844213545e-06
    - -1.1170259724865278e-05
    - 4.800815306467e-08
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

# Pick up the alphabet soup and put it in the tray

A Franka Panda arm with a parallel-jaw gripper works at a low table in a LIBERO scene (MuJoCo / robosuite; LIBERO suite `libero_90`, task `LIVING_ROOM_SCENE3_pick_up_the_alphabet_soup_and_put_it_in_the_tray`). LIBERO-90: 90 short-horizon tasks in 20 kitchen, living-room and study scenes (the task name starts with the scene); LIBERO uses them for pretraining.

**Goal (LIBERO's language instruction):** pick up the alphabet soup and put it in the tray.

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
