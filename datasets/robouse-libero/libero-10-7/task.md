---
schema_version: '1.3'
task:
  name: robouse/libero-10-7
  description: Put both the alphabet soup and the cream cheese box in the basket.
metadata:
  author_name: benchflow
  source_benchmark: LIBERO (Liu et al. 2023)
  source_task: LIVING_ROOM_SCENE1_put_both_the_alphabet_soup_and_the_cream_cheese_box_in_the_basket
  suite: libero
  libero_suite: libero_10
  libero_task_index: 7
  language_instruction: put both the alphabet soup and the cream cheese box in the basket
  category: manipulation
  difficulty: hard
  tags:
  - franka
  - mujoco
  - robosuite
  - single-arm
  - language
  reference_solution: LIBERO demonstration demo_0 of yifengzhu-hf/LIBERO-datasets/libero_10/LIVING_ROOM_SCENE1_put_both_the_alphabet_soup_and_the_cream_cheese_box_in_the_basket_demo.hdf5 (replay reaches success at step 231)
  robouse:
    id: libero-10-7
    backend: libero
    suite: libero_10
    env: LIVING_ROOM_SCENE1_put_both_the_alphabet_soup_and_the_cream_cheese_box_in_the_basket
    task_index: 7
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
    init_source: yifengzhu-hf/LIBERO-datasets/libero_10/LIVING_ROOM_SCENE1_put_both_the_alphabet_soup_and_the_cream_cheese_box_in_the_basket_demo.hdf5 demo_0 states[0]
    init_state:
    - 0.25000000000000017
    - -0.011819402943705707
    - -0.13634531210984366
    - -0.010035010603386628
    - -2.433407847787233
    - -0.017741500238709195
    - 2.282328760627119
    - 0.8362171647635671
    - 0.034050950041412974
    - -0.03406039147577638
    - 0.06658482483750384
    - -0.08241899658909513
    - 0.4751648626212499
    - -0.0024825034538958563
    - 0.0021580325352465652
    - 0.7074504440013328
    - 0.706755296658253
    - -0.14257025609556367
    - 0.06720399152736473
    - 0.44569584300064613
    - -3.488252203328141e-17
    - -3.419913642233461e-09
    - -1.1564254659834333e-08
    - 1.0
    - 0.10074481706257689
    - -0.1792706885857308
    - 0.47516914741370325
    - -0.0023299931301753057
    - 0.0023447329691131677
    - 0.7074532159718667
    - 0.7067524464551789
    - -0.19839331691606357
    - -0.1289573197796389
    - 0.6312454499278823
    - 0.5000266167646799
    - 0.4999734008287586
    - 0.49994359102672226
    - 0.5000563836027886
    - -0.004618329362747683
    - 0.2624008547922468
    - 0.4321919610888971
    - 0.7071047490794425
    - -0.0016949793370027826
    - 0.0016949779842838739
    - 0.7071047503192003
    - 0.011544557118269173
    - 0.2674011004011241
    - 0.0333520384403372
    - 0.23502228464450142
    - -0.05675049777681409
    - 0.09508319500006641
    - 0.2996218044781801
    - 0.05771182844814812
    - -0.057406090218559
    - 0.0002173125621122767
    - 4.08941243693112e-05
    - 0.0003760446939787396
    - 0.001066141026123903
    - -1.3522369246879962e-06
    - 0.005679699385985804
    - -7.359362297505419e-16
    - -1.6372238297664068e-15
    - 2.810350441038534e-09
    - -2.0897642366313955e-13
    - 8.661867530762833e-14
    - 2.727482150074078e-17
    - 1.1674503787295379e-08
    - 5.549467764487965e-09
    - 1.8531305089410475e-08
    - 1.450274068265925e-07
    - -6.511204589525183e-10
    - 3.0507396630291757e-07
    - -5.833401812276912e-05
    - -1.5534314233394917e-05
    - -0.6124323020550402
    - -0.00030369496677435965
    - -5.072348092406855e-07
    - 0.00011628228721785122
    - -9.77425454008227e-13
    - -7.846063078109304e-09
    - 4.6803922573963355e-06
    - 2.1184002203642116e-10
    - 1.7156703158034912e-06
    - 1.5481245410607929e-12
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
oracle:
  env:
    ROBOUSE_ORACLE_TOKEN: ${ROBOUSE_ORACLE_TOKEN:-}
---

You are controlling a simulated robot. Read the task below, then solve it by running the `robo` command in your shell (start with `robo info` and `robo observe`). Keep going until the task is done, then call `robo done` once. Do not stop to ask questions; there is no human to answer.

# Put both the alphabet soup and the cream cheese box in the basket

A Franka Panda arm with a parallel-jaw gripper works at a low table in a LIBERO scene (MuJoCo / robosuite; LIBERO suite `libero_10`, task `LIVING_ROOM_SCENE1_put_both_the_alphabet_soup_and_the_cream_cheese_box_in_the_basket`). LIBERO-Long (libero_10): long-horizon tasks, most with two sub-goals.

**Goal (LIBERO's language instruction):** put both the alphabet soup and the cream cheese box in the basket.

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
