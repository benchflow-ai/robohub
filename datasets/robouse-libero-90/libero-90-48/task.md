---
schema_version: '1.3'
task:
  name: robouse/libero-90-48
  description: Pick up the ketchup and put it in the basket.
metadata:
  author_name: benchflow
  source_benchmark: LIBERO (Liu et al. 2023)
  source_task: LIVING_ROOM_SCENE1_pick_up_the_ketchup_and_put_it_in_the_basket
  suite: libero-90
  libero_suite: libero_90
  libero_task_index: 48
  language_instruction: pick up the ketchup and put it in the basket
  category: manipulation
  difficulty: medium
  tags:
  - franka
  - mujoco
  - robosuite
  - single-arm
  - language
  reference_solution: LIBERO demonstration demo_0 of yifengzhu-hf/LIBERO-datasets/libero_90/LIVING_ROOM_SCENE1_pick_up_the_ketchup_and_put_it_in_the_basket_demo.hdf5 (replay reaches success at step 163)
  robouse:
    id: libero-90-48
    backend: libero
    suite: libero_90
    env: LIVING_ROOM_SCENE1_pick_up_the_ketchup_and_put_it_in_the_basket
    task_index: 48
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
    init_source: yifengzhu-hf/LIBERO-datasets/libero_90/LIVING_ROOM_SCENE1_pick_up_the_ketchup_and_put_it_in_the_basket_demo.hdf5 demo_0 states[0]
    init_state:
    - 0.25000000000000017
    - 0.00954048156530859
    - -0.16655422069148254
    - 0.014611344961605955
    - -2.4470029359566636
    - 0.013441246488284007
    - 2.2130020970739768
    - 0.809219563687937
    - 0.03406930760503964
    - -0.03404229738080758
    - 0.04536751958531917
    - -0.10708553492518831
    - 0.4751648626212499
    - -0.002482503453895785
    - 0.0021580325352465964
    - 0.7074504440013328
    - 0.7067552966582531
    - -0.16841508599921987
    - 0.08272987756432869
    - 0.44569584300064613
    - -3.4882826695445066e-17
    - -3.4199136418204795e-09
    - -1.1564254660233148e-08
    - 1.0
    - 0.12222736445035039
    - -0.20258267678009054
    - 0.47516914741370325
    - -0.0023299931301754015
    - 0.0023447329691130814
    - 0.7074532159718667
    - 0.7067524464551789
    - -0.22195450862641755
    - -0.14273580160496768
    - 0.6312454499278823
    - 0.5000266167646799
    - 0.4999734008287587
    - 0.4999435910267221
    - 0.5000563836027887
    - 0.005954236061617776
    - 0.2616966925393151
    - 0.4321919610888971
    - 0.7071047490794425
    - -0.001694979337002777
    - 0.0016949779842838823
    - 0.7071047503192003
    - -0.0045713333974502154
    - 8.673302603347389e-05
    - 0.0010997095328753002
    - 3.175160520933499e-05
    - 0.0016602148498510748
    - 6.538106614531958e-05
    - 0.01171751457662579
    - 0.05749491038476483
    - -0.0576389380489944
    - 0.00021731256211248592
    - 4.089412436985414e-05
    - 0.00037604469397853454
    - 0.0010661410261378276
    - -1.352236924776804e-06
    - 0.005679699385990887
    - -7.359356164459253e-16
    - -1.6368580737003084e-15
    - 2.8103504431656006e-09
    - -2.0904109569011998e-13
    - 8.661543540688649e-14
    - 2.7353731814624808e-17
    - 1.1674503787170739e-08
    - 5.549467766183573e-09
    - 1.8531305086511055e-08
    - 1.4502740688761528e-07
    - -6.511204590764069e-10
    - 3.0507396633570685e-07
    - -5.8334018122903596e-05
    - -1.5534314233488833e-05
    - -0.6124323020550401
    - -0.0003036949667747103
    - -5.072348092427988e-07
    - 0.00011628228721845605
    - -9.774271624216668e-13
    - -7.846063078990825e-09
    - 4.680392257392545e-06
    - 2.1184004764901831e-10
    - 1.7156703158167036e-06
    - 1.5481245511512012e-12
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

# Pick up the ketchup and put it in the basket

A Franka Panda arm with a parallel-jaw gripper works at a low table in a LIBERO scene (MuJoCo / robosuite; LIBERO suite `libero_90`, task `LIVING_ROOM_SCENE1_pick_up_the_ketchup_and_put_it_in_the_basket`). LIBERO-90: 90 short-horizon tasks in 20 kitchen, living-room and study scenes (the task name starts with the scene); LIBERO uses them for pretraining.

**Goal (LIBERO's language instruction):** pick up the ketchup and put it in the basket.

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
