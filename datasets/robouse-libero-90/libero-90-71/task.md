---
schema_version: '1.3'
task:
  name: robouse/libero-90-71
  description: Put the red mug on the plate.
metadata:
  author_name: benchflow
  source_benchmark: LIBERO (Liu et al. 2023)
  source_task: LIVING_ROOM_SCENE6_put_the_red_mug_on_the_plate
  suite: libero-90
  libero_suite: libero_90
  libero_task_index: 71
  language_instruction: put the red mug on the plate
  category: manipulation
  difficulty: medium
  tags:
  - franka
  - mujoco
  - robosuite
  - single-arm
  - language
  reference_solution: LIBERO demonstration demo_0 of yifengzhu-hf/LIBERO-datasets/libero_90/LIVING_ROOM_SCENE6_put_the_red_mug_on_the_plate_demo.hdf5 (replay reaches success at step 106)
  robouse:
    id: libero-90-71
    backend: libero
    suite: libero_90
    env: LIVING_ROOM_SCENE6_put_the_red_mug_on_the_plate
    task_index: 71
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
    init_source: yifengzhu-hf/LIBERO-datasets/libero_90/LIVING_ROOM_SCENE6_put_the_red_mug_on_the_plate_demo.hdf5 demo_0 states[0]
    init_state:
    - 0.25000000000000017
    - 0.0029061881752884995
    - -0.16669369015445992
    - -0.0023431562358388076
    - -2.4634770111093554
    - 0.01089557690676314
    - 2.2377326592945423
    - 0.805924392834643
    - 0.03407123703615607
    - -0.034040706045734885
    - -0.11633735271227566
    - -0.1708933490515991
    - 0.4343699488393418
    - -0.7071069013731585
    - 1.0172763262652443e-07
    - 6.67573136829922e-08
    - 0.7071066609999057
    - -0.19173482224680682
    - -0.011711448882415989
    - 0.4385989872647111
    - -0.7071068579899178
    - -5.528613175748412e-07
    - 3.8311997404154767e-07
    - 0.707106704382849
    - 0.15327849628372262
    - 0.006255901152650796
    - 0.4392728083556114
    - 0.7047147476297929
    - 1.243005098421315e-06
    - -4.315434595846152e-07
    - 0.7094907500956908
    - -0.05695769266336671
    - 0.12396524555195487
    - 0.4498398389808574
    - 5.02886788411506e-09
    - -3.7824570034626364e-09
    - 1.2586724035237793e-08
    - 1.0
    - 0.0014560863914478285
    - -0.050945519921505575
    - 0.021355060363358717
    - -0.04499299045234394
    - 0.006263940164827787
    - 0.02752483903347229
    - 0.012214516851118502
    - 0.05753780964446865
    - -0.05758274463994083
    - -7.114629454338284e-11
    - -4.527150192489345e-10
    - 1.1368093065859128e-06
    - -2.8896237769436556e-08
    - -1.8466801296313248e-07
    - -1.083053691224047e-12
    - 8.910653462104873e-12
    - -1.8765526692380743e-10
    - 1.8430481127052276e-06
    - -4.9171912758403945e-09
    - 1.0451615551637408e-07
    - -2.7722836824704424e-13
    - -5.925170751514278e-08
    - -5.202517333899426e-08
    - 1.234521207975919e-06
    - -2.4216532402215752e-05
    - -2.100888008096003e-05
    - -6.486151971838161e-10
    - -6.061720488746668e-11
    - -1.478529490858464e-11
    - 2.895975396023345e-07
    - -1.1621831253153357e-09
    - 4.6857718971292554e-09
    - 1.2428081656330387e-13
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

# Put the red mug on the plate

A Franka Panda arm with a parallel-jaw gripper works at a low table in a LIBERO scene (MuJoCo / robosuite; LIBERO suite `libero_90`, task `LIVING_ROOM_SCENE6_put_the_red_mug_on_the_plate`). LIBERO-90: 90 short-horizon tasks in 20 kitchen, living-room and study scenes (the task name starts with the scene); LIBERO uses them for pretraining.

**Goal (LIBERO's language instruction):** put the red mug on the plate.

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
