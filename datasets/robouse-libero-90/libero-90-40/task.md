---
schema_version: '1.3'
task:
  name: robouse/libero-90-40
  description: Put the frying pan on the cabinet shelf.
metadata:
  author_name: benchflow
  source_benchmark: LIBERO (Liu et al. 2023)
  source_task: KITCHEN_SCENE9_put_the_frying_pan_on_the_cabinet_shelf
  suite: libero-90
  libero_suite: libero_90
  libero_task_index: 40
  language_instruction: put the frying pan on the cabinet shelf
  category: manipulation
  difficulty: medium
  tags:
  - franka
  - mujoco
  - robosuite
  - single-arm
  - language
  reference_solution: LIBERO demonstration demo_0 of yifengzhu-hf/LIBERO-datasets/libero_90/KITCHEN_SCENE9_put_the_frying_pan_on_the_cabinet_shelf_demo.hdf5 (replay reaches success at step 166)
  robouse:
    id: libero-90-40
    backend: libero
    suite: libero_90
    env: KITCHEN_SCENE9_put_the_frying_pan_on_the_cabinet_shelf
    task_index: 40
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
    init_source: yifengzhu-hf/LIBERO-datasets/libero_90/KITCHEN_SCENE9_put_the_frying_pan_on_the_cabinet_shelf_demo.hdf5 demo_0 states[0]
    init_state:
    - 0.25000000000000017
    - 0.0032312691073076473
    - -0.16035747898126287
    - -0.015982811026457065
    - -2.4526749948089983
    - -0.00870523183182574
    - 2.232170830154618
    - 0.7724423634981048
    - 0.034051329332667364
    - -0.03406025122176735
    - -0.15577085788371342
    - 0.11670047730709117
    - 0.8992356038234838
    - 0.7071067699909046
    - -9.692648961575562e-06
    - -6.885763903807989e-07
    - 0.7071067923154242
    - 0.03371282429079195
    - -0.01359944312995209
    - 0.8989956691834108
    - 0.7071067559272116
    - -6.0782232043051076e-05
    - 6.142891735541575e-05
    - 0.7071068011652152
    - 0.0
    - -0.0021531877416752637
    - -0.0039016736309209826
    - 0.012162970671883707
    - -0.0027562039860300237
    - 0.042645749327997876
    - 0.01199947504905036
    - 0.02579222732763947
    - 0.05763722023591999
    - -0.05749012018885269
    - 1.6990738082601534e-08
    - -1.7857001962372116e-08
    - 0.0046987322810236555
    - -2.4368576917311752e-05
    - 2.4192889676566062e-05
    - 5.294195791312748e-10
    - -1.574768277464923e-09
    - -3.582765494015284e-07
    - 0.004697247236681478
    - 1.7128964205947879e-06
    - 0.0003906167432700457
    - 4.370247684700251e-12
    - 0.0
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

# Put the frying pan on the cabinet shelf

A Franka Panda arm with a parallel-jaw gripper works at a table in a LIBERO scene (MuJoCo / robosuite; LIBERO suite `libero_90`, task `KITCHEN_SCENE9_put_the_frying_pan_on_the_cabinet_shelf`). LIBERO-90: 90 short-horizon tasks in 20 kitchen, living-room and study scenes (the task name starts with the scene); LIBERO uses them for pretraining.

**Goal (LIBERO's language instruction):** put the frying pan on the cabinet shelf.

The episode ends as solved the moment LIBERO's own success check (the task's goal predicates) passes; you do not need to call `robo done` after that. If you finish without the check passing, call `robo done` (or `robo give-up`).

## This robot

- The action has 7 numbers, not 4: `robo act DX DY DZ DROLL DPITCH DYAW GRIPPER`, each in [-1, 1]. It is LIBERO's delta end-effector command (robosuite's OSC_POSE controller, 20 steps per second). Holding DX/DY/DZ at 1.0 moves the hand about 1 cm per step; DROLL/DPITCH/DYAW rotate the hand (1.0 asks for 0.5 rad; usually leave them at 0). GRIPPER -1 opens, +1 closes.
- `robo move-to X Y Z [--grip G]` and `robo grip G` are available (they keep the hand's orientation).
- World frame, metres: +x points forward, away from the robot's base; +y is to the robot's left; +z is up. The table surface is at about z = 0.90.
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
