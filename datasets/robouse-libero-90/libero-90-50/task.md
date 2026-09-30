---
schema_version: '1.3'
task:
  name: robouse/libero-90-50
  description: Pick up the alphabet soup and put it in the basket.
metadata:
  author_name: benchflow
  source_benchmark: LIBERO (Liu et al. 2023)
  source_task: LIVING_ROOM_SCENE2_pick_up_the_alphabet_soup_and_put_it_in_the_basket
  suite: libero-90
  libero_suite: libero_90
  libero_task_index: 50
  language_instruction: pick up the alphabet soup and put it in the basket
  category: manipulation
  difficulty: medium
  tags:
  - franka
  - mujoco
  - robosuite
  - single-arm
  - language
  reference_solution: LIBERO demonstration demo_0 of yifengzhu-hf/LIBERO-datasets/libero_90/LIVING_ROOM_SCENE2_pick_up_the_alphabet_soup_and_put_it_in_the_basket_demo.hdf5 (replay reaches success at step 155)
  robouse:
    id: libero-90-50
    backend: libero
    suite: libero_90
    env: LIVING_ROOM_SCENE2_pick_up_the_alphabet_soup_and_put_it_in_the_basket
    task_index: 50
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
    init_source: yifengzhu-hf/LIBERO-datasets/libero_90/LIVING_ROOM_SCENE2_pick_up_the_alphabet_soup_and_put_it_in_the_basket_demo.hdf5 demo_0 states[0]
    init_state:
    - 0.25000000000000017
    - -0.006304897031940452
    - -0.16312062590422116
    - 0.013191645814952102
    - -2.428092358246225
    - -0.004211620380644561
    - 2.2338403281627204
    - 0.7923117802437982
    - 0.03401783147673511
    - -0.03407799046432597
    - -0.09986407574123696
    - -0.16053812409445983
    - 0.4751648626212499
    - -0.00248250345389535
    - 0.0021580325352469703
    - 0.7074504440013328
    - 0.7067552966582531
    - 0.07598630414254699
    - -0.1991285158232938
    - 0.44569584300064613
    - -3.487894741513943e-17
    - -3.4199136443683906e-09
    - -1.15642546601964e-08
    - 1.0
    - -0.09675593036010777
    - 0.058077789064874194
    - 0.47516914741370325
    - -0.0023299931301753885
    - 0.0023447329691130953
    - 0.7074532159718667
    - 0.7067524464551789
    - -0.22660167153447675
    - -0.1700904140409504
    - 0.6312454499278823
    - 0.5000266167646801
    - 0.4999734008287586
    - 0.4999435910267219
    - 0.5000563836027886
    - -0.014317568513023382
    - -0.2543739533688075
    - 0.5897167696579936
    - 0.5000106862844939
    - 0.4999893085760512
    - 0.5000124215800896
    - 0.49998758302238544
    - 0.07391313378473523
    - -0.11497225090683495
    - 0.5897144010070282
    - 0.49998385081262614
    - 0.5000162063786705
    - 0.49998762299598376
    - 0.5000123189843294
    - 0.07182208413338016
    - 0.04894004358332928
    - 0.44546984280974117
    - 3.3358388002873064e-10
    - -4.512586028890128e-09
    - -2.6176911301982663e-08
    - 0.9999999999999998
    - -0.0028563894037457295
    - 0.2545824613739403
    - 0.4321919610888971
    - 0.7071047490794425
    - -0.001694979337002777
    - 0.0016949779842838804
    - 0.7071047503192003
    - -0.012127460994416809
    - 0.00028382864861764464
    - -0.07787769025168984
    - -0.0014693480046772975
    - -0.08347120051373783
    - -0.004521382467366274
    - -0.030829460945873934
    - 0.05762733942736746
    - -0.05745623296851949
    - 0.0002173125621119214
    - 4.0894124369749666e-05
    - 0.00037604469397806167
    - 0.0010661410261351687
    - -1.3522369247738824e-06
    - 0.005679699385976303
    - -7.359351893969499e-16
    - -1.6369270628983094e-15
    - 2.810350440446192e-09
    - -2.090653508136355e-13
    - 8.657388609026288e-14
    - 2.7272208799417637e-17
    - 1.167450378677469e-08
    - 5.549467768718863e-09
    - 1.853130509277143e-08
    - 1.4502740694118477e-07
    - -6.51120459482984e-10
    - 3.050739663333941e-07
    - -5.833401812284638e-05
    - -1.5534314233470473e-05
    - -0.6124323020550405
    - -0.00030369496677494647
    - -5.07234809243603e-07
    - 0.00011628228721872041
    - -7.034107957834328e-06
    - 5.286007837535612e-07
    - -0.7779263010058409
    - 8.180346961918156e-06
    - 3.43691562191169e-07
    - -8.33073190355413e-05
    - 6.390742457208074e-06
    - -5.1677615701398984e-05
    - -0.7779361556897684
    - 1.8117381698657253e-05
    - 7.863613778472811e-08
    - 0.00010802355310616085
    - -2.5312627661520416e-13
    - -1.3150295948961726e-13
    - 1.644330124875856e-08
    - -1.7037864227561705e-11
    - 2.994300337915281e-11
    - 1.9111790352847317e-15
    - -9.774263917270609e-13
    - -7.84606308075689e-09
    - 4.6803922573926585e-06
    - 2.118400361491191e-10
    - 1.7156703158431691e-06
    - 1.548124698342596e-12
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

# Pick up the alphabet soup and put it in the basket

A Franka Panda arm with a parallel-jaw gripper works at a low table in a LIBERO scene (MuJoCo / robosuite; LIBERO suite `libero_90`, task `LIVING_ROOM_SCENE2_pick_up_the_alphabet_soup_and_put_it_in_the_basket`). LIBERO-90: 90 short-horizon tasks in 20 kitchen, living-room and study scenes (the task name starts with the scene); LIBERO uses them for pretraining.

**Goal (LIBERO's language instruction):** pick up the alphabet soup and put it in the basket.

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
