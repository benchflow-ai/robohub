---
schema_version: '1.3'
task:
  name: robouse/adroit-pen-e11
  description: Rotate the pen in the hand to match the target orientation.
metadata:
  author_name: benchflow
  source_benchmark: Gymnasium-Robotics Adroit hand (Farama; DAPG, Rajeswaran et al. 2018)
  source_task: AdroitHandPen-v1
  suite: adroit
  category: dexterous-manipulation
  difficulty: hard
  tags:
  - adroit
  - mujoco
  - dexterous-hand
  - pen
  robouse:
    id: adroit-pen-e11
    backend: adroit
    env: AdroitHandPen-v1
    seed: 0
    max_steps: 600
    camera: fixed
    frame_every: 5
    skills: true
    success_mode: final
    init_source: Minari farama-minari/D4RL@a0fd465d1d9d pen/human-v2 episode_11 (options/initial_state_dict)
    init_state:
      desired_orien:
      - 0.9094308018684387
      - 0.41558361053466797
      - -0.01366843655705452
      - -0.0062460810877382755
      qpos:
      - -0.017124243080615997
      - -0.004374719224870205
      - 0.02350861020386219
      - 0.03484887629747391
      - 0.0720360204577446
      - 0.07447120547294617
      - -0.013790377415716648
      - 0.0009057857096195221
      - 0.001000635209493339
      - 0.001031163614243269
      - -0.013881263323128223
      - 0.0009083858458325267
      - 0.0010012793354690075
      - 0.0010313125094398856
      - 0.0117140281945467
      - -0.014095628634095192
      - 0.06557795405387878
      - 0.016050126403570175
      - 0.07163728028535843
      - -0.005865015089511871
      - 0.0009676026529632509
      - 0.011576509103178978
      - 0.025986120104789734
      - -0.00103374186437577
      - 0.0005838687065988779
      - 2.905573469899247e-29
      - -2.763696294039164e-08
      - 1.4480428546992107e-32
      - -2.1316282826778134e-20
      - -1.2774440829841438e-37
      qvel:
      - -2.7288830280303955
      - -0.6395633816719055
      - 3.6542320251464844
      - 5.4997687339782715
      - 11.261236190795898
      - 11.618363380432129
      - -2.2057061195373535
      - 0.13224205374717712
      - 0.14470520615577698
      - 0.14875005185604095
      - -2.2194104194641113
      - 0.13257332146167755
      - 0.14478778839111328
      - 0.14876927435398102
      - 1.906911015510559
      - -2.2595224380493164
      - 10.297928810119629
      - 2.5425477027893066
      - 11.174997329711914
      - -0.9136490821838379
      - 0.14005054533481598
      - 1.804321050643921
      - 4.0414652824401855
      - -0.1491336077451706
      - 0.09731144458055496
      - 4.1928843040271196e-27
      - -4.606160473485943e-06
      - 2.0917363377967095e-30
      - -3.552713847545346e-18
      - -2.1163903349341786e-35
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

# Adroit Pen (start state 11)

The robot is the ADROIT hand: a 24-joint, human-sized, Shadow-style right hand (wrist WRJ1 side-to-side and WRJ0 up-down; first (FF), middle (MF) and ring (RF) fingers with J3 spread, J2 knuckle, J1 middle and J0 tip joints; the little finger (LF) with an extra J4 palm joint; the thumb (TH) with J4 rotation, J3, J2, J1 and J0 tip joints). Finger joints J2, J1, J0 bend the finger toward the palm as they increase (0 = straight, 1.6 rad = fully bent). Every joint is driven by a position servo toward its target (the joint position it should settle at). The arm servos are weak (upstream gains): an arm joint settles near, not exactly at, its target, and gravity makes the hand sag (the vertical slide and the tilt settle a little low). World frame in metres: z up, the table top at z = 0, the forearm lies along +y behind the hand and the straight fingers point along +y, about 10 cm beyond the palm. The hand has no arm joints: the forearm is fixed and only the wrist and fingers move.

The hand is held palm-up and holds a pen (a cylinder about 23 cm long) lying across its fingers. A second, floating pen beside the hand (the target, not touchable) shows the goal orientation. Turn the pen in the hand (in-hand manipulation) so it points the same way, without dropping it or letting it drift away. At the start the pen points along [1.0, 0.0, 0.0008] and the target along [-0.0301, -0.7557, 0.6542] (similarity -0.0295).

**Goal:** Rotate the pen in the hand to match the target orientation.

Success: the pen's direction matches the target's (`orientation_similarity`, the dot product of the two unit vectors, above 0.95) and the pen's centre is within 7.5 cm of its starting point `pen_home_pos` (`pen_home_distance` < 0.075), which is the env's own success signal (`info['success']`). It is judged after you call `robo done` and the hand has held its targets for 10 more steps, so the goal must still hold then. The step budget is 600 steps (6 s of simulated time).

This task is adapted from Gymnasium-Robotics `AdroitHandPen-v1` and starts from the initial state of human demonstration 11 of the Minari dataset `D4RL/pen/human-v2`.

**Observation.** `robo observe` reports `palm_pos` (the grasp point just below the palm, where a held object sits), `fingertips` (ff, mf, rf, lf, th tip positions), `joints` (every actuated joint's position, rad or m) and `targets` (its current target), plus `pen_pos` (centre), `pen_dir` and `target_dir` (unit vectors along the pen and the target, from the bottom end to the top end), `orientation_similarity`, `pen_home_pos`, `pen_home_distance` (m). Camera images (`robo observe --image`) are taken from above the table, looking along +y.

**Controls.** `robo act` takes 24 numbers, one per actuator in this order: WRJ1 WRJ0 FFJ3 FFJ2 FFJ1 FFJ0 MFJ3 MFJ2 MFJ1 MFJ0 RFJ3 RFJ2 RFJ1 RFJ0 LFJ4 LFJ3 LFJ2 LFJ1 LFJ0 THJ4 THJ3 THJ2 THJ1 THJ0. Each number is the change of that actuator's position target this step, in rad (m for the ARTx/ARTy/ARTz slides); the target is clipped to the actuator's range (`robo info` lists every range). All zeros keeps the targets, so `robo act 0 0 ... 0 --repeat N` waits N steps. One step is 10 ms. The servos need several steps to reach a new target, and fingers stop where they touch something. `robo skill set NAME VALUE [NAME VALUE ...] [steps=N]` moves the named targets (absolute, rad or m) there in a straight ramp over N steps (default 10); `robo skill hand open|close|pinch [AMOUNT] [steps=N]` sets a finger synergy (open: all finger and thumb joints straight; close AMOUNT 0..1: the four fingers curl and the thumb swings across, a power grasp; pinch: only the first finger and the thumb); `robo skill wait [N]` holds the targets for N steps (default 10). Every skill step counts against the budget. `robo move-to` and `robo grip` are not available. All angles in this task are in radians.

## How to control the robot

You are the robot's policy. You act only through the `robo` command in your shell. There is no other way to move the robot, and you cannot read or change the simulator, the scoring, or other files to succeed; the episode server judges the final physical state itself.

```
robo info                          # the robot, its sensors, action groups, skills and step budget
robo observe                       # robot and scene state as numbers
robo observe --image [--camera C]  # also saves a camera image and prints its path (open it to look)
robo act V1 V2 ... [--repeat N]    # one low-level action (the action groups under Controls), applied N times (N <= 50)
robo skill NAME ARG ...            # run a skill listed by `robo info`; it runs until it finishes and reports the result
robo done "short summary"          # end the episode and ask for scoring
robo give-up "reason"              # end the episode without claiming success
```

- Positions are in metres in the world frame (+z up); angles are in degrees unless a field says otherwise.
- The episode has a fixed step budget (see `robo info`); every simulated control step counts, including the steps a skill runs.
- Skills are ordinary controllers: they can fail, stop early or be blocked by the scene. Read what they report and re-observe.
- Success is judged about 10 steps after you call `robo done`, with the robot holding still (each action group's hold value: zero for velocity and delta commands, full brake for a car), so the goal must still be true when the robot stops.
- Call `robo done` exactly once when finished.
