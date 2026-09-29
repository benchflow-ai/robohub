---
schema_version: '1.3'
task:
  name: robouse/adroit-door-e23
  description: Unlatch the door and swing it open.
metadata:
  author_name: benchflow
  source_benchmark: Gymnasium-Robotics Adroit hand (Farama; DAPG, Rajeswaran et al. 2018)
  source_task: AdroitHandDoor-v1
  suite: adroit
  category: dexterous-manipulation
  difficulty: hard
  tags:
  - adroit
  - mujoco
  - dexterous-hand
  - door
  robouse:
    id: adroit-door-e23
    backend: adroit
    env: AdroitHandDoor-v1
    seed: 0
    max_steps: 1000
    camera: fixed
    frame_every: 5
    skills: true
    success_mode: final
    init_source: Minari farama-minari/D4RL@a0fd465d1d9d door/human-v2 episode_23 (options/initial_state_dict)
    init_state:
      door_body_pos:
      - -0.16792899370193481
      - 0.3587769865989685
      - 0.2905159890651703
      qpos:
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      qvel:
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
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

# Adroit Door (start state 23)

The robot is the ADROIT hand: a 24-joint, human-sized, Shadow-style right hand (wrist WRJ1 side-to-side and WRJ0 up-down; first (FF), middle (MF) and ring (RF) fingers with J3 spread, J2 knuckle, J1 middle and J0 tip joints; the little finger (LF) with an extra J4 palm joint; the thumb (TH) with J4 rotation, J3, J2, J1 and J0 tip joints). Finger joints J2, J1, J0 bend the finger toward the palm as they increase (0 = straight, 1.6 rad = fully bent). Every joint is driven by a position servo toward its target (the joint position it should settle at). The arm servos are weak (upstream gains): an arm joint settles near, not exactly at, its target, and gravity makes the hand sag (the vertical slide and the tilt settle a little low). World frame in metres: z up, the table top at z = 0, the forearm lies along +y behind the hand and the straight fingers point along +y, about 10 cm beyond the palm. The hand is on an arm with 4 actuators: ARTz (slide, m: moves the whole arm along world +y, toward the door), ARRx (tilts the forearm about world x at its base, 0.5 m behind the palm: increasing it lowers the hand), ARRy (swings the forearm about the vertical: increasing it moves the hand toward -x) and ARRz (rolls the forearm about its own axis, world y), all rad except ARTz.

A door stands on the table in front of the hand. It is shut by a latch: its lever handle (`handle_pos`) must be turned down (the `latch` angle increases from 0 as it turns; the latch has strong dry friction and a spring that holds the door shut) before the door can swing. The door opens by pulling it toward the hand (the hinge is on the door's left edge). The door's handle starts at [0.1221, 0.2088, 0.2655].

**Goal:** Unlatch the door and swing it open.

Success: `door_hinge` (the door angle, rad) is at least 1.35 (the door touches its stopper), which is the env's own success signal (`info['success']`). It is judged after you call `robo done` and the hand has held its targets for 10 more steps, so the goal must still hold then. The step budget is 1000 steps (10 s of simulated time).

This task is adapted from Gymnasium-Robotics `AdroitHandDoor-v1` and starts from the initial state of human demonstration 23 of the Minari dataset `D4RL/door/human-v2`.

**Observation.** `robo observe` reports `palm_pos` (the grasp point just below the palm, where a held object sits), `fingertips` (ff, mf, rf, lf, th tip positions), `joints` (every actuated joint's position, rad or m) and `targets` (its current target), plus `handle_pos` (the lever handle), `door_hinge` (rad, 0 = shut), `latch` (rad, 0 = latched), `door_open` (`door_hinge` >= 1.35). Camera images (`robo observe --image`) are taken from above the table, looking along +y.

**Controls.** `robo act` takes 28 numbers, one per actuator in this order: ARTz ARRx ARRy ARRz WRJ1 WRJ0 FFJ3 FFJ2 FFJ1 FFJ0 MFJ3 MFJ2 MFJ1 MFJ0 RFJ3 RFJ2 RFJ1 RFJ0 LFJ4 LFJ3 LFJ2 LFJ1 LFJ0 THJ4 THJ3 THJ2 THJ1 THJ0. Each number is the change of that actuator's position target this step, in rad (m for the ARTx/ARTy/ARTz slides); the target is clipped to the actuator's range (`robo info` lists every range). All zeros keeps the targets, so `robo act 0 0 ... 0 --repeat N` waits N steps. One step is 10 ms. The servos need several steps to reach a new target, and fingers stop where they touch something. `robo skill set NAME VALUE [NAME VALUE ...] [steps=N]` moves the named targets (absolute, rad or m) there in a straight ramp over N steps (default 10); `robo skill hand open|close|pinch [AMOUNT] [steps=N]` sets a finger synergy (open: all finger and thumb joints straight; close AMOUNT 0..1: the four fingers curl and the thumb swings across, a power grasp; pinch: only the first finger and the thumb); `robo skill wait [N]` holds the targets for N steps (default 10). Every skill step counts against the budget. `robo move-to` and `robo grip` are not available. All angles in this task are in radians.

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
