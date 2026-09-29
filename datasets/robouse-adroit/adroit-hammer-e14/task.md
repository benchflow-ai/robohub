---
schema_version: '1.3'
task:
  name: robouse/adroit-hammer-e14
  description: Pick up the hammer and drive the nail into the board.
metadata:
  author_name: benchflow
  source_benchmark: Gymnasium-Robotics Adroit hand (Farama; DAPG, Rajeswaran et al. 2018)
  source_task: AdroitHandHammer-v1
  suite: adroit
  category: dexterous-manipulation
  difficulty: hard
  tags:
  - adroit
  - mujoco
  - dexterous-hand
  - hammer
  robouse:
    id: adroit-hammer-e14
    backend: adroit
    env: AdroitHandHammer-v1
    seed: 0
    max_steps: 1500
    camera: fixed
    frame_every: 5
    skills: true
    success_mode: final
    init_source: Minari farama-minari/D4RL@a0fd465d1d9d hammer/human-v2 episode_14 (options/initial_state_dict)
    init_state:
      board_pos:
      - 0.05000000074505806
      - 0.0
      - 0.11292125284671783
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

# Adroit Hammer (start state 14)

The robot is the ADROIT hand: a 24-joint, human-sized, Shadow-style right hand (wrist WRJ1 side-to-side and WRJ0 up-down; first (FF), middle (MF) and ring (RF) fingers with J3 spread, J2 knuckle, J1 middle and J0 tip joints; the little finger (LF) with an extra J4 palm joint; the thumb (TH) with J4 rotation, J3, J2, J1 and J0 tip joints). Finger joints J2, J1, J0 bend the finger toward the palm as they increase (0 = straight, 1.6 rad = fully bent). Every joint is driven by a position servo toward its target (the joint position it should settle at). The arm servos are weak (upstream gains): an arm joint settles near, not exactly at, its target, and gravity makes the hand sag (the vertical slide and the tilt settle a little low). World frame in metres: z up, the table top at z = 0, the forearm lies along +y behind the hand and the straight fingers point along +y, about 10 cm beyond the palm. The hand is on an arm with 2 actuators (rad): ARRx (tilts the forearm about world x at its base, 0.5 m behind the palm: increasing it lowers the hand) and ARRy (swings the forearm about the vertical: increasing it moves the hand toward -x); the arm cannot slide.

A hammer lies on the table under the hand, its handle along x (`hammer_pos` is its centre, `hammer_head_pos` the striking face of its head). A board stands to the side with a nail sticking out of it toward the hammer (`nail_pos` is the nail head; `nail_goal_pos` is where the nail head is when it is driven fully in). Pick up the hammer and strike the nail head along the nail's axis. The nail head starts at [-0.0441, -0.0366, 0.113], 0.091 m out of the board; the hammer lies at [0.0, -0.2, 0.035].

**Goal:** Pick up the hammer and drive the nail into the board.

Success: the nail head is within 1 cm of its fully-driven position (`nail_goal_distance` < 0.01), which is the env's own success signal (`info['success']`). It is judged after you call `robo done` and the hand has held its targets for 10 more steps, so the goal must still hold then. The step budget is 1500 steps (15 s of simulated time).

This task is adapted from Gymnasium-Robotics `AdroitHandHammer-v1` and starts from the initial state of human demonstration 14 of the Minari dataset `D4RL/hammer/human-v2`.

**Observation.** `robo observe` reports `palm_pos` (the grasp point just below the palm, where a held object sits), `fingertips` (ff, mf, rf, lf, th tip positions), `joints` (every actuated joint's position, rad or m) and `targets` (its current target), plus `hammer_pos`, `hammer_rot` (Euler xyz, rad), `hammer_head_pos`, `nail_pos`, `nail_goal_pos`, `nail_goal_distance` (m). Camera images (`robo observe --image`) are taken from above the table, looking along +y.

**Controls.** `robo act` takes 26 numbers, one per actuator in this order: ARRx ARRy WRJ1 WRJ0 FFJ3 FFJ2 FFJ1 FFJ0 MFJ3 MFJ2 MFJ1 MFJ0 RFJ3 RFJ2 RFJ1 RFJ0 LFJ4 LFJ3 LFJ2 LFJ1 LFJ0 THJ4 THJ3 THJ2 THJ1 THJ0. Each number is the change of that actuator's position target this step, in rad (m for the ARTx/ARTy/ARTz slides); the target is clipped to the actuator's range (`robo info` lists every range). All zeros keeps the targets, so `robo act 0 0 ... 0 --repeat N` waits N steps. One step is 10 ms. The servos need several steps to reach a new target, and fingers stop where they touch something. `robo skill set NAME VALUE [NAME VALUE ...] [steps=N]` moves the named targets (absolute, rad or m) there in a straight ramp over N steps (default 10); `robo skill hand open|close|pinch [AMOUNT] [steps=N]` sets a finger synergy (open: all finger and thumb joints straight; close AMOUNT 0..1: the four fingers curl and the thumb swings across, a power grasp; pinch: only the first finger and the thumb); `robo skill wait [N]` holds the targets for N steps (default 10). Every skill step counts against the budget. `robo move-to` and `robo grip` are not available. All angles in this task are in radians.

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
