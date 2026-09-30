---
schema_version: '1.3'
task:
  name: robouse/dmcontrol-jaco-place-cradle
  description: 'Jaco: place the brick on the cradle'
metadata:
  author_name: benchflow
  source_benchmark: DeepMind dm_control 1.0.47 (google-deepmind/dm_control, Apache-2.0)
  source_task: dm_control.manipulation `place_cradle_features`, seed=2
  suite: dmcontrol
  category: pick and place
  difficulty: medium
  tags:
  - dm_control
  - mujoco
  - jaco
  upstream_task: place_cradle_features
  simulator: dm_control 1.0.47, MuJoCo 3.14.0
  robouse:
    id: dmcontrol-jaco-place-cradle
    backend: dmcontrol
    env: dmcontrol-jaco-place-cradle
    seed: 0
    max_steps: 800
    camera: front_close
    cameras:
    - front_close
    skills: true
    success_mode: first
agent:
  timeout_sec: 2400
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

# Jaco: place the brick on the cradle (dm_control)

A Kinova Jaco robot arm (6 joints) with the three-finger Jaco hand stands at the edge of a table, its base at (0, 0.4, 0), from dm_control.manipulation (MuJoCo). World frame in metres, z up, the table top at z = 0. The hand is controlled at its pinch point, the point between the fingertips (dm_control's tool centre point), and always points straight down; you set its position and its yaw (turn about the vertical). The objects are Duplo 2x4 bricks (6.4 x 3.2 cm, 1.9 cm tall plus studs); a brick's `_pos` is the centre of its bottom face and `_yaw_deg` the direction of its long side. The fingers close along the hand's yaw direction: with `hand_yaw_deg` at a brick's yaw + 90 degrees they grip the brick across its 3.2 cm width. Simulated time advances only when you act; the scene is paused while you think. A feedback controller can run as a script: Python 3 with numpy is available, and `robo observe --json` / `robo act ... --json` print machine-readable output.

## Task

Put the brick on a cradle of three spheres on top of a pedestal (a pillar about 0.2 m tall), let go and take the hand away. The brick lies on the table at (0.069, -0.084, 0.000) (long side at -156 degrees); the target point on the pedestal is (-0.095, 0.010, 0.114). Task fields: `target_pos`, `target_radius`, `brick_to_target` and `hand_to_target`.

**Success:** the brick's `brick_pos` within 5 cm of the target point (`brick_to_target` <= 0.05 m) and the pinch point at least 20 cm away from it (`hand_to_target` >= 0.2 m), where dm_control's place reward is 1, held for 25 consecutive steps (1 s of simulated time), judged by the episode server from the simulated state after every step. The episode ends as solved the moment that happens; `robo done` before that scores 0.

**Controls.** `robo act DX DY DZ DYAW GRIP [--repeat N]` applies one 40 ms step N times. DX DY DZ in [-2, 2] move the commanded pinch point by 1 cm per unit along world x, y, z; DYAW in [-2, 2] turns the commanded hand yaw by 0.1 rad per unit (positive = counter-clockwise seen from above). A Jacobian controller drives the arm's joint velocities towards the command (the hand lags a fast command, and the command is kept within 8 cm of the hand and inside the box x, y in [-0.35, 0.35], z in [-0.02, 0.6]). GRIP in [-1, 1] is the fingers' velocity: +1 closes, -1 opens; at 0 the fingers stop where they are and no longer squeeze, so keep GRIP at +1 while you carry something. Skills (`robo info` lists their arguments): `move_to X Y Z [YAW] [GRIP]` moves the pinch point in a straight line to a point (and turns the hand to YAW degrees; 999 keeps it) while holding GRIP; `grip VALUE [STEPS]` sets the fingers without moving the arm.

**Observation.** `robo observe` reports `hand_pos` (the pinch point), `hand_target_pos` (the commanded point), `hand_yaw_deg`, `hand_target_yaw_deg`, `hand_z_axis` (the hand's pointing direction), `arm_joints` and `finger_joints` (rad; each finger ranges from 0.15, open, to 1.35, closed), `grip_command`, and for each brick `<name>_pos`, `<name>_quat_wxyz` and `<name>_yaw_deg`, plus the task fields below. `robo observe --image` saves a picture from the `front_close` camera. Every task also reports `reward` (dm_control's own reward for the last step, for information), `in_target` (whether the success condition holds right now), `hold_steps` (for how many consecutive steps it has held) and `hold_required`, and `time_s` (simulated time).

The step budget is 800 steps (32 s of simulated time).

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
- The episode ends as solved the moment the task's success rule holds (see Success); `robo done` before that scores 0.
- Call `robo done` exactly once when finished.
