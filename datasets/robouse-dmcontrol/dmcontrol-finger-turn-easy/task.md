---
schema_version: '1.3'
task:
  name: robouse/dmcontrol-finger-turn-easy
  description: 'Finger: turn the spinner (large target)'
metadata:
  author_name: benchflow
  source_benchmark: DeepMind dm_control 1.0.47 (google-deepmind/dm_control, Apache-2.0)
  source_task: dm_control Control Suite `finger` / `turn_easy`, random=1
  suite: dmcontrol
  category: manipulation
  difficulty: medium
  tags:
  - dm_control
  - mujoco
  - finger
  upstream_task: finger-turn_easy
  simulator: dm_control 1.0.47, MuJoCo 3.14.0
  robouse:
    id: dmcontrol-finger-turn-easy
    backend: dmcontrol
    env: dmcontrol-finger-turn-easy
    seed: 0
    max_steps: 1000
    camera: cam1
    cameras:
    - cam0
    - cam1
    skills: false
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
---

You are controlling a simulated robot. Read the task below, then solve it by running the `robo` command in your shell (start with `robo info` and `robo observe`). Keep going until the task is done, then call `robo done` once. Do not stop to ask questions; there is no human to answer.

# Finger: turn the spinner (large target) (dm_control)

The finger domain of the DeepMind Control Suite (MuJoCo), in the x-z plane (z up, no gravity). A two-link finger is mounted on a hinge at (x, z) = (-0.2, 0.4): the proximal link is 0.18 m long; the distal link ends in a rounded fingertip whose far end is 0.161 m from the distal joint (fingertip radius 3 cm). Both joints are damped and limited to +-110 degrees (+-1.92 rad). Angles are in radians: `proximal_angle` is the proximal link's direction measured counter-clockwise from +x in the x-z plane (0 = pointing towards +x, -pi/2 = pointing down), `distal_angle` the distal link's angle relative to it. The spinner is a free body on an unactuated, damped hinge at (0.2, 0.4): a bar 0.26 m long and 0.12 m wide made of two capsules, which stops turning almost at once when nothing pushes it. One end is marked: its tip (`spinner_tip_pos`) lies 0.13 m from the hinge. The target is a point on the same 0.13 m circle (`target_pos`). Only the fingertip and the proximal link collide; the finger can reach the spinner only on its left side. Simulated time advances only when you act; the scene is paused while you think. A feedback controller can run as a script: Python 3 with numpy is available, and `robo observe --json` / `robo act ... --json` print machine-readable output.

## Task

Turn the spinner with the fingertip until its marked tip lies in the target circle (radius 0.07 m around (0.135, 0.513)), and leave it there. At the start the marked tip is at (0.323, 0.442), 0.201 m from the target; the finger is at rest.

**Success:** the spinner's marked tip inside the target circle (`dist_to_target` <= 0, the task's own reward condition), held for 25 consecutive steps (0.5 s of simulated time), judged by the episode server from the simulated state after every step. The episode ends as solved the moment that happens; `robo done` before that scores 0.

**Controls.** `robo act PROXIMAL DISTAL [--repeat N]` applies torques of 30 N m and 15 N m per unit at the proximal and distal joints, each in [-1, 1] (positive increases the joint angle), for N steps of 20 ms. There are no skills.

**Observation.** `robo observe` reports dm_control's own observation, `position` (the two joint angles, then the spinner tip's x and z relative to the hinge), `velocity` (the two joints' and the hinge's angular velocities), `touch` (log(1 + force) of the fingertip's two touch sensors), `target_position` (target relative to the hinge) and `dist_to_target` (distance from the tip to the target minus the target radius), plus by name: `proximal_angle`, `distal_angle`, `proximal_vel`, `distal_vel`, `fingertip_pos` (centre of the fingertip), `fingertip_end_pos` (its far end), `spinner_center`, `spinner_angle` (the hinge angle, rad), `spinner_vel`, `spinner_tip_pos`, `target_pos`, `target_radius` and `tip_to_target` (all positions (x, z) in m). Every task also reports `reward` (dm_control's own reward for the last step, for information), `in_target` (whether the success condition holds right now), `hold_steps` (for how many consecutive steps it has held) and `hold_required`, and `time_s` (simulated time). Cameras: `cam0`, `cam1`; `robo observe --image [--camera C]` saves a picture.

The step budget is 1000 steps (20 s of simulated time).

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
