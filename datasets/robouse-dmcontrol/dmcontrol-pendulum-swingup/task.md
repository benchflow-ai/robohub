---
schema_version: '1.3'
task:
  name: robouse/dmcontrol-pendulum-swingup
  description: Pendulum swing-up
metadata:
  author_name: benchflow
  source_benchmark: DeepMind dm_control 1.0.47 (google-deepmind/dm_control, Apache-2.0)
  source_task: dm_control Control Suite `pendulum` / `swingup`, random=9
  suite: dmcontrol
  category: swing-up
  difficulty: medium
  tags:
  - dm_control
  - mujoco
  - pendulum
  upstream_task: pendulum-swingup
  simulator: dm_control 1.0.47, MuJoCo 3.14.0
  robouse:
    id: dmcontrol-pendulum-swingup
    backend: dmcontrol
    env: dmcontrol-pendulum-swingup
    seed: 0
    max_steps: 1000
    camera: fixed
    cameras:
    - fixed
    - lookat
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
oracle:
  env:
    ROBOUSE_ORACLE_TOKEN: ${ROBOUSE_ORACLE_TOKEN:-}
---

You are controlling a simulated robot. Read the task below, then solve it by running the `robo` command in your shell (start with `robo info` and `robo observe`). Keep going until the task is done, then call `robo done` once. Do not stop to ask questions; there is no human to answer.

# Pendulum swing-up (dm_control)

A torque-limited pendulum from the DeepMind Control Suite (MuJoCo): a 1 kg ball on a 0.5 m massless rod, swinging in the x-z plane (z up) about a hinge with light damping. `pole_angle` is in radians: 0 = pointing straight up, +-pi = hanging straight down; a positive angle tilts the ball towards +x. The motor is too weak to lift the pendulum directly (gravity's torque reaches 4.9 N m, the motor gives at most 1 N m), so it has to be swung up. Simulated time advances only when you act; the scene is paused while you think. A feedback controller can run as a script: Python 3 with numpy is available, and `robo observe --json` / `robo act ... --json` print machine-readable output.

## Task

Swing the pendulum up and keep it upright. It starts at rest near the bottom (`pole_angle` = -3.08 rad).

**Success:** the pendulum within 30 degrees of upright (cosine of `pole_angle` >= cos 30 deg), the task's own reward condition, held for 100 consecutive steps (2 s of simulated time), judged by the episode server from the simulated state after every step. The episode ends as solved the moment that happens; `robo done` before that scores 0.

**Controls.** `robo act TAU [--repeat N]` applies a torque of TAU N m at the hinge, TAU in [-1, 1] (positive increases `pole_angle`), for N steps of 20 ms. There are no skills.

**Observation.** `robo observe` reports dm_control's own observation, `orientation` = [cos, sin] of the pole angle and `velocity` = [angular velocity (rad/s)], plus `pole_angle` and `pole_angvel` by name. Every task also reports `reward` (dm_control's own reward for the last step, for information), `in_target` (whether the success condition holds right now), `hold_steps` (for how many consecutive steps it has held) and `hold_required`, and `time_s` (simulated time). Cameras: `fixed`, `lookat`; `robo observe --image [--camera C]` saves a picture.

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
