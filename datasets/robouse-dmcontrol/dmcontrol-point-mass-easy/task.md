---
schema_version: '1.3'
task:
  name: robouse/dmcontrol-point-mass-easy
  description: Point mass to the target
metadata:
  author_name: benchflow
  source_benchmark: DeepMind dm_control 1.0.47 (google-deepmind/dm_control, Apache-2.0)
  source_task: dm_control Control Suite `point_mass` / `easy`, random=9
  suite: dmcontrol
  category: reaching
  difficulty: easy
  tags:
  - dm_control
  - mujoco
  - point-mass
  upstream_task: point_mass-easy
  simulator: dm_control 1.0.47, MuJoCo 3.14.0
  robouse:
    id: dmcontrol-point-mass-easy
    backend: dmcontrol
    env: dmcontrol-point-mass-easy
    seed: 0
    max_steps: 500
    camera: fixed
    cameras:
    - fixed
    - cam0
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

# Point mass to the target (dm_control)

A point mass from the DeepMind Control Suite (MuJoCo): a 0.3 kg ball (1 cm radius) slides on a table (the x-y plane, z up) inside a square arena with walls at x, y = +-0.3 m. The target is a small sphere at the origin. Simulated time advances only when you act; the scene is paused while you think. A feedback controller can run as a script: Python 3 with numpy is available, and `robo observe --json` / `robo act ... --json` print machine-readable output.

## Task

Push the mass onto the target at the origin and keep it there. It starts at rest at (-0.284, 0.001).

**Success:** the centre of the mass within the target's radius of its centre (`mass_to_target` <= `target_radius` = 0.015 m, the near-target condition of the task's reward), held for 25 consecutive steps (0.5 s of simulated time), judged by the episode server from the simulated state after every step. The episode ends as solved the moment that happens; `robo done` before that scores 0.

**Controls.** `robo act FX FY [--repeat N]` pushes the mass with 0.1 N per unit along x and along y, each in [-1, 1], for N steps of 20 ms. There are no skills.

**Observation.** `robo observe` reports dm_control's own observation, `position` (x, y in m) and `velocity` (m/s), plus `mass_pos`, `mass_vel`, `target_pos`, `target_radius` and `mass_to_target` (m). Every task also reports `reward` (dm_control's own reward for the last step, for information), `in_target` (whether the success condition holds right now), `hold_steps` (for how many consecutive steps it has held) and `hold_required`, and `time_s` (simulated time). Cameras: `fixed`, `cam0`; `robo observe --image [--camera C]` saves a picture.

The step budget is 500 steps (10 s of simulated time).

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
