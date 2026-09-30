---
schema_version: '1.3'
task:
  name: robouse/dmcontrol-acrobot-swingup
  description: Acrobot swing-up
metadata:
  author_name: benchflow
  source_benchmark: DeepMind dm_control 1.0.47 (google-deepmind/dm_control, Apache-2.0)
  source_task: dm_control Control Suite `acrobot` / `swingup`, random=9
  suite: dmcontrol
  category: swing-up
  difficulty: hard
  tags:
  - dm_control
  - mujoco
  - acrobot
  upstream_task: acrobot-swingup_sparse
  simulator: dm_control 1.0.47, MuJoCo 3.14.0
  robouse:
    id: dmcontrol-acrobot-swingup
    backend: dmcontrol
    env: dmcontrol-acrobot-swingup
    seed: 0
    max_steps: 3000
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

# Acrobot swing-up (dm_control)

An acrobot from the DeepMind Control Suite (MuJoCo): a two-link pendulum in the x-z plane (z up). The upper arm hangs from an unactuated shoulder hinge at (x, z) = (0, 2); the lower arm is attached at the elbow, the only motor. Each link is 1 m long and weighs 1 kg; both joints have light damping. Angles are in radians: `shoulder_angle` 0 = upper arm pointing straight up (+-pi = hanging down), `elbow_angle` is the lower arm's angle relative to the upper arm (0 = straight); positive angles turn towards +x. The target is the point straight above the shoulder at full height, (0, 4). Simulated time advances only when you act; the scene is paused while you think. A feedback controller can run as a script: Python 3 with numpy is available, and `robo observe --json` / `robo act ... --json` print machine-readable output.

## Task

Swing the acrobot up until the tip of the lower arm reaches the target at the top and keep it there. It starts hanging nearly straight down (`shoulder_angle` = -3.08, `elbow_angle` = 0.01 rad), at rest.

**Success:** the tip of the lower arm inside the target sphere (`tip_to_target` <= 0.2 m, dm_control's swingup_sparse condition), held for 20 consecutive steps (0.2 s of simulated time), judged by the episode server from the simulated state after every step. The episode ends as solved the moment that happens; `robo done` before that scores 0.

**Controls.** `robo act TAU [--repeat N]` applies a torque of 2 N m per unit at the elbow, TAU in [-1, 1] (positive increases `elbow_angle`), for N steps of 10 ms. The motor is weak: the acrobot has to be pumped up by swinging, and balancing it upright needs fast, precise feedback. There are no skills.

**Observation.** `robo observe` reports dm_control's own observation, `orientations` = [sin shoulder, sin(shoulder + elbow), cos shoulder, cos(shoulder + elbow)] and `velocity` = [shoulder, elbow angular velocity (rad/s)], plus `shoulder_angle`, `elbow_angle`, `shoulder_vel`, `elbow_vel`, the tip of the lower arm `tip_pos` and `target_pos` as (x, z) in metres, `target_radius` and `tip_to_target` (m). Every task also reports `reward` (dm_control's own reward for the last step, for information), `in_target` (whether the success condition holds right now), `hold_steps` (for how many consecutive steps it has held) and `hold_required`, and `time_s` (simulated time). Cameras: `fixed`, `lookat`; `robo observe --image [--camera C]` saves a picture.

The step budget is 3000 steps (30 s of simulated time).

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
