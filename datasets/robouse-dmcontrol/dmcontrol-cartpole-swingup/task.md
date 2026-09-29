---
schema_version: '1.3'
task:
  name: robouse/dmcontrol-cartpole-swingup
  description: Cart-pole swing-up
metadata:
  author_name: benchflow
  source_benchmark: DeepMind dm_control 1.0.47 (google-deepmind/dm_control, Apache-2.0)
  source_task: dm_control Control Suite `cartpole` / `swingup`, random=1
  suite: dmcontrol
  category: swing-up
  difficulty: medium
  tags:
  - dm_control
  - mujoco
  - cartpole
  upstream_task: cartpole-swingup
  simulator: dm_control 1.0.47, MuJoCo 3.14.0
  robouse:
    id: dmcontrol-cartpole-swingup
    backend: dmcontrol
    env: dmcontrol-cartpole-swingup
    seed: 0
    max_steps: 1500
    camera: fixed
    cameras:
    - fixed
    - lookatcart
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

# Cart-pole swing-up (dm_control)

A cart-pole from the DeepMind Control Suite (MuJoCo). A cart (1 kg) slides on a rail along x (x = 0 in the middle, stops at x = +-1.8 m); a pole (1 m long, 0.1 kg) swings freely on an unactuated hinge on the cart, in the x-z plane (z up). `pole_angle` is in radians: 0 = pointing straight up, +-pi = hanging straight down; a positive angle tilts the top of the pole towards +x. The fixed camera looks along +y, so +x is to the right in the image. Simulated time advances only when you act; the scene is paused while you think. A feedback controller can run as a script: Python 3 with numpy is available, and `robo observe --json` / `robo act ... --json` print machine-readable output.

## Task

Swing the pole up from hanging down and balance it upright with the cart near the middle of the rail. The pole starts hanging (`pole_angle` = 3.14 rad) with the cart at x = 0.016 m.

**Success:** the cart within 0.25 m of the rail's centre (|`cart_x`| <= 0.25) and the pole within about 5.7 degrees of vertical (cosine of `pole_angle` >= 0.995), dm_control's sparse cart-pole condition (balance_sparse / swingup_sparse), held for 200 consecutive steps (2 s of simulated time), judged by the episode server from the simulated state after every step. The episode ends as solved the moment that happens; `robo done` before that scores 0.

**Controls.** `robo act F [--repeat N]` pushes the cart along x with a force of 10 N per unit, F in [-1, 1], for N steps of 10 ms (one step = one dm_control control step). There are no skills.

**Observation.** `robo observe` reports dm_control's own observation, `position` = [cart x (m), cos(pole angle), sin(pole angle)] and `velocity` = [cart velocity (m/s), pole angular velocity (rad/s)], plus the same values by name: `cart_x`, `cart_vel`, `pole_angle`, `pole_angvel`. Every task also reports `reward` (dm_control's own reward for the last step, for information), `in_target` (whether the success condition holds right now), `hold_steps` (for how many consecutive steps it has held) and `hold_required`, and `time_s` (simulated time). Cameras: `fixed`, `lookatcart`; `robo observe --image [--camera C]` saves a picture.

The step budget is 1500 steps (15 s of simulated time).

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
