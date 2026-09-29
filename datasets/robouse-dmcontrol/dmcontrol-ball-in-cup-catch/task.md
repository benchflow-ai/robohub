---
schema_version: '1.3'
task:
  name: robouse/dmcontrol-ball-in-cup-catch
  description: Ball in cup
metadata:
  author_name: benchflow
  source_benchmark: DeepMind dm_control 1.0.47 (google-deepmind/dm_control, Apache-2.0)
  source_task: dm_control Control Suite `ball_in_cup` / `catch`, random=3
  suite: dmcontrol
  category: manipulation
  difficulty: hard
  tags:
  - dm_control
  - mujoco
  - ball-in-cup
  upstream_task: ball_in_cup-catch
  simulator: dm_control 1.0.47, MuJoCo 3.14.0
  robouse:
    id: dmcontrol-ball-in-cup-catch
    backend: dmcontrol
    env: dmcontrol-ball-in-cup-catch
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

# Ball in cup (dm_control)

Ball in cup from the DeepMind Control Suite (MuJoCo), in the x-z plane (z up, gravity on). A cup (0.065 kg; an open V-bottomed cup, about 8 cm wide inside the rim and 10 cm deep, opening upwards) is held by springs: it slides in x and z about its home position (0, 0.6) with a stiffness of 20 N/m and damping of 3 N s/m per axis. A ball (0.065 kg, 2.5 cm radius) hangs from the bottom of the cup on a 0.3 m string, which goes slack when the ball comes closer. The cup's position `cup_pos` is the centre of its rim (the bottom is 0.1 m lower). The ball starts at rest somewhere below the cup. Simulated time advances only when you act; the scene is paused while you think. A feedback controller can run as a script: Python 3 with numpy is available, and `robo observe --json` / `robo act ... --json` print machine-readable output.

## Task

Swing the ball on its string into the cup and keep it there. The ball starts at (-0.084, 0.553), at rest, and falls when the episode starts; the cup is at its home (0.000, 0.600).

**Success:** the ball entirely inside the cup: its centre within 0.025 m of `cup_target_center` along both x and z (dm_control's own in-target check: the target box's half-size 0.05 m minus the ball radius), held for 50 consecutive steps (1 s of simulated time), judged by the episode server from the simulated state after every step. The episode ends as solved the moment that happens; `robo done` before that scores 0.

**Controls.** `robo act FX FZ [--repeat N]` pushes the cup with 5 N per unit along x and along z, each in [-1, 1], for N steps of 20 ms. The springs limit how far the cup can go (about 0.25 m from home at full force). There are no skills.

**Observation.** `robo observe` reports dm_control's own observation, `position` = [cup x, cup z, ball x, ball z] (joint positions: cup relative to its home, ball relative to its reference 0.2 m above the floor) and `velocity`, plus by name (world frame): `cup_pos`, `cup_vel`, `ball_pos`, `ball_vel`, `cup_target_center` (the middle of the inside of the cup) and `string_length`. Every task also reports `reward` (dm_control's own reward for the last step, for information), `in_target` (whether the success condition holds right now), `hold_steps` (for how many consecutive steps it has held) and `hold_required`, and `time_s` (simulated time). Cameras: `cam0`, `cam1`; `robo observe --image [--camera C]` saves a picture.

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
