---
schema_version: '1.3'
task:
  name: robouse/dmcontrol-manipulator-bring-ball
  description: 'Planar manipulator: bring the ball'
metadata:
  author_name: benchflow
  source_benchmark: DeepMind dm_control 1.0.47 (google-deepmind/dm_control, Apache-2.0)
  source_task: dm_control Control Suite `manipulator` / `bring_ball`, random=2
  suite: dmcontrol
  category: manipulation
  difficulty: hard
  tags:
  - dm_control
  - mujoco
  - manipulator
  upstream_task: manipulator-bring_ball
  simulator: dm_control 1.0.47, MuJoCo 3.14.0
  robouse:
    id: dmcontrol-manipulator-bring-ball
    backend: dmcontrol
    env: dmcontrol-manipulator-bring-ball
    seed: 0
    max_steps: 2000
    camera: fixed
    cameras:
    - fixed
    - hand
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

# Planar manipulator: bring the ball (dm_control)

The planar manipulator of the DeepMind Control Suite (MuJoCo), in the x-z plane (z up, gravity on). An arm is mounted on a hinge at (x, z) = (0, 0.4) above a floor (z = 0) that is bounded by walls sloping up from x = +-0.4 m. Links: 0.18 m (root to shoulder), 0.15 m (shoulder to elbow), 0.12 m (elbow to wrist); the hand's grasp point (`grasp_site_pos`) is 0.065 m past the wrist and the pinch point between the fingertips 0.09 m. Joint angles are in radians; each joint turns about -y, so with 0 every link points straight up and a positive angle turns it towards -x: the direction of link k is (-sin Q, cos Q) with Q the sum of the joint angles up to it. `arm_root` turns without limit; `arm_shoulder` and `arm_elbow` are limited to +-160 degrees, `arm_wrist` to +-140 degrees. The hand has a thumb and a finger. A ball (2.2 cm radius) starts somewhere in the arena, possibly moving; it rolls freely on the floor. Simulated time advances only when you act; the scene is paused while you think. A feedback controller can run as a script: Python 3 with numpy is available, and `robo observe --json` / `robo act ... --json` print machine-readable output.

## Task

Pick up the ball and hold it at the target, a ball-sized ghost at (-0.160, 0.180) (x, z) in mid-air. The ball starts at (-0.365, 0.359) moving at 2.85 m/s along x; it falls to the floor and rolls.

**Success:** the ball's centre within 1 cm of the target's centre (`ball_to_target` <= 0.01 m, where the task's reward is 1), held for 25 consecutive steps (0.25 s of simulated time), judged by the episode server from the simulated state after every step. The episode ends as solved the moment that happens; `robo done` before that scores 0.

**Controls.** `robo act ROOT SHOULDER ELBOW WRIST GRASP [--repeat N]` applies joint torques of 12, 8, 4 and 2 N m per unit (positive increases the joint angle) and a grip force of 2 N per unit on the fingers (positive closes them, negative opens them), each in [-1, 1], for N steps of 10 ms. There are no skills.

**Observation.** `robo observe` reports dm_control's own observation: `arm_pos` (sin and cos of the 8 joints: root, shoulder, elbow, wrist, finger, fingertip, thumb, thumbtip), `arm_vel`, `touch` (palm, finger, thumb, fingertip, thumbtip), and `hand_pos`, `object_pos`, `target_pos` as [x, z, qw, qy] (position and rotation about y), `object_vel`; plus by name: `joint_angles` and `joint_vels` (rad, rad/s), `grasp_site_pos`, `pinch_site_pos`, `ball_site_pos`, `target_ball_site_pos` ((x, z) in m), `ball_angle` and `ball_to_target` (m). Every task also reports `reward` (dm_control's own reward for the last step, for information), `in_target` (whether the success condition holds right now), `hold_steps` (for how many consecutive steps it has held) and `hold_required`, and `time_s` (simulated time). Cameras: `fixed`, `hand`; `robo observe --image [--camera C]` saves a picture.

The step budget is 2000 steps (20 s of simulated time).

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
