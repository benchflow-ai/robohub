---
schema_version: '1.3'
task:
  name: robouse/inspect-tag--crazyflie-2--lab-bench--s3
  description: 'Read the number on an inspection tag: Bitcraze Crazyflie 2 in the lab bench'
metadata:
  author_name: benchflow
  source_benchmark: robouse remix (components)
  source_task: inspect-tag--crazyflie-2--lab-bench--s3
  suite: remix
  category: inspect-tag
  difficulty: medium
  tags:
  - remix
  - crazyflie-2
  - lab-bench
  - lab
  - aerial
  - perception
  - partial-observability
  - navigation
  embodiment: crazyflie-2
  scene: lab-bench
  scenario: lab
  track: aerial
  capabilities:
  - perception
  - partial-observability
  - navigation
  validation:
    oracle: 1.0
    noop: 0.0
    oracle_steps: 279
  robouse:
    id: inspect-tag--crazyflie-2--lab-bench--s3
    backend: remix
    env: inspect-tag--crazyflie-2--lab-bench--s3
    seed: 3
    max_steps: 1400
    camera: chase
    cameras:
    - overview
    - chase
    - robot/fpv
    - bench_cam
    - top
    skills: true
    success_mode: final
    components:
      embodiment: crazyflie-2@0.1.0
      scene: lab-bench@0.1.0
      task: inspect-tag@0.1.0
      modifiers:
        obs: state
        budget: normal
        perturbation: none
        safety: none
        roles: single
    remix:
      embodiment: crazyflie-2
      scene: lab-bench
      task: inspect-tag
      seed: 3
      modifiers:
        obs: state
        budget: normal
        perturbation: none
        safety: none
        roles: single
      instance:
        objects: []
        fixtures:
        - name: tag
          kind: tag
          color: yellow
          pos:
          - 2.35
          - 0.7
          - 0.9
          normal:
          - -1
          - 0
          reading: 830
          label: the yellow tag on the east wall above the side bench
          post: false
        goal:
          tag: tag
          reading: 830
        task: A yellow inspection tag hangs on the east wall above the side bench; a three-digit number is printed on it. Go and look at it with your camera, read the number, and report it in the `robo done` message (for example `robo done "the tag reads 123"`).
        success: the tag was seen by the robot's own camera (its centre in the middle 70 % of the image, facing the camera, at most 2.5 m away, not blocked) for at least one second, and the `robo done` message contains the number
        steps: 1400
  embodied:
    format: robouse
    base_task: inspect-tag--crazyflie-2--lab-bench--s3
    seed: 3
    seed_override: false
    noop: false
agent:
  timeout_sec: 2100
verifier:
  service: simulator
  user: root
  timeout_sec: 300
sandbox:
  cpus: 1
  memory_mb: 2048
  build_timeout_sec: 1800
---

You are controlling a simulated robot. Read the task below, then solve it by running the `robo` command in your shell (start with `robo info` and `robo observe`). Keep going until the task is done, then call `robo done` once. Do not stop to ask questions; there is no human to answer.

# Read the number on an inspection tag: Bitcraze Crazyflie 2 in the lab bench

You control a Bitcraze Crazyflie 2 (resting on its take-off pad at (-0.8, -1.6) with the motors off). A 27 g quadrotor with a cascaded velocity and attitude controller and a forward camera; it cannot carry anything. The scene is a laboratory: a white bench, a side bench, a fume hood and a sample freezer. Everything is simulated with MuJoCo physics: objects are free rigid bodies that move only through contact and friction, so a loose grip or a collision can drop or knock things over. World frame in metres: +x east, +y north, +z up, floor at z = 0 (the floor area spans x -2.4..2.4, y -2..1.85); headings in degrees counter-clockwise from +x.

## Task

A yellow inspection tag hangs on the east wall above the side bench; a three-digit number is printed on it. Go and look at it with your camera, read the number, and report it in the `robo done` message (for example `robo done "the tag reads 123"`).

**Success:** the tag was seen by the robot's own camera (its centre in the middle 70 % of the image, facing the camera, at most 2.5 m away, not blocked) for at least one second, and the `robo done` message contains the number; and no safety event happened (a hard collision with furniture or walls, an object falling to the floor, the robot falling over, or harm to a person; `safety_events` in `robo observe` lists them). Judged by the episode server from the simulated state after you call `robo done` and the robot has held still for 10 steps.

**Controls.** `robo act VX VY VZ YAW_RATE` (4 values; one step is 50 ms; `--repeat N` holds it for N steps). `velocity_setpoint` VX, VY, VZ, YAW_RATE (each in [-1, 1]; x 1 m/s, x 1 m/s, x 1 m/s, x 90 deg/s): world-frame velocity setpoint and yaw rate; all zero = hold position (or stay landed). All zero holds still. Skills (`robo skill NAME ARG ...`; `robo info` lists them with their arguments): `takeoff [Z=1.0]`: climb straight up to height Z and hover; `fly_to X Y Z`: fly in a straight line to (X, Y, Z) and hover; not obstacle-aware (give it waypoints); `land`: descend where it is until it touches down; the motors stop on contact; `turn YAW_DEG`: turn in place to heading YAW_DEG; `look_at X Y Z`: turn to face (X, Y) so the forward camera looks at it, then hover for a second.

**Observation.** `robo observe` reports `robot` (what the robot is, its capabilities and grasp, its workspace, and its current state: position, heading and motion state), `objects` (per task object: kind, colour, centre `pos`, tilt, yaw, size, mass and `resting_on`: a surface, a fixture, another object, `gripper` or the floor), `fixtures` (containers with their inner size and rim height, pegs, marks, pads, tags), `scene` (surfaces with their top heights and sizes, obstacles, the floor area) and `safety_events`. Cameras (`robo observe --image --camera NAME`): `overview`, `chase`, `robot/fpv`, `bench_cam`, `top`.

The step budget is 1400 steps (70 s of simulated time).

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
