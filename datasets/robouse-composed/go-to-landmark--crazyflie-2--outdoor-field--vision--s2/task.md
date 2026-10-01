---
schema_version: '1.3'
task:
  name: robouse/go-to-landmark--crazyflie-2--outdoor-field--vision--s2
  description: 'Go to the mat described by a spatial relation: Bitcraze Crazyflie 2 in the outdoor field'
metadata:
  author_name: benchflow
  source_benchmark: robouse composed task (components)
  source_task: go-to-landmark--crazyflie-2--outdoor-field--vision--s2
  suite: composed
  category: go-to-landmark
  difficulty: easy
  tags:
  - composed
  - crazyflie-2
  - outdoor-field
  - outdoor-field
  - aerial
  - spatial-reasoning
  - navigation
  - perception
  - partial-observability
  embodiment: crazyflie-2
  scene: outdoor-field
  scenario: outdoor-field
  track: aerial
  capabilities:
  - spatial-reasoning
  - navigation
  - perception
  - partial-observability
  validation:
    oracle: 1.0
    noop: 0.0
    oracle_steps: 441
  robouse:
    id: go-to-landmark--crazyflie-2--outdoor-field--vision--s2
    backend: components
    env: go-to-landmark--crazyflie-2--outdoor-field--vision--s2
    seed: 2
    max_steps: 1200
    camera: chase
    cameras:
    - overview
    - chase
    - robot/fpv
    - top
    skills: true
    success_mode: final
    instance:
      embodiment:
        id: crazyflie-2
        version: 0.1.0
      scene:
        id: outdoor-field
        version: 0.1.0
      task:
        id: go-to-landmark
        version: 0.1.0
      modifiers:
        obs: vision
        budget: normal
        perturbation: none
        safety: none
        roles: single
      seed: 2
      layout:
        objects: []
        fixtures:
        - name: yellow_pad
          kind: pad
          color: yellow
          pos:
          - 2.373
          - -1.053
          - 0.0
          half:
          - 0.2
          - 0.2
          'on': floor
        - name: green_pad
          kind: pad
          color: green
          pos:
          - 2.699
          - 1.26
          - 0.0
          half:
          - 0.2
          - 0.2
          'on': floor
        - name: red_pad
          kind: pad
          color: red
          pos:
          - -1.375
          - -1.2
          - 0.0
          half:
          - 0.2
          - 0.2
          'on': floor
        goal:
          target: yellow_pad
          landmark: the solar panel array
        task: Three coloured landing pads lie on the floor. Take off, fly to the pad closest to the solar panel array and land on it. (Closest means the smallest distance between the landing pad's centre and the centre of the solar panel array.)
        success: the drone rests on that pad with its motors off (its centre above the pad), and it never touched anything but the floor, a pad or a surface top
        steps: 1200
    obs_mode: vision
    visible_fields:
    - robot
    - scene
    - safety_events
    - time_s
  embodied:
    format: robouse
    base_task: go-to-landmark--crazyflie-2--outdoor-field--vision--s2
    seed: 2
    seed_override: false
    noop: false
agent:
  timeout_sec: 1800
verifier:
  service: simulator
  user: root
  timeout_sec: 300
sandbox:
  cpus: 1
  memory_mb: 2048
  build_timeout_sec: 1800
oracle:
  env:
    ROBOUSE_ORACLE_TOKEN: ${ROBOUSE_ORACLE_TOKEN:-}
---

You are controlling a simulated robot. Read the task below, then solve it by running the `robo` command in your shell (start with `robo info` and `robo observe`). Keep going until the task is done, then call `robo done` once. Do not stop to ask questions; there is no human to answer.

# Go to the mat described by a spatial relation: Bitcraze Crazyflie 2 in the outdoor field

You control a Bitcraze Crazyflie 2 (resting on its take-off pad at (-4.2, -3.4) with the motors off). A 27 g quadrotor with a cascaded velocity and attitude controller and a forward camera; it cannot carry anything. The scene is a 10 m x 8 m grass field with a shed, a boulder, a tree and a solar panel array; no work surfaces, so no arm mounts. Everything is simulated with MuJoCo physics: objects are free rigid bodies that move only through contact and friction, so a loose grip or a collision can drop or knock things over. World frame in metres: +x east, +y north, +z up, floor at z = 0 (the floor area spans x -5..5, y -4..4); headings in degrees counter-clockwise from +x.

## Task

Three coloured landing pads lie on the floor. Take off, fly to the pad closest to the solar panel array and land on it. (Closest means the smallest distance between the landing pad's centre and the centre of the solar panel array.)

**Success:** the drone rests on that pad with its motors off (its centre above the pad), and it never touched anything but the floor, a pad or a surface top; and no safety event happened (a hard collision with furniture or walls, an object falling to the floor, the robot falling over, or harm to a person; `safety_events` in `robo observe` lists them). Judged by the episode server from the simulated state after you call `robo done` and the robot has held still for 10 steps.

**Controls.** `robo act VX VY VZ YAW_RATE` (4 values; one step is 50 ms; `--repeat N` holds it for N steps). `velocity_setpoint` VX, VY, VZ, YAW_RATE (each in [-1, 1]; x 1 m/s, x 1 m/s, x 1 m/s, x 90 deg/s): world-frame velocity setpoint and yaw rate; all zero = hold position (or stay landed). All zero holds still. Skills (`robo skill NAME ARG ...`; `robo info` lists them with their arguments): `takeoff [Z=1.0]`: climb straight up to height Z and hover; `fly_to X Y Z`: fly in a straight line to (X, Y, Z) and hover; not obstacle-aware (give it waypoints); `land`: descend where it is until it touches down; the motors stop on contact; `turn YAW_DEG`: turn in place to heading YAW_DEG; `look_at X Y Z`: turn to face (X, Y) so the forward camera looks at it, then hover for a second.

**Observation.** `robo observe` reports `robot` (what the robot is, its capabilities and grasp, its workspace, and its current state: position, heading and motion state), `objects` (per task object: kind, colour, centre `pos`, tilt, yaw, size, mass and `resting_on`: a surface, a fixture, another object, `gripper` or the floor), `fixtures` (containers with their inner size and rim height, pegs, marks, pads, tags), `scene` (surfaces with their top heights and sizes, obstacles, the floor area) and `safety_events`. In this task `robo observe` does not report `objects` or `fixtures`: find them in the camera images (`robo info` gives each camera's calibration where it is fixed). Cameras (`robo observe --image --camera NAME`): `overview`, `chase`, `robot/fpv`, `top`.

The step budget is 1200 steps (60 s of simulated time).

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
