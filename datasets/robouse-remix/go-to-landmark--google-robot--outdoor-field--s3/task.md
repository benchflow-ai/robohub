---
schema_version: '1.3'
task:
  name: robouse/go-to-landmark--google-robot--outdoor-field--s3
  description: 'Go to the mat described by a spatial relation: Google Robot in the outdoor field'
metadata:
  author_name: benchflow
  source_benchmark: robouse remix (components)
  source_task: go-to-landmark--google-robot--outdoor-field--s3
  suite: remix
  category: go-to-landmark
  difficulty: easy
  tags:
  - remix
  - google-robot
  - outdoor-field
  - outdoor-field
  - mobile-manipulation
  - spatial-reasoning
  - navigation
  embodiment: google-robot
  scene: outdoor-field
  scenario: outdoor-field
  track: mobile-manipulation
  capabilities:
  - spatial-reasoning
  - navigation
  validation:
    oracle: 1.0
    noop: 0.0
    oracle_steps: 312
  robouse:
    id: go-to-landmark--google-robot--outdoor-field--s3
    backend: remix
    env: go-to-landmark--google-robot--outdoor-field--s3
    seed: 3
    max_steps: 1200
    camera: chase
    cameras:
    - overview
    - chase
    - robot/head
    - top
    skills: true
    success_mode: final
    components:
      embodiment: google-robot@0.1.0
      scene: outdoor-field@0.1.0
      task: go-to-landmark@0.1.0
      modifiers:
        obs: state
        budget: normal
        perturbation: none
        safety: none
        roles: single
    remix:
      embodiment: google-robot
      scene: outdoor-field
      task: go-to-landmark
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
        - name: yellow_mat
          kind: mat
          color: yellow
          pos:
          - -1.336
          - -1.312
          - 0.0
          half:
          - 0.3
          - 0.3
          'on': floor
        - name: green_mat
          kind: mat
          color: green
          pos:
          - -0.22
          - 2.566
          - 0.0
          half:
          - 0.3
          - 0.3
          'on': floor
        - name: blue_mat
          kind: mat
          color: blue
          pos:
          - 2.19
          - -1.018
          - 0.0
          half:
          - 0.3
          - 0.3
          'on': floor
        goal:
          target: green_mat
          landmark: the tool shed
        task: Three coloured mats lie on the floor. Go to the mat closest to the tool shed and stop on it. (Closest means the smallest distance between the mat's centre and the centre of the tool shed.)
        success: the robot's base centre is on that mat and the robot is standing still (not stepping or driving)
        steps: 1200
  embodied:
    format: robouse
    base_task: go-to-landmark--google-robot--outdoor-field--s3
    seed: 3
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
---

You are controlling a simulated robot. Read the task below, then solve it by running the `robo` command in your shell (start with `robo info` and `robo observe`). Keep going until the task is done, then call `robo done` once. Do not stop to ask questions; there is no human to answer.

# Go to the mat described by a spatial relation: Google Robot in the outdoor field

You control a Google Robot (on a holonomic base) (standing on the floor at (-3.8, -3.2), facing 45 degrees). The Google (RT-1) robot's arm and gripper on a holonomic planar base added in robouse; front grasps; a head camera. The scene is a 10 m x 8 m grass field with a shed, a boulder, a tree and a solar panel array; no work surfaces, so no arm mounts. Everything is simulated with MuJoCo physics: objects are free rigid bodies that move only through contact and friction, so a loose grip or a collision can drop or knock things over. World frame in metres: +x east, +y north, +z up, floor at z = 0 (the floor area spans x -5..5, y -4..4); headings in degrees counter-clockwise from +x.

## Task

Three coloured mats lie on the floor. Go to the mat closest to the tool shed and stop on it. (Closest means the smallest distance between the mat's centre and the centre of the tool shed.)

**Success:** the robot's base centre is on that mat and the robot is standing still (not stepping or driving); and no safety event happened (a hard collision with furniture or walls, an object falling to the floor, the robot falling over, or harm to a person; `safety_events` in `robo observe` lists them). Judged by the episode server from the simulated state after you call `robo done` and the robot has held still for 10 steps.

**Controls.** `robo act VX VY WZ DX DY DZ G` (7 values; one step is 50 ms; `--repeat N` holds it for N steps). `base.twist` VX, VY, WZ (each in [-1, 1]; x 0.5 m/s, x 0.5 m/s, x 60 deg/s): forward, leftward and yaw-rate command of the holonomic base (robot frame). `arm.ee_delta` DX, DY, DZ (each in [-1, 1]; x 2 cm per step): moves the gripper target (world axes; IK; the gripper stays level and points along the heading). `gripper` G (each in [-1, 1]; rate): > 0 closes, < 0 opens (1 = 10 % of the stroke per step), 0 holds; closing stops squeezing at a bounded force. All zero holds still. Skills (`robo skill NAME ARG ...`; `robo info` lists them with their arguments): `go_to X Y [YAW_DEG=none] [SPEED=0.5]`: drive to (X, Y) in a straight line, then turn to YAW_DEG; it does not plan around obstacles (give it waypoints) and stops if the base is blocked; `turn YAW_DEG`: rotate in place to heading YAW_DEG (0 = +x, 90 = +y); `look_at X Y Z`: point the head camera at (X, Y, Z) (pan and tilt; the base does not move); `move_to X Y Z`: move the grasp point (between the fingertips) to (X, Y, Z) in a straight line with the IK arm (the first call unfolds the arm); reports whether it got there; `grasp`: close the gripper until the fingers stop; reports what is held; `release`: open the gripper fully; `home`: fold the arm in for driving.

**Observation.** `robo observe` reports `robot` (what the robot is, its capabilities and grasp, its workspace, and its current state: base pose, gripper point and opening, head angles), `objects` (per task object: kind, colour, centre `pos`, tilt, yaw, size, mass and `resting_on`: a surface, a fixture, another object, `gripper` or the floor), `fixtures` (containers with their inner size and rim height, pegs, marks, pads, tags), `scene` (surfaces with their top heights and sizes, obstacles, the floor area) and `safety_events`. Cameras (`robo observe --image --camera NAME`): `overview`, `chase`, `robot/head`, `top`.

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
