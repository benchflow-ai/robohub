---
schema_version: '1.3'
task:
  name: robouse/inspect-tag--pal-tiago--kitchen-counter--s2
  description: 'Read the number on an inspection tag: PAL TIAGo in the kitchen counter'
metadata:
  author_name: benchflow
  source_benchmark: robouse composed task (components)
  source_task: inspect-tag--pal-tiago--kitchen-counter--s2
  suite: composed
  category: inspect-tag
  difficulty: medium
  tags:
  - composed
  - pal-tiago
  - kitchen-counter
  - household-kitchen
  - mobile-manipulation
  - perception
  - partial-observability
  - navigation
  embodiment: pal-tiago
  scene: kitchen-counter
  scenario: household-kitchen
  track: mobile-manipulation
  capabilities:
  - perception
  - partial-observability
  - navigation
  validation:
    oracle: 1.0
    noop: 0.0
    oracle_steps: 260
  robouse:
    id: inspect-tag--pal-tiago--kitchen-counter--s2
    backend: components
    env: inspect-tag--pal-tiago--kitchen-counter--s2
    seed: 2
    max_steps: 1400
    camera: chase
    cameras:
    - overview
    - chase
    - robot/head
    - counter_cam
    - top
    skills: true
    success_mode: final
    instance:
      embodiment:
        id: pal-tiago
        version: 0.1.0
      scene:
        id: kitchen-counter
        version: 0.1.0
      task:
        id: inspect-tag
        version: 0.1.0
      modifiers:
        obs: state
        budget: normal
        perturbation: none
        safety: none
        roles: single
      seed: 2
      layout:
        objects: []
        fixtures:
        - name: tag
          kind: tag
          color: yellow
          pos:
          - -2.45
          - -0.2
          - 0.7
          normal:
          - 1
          - 0
          reading: 335
          label: the yellow tag on the west wall
          post: false
        goal:
          tag: tag
          reading: 335
        task: A yellow inspection tag hangs on the west wall; a three-digit number is printed on it. Go and look at it with your camera, read the number, and report it in the `robo done` message (for example `robo done "the tag reads 123"`).
        success: the tag was seen by the robot's own camera (its centre in the middle 70 % of the image, facing the camera, at most 2.5 m away, not blocked) for at least one second, and the `robo done` message contains the number
        steps: 1400
  embodied:
    format: robouse
    base_task: inspect-tag--pal-tiago--kitchen-counter--s2
    seed: 2
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
oracle:
  env:
    ROBOUSE_ORACLE_TOKEN: ${ROBOUSE_ORACLE_TOKEN:-}
---

You are controlling a simulated robot. Read the task below, then solve it by running the `robo` command in your shell (start with `robo info` and `robo observe`). Keep going until the task is done, then call `robo done` once. Do not stop to ask questions; there is no human to answer.

# Read the number on an inspection tag: PAL TIAGo in the kitchen counter

You control a PAL TIAGo (standing on the floor at (-0.6, 0.35), facing 0 degrees). A differential-drive mobile manipulator with a torso lift, a 7-DoF arm and a parallel gripper that grasps from the front (level); a pan-tilt head camera. The scene is a kitchen: a 0.9 m counter with a stove along the north wall, a dining table, a fridge and a chair. Arms mount at the back of the counter. Everything is simulated with MuJoCo physics: objects are free rigid bodies that move only through contact and friction, so a loose grip or a collision can drop or knock things over. World frame in metres: +x east, +y north, +z up, floor at z = 0 (the floor area spans x -2.5..2.5, y -2.2..2.45); headings in degrees counter-clockwise from +x.

## Task

A yellow inspection tag hangs on the west wall; a three-digit number is printed on it. Go and look at it with your camera, read the number, and report it in the `robo done` message (for example `robo done "the tag reads 123"`).

**Success:** the tag was seen by the robot's own camera (its centre in the middle 70 % of the image, facing the camera, at most 2.5 m away, not blocked) for at least one second, and the `robo done` message contains the number; and no safety event happened (a hard collision with furniture or walls, an object falling to the floor, the robot falling over, or harm to a person; `safety_events` in `robo observe` lists them). Judged by the episode server from the simulated state after you call `robo done` and the robot has held still for 10 steps.

**Controls.** `robo act V WZ DX DY DZ G` (6 values; one step is 50 ms; `--repeat N` holds it for N steps). `base.twist` V, WZ (each in [-1, 1]; x 0.45 m/s, x 60 deg/s): forward speed and yaw rate of the differential drive. `arm.ee_delta` DX, DY, DZ (each in [-1, 1]; x 2 cm per step): moves the gripper target (world axes; IK; the gripper stays level and points along the heading). `gripper` G (each in [-1, 1]; rate): > 0 closes, < 0 opens (1 = 10 % of the stroke per step), 0 holds; closing stops squeezing at a bounded force. All zero holds still. Skills (`robo skill NAME ARG ...`; `robo info` lists them with their arguments): `go_to X Y [YAW_DEG=none] [SPEED=0.45]`: drive to (X, Y) in a straight line, then turn to YAW_DEG; it does not plan around obstacles (give it waypoints) and stops if the base is blocked; `turn YAW_DEG`: rotate in place to heading YAW_DEG (0 = +x, 90 = +y); `look_at X Y Z`: point the head camera at (X, Y, Z) (pan and tilt; the base does not move); `move_to X Y Z`: move the grasp point (between the fingertips) to (X, Y, Z) in a straight line with the IK arm (the first call unfolds the arm); reports whether it got there; `grasp`: close the gripper until the fingers stop; reports what is held; `release`: open the gripper fully; `home`: fold the arm in for driving.

**Observation.** `robo observe` reports `robot` (what the robot is, its capabilities and grasp, its workspace, and its current state: base pose, gripper point and opening, head angles), `objects` (per task object: kind, colour, centre `pos`, tilt, yaw, size, mass and `resting_on`: a surface, a fixture, another object, `gripper` or the floor), `fixtures` (containers with their inner size and rim height, pegs, marks, pads, tags), `scene` (surfaces with their top heights and sizes, obstacles, the floor area) and `safety_events`. Cameras (`robo observe --image --camera NAME`): `overview`, `chase`, `robot/head`, `counter_cam`, `top`.

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
