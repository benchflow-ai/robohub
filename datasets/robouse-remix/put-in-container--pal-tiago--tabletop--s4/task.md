---
schema_version: '1.3'
task:
  name: robouse/put-in-container--pal-tiago--tabletop--s4
  description: 'Put an object in a container: PAL TIAGo in the tabletop'
metadata:
  author_name: benchflow
  source_benchmark: robouse remix (components)
  source_task: put-in-container--pal-tiago--tabletop--s4
  suite: remix
  category: put-in-container
  difficulty: easy
  tags:
  - remix
  - pal-tiago
  - tabletop
  - tabletop
  - mobile-manipulation
  - pick-and-place
  embodiment: pal-tiago
  scene: tabletop
  scenario: tabletop
  track: mobile-manipulation
  capabilities:
  - pick-and-place
  validation:
    oracle: 1.0
    noop: 0.0
    oracle_steps: 899
  robouse:
    id: put-in-container--pal-tiago--tabletop--s4
    backend: remix
    env: put-in-container--pal-tiago--tabletop--s4
    seed: 4
    max_steps: 2400
    camera: chase
    cameras:
    - overview
    - chase
    - robot/head
    - front
    - top
    skills: true
    success_mode: final
    components:
      embodiment: pal-tiago@0.1.0
      scene: tabletop@0.1.0
      task: put-in-container@0.1.0
      modifiers:
        obs: state
        budget: normal
        perturbation: none
        safety: none
        roles: single
    remix:
      embodiment: pal-tiago
      scene: tabletop
      task: put-in-container
      seed: 4
      modifiers:
        obs: state
        budget: normal
        perturbation: none
        safety: none
        roles: single
      instance:
        objects:
        - name: blue_tall_block
          kind: tall_block
          color: blue
          pos:
          - -0.121
          - -0.257
          - 0.75
          yaw: 11.619671998346913
          'on': table
        - name: green_cup
          kind: cup
          color: green
          pos:
          - 0.809
          - -0.246
          - 0.75
          yaw: -7.7353779417124535
          'on': table
        fixtures:
        - name: tray
          kind: tray
          color: grey
          pos:
          - 0.538
          - -0.276
          - 0.75
          'on': table
        goal:
          object: blue_tall_block
          container: tray
        task: Put the blue tall block in the grey tray. Leave the other objects where they are.
        success: the blue tall block rests inside the grey tray (its centre within the container's inner footprint and below its rim), no part of the robot touches it, and no other object is inside the grey tray
        steps: 2400
  embodied:
    format: robouse
    base_task: put-in-container--pal-tiago--tabletop--s4
    seed: 4
    seed_override: false
    noop: false
agent:
  timeout_sec: 3600
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

# Put an object in a container: PAL TIAGo in the tabletop

You control a PAL TIAGo (standing on the floor at (0, -1.5), facing 90 degrees). A differential-drive mobile manipulator with a torso lift, a 7-DoF arm and a parallel gripper that grasps from the front (level); a pan-tilt head camera. The scene is a 2 m x 0.8 m table in a plain room; arms mount at its rear edge, floor robots reach it from the south, drones take off from a pad in the corner. Everything is simulated with MuJoCo physics: objects are free rigid bodies that move only through contact and friction, so a loose grip or a collision can drop or knock things over. World frame in metres: +x east, +y north, +z up, floor at z = 0 (the floor area spans x -2.5..2.5, y -2.2..1.6); headings in degrees counter-clockwise from +x.

## Task

Put the blue tall block in the grey tray. Leave the other objects where they are.

**Success:** the blue tall block rests inside the grey tray (its centre within the container's inner footprint and below its rim), no part of the robot touches it, and no other object is inside the grey tray; and no safety event happened (a hard collision with furniture or walls, an object falling to the floor, the robot falling over, or harm to a person; `safety_events` in `robo observe` lists them). Judged by the episode server from the simulated state after you call `robo done` and the robot has held still for 10 steps.

**Controls.** `robo act V WZ DX DY DZ G` (6 values; one step is 50 ms; `--repeat N` holds it for N steps). `base.twist` V, WZ (each in [-1, 1]; x 0.45 m/s, x 60 deg/s): forward speed and yaw rate of the differential drive. `arm.ee_delta` DX, DY, DZ (each in [-1, 1]; x 2 cm per step): moves the gripper target (world axes; IK; the gripper stays level and points along the heading). `gripper` G (each in [-1, 1]; rate): > 0 closes, < 0 opens (1 = 10 % of the stroke per step), 0 holds; closing stops squeezing at a bounded force. All zero holds still. Skills (`robo skill NAME ARG ...`; `robo info` lists them with their arguments): `go_to X Y [YAW_DEG=none] [SPEED=0.45]`: drive to (X, Y) in a straight line, then turn to YAW_DEG; it does not plan around obstacles (give it waypoints) and stops if the base is blocked; `turn YAW_DEG`: rotate in place to heading YAW_DEG (0 = +x, 90 = +y); `look_at X Y Z`: point the head camera at (X, Y, Z) (pan and tilt; the base does not move); `move_to X Y Z`: move the grasp point (between the fingertips) to (X, Y, Z) in a straight line with the IK arm (the first call unfolds the arm); reports whether it got there; `grasp`: close the gripper until the fingers stop; reports what is held; `release`: open the gripper fully; `home`: fold the arm in for driving.

**Observation.** `robo observe` reports `robot` (what the robot is, its capabilities and grasp, its workspace, and its current state: base pose, gripper point and opening, head angles), `objects` (per task object: kind, colour, centre `pos`, tilt, yaw, size, mass and `resting_on`: a surface, a fixture, another object, `gripper` or the floor), `fixtures` (containers with their inner size and rim height, pegs, marks, pads, tags), `scene` (surfaces with their top heights and sizes, obstacles, the floor area) and `safety_events`. Cameras (`robo observe --image --camera NAME`): `overview`, `chase`, `robot/head`, `front`, `top`.

The step budget is 2400 steps (120 s of simulated time).

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
