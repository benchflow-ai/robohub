---
schema_version: '1.3'
task:
  name: robouse/put-in-container--aloha-2--tabletop--pert-dynamics--s4
  description: 'Put an object in a container: ALOHA 2 in the tabletop'
metadata:
  author_name: benchflow
  source_benchmark: robouse remix (components)
  source_task: put-in-container--aloha-2--tabletop--pert-dynamics--s4
  suite: remix
  category: put-in-container
  difficulty: easy
  tags:
  - remix
  - aloha-2
  - tabletop
  - tabletop
  - arm
  - pick-and-place
  embodiment: aloha-2
  scene: tabletop
  scenario: tabletop
  track: arm
  capabilities:
  - pick-and-place
  validation:
    oracle: 1.0
    noop: 0.0
    oracle_steps: 175
  robouse:
    id: put-in-container--aloha-2--tabletop--pert-dynamics--s4
    backend: remix
    env: put-in-container--aloha-2--tabletop--pert-dynamics--s4
    seed: 4
    max_steps: 700
    camera: front
    cameras:
    - overview
    - robot/left_wrist
    - robot/right_wrist
    - front
    - top
    skills: true
    success_mode: final
    components:
      embodiment: aloha-2@0.1.0
      scene: tabletop@0.1.0
      task: put-in-container@0.1.0
      modifiers:
        obs: state
        budget: normal
        perturbation: dynamics
        safety: none
        roles: single
    remix:
      embodiment: aloha-2
      scene: tabletop
      task: put-in-container
      seed: 4
      modifiers:
        obs: state
        budget: normal
        perturbation: dynamics
        safety: none
        roles: single
      instance:
        objects:
        - name: blue_block
          kind: block
          color: blue
          pos:
          - 0.434
          - -0.115
          - 0.75
          yaw: 8.919246580503753
          'on': table
        - name: green_block
          kind: block
          color: green
          pos:
          - 0.578
          - -0.055
          - 0.75
          yaw: -1.668089334494085
          'on': table
        fixtures:
        - name: tray
          kind: tray
          color: grey
          pos:
          - 0.123
          - -0.075
          - 0.75
          'on': table
        goal:
          object: blue_block
          container: tray
        task: Put the blue block in the grey tray. Leave the other objects where they are.
        success: the blue block rests inside the grey tray (its centre within the container's inner footprint and below its rim), no part of the robot touches it, and no other object is inside the grey tray
        steps: 700
  embodied:
    format: robouse
    base_task: put-in-container--aloha-2--tabletop--pert-dynamics--s4
    seed: 4
    seed_override: false
    noop: false
agent:
  timeout_sec: 1050
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

# Put an object in a container: ALOHA 2 in the tabletop

You control a ALOHA 2 (two ViperX 300 s arms) (mounted on the table (top at z = 0.75 m), facing south: the left arm at (-0.4, 0.2); the right arm at (0.4, 0.2)). Two ViperX 300 s arms side by side on a work surface; each has its own gripper target and gripper. Together they span more than either arm reaches alone. The scene is a 2 m x 0.8 m table in a plain room; arms mount at its rear edge, floor robots reach it from the south, drones take off from a pad in the corner. Everything is simulated with MuJoCo physics: objects are free rigid bodies that move only through contact and friction, so a loose grip or a collision can drop or knock things over. World frame in metres: +x east, +y north, +z up, floor at z = 0 (the floor area spans x -2.5..2.5, y -2.2..1.6); headings in degrees counter-clockwise from +x.

## Task

Put the blue block in the grey tray. Leave the other objects where they are.

**Success:** the blue block rests inside the grey tray (its centre within the container's inner footprint and below its rim), no part of the robot touches it, and no other object is inside the grey tray; and no safety event happened (a hard collision with furniture or walls, an object falling to the floor, the robot falling over, or harm to a person; `safety_events` in `robo observe` lists them). Judged by the episode server from the simulated state after you call `robo done` and the robot has held still for 10 steps.

**Controls.** `robo act L_DX L_DY L_DZ L_G R_DX R_DY R_DZ R_G` (8 values; one step is 50 ms; `--repeat N` holds it for N steps). `left.ee_delta` L_DX, L_DY, L_DZ (each in [-1, 1]; x 2 cm per step): moves the gripper target (world axes); IK follows it with the gripper pointing down. `left.gripper` L_G (each in [-1, 1]; command): 0 keeps the fingers, > 0 closes, -f opens to fraction f (-1 fully open). `right.ee_delta` R_DX, R_DY, R_DZ (each in [-1, 1]; x 2 cm per step): moves the gripper target (world axes); IK follows it with the gripper pointing down. `right.gripper` R_G (each in [-1, 1]; command): 0 keeps the fingers, > 0 closes, -f opens to fraction f (-1 fully open). All zero holds still. Skills (`robo skill NAME ARG ...`; `robo info` lists them with their arguments): `move_to X Y Z [ARM=auto]`: move the grasp point (between the fingertips) to (X, Y, Z) along a straight line, gripper pointing down; reports whether it got there (it stops if blocked or out of reach); `grasp [ARM=auto]`: close the fingers until they stop; reports what is held; `release [ARM=auto]`: open the fingers fully; `home [ARM=auto]`: move the gripper back to its start point above the work area.

**Observation.** `robo observe` reports `robot` (what the robot is, its capabilities and grasp, its workspace, and its current state: gripper points, targets and openings per arm), `objects` (per task object: kind, colour, centre `pos`, tilt, yaw, size, mass and `resting_on`: a surface, a fixture, another object, `gripper` or the floor), `fixtures` (containers with their inner size and rim height, pegs, marks, pads, tags), `scene` (surfaces with their top heights and sizes, obstacles, the floor area) and `safety_events`. Cameras (`robo observe --image --camera NAME`): `overview`, `robot/left_wrist`, `robot/right_wrist`, `front`, `top`.

The step budget is 700 steps (35 s of simulated time).

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
