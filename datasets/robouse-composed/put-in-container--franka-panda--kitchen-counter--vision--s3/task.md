---
schema_version: '1.3'
task:
  name: robouse/put-in-container--franka-panda--kitchen-counter--vision--s3
  description: 'Put an object in a container: Franka Emika Panda in the kitchen counter'
metadata:
  author_name: benchflow
  source_benchmark: robouse composed task (components)
  source_task: put-in-container--franka-panda--kitchen-counter--vision--s3
  suite: composed
  category: put-in-container
  difficulty: easy
  tags:
  - composed
  - franka-panda
  - kitchen-counter
  - household-kitchen
  - arm
  - pick-and-place
  - perception
  - partial-observability
  embodiment: franka-panda
  scene: kitchen-counter
  scenario: household-kitchen
  track: arm
  capabilities:
  - pick-and-place
  - perception
  - partial-observability
  validation:
    oracle: 1.0
    noop: 0.0
    oracle_steps: 171
  robouse:
    id: put-in-container--franka-panda--kitchen-counter--vision--s3
    backend: components
    env: put-in-container--franka-panda--kitchen-counter--vision--s3
    seed: 3
    max_steps: 700
    camera: counter_cam
    cameras:
    - overview
    - robot/arm_wrist
    - counter_cam
    - top
    skills: true
    success_mode: final
    instance:
      embodiment:
        id: franka-panda
        version: 0.1.0
      scene:
        id: kitchen-counter
        version: 0.1.0
      task:
        id: put-in-container
        version: 0.1.0
      modifiers:
        obs: vision
        budget: normal
        perturbation: none
        safety: none
        roles: single
      seed: 3
      layout:
        objects:
        - name: purple_block
          kind: block
          color: purple
          pos:
          - -0.018
          - 1.608
          - 0.9
          yaw: 3.565132969915801
          'on': counter
        - name: red_block
          kind: block
          color: red
          pos:
          - -0.236
          - 1.61
          - 0.9
          yaw: -11.964237995787933
          'on': counter
        fixtures:
        - name: tote
          kind: tote
          color: grey
          pos:
          - -0.546
          - 1.685
          - 0.9
          'on': counter
        goal:
          object: purple_block
          container: tote
        task: Put the purple block in the grey tote. Leave the other objects where they are.
        success: the purple block rests inside the grey tote (its centre within the container's inner footprint and below its rim), no part of the robot touches it, and no other object is inside the grey tote
        steps: 700
    obs_mode: vision
    visible_fields:
    - robot
    - scene
    - safety_events
    - time_s
  embodied:
    format: robouse
    base_task: put-in-container--franka-panda--kitchen-counter--vision--s3
    seed: 3
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
oracle:
  env:
    ROBOUSE_ORACLE_TOKEN: ${ROBOUSE_ORACLE_TOKEN:-}
---

You are controlling a simulated robot. Read the task below, then solve it by running the `robo` command in your shell (start with `robo info` and `robo observe`). Keep going until the task is done, then call `robo done` once. Do not stop to ask questions; there is no human to answer.

# Put an object in a container: Franka Emika Panda in the kitchen counter

You control a Franka Emika Panda (mounted on the kitchen counter (top at z = 0.9 m), facing south: its base at (-0.4, 2.05)). A 7-DoF arm with a parallel gripper, mounted on a work surface; Cartesian gripper-target control through IK, top-down grasps, a wrist camera. The scene is a kitchen: a 0.9 m counter with a stove along the north wall, a dining table, a fridge and a chair. Arms mount at the back of the counter. Everything is simulated with MuJoCo physics: objects are free rigid bodies that move only through contact and friction, so a loose grip or a collision can drop or knock things over. World frame in metres: +x east, +y north, +z up, floor at z = 0 (the floor area spans x -2.5..2.5, y -2.2..2.45); headings in degrees counter-clockwise from +x.

## Task

Put the purple block in the grey tote. Leave the other objects where they are.

**Success:** the purple block rests inside the grey tote (its centre within the container's inner footprint and below its rim), no part of the robot touches it, and no other object is inside the grey tote; and no safety event happened (a hard collision with furniture or walls, an object falling to the floor, the robot falling over, or harm to a person; `safety_events` in `robo observe` lists them). Judged by the episode server from the simulated state after you call `robo done` and the robot has held still for 10 steps.

**Controls.** `robo act DX DY DZ G` (4 values; one step is 50 ms; `--repeat N` holds it for N steps). `arm.ee_delta` DX, DY, DZ (each in [-1, 1]; x 2 cm per step): moves the gripper target (world axes); IK follows it with the gripper pointing down. `gripper` G (each in [-1, 1]; command): 0 keeps the fingers, > 0 closes, -f opens to fraction f (-1 fully open). All zero holds still. Skills (`robo skill NAME ARG ...`; `robo info` lists them with their arguments): `move_to X Y Z`: move the grasp point (between the fingertips) to (X, Y, Z) along a straight line, gripper pointing down; reports whether it got there (it stops if blocked or out of reach); `grasp`: close the fingers until they stop; reports what is held; `release`: open the fingers fully; `home`: move the gripper back to its start point above the work area.

**Observation.** `robo observe` reports `robot` (what the robot is, its capabilities and grasp, its workspace, and its current state: gripper points, targets and openings per arm), `objects` (per task object: kind, colour, centre `pos`, tilt, yaw, size, mass and `resting_on`: a surface, a fixture, another object, `gripper` or the floor), `fixtures` (containers with their inner size and rim height, pegs, marks, pads, tags), `scene` (surfaces with their top heights and sizes, obstacles, the floor area) and `safety_events`. In this task `robo observe` does not report `objects` or `fixtures`: find them in the camera images (`robo info` gives each camera's calibration where it is fixed). Cameras (`robo observe --image --camera NAME`): `overview`, `robot/arm_wrist`, `counter_cam`, `top`.

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
