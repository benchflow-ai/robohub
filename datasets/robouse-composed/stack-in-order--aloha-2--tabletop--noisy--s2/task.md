---
schema_version: '1.3'
task:
  name: robouse/stack-in-order--aloha-2--tabletop--noisy--s2
  description: 'Stack blocks in a stated order: ALOHA 2 in the tabletop'
metadata:
  author_name: benchflow
  source_benchmark: robouse composed task (components)
  source_task: stack-in-order--aloha-2--tabletop--noisy--s2
  suite: composed
  category: stack-in-order
  difficulty: medium
  tags:
  - composed
  - aloha-2
  - tabletop
  - tabletop
  - arm
  - spatial-reasoning
  - pick-and-place
  embodiment: aloha-2
  scene: tabletop
  scenario: tabletop
  track: arm
  capabilities:
  - spatial-reasoning
  - pick-and-place
  validation:
    oracle: 1.0
    noop: 0.0
    oracle_steps: 956
  robouse:
    id: stack-in-order--aloha-2--tabletop--noisy--s2
    backend: components
    env: stack-in-order--aloha-2--tabletop--noisy--s2
    seed: 2
    max_steps: 1400
    camera: front
    cameras:
    - overview
    - robot/left_wrist
    - robot/right_wrist
    - front
    - top
    skills: true
    success_mode: final
    instance:
      embodiment:
        id: aloha-2
        version: 0.1.0
      scene:
        id: tabletop
        version: 0.1.0
      task:
        id: stack-in-order
        version: 0.1.0
      modifiers:
        obs: noisy
        budget: normal
        perturbation: none
        safety: none
        roles: single
      seed: 2
      layout:
        objects:
        - name: yellow_block
          kind: block
          color: yellow
          pos:
          - -0.323
          - -0.019
          - 0.75
          yaw: 1.494375906730272
          'on': table
        - name: green_block
          kind: block
          color: green
          pos:
          - 0.243
          - -0.065
          - 0.75
          yaw: 3.1964255825787937
          'on': table
        - name: purple_block
          kind: block
          color: purple
          pos:
          - 0.671
          - -0.013
          - 0.75
          yaw: -2.601004006079372
          'on': table
        fixtures:
        - name: base_mark
          kind: zone
          color: black
          pos:
          - -0.531
          - -0.03
          - 0.75
          half:
          - 0.045
          - 0.045
          'on': table
        goal:
          order:
          - yellow_block
          - purple_block
          - green_block
          mark: base_mark
          object: yellow_block
        task: 'Build a tower of the three blocks on the black mark: the yellow block at the bottom, the green block on top, and the remaining block in the middle.'
        success: the bottom block rests on the surface with its centre on the black mark (within 3 cm), each other block rests on the one below it (centres within 2.5 cm horizontally, touching), every block is level (tilted less than 15 degrees) and the robot touches none of them
        steps: 1400
  embodied:
    format: robouse
    base_task: stack-in-order--aloha-2--tabletop--noisy--s2
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

# Stack blocks in a stated order: ALOHA 2 in the tabletop

You control a ALOHA 2 (two ViperX 300 s arms) (mounted on the table (top at z = 0.75 m), facing south: the left arm at (-0.4, 0.2); the right arm at (0.4, 0.2)). Two ViperX 300 s arms side by side on a work surface; each has its own gripper target and gripper. Together they span more than either arm reaches alone. The scene is a 2 m x 0.8 m table in a plain room; arms mount at its rear edge, floor robots reach it from the south, drones take off from a pad in the corner. Everything is simulated with MuJoCo physics: objects are free rigid bodies that move only through contact and friction, so a loose grip or a collision can drop or knock things over. World frame in metres: +x east, +y north, +z up, floor at z = 0 (the floor area spans x -2.5..2.5, y -2.2..1.6); headings in degrees counter-clockwise from +x.

## Task

Build a tower of the three blocks on the black mark: the yellow block at the bottom, the green block on top, and the remaining block in the middle.

**Success:** the bottom block rests on the surface with its centre on the black mark (within 3 cm), each other block rests on the one below it (centres within 2.5 cm horizontally, touching), every block is level (tilted less than 15 degrees) and the robot touches none of them; and no safety event happened (a hard collision with furniture or walls, an object falling to the floor, the robot falling over, or harm to a person; `safety_events` in `robo observe` lists them). Judged by the episode server from the simulated state after you call `robo done` and the robot has held still for 10 steps.

**Controls.** `robo act L_DX L_DY L_DZ L_G R_DX R_DY R_DZ R_G` (8 values; one step is 50 ms; `--repeat N` holds it for N steps). `left.ee_delta` L_DX, L_DY, L_DZ (each in [-1, 1]; x 2 cm per step): moves the gripper target (world axes); IK follows it with the gripper pointing down. `left.gripper` L_G (each in [-1, 1]; command): 0 keeps the fingers, > 0 closes, -f opens to fraction f (-1 fully open). `right.ee_delta` R_DX, R_DY, R_DZ (each in [-1, 1]; x 2 cm per step): moves the gripper target (world axes); IK follows it with the gripper pointing down. `right.gripper` R_G (each in [-1, 1]; command): 0 keeps the fingers, > 0 closes, -f opens to fraction f (-1 fully open). All zero holds still. Skills (`robo skill NAME ARG ...`; `robo info` lists them with their arguments): `move_to X Y Z [ARM=auto]`: move the grasp point (between the fingertips) to (X, Y, Z) along a straight line, gripper pointing down; reports whether it got there (it stops if blocked or out of reach); `grasp [ARM=auto]`: close the fingers until they stop; reports what is held; `release [ARM=auto]`: open the fingers fully; `home [ARM=auto]`: move the gripper back to its start point above the work area.

**Observation.** `robo observe` reports `robot` (what the robot is, its capabilities and grasp, its workspace, and its current state: gripper points, targets and openings per arm), `objects` (per task object: kind, colour, centre `pos`, tilt, yaw, size, mass and `resting_on`: a surface, a fixture, another object, `gripper` or the floor), `fixtures` (containers with their inner size and rim height, pegs, marks, pads, tags), `scene` (surfaces with their top heights and sizes, obstacles, the floor area) and `safety_events`. Object and fixture positions carry about 1.5 cm of sensor noise; each observation draws new noise. Cameras (`robo observe --image --camera NAME`): `overview`, `robot/left_wrist`, `robot/right_wrist`, `front`, `top`.

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
