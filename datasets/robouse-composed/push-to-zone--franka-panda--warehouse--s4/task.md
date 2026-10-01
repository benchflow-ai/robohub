---
schema_version: '1.3'
task:
  name: robouse/push-to-zone--franka-panda--warehouse--s4
  description: 'Push an object that cannot be grasped into a marked zone: Franka Emika Panda in the warehouse aisle'
metadata:
  author_name: benchflow
  source_benchmark: robouse composed task (components)
  source_task: push-to-zone--franka-panda--warehouse--s4
  suite: composed
  category: push-to-zone
  difficulty: medium
  tags:
  - composed
  - franka-panda
  - warehouse
  - warehouse
  - arm
  - contact-rich
  embodiment: franka-panda
  scene: warehouse
  scenario: warehouse
  track: arm
  capabilities:
  - contact-rich
  validation:
    oracle: 1.0
    noop: 0.0
    oracle_steps: 117
  robouse:
    id: push-to-zone--franka-panda--warehouse--s4
    backend: components
    env: push-to-zone--franka-panda--warehouse--s4
    seed: 4
    max_steps: 900
    camera: station_cam
    cameras:
    - overview
    - robot/arm_wrist
    - station_cam
    - top
    skills: true
    success_mode: final
    instance:
      embodiment:
        id: franka-panda
        version: 0.1.0
      scene:
        id: warehouse
        version: 0.1.0
      task:
        id: push-to-zone
        version: 0.1.0
      modifiers:
        obs: state
        budget: normal
        perturbation: none
        safety: none
        roles: single
      seed: 4
      layout:
        objects:
        - name: crate
          kind: crate
          color: cardboard
          pos:
          - 2.156
          - -1.684
          - 0.85
          yaw: 0.0
          'on': packing_station
        fixtures:
        - name: target_zone
          kind: zone
          color: green
          pos:
          - 2.466
          - -1.743
          - 0.85
          half:
          - 0.1
          - 0.08
          'on': packing_station
        goal:
          object: crate
          zone: target_zone
        task: The crate is too wide to grasp. Push it into the green zone.
        success: the centre of the crate is inside the green zone, it is upright (tilted less than 15 degrees) and nothing of the robot touches it
        steps: 900
  embodied:
    format: robouse
    base_task: push-to-zone--franka-panda--warehouse--s4
    seed: 4
    seed_override: false
    noop: false
agent:
  timeout_sec: 1350
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

# Push an object that cannot be grasped into a marked zone: Franka Emika Panda in the warehouse aisle

You control a Franka Emika Panda (mounted on the packing station (top at z = 0.85 m), facing south: its base at (2.2, -1.32)). A 7-DoF arm with a parallel gripper, mounted on a work surface; Cartesian gripper-target control through IK, top-down grasps, a wrist camera. The scene is a warehouse aisle: two blue racks with shelves at 0.6 m and 0.95 m, a packing station, a stack of pallets and a steel column; a 4 m ceiling. Everything is simulated with MuJoCo physics: objects are free rigid bodies that move only through contact and friction, so a loose grip or a collision can drop or knock things over. World frame in metres: +x east, +y north, +z up, floor at z = 0 (the floor area spans x -3.5..3.5, y -2.6..2); headings in degrees counter-clockwise from +x.

## Task

The crate is too wide to grasp. Push it into the green zone.

**Success:** the centre of the crate is inside the green zone, it is upright (tilted less than 15 degrees) and nothing of the robot touches it; and no safety event happened (a hard collision with furniture or walls, an object falling to the floor, the robot falling over, or harm to a person; `safety_events` in `robo observe` lists them). Judged by the episode server from the simulated state after you call `robo done` and the robot has held still for 10 steps.

**Controls.** `robo act DX DY DZ G` (4 values; one step is 50 ms; `--repeat N` holds it for N steps). `arm.ee_delta` DX, DY, DZ (each in [-1, 1]; x 2 cm per step): moves the gripper target (world axes); IK follows it with the gripper pointing down. `gripper` G (each in [-1, 1]; command): 0 keeps the fingers, > 0 closes, -f opens to fraction f (-1 fully open). All zero holds still. Skills (`robo skill NAME ARG ...`; `robo info` lists them with their arguments): `move_to X Y Z`: move the grasp point (between the fingertips) to (X, Y, Z) along a straight line, gripper pointing down; reports whether it got there (it stops if blocked or out of reach); `grasp`: close the fingers until they stop; reports what is held; `release`: open the fingers fully; `home`: move the gripper back to its start point above the work area.

**Observation.** `robo observe` reports `robot` (what the robot is, its capabilities and grasp, its workspace, and its current state: gripper points, targets and openings per arm), `objects` (per task object: kind, colour, centre `pos`, tilt, yaw, size, mass and `resting_on`: a surface, a fixture, another object, `gripper` or the floor), `fixtures` (containers with their inner size and rim height, pegs, marks, pads, tags), `scene` (surfaces with their top heights and sizes, obstacles, the floor area) and `safety_events`. Cameras (`robo observe --image --camera NAME`): `overview`, `robot/arm_wrist`, `station_cam`, `top`.

The step budget is 900 steps (45 s of simulated time).

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
