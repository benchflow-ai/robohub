---
schema_version: '1.3'
task:
  name: robouse/inspect-tag--unitree-go2--tabletop--s1
  description: 'Read the number on an inspection tag: Unitree Go2 in the tabletop'
metadata:
  author_name: benchflow
  source_benchmark: robouse composed task (components)
  source_task: inspect-tag--unitree-go2--tabletop--s1
  suite: composed
  category: inspect-tag
  difficulty: medium
  tags:
  - composed
  - unitree-go2
  - tabletop
  - tabletop
  - move
  - perception
  - partial-observability
  - navigation
  embodiment: unitree-go2
  scene: tabletop
  scenario: tabletop
  track: move
  capabilities:
  - perception
  - partial-observability
  - navigation
  validation:
    oracle: 1.0
    noop: 0.0
    oracle_steps: 149
  robouse:
    id: inspect-tag--unitree-go2--tabletop--s1
    backend: components
    env: inspect-tag--unitree-go2--tabletop--s1
    seed: 1
    max_steps: 1400
    camera: chase
    cameras:
    - overview
    - chase
    - robot/head
    - front
    - top
    skills: true
    success_mode: final
    instance:
      embodiment:
        id: unitree-go2
        version: 0.1.0
      scene:
        id: tabletop
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
      seed: 1
      layout:
        objects: []
        fixtures:
        - name: tag
          kind: tag
          color: yellow
          pos:
          - 2.45
          - -0.4
          - 0.55
          normal:
          - -1
          - 0
          reading: 560
          label: the yellow tag on the east wall
          post: false
        goal:
          tag: tag
          reading: 560
        task: A yellow inspection tag hangs on the east wall; a three-digit number is printed on it. Go and look at it with your camera, read the number, and report it in the `robo done` message (for example `robo done "the tag reads 123"`).
        success: the tag was seen by the robot's own camera (its centre in the middle 70 % of the image, facing the camera, at most 2.5 m away, not blocked) for at least one second, and the `robo done` message contains the number
        steps: 1400
  embodied:
    format: robouse
    base_task: inspect-tag--unitree-go2--tabletop--s1
    seed: 1
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

# Read the number on an inspection tag: Unitree Go2 in the tabletop

You control a Unitree Go2 (standing on the floor at (0, -1.5), facing 90 degrees). A 15 kg quadruped driven by a model-based trot controller (body velocity commands); a forward head camera; it can push things with its body. The scene is a 2 m x 0.8 m table in a plain room; arms mount at its rear edge, floor robots reach it from the south, drones take off from a pad in the corner. Everything is simulated with MuJoCo physics: objects are free rigid bodies that move only through contact and friction, so a loose grip or a collision can drop or knock things over. World frame in metres: +x east, +y north, +z up, floor at z = 0 (the floor area spans x -2.5..2.5, y -2.2..1.6); headings in degrees counter-clockwise from +x.

## Task

A yellow inspection tag hangs on the east wall; a three-digit number is printed on it. Go and look at it with your camera, read the number, and report it in the `robo done` message (for example `robo done "the tag reads 123"`).

**Success:** the tag was seen by the robot's own camera (its centre in the middle 70 % of the image, facing the camera, at most 2.5 m away, not blocked) for at least one second, and the `robo done` message contains the number; and no safety event happened (a hard collision with furniture or walls, an object falling to the floor, the robot falling over, or harm to a person; `safety_events` in `robo observe` lists them). Judged by the episode server from the simulated state after you call `robo done` and the robot has held still for 10 steps.

**Controls.** `robo act VX VY WZ` (3 values; one step is 50 ms; `--repeat N` holds it for N steps). `base.twist` VX, VY, WZ ([-0.6, -0.4, -69] to [0.6, 0.4, 69]; m/s, m/s, deg/s): body-frame velocity command for the trot controller (x forward, y left, WZ counter-clockwise); all zero = stop stepping and stand. All zero holds still. Skills (`robo skill NAME ARG ...`; `robo info` lists them with their arguments): `go_to X Y [YAW_DEG=none] [SPEED=0.5]`: walk to (X, Y): turn towards it, trot along the straight line, stop (then turn to YAW_DEG); it does not plan around obstacles (give it waypoints) and reports if blocked; `turn YAW_DEG`: turn in place to heading YAW_DEG (0 = +x east, 90 = +y north); `look_at X Y Z`: turn to face (X, Y) so the head camera looks at it, and hold still for a second.

**Observation.** `robo observe` reports `robot` (what the robot is, its capabilities and grasp, its workspace, and its current state: position, heading and motion state), `objects` (per task object: kind, colour, centre `pos`, tilt, yaw, size, mass and `resting_on`: a surface, a fixture, another object, `gripper` or the floor), `fixtures` (containers with their inner size and rim height, pegs, marks, pads, tags), `scene` (surfaces with their top heights and sizes, obstacles, the floor area) and `safety_events`. Cameras (`robo observe --image --camera NAME`): `overview`, `chase`, `robot/head`, `front`, `top`.

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
