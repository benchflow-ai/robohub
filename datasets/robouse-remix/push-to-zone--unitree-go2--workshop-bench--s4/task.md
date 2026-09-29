---
schema_version: '1.3'
task:
  name: robouse/push-to-zone--unitree-go2--workshop-bench--s4
  description: 'Push an object that cannot be grasped into a marked zone: Unitree Go2 in the workshop bench'
metadata:
  author_name: benchflow
  source_benchmark: robouse remix (components)
  source_task: push-to-zone--unitree-go2--workshop-bench--s4
  suite: remix
  category: push-to-zone
  difficulty: medium
  tags:
  - remix
  - unitree-go2
  - workshop-bench
  - workshop-assembly
  - move
  - contact-rich
  embodiment: unitree-go2
  scene: workshop-bench
  scenario: workshop-assembly
  track: move
  capabilities:
  - contact-rich
  validation:
    oracle: 1.0
    noop: 0.0
    oracle_steps: 449
  robouse:
    id: push-to-zone--unitree-go2--workshop-bench--s4
    backend: remix
    env: push-to-zone--unitree-go2--workshop-bench--s4
    seed: 4
    max_steps: 2400
    camera: chase
    cameras:
    - overview
    - chase
    - robot/head
    - bench_cam
    - top
    skills: true
    success_mode: final
    components:
      embodiment: unitree-go2@0.1.0
      scene: workshop-bench@0.1.0
      task: push-to-zone@0.1.0
      modifiers:
        obs: state
        budget: normal
        perturbation: none
        safety: none
        roles: single
    remix:
      embodiment: unitree-go2
      scene: workshop-bench
      task: push-to-zone
      seed: 4
      modifiers:
        obs: state
        budget: normal
        perturbation: none
        safety: none
        roles: single
      instance:
        objects:
        - name: carton
          kind: carton
          color: cardboard
          pos:
          - 0.25
          - 0.085
          - 0.0
          yaw: 2.909880809271179
          'on': floor
        fixtures:
        - name: target_zone
          kind: mat
          color: green
          pos:
          - 1.685
          - 0.34
          - 0.0
          half:
          - 0.55
          - 0.55
          'on': floor
        goal:
          object: carton
          zone: target_zone
        task: The carton is too wide to grasp. Push it into the green zone.
        success: the centre of the carton is inside the green zone, it is upright (tilted less than 15 degrees) and nothing of the robot touches it
        steps: 2400
  embodied:
    format: robouse
    base_task: push-to-zone--unitree-go2--workshop-bench--s4
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

# Push an object that cannot be grasped into a marked zone: Unitree Go2 in the workshop bench

You control a Unitree Go2 (standing on the floor at (0.3, -1.4), facing 90 degrees). A 15 kg quadruped driven by a model-based trot controller (body velocity commands); a forward head camera; it can push things with its body. The scene is a workshop: a 0.9 m workbench under a pegboard, a parts cart, a tool cabinet and a drill press. Arms mount at the back of the bench. Everything is simulated with MuJoCo physics: objects are free rigid bodies that move only through contact and friction, so a loose grip or a collision can drop or knock things over. World frame in metres: +x east, +y north, +z up, floor at z = 0 (the floor area spans x -2.6..2.6, y -2.2..1.95); headings in degrees counter-clockwise from +x.

## Task

The carton is too wide to grasp. Push it into the green zone.

**Success:** the centre of the carton is inside the green zone, it is upright (tilted less than 15 degrees) and nothing of the robot touches it; and no safety event happened (a hard collision with furniture or walls, an object falling to the floor, the robot falling over, or harm to a person; `safety_events` in `robo observe` lists them). Judged by the episode server from the simulated state after you call `robo done` and the robot has held still for 10 steps.

**Controls.** `robo act VX VY WZ` (3 values; one step is 50 ms; `--repeat N` holds it for N steps). `base.twist` VX, VY, WZ ([-0.6, -0.4, -69] to [0.6, 0.4, 69]; m/s, m/s, deg/s): body-frame velocity command for the trot controller (x forward, y left, WZ counter-clockwise); all zero = stop stepping and stand. All zero holds still. Skills (`robo skill NAME ARG ...`; `robo info` lists them with their arguments): `go_to X Y [YAW_DEG=none] [SPEED=0.5]`: walk to (X, Y): turn towards it, trot along the straight line, stop (then turn to YAW_DEG); it does not plan around obstacles (give it waypoints) and reports if blocked; `turn YAW_DEG`: turn in place to heading YAW_DEG (0 = +x east, 90 = +y north); `look_at X Y Z`: turn to face (X, Y) so the head camera looks at it, and hold still for a second.

**Observation.** `robo observe` reports `robot` (what the robot is, its capabilities and grasp, its workspace, and its current state: position, heading and motion state), `objects` (per task object: kind, colour, centre `pos`, tilt, yaw, size, mass and `resting_on`: a surface, a fixture, another object, `gripper` or the floor), `fixtures` (containers with their inner size and rim height, pegs, marks, pads, tags), `scene` (surfaces with their top heights and sizes, obstacles, the floor area) and `safety_events`. Cameras (`robo observe --image --camera NAME`): `overview`, `chase`, `robot/head`, `bench_cam`, `top`.

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
