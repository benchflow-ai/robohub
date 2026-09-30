---
schema_version: '1.3'
task:
  name: robouse/fetch-and-deliver--google-robot--workshop-bench--s3
  description: 'Fetch an object from one surface and deliver it to another: Google Robot in the workshop bench'
metadata:
  author_name: benchflow
  source_benchmark: robouse composed task (components)
  source_task: fetch-and-deliver--google-robot--workshop-bench--s3
  suite: composed
  category: fetch-and-deliver
  difficulty: hard
  tags:
  - composed
  - google-robot
  - workshop-bench
  - workshop-assembly
  - mobile-manipulation
  - long-horizon
  - navigation
  - pick-and-place
  embodiment: google-robot
  scene: workshop-bench
  scenario: workshop-assembly
  track: mobile-manipulation
  capabilities:
  - long-horizon
  - navigation
  - pick-and-place
  validation:
    oracle: 1.0
    noop: 0.0
    oracle_steps: 1626
  robouse:
    id: fetch-and-deliver--google-robot--workshop-bench--s3
    backend: components
    env: fetch-and-deliver--google-robot--workshop-bench--s3
    seed: 3
    max_steps: 3600
    camera: chase
    cameras:
    - overview
    - chase
    - robot/head
    - bench_cam
    - top
    skills: true
    success_mode: final
    instance:
      embodiment:
        id: google-robot
        version: 0.1.0
      scene:
        id: workshop-bench
        version: 0.1.0
      task:
        id: fetch-and-deliver
        version: 0.1.0
      modifiers:
        obs: state
        budget: normal
        perturbation: none
        safety: none
        roles: single
      seed: 3
      layout:
        objects:
        - name: yellow_tall_block
          kind: tall_block
          color: yellow
          pos:
          - 1.691
          - -0.829
          - 0.8
          yaw: 0.0
          'on': parts_cart
        fixtures:
        - name: drop_zone
          kind: zone
          color: green
          pos:
          - -0.174
          - 0.996
          - 0.9
          half:
          - 0.1
          - 0.1
          'on': workbench
        goal:
          object: yellow_tall_block
          zone: drop_zone
          surface: workbench
          from: parts_cart
        task: Bring the yellow tall block from the parts cart to the workbench and leave it standing on the green mark there.
        success: the yellow tall block stands on the workbench (touching its top, tilted less than 20 degrees) with its centre on the green mark, and no part of the robot touches it
        steps: 3600
  embodied:
    format: robouse
    base_task: fetch-and-deliver--google-robot--workshop-bench--s3
    seed: 3
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
oracle:
  env:
    ROBOUSE_ORACLE_TOKEN: ${ROBOUSE_ORACLE_TOKEN:-}
---

You are controlling a simulated robot. Read the task below, then solve it by running the `robo` command in your shell (start with `robo info` and `robo observe`). Keep going until the task is done, then call `robo done` once. Do not stop to ask questions; there is no human to answer.

# Fetch an object from one surface and deliver it to another: Google Robot in the workshop bench

You control a Google Robot (on a holonomic base) (standing on the floor at (0.3, -1.4), facing 90 degrees). The Google (RT-1) robot's arm and gripper on a holonomic planar base added in robouse; front grasps; a head camera. The scene is a workshop: a 0.9 m workbench under a pegboard, a parts cart, a tool cabinet and a drill press. Arms mount at the back of the bench. Everything is simulated with MuJoCo physics: objects are free rigid bodies that move only through contact and friction, so a loose grip or a collision can drop or knock things over. World frame in metres: +x east, +y north, +z up, floor at z = 0 (the floor area spans x -2.6..2.6, y -2.2..1.95); headings in degrees counter-clockwise from +x.

## Task

Bring the yellow tall block from the parts cart to the workbench and leave it standing on the green mark there.

**Success:** the yellow tall block stands on the workbench (touching its top, tilted less than 20 degrees) with its centre on the green mark, and no part of the robot touches it; and no safety event happened (a hard collision with furniture or walls, an object falling to the floor, the robot falling over, or harm to a person; `safety_events` in `robo observe` lists them). Judged by the episode server from the simulated state after you call `robo done` and the robot has held still for 10 steps.

**Controls.** `robo act VX VY WZ DX DY DZ G` (7 values; one step is 50 ms; `--repeat N` holds it for N steps). `base.twist` VX, VY, WZ (each in [-1, 1]; x 0.5 m/s, x 0.5 m/s, x 60 deg/s): forward, leftward and yaw-rate command of the holonomic base (robot frame). `arm.ee_delta` DX, DY, DZ (each in [-1, 1]; x 2 cm per step): moves the gripper target (world axes; IK; the gripper stays level and points along the heading). `gripper` G (each in [-1, 1]; rate): > 0 closes, < 0 opens (1 = 10 % of the stroke per step), 0 holds; closing stops squeezing at a bounded force. All zero holds still. Skills (`robo skill NAME ARG ...`; `robo info` lists them with their arguments): `go_to X Y [YAW_DEG=none] [SPEED=0.5]`: drive to (X, Y) in a straight line, then turn to YAW_DEG; it does not plan around obstacles (give it waypoints) and stops if the base is blocked; `turn YAW_DEG`: rotate in place to heading YAW_DEG (0 = +x, 90 = +y); `look_at X Y Z`: point the head camera at (X, Y, Z) (pan and tilt; the base does not move); `move_to X Y Z`: move the grasp point (between the fingertips) to (X, Y, Z) in a straight line with the IK arm (the first call unfolds the arm); reports whether it got there; `grasp`: close the gripper until the fingers stop; reports what is held; `release`: open the gripper fully; `home`: fold the arm in for driving.

**Observation.** `robo observe` reports `robot` (what the robot is, its capabilities and grasp, its workspace, and its current state: base pose, gripper point and opening, head angles), `objects` (per task object: kind, colour, centre `pos`, tilt, yaw, size, mass and `resting_on`: a surface, a fixture, another object, `gripper` or the floor), `fixtures` (containers with their inner size and rim height, pegs, marks, pads, tags), `scene` (surfaces with their top heights and sizes, obstacles, the floor area) and `safety_events`. Cameras (`robo observe --image --camera NAME`): `overview`, `chase`, `robot/head`, `bench_cam`, `top`.

The step budget is 3600 steps (180 s of simulated time).

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
