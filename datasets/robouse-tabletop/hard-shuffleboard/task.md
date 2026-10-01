---
schema_version: '1.3'
task:
  name: robouse/hard-shuffleboard
  description: 'Shuffleboard: slide the puck into an unreachable zone'
metadata:
  author_name: benchflow
  source_benchmark: robouse original
  source_task: shuffleboard (dynamic release)
  source_suite: hard
  suite: hard
  category: dynamics
  difficulty: hard
  tags:
  - dynamics
  - no-skills
  - max-repeat-5
  - one-shot
  - unreachable-goal
  robouse:
    id: hard-shuffleboard
    backend: tabletop
    seed: 0
    max_steps: 650
    camera: front
    skills: false
    success_mode: final
    max_repeat: 5
    scenario:
      id: hard-shuffleboard
      title: 'Shuffleboard: slide the puck into an unreachable zone'
      goal: 'Slide the red puck along the slippery lane so that it comes to rest inside the blue zone. The lane is a smooth white slab 24 cm wide (x from -0.12 to 0.12) and 4 mm thick whose surface is very slippery (friction coefficient about 0.03 against the pucks); it runs from y = -0.25 past the back edge of the table and ends in mid-air at y = 0.85. A puck that slides past the end falls off. The gripper can only reach y <= 0.28, so the zone (the blue rectangle painted on the lane, x from -0.10 to 0.10, y from 0.44 to 0.62) cannot be reached: a puck must be pushed so that it slides there by itself, and a puck that stops beyond y = 0.28 can never be touched again. Each puck is a disc 5 cm across and 2 cm thick. Success is judged after `robo done`: the puck''s centre must be inside the zone, resting on the lane and no longer moving.'
      objects:
      - name: puck
        kind: cyl
        color: red
        pos:
        - 0.0
        - -0.1
        size:
        - 0.025
        - 0.01
        friction: 0.03
        z: 0.014
      fixtures:
      - name: lane
        kind: lane
        pos:
        - 0.0
        - 0.3
        size:
        - 0.12
        - 0.55
        height: 0.004
        friction: 0.03
        color: white
      - name: zone
        kind: pad
        pos:
        - 0.0
        - 0.53
        size:
        - 0.1
        - 0.09
        z: 0.004
        color: blue
      goals:
      - type: in_zone
        obj: puck
        rect:
        - -0.1
        - 0.1
        - 0.44
        - 0.62
        z: 0.004
      max_steps: 650
agent:
  timeout_sec: 1200
verifier:
  service: simulator
  user: root
  timeout_sec: 300
sandbox:
  cpus: 1
  memory_mb: 2048
  build_timeout_sec: 3600
oracle:
  env:
    ROBOUSE_ORACLE_TOKEN: ${ROBOUSE_ORACLE_TOKEN:-}
---

You are controlling a simulated robot. Read the task below, then solve it by running the `robo` command in your shell (start with `robo info` and `robo observe`). Keep going until the task is done, then call `robo done` once. Do not stop to ask questions; there is no human to answer.

# Shuffleboard: slide the puck into an unreachable zone

A two-finger parallel gripper hangs over a table (a floating gripper, no arm; it cannot rotate). x points to the right as seen from the front of the table, y away from the front (toward the back of the table), z up; the table top is at z = 0 and spans x from -0.35 to 0.35 and y from -0.30 to 0.30. The gripper's tool point can reach x in [-0.34, 0.34], y in [-0.27, 0.28], z in [0.012, 0.35]. The fingers close along the y axis; the fully open gap is 10 cm.

**Goal:** Slide the red puck along the slippery lane so that it comes to rest inside the blue zone. The lane is a smooth white slab 24 cm wide (x from -0.12 to 0.12) and 4 mm thick whose surface is very slippery (friction coefficient about 0.03 against the pucks); it runs from y = -0.25 past the back edge of the table and ends in mid-air at y = 0.85. A puck that slides past the end falls off. The gripper can only reach y <= 0.28, so the zone (the blue rectangle painted on the lane, x from -0.10 to 0.10, y from 0.44 to 0.62) cannot be reached: a puck must be pushed so that it slides there by itself, and a puck that stops beyond y = 0.28 can never be touched again. Each puck is a disc 5 cm across and 2 cm thick. Success is judged after `robo done`: the puck's centre must be inside the zone, resting on the lane and no longer moving.

## What `robo observe` shows

- `hand_pos`: the gripper's tool point (between the fingertips), metres. `gripper_open`: 0 = closed, 1 = fully open (10 cm gap). `holding`: names of objects currently touching the fingers.
- `objects`: every movable object by name, with `kind`, `color`, `pos` (centre, metres), `size` (full x/y/z extent in its own frame), `yaw_deg`, `half_height` (current half extent along z), `tilt_deg` (0 = upright).
- `fixtures`: static things by name (pads, bowls, trays, plates, sockets, the drawer cabinet, the lane) with their positions and sizes. Pads are flat coloured squares on the table; bowls and trays have a `floor_z` and a `rim_z`.

## Gripper

The grip value is a finger position target, not a hold command: +1 = fully closed, -1 = fully open (10 cm gap), and values in between give a partial opening (finger gap = 10 cm * (1 - G) / 2, so 0 = half open, a 5 cm gap). There is no separate "hold" value: to keep holding an object, keep sending +1 (the fingers then squeeze it). `robo move-to X Y Z --grip G` applies G on every step of the move, starting with the first, so `--grip 1` closes the fingers at the start of the move (they take about 15 steps to close fully); without `--grip`, the last grip value is kept. `robo grip G --steps N` holds the hand still for N steps while applying G. The same values apply to the GRIP argument of `robo act`.

## Task rules

- Skills are disabled on this task: `robo move-to` and `robo grip` return an error. Use `robo act DX DY DZ GRIP` only (DX/DY/DZ move the tool point by about 1 cm per unit per step; GRIP +1 closes, -1 opens, values in between give a partial opening: finger gap = 10 cm * (1 - GRIP) / 2).
- `robo act --repeat N` is limited to N <= 5 on this task (larger values are cut to 5).
- Step budget: 650 simulated steps (one step = 20 ms). Wall-clock limit: 20 minutes.
- Objects must be released (not touching the fingers) when the episode is judged, about 10 steps after `robo done`.

## How to control the robot

You are the robot's policy. You act only through the `robo` command in your shell. There is no other way to move the robot, and you cannot read or change the simulator, the scoring, or other files to succeed; the episode server judges the final physical state itself.

```
robo info                         # action space, available skills, step budget
robo observe                      # robot and object state as numbers
robo observe --image              # also saves a camera image and prints its path (open it to look)
robo act DX DY DZ GRIP [--repeat N]   # low-level action, applied N times (N <= 50)
robo move-to X Y Z [--grip G]     # skill: move the gripper toward a point (if enabled for this task)
robo grip G [--steps N]           # skill: hold position and set the gripper (+1 close, -1 open)
robo done "short summary"         # end the episode and ask for scoring
robo give-up "reason"             # end the episode without claiming success
```

- Positions are in metres in the world frame (x, y on the table plane, z up).
- The episode has a fixed step budget (see `robo info`); every simulated step counts, including skills.
- Unless the task says otherwise, success is judged about 10 steps after you call `robo done`, with the robot holding still, so the goal must still be true when the robot stops.
- Work in small steps and re-observe after each motion. Call `robo done` exactly once when finished.
