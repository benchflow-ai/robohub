---
schema_version: '1.3'
task:
  name: robouse/robouse-sort-by-colour
  description: Sort blocks into trays by colour
metadata:
  author_name: benchflow
  source_benchmark: BenchFlow robo-use (task family, adapted to a single floating gripper)
  source_task: L-family multi-object sorting
  source_suite: robo-use L-family
  suite: robo-use-families
  category: sorting
  difficulty: medium
  tags:
  - robo-use
  - sorting
  - multi-object
  robouse:
    id: robouse-sort-by-colour
    backend: tabletop
    seed: 0
    max_steps: 1400
    camera: front
    skills: true
    success_mode: final
    scenario:
      id: robouse-sort-by-colour
      title: Sort blocks into trays by colour
      goal: Put both red blocks into the tray on the left (red_tray) and both blue blocks into the tray on the right (blue_tray).
      objects:
      - name: block_1
        kind: box
        color: red
        pos:
        - -0.05
        - -0.12
        size:
        - 0.02
        - 0.02
        - 0.02
      - name: block_2
        kind: box
        color: red
        pos:
        - 0.16
        - -0.06
        size:
        - 0.02
        - 0.02
        - 0.02
      - name: block_3
        kind: box
        color: blue
        pos:
        - -0.18
        - -0.07
        size:
        - 0.02
        - 0.02
        - 0.02
      - name: block_4
        kind: box
        color: blue
        pos:
        - 0.04
        - -0.02
        size:
        - 0.02
        - 0.02
        - 0.02
      fixtures:
      - name: red_tray
        kind: tray
        pos:
        - -0.17
        - 0.14
        size:
        - 0.075
        - 0.06
        color: pink
      - name: blue_tray
        kind: tray
        pos:
        - 0.17
        - 0.14
        size:
        - 0.075
        - 0.06
        color: cyan
      goals:
      - type: in
        obj: block_1
        container: red_tray
      - type: in
        obj: block_2
        container: red_tray
      - type: in
        obj: block_3
        container: blue_tray
      - type: in
        obj: block_4
        container: blue_tray
      max_steps: 1400
agent:
  timeout_sec: 900
verifier:
  service: simulator
  user: root
  timeout_sec: 300
sandbox:
  cpus: 1
  memory_mb: 2048
  build_timeout_sec: 3600
---

You are controlling a simulated robot. Read the task below, then solve it by running the `robo` command in your shell (start with `robo info` and `robo observe`). Keep going until the task is done, then call `robo done` once. Do not stop to ask questions; there is no human to answer.

# Sort blocks into trays by colour

A two-finger parallel gripper hangs over a table (a floating gripper, no arm). The camera looks at the table from the front: x points to the right, y away from the camera (toward the back of the table), z up; the table top is at z = 0. The fingers close along the y axis.

**Goal:** Put both red blocks into the tray on the left (red_tray) and both blue blocks into the tray on the right (blue_tray).

## What `robo observe` shows

- `hand_pos`: the gripper's tool point (between the fingertips), metres. `gripper_open`: 0 = closed, 1 = fully open (10 cm gap). `holding`: names of objects currently touching the fingers.
- `objects`: every movable object by name, with `kind`, `color`, `pos` (centre, metres), `size` (full x/y/z extent in its own frame), `yaw_deg`, `half_height` (current half extent along z), `tilt_deg` (0 = upright).
- `fixtures`: static things by name (pads, bowls, trays, plates, sockets, the drawer cabinet) with their positions and sizes. Pads are flat coloured squares on the table; bowls and trays have a `floor_z` and a `rim_z`.

## Gripper

The grip value is a finger position target, not a hold command: +1 = fully closed, -1 = fully open (10 cm gap), and values in between give a partial opening (finger gap = 10 cm * (1 - G) / 2, so 0 = half open, a 5 cm gap). There is no separate "hold" value: to keep holding an object, keep sending +1 (the fingers then squeeze it). `robo move-to X Y Z --grip G` applies G on every step of the move, starting with the first, so `--grip 1` closes the fingers at the start of the move (they take about 15 steps to close fully); without `--grip`, the last grip value is kept. `robo grip G --steps N` holds the hand still for N steps while applying G. The same values apply to the GRIP argument of `robo act`.


Objects must be released (not touching the fingers) when the episode is judged.

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
