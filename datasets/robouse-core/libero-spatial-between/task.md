---
schema_version: '1.3'
task:
  name: robouse/libero-spatial-between
  description: Move the blue block between the cylinder and the tray
metadata:
  author_name: benchflow
  source_benchmark: LIBERO (Liu et al. 2023), adapted
  source_task: pick up the black bowl between the plate and the ramekin and place it on the plate
  source_suite: libero_spatial
  suite: libero-style
  category: pick-place
  difficulty: easy
  tags:
  - libero
  - spatial
  - referring-expression
  robouse:
    id: libero-spatial-between
    backend: tabletop
    seed: 0
    max_steps: 500
    camera: front
    skills: true
    success_mode: final
    scenario:
      id: libero-spatial-between
      title: Move the blue block between the cylinder and the tray
      goal: There are two identical blue blocks. Pick up the blue block that is BETWEEN the yellow cylinder and the tray and place it on the plate. Do not move the other blue block.
      objects:
      - name: yellow_cylinder
        kind: cyl
        color: yellow
        pos:
        - -0.2
        - 0.1
        size:
        - 0.02
        - 0.035
      - name: blue_block_a
        kind: box
        color: blue
        pos:
        - -0.03
        - 0.1
        size:
        - 0.02
        - 0.02
        - 0.02
      - name: blue_block_b
        kind: box
        color: blue
        pos:
        - -0.12
        - -0.12
        size:
        - 0.02
        - 0.02
        - 0.02
      fixtures:
      - name: tray
        kind: tray
        pos:
        - 0.16
        - 0.1
        size:
        - 0.06
        - 0.06
        color: white
      - name: plate
        kind: plate
        pos:
        - 0.1
        - -0.1
        color: white
      goals:
      - type: on_plate
        obj: blue_block_a
        plate: plate
      - type: unmoved
        obj: blue_block_b
      max_steps: 500
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
oracle:
  env:
    ROBOUSE_ORACLE_TOKEN: ${ROBOUSE_ORACLE_TOKEN:-}
---

You are controlling a simulated robot. Read the task below, then solve it by running the `robo` command in your shell (start with `robo info` and `robo observe`). Keep going until the task is done, then call `robo done` once. Do not stop to ask questions; there is no human to answer.

# Move the blue block between the cylinder and the tray

A two-finger parallel gripper hangs over a table (a floating gripper, no arm). The camera looks at the table from the front: x points to the right, y away from the camera (toward the back of the table), z up; the table top is at z = 0. The fingers close along the y axis.

**Goal:** There are two identical blue blocks. Pick up the blue block that is BETWEEN the yellow cylinder and the tray and place it on the plate. Do not move the other blue block.

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
