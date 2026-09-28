---
schema_version: '1.3'
task:
  name: robouse/arc-shift-right-vision
  description: 'Rule inference: translation'
metadata:
  author_name: benchflow
  source_benchmark: ARC-AGI (style), robouse original
  source_task: shift-right
  source_suite: arc-style
  suite: vision
  category: rule-inference
  difficulty: easy
  tags:
  - arc
  - few-shot
  base_suite: arc-style
  base_task: arc-shift-right
  robouse:
    id: arc-shift-right-vision
    backend: tabletop
    seed: 0
    max_steps: 1400
    camera: front
    skills: true
    success_mode: final
    scenario:
      id: arc-shift-right
      title: 'Rule inference: translation'
      grid:
        rows: 3
        cols: 4
        cell: 0.08
        origin:
        - -0.12
        - 0.08
      objects:
      - name: block_1
        kind: box
        color: blue
        cell:
        - 0
        - 2
        pos:
        - 0.0
        - 0.0
        size:
        - 0.02
        - 0.02
        - 0.02
      - name: block_2
        kind: box
        color: green
        cell:
        - 0
        - 1
        pos:
        - 0.0
        - 0.0
        size:
        - 0.02
        - 0.02
        - 0.02
      - name: block_3
        kind: box
        color: yellow
        cell:
        - 1
        - 0
        pos:
        - 0.0
        - 0.0
        size:
        - 0.02
        - 0.02
        - 0.02
      - name: block_4
        kind: box
        color: red
        cell:
        - 2
        - 2
        pos:
        - 0.0
        - 0.0
        size:
        - 0.02
        - 0.02
        - 0.02
      fixtures: []
      goals:
      - type: grid
        expected:
        - color: blue
          cell:
          - 0
          - 3
        - color: green
          cell:
          - 0
          - 2
        - color: yellow
          cell:
          - 1
          - 1
        - color: red
          cell:
          - 2
          - 3
      max_steps: 1400
      public:
        examples:
        - before:
          - color: red
            cell:
            - 0
            - 0
          - color: blue
            cell:
            - 1
            - 2
          after:
          - color: red
            cell:
            - 0
            - 1
          - color: blue
            cell:
            - 1
            - 3
        - before:
          - color: green
            cell:
            - 2
            - 1
          - color: yellow
            cell:
            - 0
            - 2
          - color: red
            cell:
            - 1
            - 0
          after:
          - color: green
            cell:
            - 2
            - 2
          - color: yellow
            cell:
            - 0
            - 3
          - color: red
            cell:
            - 1
            - 1
    obs_mode: vision
    cameras:
    - front
    - top
    visible_fields:
    - hand_pos
    - gripper_open
    - holding
    - examples
    - grid
    - constraint_violations
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

# Rule inference: translation

## Vision variant

This is the vision version of the task. `robo observe` does **not** give object or goal positions: you get only the robot's own state (`hand_pos`, `gripper_open`, `holding`, `examples`, `grid`, `constraint_violations`) and two camera images (`front`, `top`), saved on every `robo observe` and printed as paths. Open the images to see the scene. `robo info` lists each camera with a 3x4 `projection` matrix P: for a world point (x, y, z), `[u*w, v*w, w] = P @ [x, y, z, 1]` gives its pixel (u, v) in that camera's saved image (u to the right, v down, origin at the top-left). With two cameras you can triangulate a point you see in both, or intersect a pixel's ray with a known height. Any field names in the description below that are not in your observation are hidden in this variant.

A two-finger parallel gripper hangs over a table (a floating gripper, no arm). The camera looks at the table from the front: x points to the right, y away from the camera (toward the back of the table), z up; the table top is at z = 0. The fingers close along the y axis.

**Goal:** Coloured blocks stand on a grid of cells marked on the table. The observation field `examples` holds demonstrations of a hidden rule: each shows an arrangement of coloured blocks on a grid of the same shape *before* and *after* the rule was applied (cells are `[row, col]`, row 0 = back row, col 0 = left column). Work out the rule from the examples and apply it to the blocks in front of you by moving them. Some examples mention extra items (spare blocks off the grid, pads, a marker, block sizes); these correspond to the same kinds of items in your scene. Blocks of the same colour are interchangeable. Only the final arrangement is judged; blocks must be released and resting on the table.

## What `robo observe` shows

- `hand_pos`: the gripper's tool point (between the fingertips), metres. `gripper_open`: 0 = closed, 1 = fully open (10 cm gap). `holding`: names of objects currently touching the fingers.
- `objects`: every movable object by name, with `kind`, `color`, `pos` (centre, metres), `size` (full x/y/z extent in its own frame), `yaw_deg`, `half_height` (current half extent along z), `tilt_deg` (0 = upright), and `cell` (`[row, col]` or null when off the grid).
- `fixtures`: static things by name (pads, bowls, trays, plates, sockets, the drawer cabinet) with their positions and sizes. Pads are flat coloured squares on the table; bowls and trays have a `floor_z` and a `rim_z`.
- `grid`: rows, cols, `cell_size` and the centre of every cell (`cell_centers[row][col]`). Row 0 is the back row, col 0 the left column.
- `examples`: the before/after demonstrations described above.

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
