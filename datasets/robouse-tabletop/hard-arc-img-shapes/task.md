---
schema_version: '1.3'
task:
  name: robouse/hard-arc-img-shapes
  description: 'Rule inference from pictures: shapes'
metadata:
  author_name: benchflow
  source_benchmark: ARC-AGI (style), robouse original
  source_task: sort by shape into rows
  source_suite: hard
  suite: hard
  category: rule-inference
  difficulty: hard
  tags:
  - arc
  - few-shot
  - examples-as-images-only
  - vision-only
  - single-camera
  - uncalibrated-camera
  - no-projection-matrix
  - no-grid-coordinates
  - no-skills
  - max-repeat-10
  - shape-perception
  robouse:
    id: hard-arc-img-shapes
    backend: tabletop
    seed: 0
    max_steps: 1050
    camera: top
    skills: false
    success_mode: final
    max_repeat: 10
    obs_mode: vision
    cameras:
    - top
    uncalibrated_cameras:
    - top
    visible_fields:
    - hand_pos
    - gripper_open
    - constraint_violations
    scenario:
      id: hard-arc-img-shapes
      title: 'Rule inference from pictures: shapes'
      grid:
        rows: 4
        cols: 4
        cell: 0.07
        origin:
        - -0.105
        - 0.17
      objects:
      - name: item_1
        kind: box
        color: red
        cell:
        - 1
        - 0
        pos:
        - 0.0
        - 0.0
        size:
        - 0.018
        - 0.018
        - 0.018
      - name: item_2
        kind: cyl
        color: red
        cell:
        - 2
        - 1
        pos:
        - 0.0
        - 0.0
        size:
        - 0.018
        - 0.018
      - name: item_3
        kind: cyl
        color: blue
        cell:
        - 1
        - 2
        pos:
        - 0.0
        - 0.0
        size:
        - 0.018
        - 0.018
      - name: item_4
        kind: box
        color: green
        cell:
        - 0
        - 3
        pos:
        - 0.0
        - 0.0
        size:
        - 0.018
        - 0.018
        - 0.018
      - name: item_5
        kind: box
        color: yellow
        cell:
        - 2
        - 2
        pos:
        - 0.0
        - 0.0
        size:
        - 0.018
        - 0.018
        - 0.018
      fixtures: []
      goals:
      - type: grid
        expected:
        - color: red
          cell:
          - 3
          - 0
          kind: box
        - color: red
          cell:
          - 0
          - 1
          kind: cyl
        - color: blue
          cell:
          - 0
          - 2
          kind: cyl
        - color: green
          cell:
          - 3
          - 3
          kind: box
        - color: yellow
          cell:
          - 3
          - 2
          kind: box
        match_kind: true
      image_examples:
      - before:
          cells:
          - color: blue
            cell:
            - 1
            - 1
            kind: box
          - color: red
            cell:
            - 2
            - 3
            kind: cyl
          - color: green
            cell:
            - 1
            - 0
            kind: cyl
        after:
          cells:
          - color: blue
            cell:
            - 3
            - 1
            kind: box
          - color: red
            cell:
            - 0
            - 3
            kind: cyl
          - color: green
            cell:
            - 0
            - 0
            kind: cyl
      - before:
          cells:
          - color: yellow
            cell:
            - 0
            - 2
            kind: box
          - color: blue
            cell:
            - 2
            - 0
            kind: box
          - color: blue
            cell:
            - 3
            - 1
            kind: cyl
          - color: red
            cell:
            - 1
            - 3
            kind: box
        after:
          cells:
          - color: yellow
            cell:
            - 3
            - 2
            kind: box
          - color: blue
            cell:
            - 3
            - 0
            kind: box
          - color: blue
            cell:
            - 0
            - 1
            kind: cyl
          - color: red
            cell:
            - 3
            - 3
            kind: box
      - before:
          cells:
          - color: green
            cell:
            - 2
            - 2
            kind: cyl
          - color: red
            cell:
            - 1
            - 1
            kind: box
          - color: yellow
            cell:
            - 3
            - 0
            kind: cyl
        after:
          cells:
          - color: green
            cell:
            - 0
            - 2
            kind: cyl
          - color: red
            cell:
            - 3
            - 1
            kind: box
          - color: yellow
            cell:
            - 0
            - 0
            kind: cyl
      example_view:
        center:
        - 0.0
        - 0.03
        distance: 0.8
      max_steps: 1050
      camera: top
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
---

You are controlling a simulated robot. Read the task below, then solve it by running the `robo` command in your shell (start with `robo info` and `robo observe`). Keep going until the task is done, then call `robo done` once. Do not stop to ask questions; there is no human to answer.

# Rule inference from pictures: shapes

A two-finger parallel gripper hangs over a table (a floating gripper, no arm; it cannot rotate). x points to the right as seen from the front of the table, y away from the front (toward the back of the table), z up; the table top is at z = 0 and spans x from -0.35 to 0.35 and y from -0.30 to 0.30. The gripper's tool point can reach x in [-0.34, 0.34], y in [-0.27, 0.28], z in [0.012, 0.35]. The fingers close along the y axis; the fully open gap is 10 cm.

**Goal:** Coloured objects stand on a 4 x 4 grid of square cells marked on the table (cells 7 cm apart, centre to centre; row 0 is the back row, col 0 the left column). The cell positions are NOT given as data: find the grid in the camera image. A hidden rule transforms one arrangement into another. The rule is shown ONLY as pictures: at the start of the episode the files `observations/example_1.png`, `observations/example_2.png`, `observations/example_3.png` are placed in your working directory. Each picture shows one example, BEFORE on the left and AFTER on the right, seen from straight above (the back of the table is at the top of the picture) in this same scene with the gripper hidden. Nothing about the examples is given as data. Work out the rule and apply it to the objects in front of you by moving them. Objects of the same colour and shape are interchangeable. Only the final arrangement on the grid is judged: every object in the grid area must rest on the table within 2.5 cm of a cell centre, and objects off the grid are ignored. Objects must be released at the end.

## What you can observe

`robo observe` does **not** give object or goal positions. It returns only `hand_pos`, `gripper_open`, `constraint_violations` and saves one image per camera (`top`; 320 x 320 pixels), printing the paths. Open the images to see the scene. `hand_pos` is the gripper's tool point (between the fingertips) in metres, `gripper_open` is 0 (closed) to 1 (10 cm gap) (there is no contact sensor that says what the fingers hold: a finger opening above zero after closing means something is between them, and the images show what), and `constraint_violations` lists broken rules (any entry means the task has failed).

There is only one camera, and `robo info` does NOT give its projection matrix (only its image size and field of view). You know where the gripper is from `hand_pos`, so you can relate image positions to world positions by moving the gripper and looking where it appears.

## Gripper

The grip value is a finger position target, not a hold command: +1 = fully closed, -1 = fully open (10 cm gap), and values in between give a partial opening (finger gap = 10 cm * (1 - G) / 2, so 0 = half open, a 5 cm gap). There is no separate "hold" value: to keep holding an object, keep sending +1 (the fingers then squeeze it). `robo move-to X Y Z --grip G` applies G on every step of the move, starting with the first, so `--grip 1` closes the fingers at the start of the move (they take about 15 steps to close fully); without `--grip`, the last grip value is kept. `robo grip G --steps N` holds the hand still for N steps while applying G. The same values apply to the GRIP argument of `robo act`.

## Task rules

- Skills are disabled on this task: `robo move-to` and `robo grip` return an error. Use `robo act DX DY DZ GRIP` only (DX/DY/DZ move the tool point by about 1 cm per unit per step; GRIP +1 closes, -1 opens, values in between give a partial opening: finger gap = 10 cm * (1 - GRIP) / 2).
- `robo act --repeat N` is limited to N <= 10 on this task (larger values are cut to 10).
- Step budget: 1050 simulated steps (one step = 20 ms). Wall-clock limit: 20 minutes.
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
