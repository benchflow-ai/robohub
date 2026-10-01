---
schema_version: '1.3'
task:
  name: robouse/hard-uncal-hanoi-3
  description: 'One uncalibrated camera: Tower of Hanoi, three discs'
metadata:
  author_name: benchflow
  source_benchmark: BenchFlow robo-use L-family (Tower of Hanoi)
  source_task: L-family long-horizon sequencing
  source_suite: hard
  suite: hard
  category: sequencing
  difficulty: hard
  tags:
  - hanoi
  - long-horizon
  - single-camera
  - uncalibrated-camera
  - no-projection-matrix
  - vision-only
  robouse:
    id: hard-uncal-hanoi-3
    backend: tabletop
    seed: 0
    max_steps: 1550
    camera: front
    skills: true
    success_mode: final
    obs_mode: vision
    cameras:
    - front
    uncalibrated_cameras:
    - front
    visible_fields:
    - hand_pos
    - gripper_open
    - constraint_violations
    scenario:
      id: hard-uncal-hanoi-3
      title: 'One uncalibrated camera: Tower of Hanoi, three discs'
      goal: 'Move the tower of three square discs (flat plates: red 8 cm, yellow 6.4 cm, blue 4.8 cm wide, 2.4 cm thick) from the left grey pad to the right grey pad, using the middle grey pad as a buffer. Move one disc at a time, only ever the top disc of a pile (never pull a disc out from under another), and never leave a larger disc resting on a smaller one (both checked at every step and reported in `constraint_violations`). At the end all three must be stacked on the right pad, largest at the bottom, each centred on the one below within 2 cm. There is one camera (`front`) and its projection matrix is NOT given: work out where things are from the images, for example by moving the gripper (whose position you know) and seeing where it appears.'
      objects:
      - name: item_1
        kind: box
        color: red
        pos:
        - -0.16
        - 0.02
        size:
        - 0.04
        - 0.04
        - 0.012
        z: 0.012
      - name: item_2
        kind: box
        color: yellow
        pos:
        - -0.16
        - 0.02
        size:
        - 0.032
        - 0.032
        - 0.012
        z: 0.036500000000000005
      - name: item_3
        kind: box
        color: blue
        pos:
        - -0.16
        - 0.02
        size:
        - 0.024
        - 0.024
        - 0.012
        z: 0.061
      fixtures:
      - name: pad_a
        kind: pad
        pos:
        - -0.16
        - 0.02
        size:
        - 0.05
        - 0.05
        color: grey
      - name: pad_b
        kind: pad
        pos:
        - 0.0
        - 0.02
        size:
        - 0.05
        - 0.05
        color: grey
      - name: pad_c
        kind: pad
        pos:
        - 0.16
        - 0.02
        size:
        - 0.05
        - 0.05
        color: grey
      constraints:
      - type: size_order
        objects:
        - item_1
        - item_2
        - item_3
      - type: top_only
        objects:
        - item_1
        - item_2
        - item_3
      goals:
      - type: stack
        order:
        - item_1
        - item_2
        - item_3
        pad: pad_c
        tol: 0.02
      max_steps: 1550
agent:
  timeout_sec: 1800
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

# One uncalibrated camera: Tower of Hanoi, three discs

A two-finger parallel gripper hangs over a table (a floating gripper, no arm; it cannot rotate). x points to the right as seen from the front of the table, y away from the front (toward the back of the table), z up; the table top is at z = 0 and spans x from -0.35 to 0.35 and y from -0.30 to 0.30. The gripper's tool point can reach x in [-0.34, 0.34], y in [-0.27, 0.28], z in [0.012, 0.35]. The fingers close along the y axis; the fully open gap is 10 cm.

**Goal:** Move the tower of three square discs (flat plates: red 8 cm, yellow 6.4 cm, blue 4.8 cm wide, 2.4 cm thick) from the left grey pad to the right grey pad, using the middle grey pad as a buffer. Move one disc at a time, only ever the top disc of a pile (never pull a disc out from under another), and never leave a larger disc resting on a smaller one (both checked at every step and reported in `constraint_violations`). At the end all three must be stacked on the right pad, largest at the bottom, each centred on the one below within 2 cm. There is one camera (`front`) and its projection matrix is NOT given: work out where things are from the images, for example by moving the gripper (whose position you know) and seeing where it appears.

## What you can observe

`robo observe` does **not** give object or goal positions. It returns only `hand_pos`, `gripper_open`, `constraint_violations` and saves one image per camera (`front`; 320 x 320 pixels), printing the paths. Open the images to see the scene. `hand_pos` is the gripper's tool point (between the fingertips) in metres, `gripper_open` is 0 (closed) to 1 (10 cm gap) (there is no contact sensor that says what the fingers hold: a finger opening above zero after closing means something is between them, and the images show what), and `constraint_violations` lists broken rules (any entry means the task has failed).

There is only one camera, and `robo info` does NOT give its projection matrix (only its image size and field of view). You know where the gripper is from `hand_pos`, so you can relate image positions to world positions by moving the gripper and looking where it appears.

## Gripper

The grip value is a finger position target, not a hold command: +1 = fully closed, -1 = fully open (10 cm gap), and values in between give a partial opening (finger gap = 10 cm * (1 - G) / 2, so 0 = half open, a 5 cm gap). There is no separate "hold" value: to keep holding an object, keep sending +1 (the fingers then squeeze it). `robo move-to X Y Z --grip G` applies G on every step of the move, starting with the first, so `--grip 1` closes the fingers at the start of the move (they take about 15 steps to close fully); without `--grip`, the last grip value is kept. `robo grip G --steps N` holds the hand still for N steps while applying G. The same values apply to the GRIP argument of `robo act`.

## Task rules

- Step budget: 1550 simulated steps (one step = 20 ms). Wall-clock limit: 30 minutes.
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
