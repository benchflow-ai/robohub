---
schema_version: '1.3'
task:
  name: robouse/hard-wrist-count-marker
  description: Count with the wrist camera
metadata:
  author_name: benchflow
  source_benchmark: robouse original
  source_task: count objects, then act on the count
  source_suite: hard
  suite: hard
  category: active-perception
  difficulty: hard
  tags:
  - wrist-camera-only
  - no-object-coordinates
  - no-calibrated-world-camera
  - search
  - camera-offset-unknown
  - counting
  robouse:
    id: hard-wrist-count-marker
    backend: tabletop
    seed: 0
    max_steps: 550
    camera: front
    skills: true
    success_mode: final
    obs_mode: vision
    cameras:
    - wrist
    uncalibrated_cameras:
    - wrist
    visible_fields:
    - hand_pos
    - gripper_open
    - constraint_violations
    scenario:
      id: hard-wrist-count-marker
      title: Count with the wrist camera
      goal: 'Coloured blocks are scattered over the table. Count them (do not count the black marker cube or the numbered pads) and put the black marker cube onto the grey pad whose number equals the count. The six numbered pads lie in a row along the front edge of the table: pad 1 is centred at x = -0.25, pad 2 at x = -0.15, pad 3 at x = -0.05, pad 4 at x = 0.05, pad 5 at x = 0.15, pad 6 at x = 0.25, all at y = -0.235, each 6 cm square. The marker starts at x = 0.0, y = 0.0. Leave the coloured blocks where they are (each must stay within 2 cm of where it started).'
      objects:
      - name: item_1
        kind: box
        color: black
        pos:
        - 0.0
        - 0.0
        size:
        - 0.015
        - 0.015
        - 0.015
      - name: item_2
        kind: box
        color: red
        pos:
        - -0.28
        - 0.22
        size:
        - 0.02
        - 0.02
        - 0.02
      - name: item_3
        kind: box
        color: blue
        pos:
        - 0.28
        - 0.2
        size:
        - 0.02
        - 0.02
        - 0.02
      - name: item_4
        kind: box
        color: green
        pos:
        - 0.1
        - 0.08
        size:
        - 0.02
        - 0.02
        - 0.02
      - name: item_5
        kind: box
        color: yellow
        pos:
        - -0.15
        - -0.1
        size:
        - 0.02
        - 0.02
        - 0.02
      - name: item_6
        kind: box
        color: purple
        pos:
        - 0.29
        - -0.12
        size:
        - 0.02
        - 0.02
        - 0.02
      - name: item_7
        kind: box
        color: orange
        pos:
        - -0.3
        - 0.02
        size:
        - 0.02
        - 0.02
        - 0.02
      fixtures:
      - name: pad_1
        kind: pad
        pos:
        - -0.25
        - -0.235
        size:
        - 0.03
        - 0.03
        color: grey
        label: '1'
      - name: pad_2
        kind: pad
        pos:
        - -0.15
        - -0.235
        size:
        - 0.03
        - 0.03
        color: grey
        label: '2'
      - name: pad_3
        kind: pad
        pos:
        - -0.04999999999999999
        - -0.235
        size:
        - 0.03
        - 0.03
        color: grey
        label: '3'
      - name: pad_4
        kind: pad
        pos:
        - 0.050000000000000044
        - -0.235
        size:
        - 0.03
        - 0.03
        color: grey
        label: '4'
      - name: pad_5
        kind: pad
        pos:
        - 0.15000000000000002
        - -0.235
        size:
        - 0.03
        - 0.03
        color: grey
        label: '5'
      - name: pad_6
        kind: pad
        pos:
        - 0.25
        - -0.235
        size:
        - 0.03
        - 0.03
        color: grey
        label: '6'
      goals:
      - type: on_pad
        obj: item_1
        pad: pad_6
      - type: unmoved
        obj: item_2
      - type: unmoved
        obj: item_3
      - type: unmoved
        obj: item_4
      - type: unmoved
        obj: item_5
      - type: unmoved
        obj: item_6
      - type: unmoved
        obj: item_7
      constraints: []
      wrist_camera: true
      max_steps: 550
      camera: front
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

# Count with the wrist camera

A two-finger parallel gripper hangs over a table (a floating gripper, no arm; it cannot rotate). x points to the right as seen from the front of the table, y away from the front (toward the back of the table), z up; the table top is at z = 0 and spans x from -0.35 to 0.35 and y from -0.30 to 0.30. The gripper's tool point can reach x in [-0.34, 0.34], y in [-0.27, 0.28], z in [0.012, 0.35]. The fingers close along the y axis; the fully open gap is 10 cm.

**Goal:** Coloured blocks are scattered over the table. Count them (do not count the black marker cube or the numbered pads) and put the black marker cube onto the grey pad whose number equals the count. The six numbered pads lie in a row along the front edge of the table: pad 1 is centred at x = -0.25, pad 2 at x = -0.15, pad 3 at x = -0.05, pad 4 at x = 0.05, pad 5 at x = 0.15, pad 6 at x = 0.25, all at y = -0.235, each 6 cm square. The marker starts at x = 0.0, y = 0.0. Leave the coloured blocks where they are (each must stay within 2 cm of where it started).

## What you can observe

`robo observe` does **not** give object or goal positions. It returns only `hand_pos`, `gripper_open`, `constraint_violations` and saves one image per camera (`wrist`; 320 x 320 pixels), printing the paths. Open the images to see the scene. `hand_pos` is the gripper's tool point (between the fingertips) in metres, `gripper_open` is 0 (closed) to 1 (10 cm gap) (there is no contact sensor that says what the fingers hold: a finger opening above zero after closing means something is between them, and the images show what), and `constraint_violations` lists broken rules (any entry means the task has failed).

The only camera is `wrist`, an eye-in-hand camera fixed to the gripper and moving with it. It looks straight down; in its image, right is +x and up is +y (the back of the table). It is mounted to one side of the fingers, but its exact offset from the tool point is NOT given, and no projection matrix is given (`robo info` gives only its image size and field of view). You know `hand_pos`, and the task description gives the positions of some fixed things, so you can work out how image positions relate to world positions by moving the gripper and watching how the view changes. The higher the gripper, the more of the table you see. There is no other camera.

## Gripper

The grip value is a finger position target, not a hold command: +1 = fully closed, -1 = fully open (10 cm gap), and values in between give a partial opening (finger gap = 10 cm * (1 - G) / 2, so 0 = half open, a 5 cm gap). There is no separate "hold" value: to keep holding an object, keep sending +1 (the fingers then squeeze it). `robo move-to X Y Z --grip G` applies G on every step of the move, starting with the first, so `--grip 1` closes the fingers at the start of the move (they take about 15 steps to close fully); without `--grip`, the last grip value is kept. `robo grip G --steps N` holds the hand still for N steps while applying G. The same values apply to the GRIP argument of `robo act`.

## Task rules

- Step budget: 550 simulated steps (one step = 20 ms). Wall-clock limit: 20 minutes.
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
