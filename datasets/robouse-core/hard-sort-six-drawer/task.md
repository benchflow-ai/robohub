---
schema_version: '1.3'
task:
  name: robouse/hard-sort-six-drawer
  description: Sort six blocks into three containers, one of them a drawer (one uncalibrated camera, low-level control)
metadata:
  author_name: benchflow
  source_benchmark: LIBERO-10 (long-horizon drawer + sorting), adapted and extended
  source_task: put the bowl in the drawer and close it + sorting
  source_suite: hard
  suite: hard
  category: long-horizon
  difficulty: hard
  tags:
  - long-horizon
  - drawer
  - sorting
  - vision-only
  - no-object-coordinates
  - single-camera
  - uncalibrated-camera
  - no-projection-matrix
  - no-skills
  - max-repeat-10
  robouse:
    id: hard-sort-six-drawer
    backend: tabletop
    seed: 0
    max_steps: 1750
    camera: front
    skills: false
    success_mode: final
    max_repeat: 10
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
      id: hard-sort-six-drawer
      title: Sort six blocks into three containers, one of them a drawer (one uncalibrated camera, low-level control)
      goal: 'Six blocks lie on the table: two red, two blue, two green. Put both red blocks into the drawer of the cabinet and close the drawer (within 1 cm), put both blue blocks into the white bowl, and both green blocks into the grey tray. The drawer cabinet stands at the back right. Its drawer slides out toward the front (toward -y) by up to 12 cm; its handle is a horizontal bar along x, about 4 cm in front of the drawer face and 4.5 cm above the table. The fingers can close around the bar from above if the gripper is only partly open (e.g. grip 0.2; a fully open gripper hits the drawer face). The drawer''s inside is about 11.8 cm square with 5 cm high walls.'
      objects:
      - name: item_1
        kind: box
        color: red
        pos:
        - -0.22
        - -0.15
        size:
        - 0.018
        - 0.018
        - 0.018
      - name: item_2
        kind: box
        color: red
        pos:
        - 0.05
        - -0.12
        size:
        - 0.018
        - 0.018
        - 0.018
      - name: item_3
        kind: box
        color: blue
        pos:
        - -0.04
        - -0.21
        size:
        - 0.018
        - 0.018
        - 0.018
      - name: item_4
        kind: box
        color: blue
        pos:
        - 0.25
        - -0.18
        size:
        - 0.018
        - 0.018
        - 0.018
      - name: item_5
        kind: box
        color: green
        pos:
        - -0.1
        - -0.06
        size:
        - 0.018
        - 0.018
        - 0.018
      - name: item_6
        kind: box
        color: green
        pos:
        - -0.26
        - -0.02
        size:
        - 0.018
        - 0.018
        - 0.018
      fixtures:
      - name: drawer
        kind: drawer
        pos:
        - 0.17
        - 0.17
        color: darkgrey
        open: 0.0
      - name: bowl
        kind: bowl
        pos:
        - -0.2
        - 0.16
        radius: 0.065
        color: white
      - name: tray
        kind: tray
        pos:
        - -0.02
        - 0.18
        size:
        - 0.065
        - 0.05
        color: grey
      goals:
      - type: in
        obj: item_1
        container: drawer
      - type: in
        obj: item_2
        container: drawer
      - type: drawer
        drawer: drawer
        max_open: 0.01
      - type: in
        obj: item_3
        container: bowl
      - type: in
        obj: item_4
        container: bowl
      - type: in
        obj: item_5
        container: tray
      - type: in
        obj: item_6
        container: tray
      max_steps: 1750
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

# Sort six blocks into three containers, one of them a drawer (one uncalibrated camera, low-level control)

A two-finger parallel gripper hangs over a table (a floating gripper, no arm; it cannot rotate). x points to the right as seen from the front of the table, y away from the front (toward the back of the table), z up; the table top is at z = 0 and spans x from -0.35 to 0.35 and y from -0.30 to 0.30. The gripper's tool point can reach x in [-0.34, 0.34], y in [-0.27, 0.28], z in [0.012, 0.35]. The fingers close along the y axis; the fully open gap is 10 cm.

**Goal:** Six blocks lie on the table: two red, two blue, two green. Put both red blocks into the drawer of the cabinet and close the drawer (within 1 cm), put both blue blocks into the white bowl, and both green blocks into the grey tray. The drawer cabinet stands at the back right. Its drawer slides out toward the front (toward -y) by up to 12 cm; its handle is a horizontal bar along x, about 4 cm in front of the drawer face and 4.5 cm above the table. The fingers can close around the bar from above if the gripper is only partly open (e.g. grip 0.2; a fully open gripper hits the drawer face). The drawer's inside is about 11.8 cm square with 5 cm high walls.

## What you can observe

`robo observe` does **not** give object or goal positions. It returns only `hand_pos`, `gripper_open`, `constraint_violations` and saves one image per camera (`front`; 320 x 320 pixels), printing the paths. Open the images to see the scene. `hand_pos` is the gripper's tool point (between the fingertips) in metres, `gripper_open` is 0 (closed) to 1 (10 cm gap) (there is no contact sensor that says what the fingers hold: a finger opening above zero after closing means something is between them, and the images show what), and `constraint_violations` lists broken rules (any entry means the task has failed).

There is only one camera, and `robo info` does NOT give its projection matrix (only its image size and field of view). You know where the gripper is from `hand_pos`, so you can relate image positions to world positions by moving the gripper and looking where it appears.

## Gripper

The grip value is a finger position target, not a hold command: +1 = fully closed, -1 = fully open (10 cm gap), and values in between give a partial opening (finger gap = 10 cm * (1 - G) / 2, so 0 = half open, a 5 cm gap). There is no separate "hold" value: to keep holding an object, keep sending +1 (the fingers then squeeze it). `robo move-to X Y Z --grip G` applies G on every step of the move, starting with the first, so `--grip 1` closes the fingers at the start of the move (they take about 15 steps to close fully); without `--grip`, the last grip value is kept. `robo grip G --steps N` holds the hand still for N steps while applying G. The same values apply to the GRIP argument of `robo act`.

## Task rules

- Skills are disabled on this task: `robo move-to` and `robo grip` return an error. Use `robo act DX DY DZ GRIP` only (DX/DY/DZ move the tool point by about 1 cm per unit per step; GRIP +1 closes, -1 opens, values in between give a partial opening: finger gap = 10 cm * (1 - GRIP) / 2).
- `robo act --repeat N` is limited to N <= 10 on this task (larger values are cut to 10).
- Step budget: 1750 simulated steps (one step = 20 ms). Wall-clock limit: 30 minutes.
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
