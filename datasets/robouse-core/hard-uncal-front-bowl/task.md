---
schema_version: '1.3'
task:
  name: robouse/hard-uncal-front-bowl
  description: 'One uncalibrated camera: cylinder into the bowl'
metadata:
  author_name: benchflow
  source_benchmark: robouse original
  source_task: pick and place with an uncalibrated camera
  source_suite: hard
  suite: hard
  category: perception
  difficulty: hard
  tags:
  - single-camera
  - uncalibrated-camera
  - no-projection-matrix
  - vision-only
  - no-object-coordinates
  robouse:
    id: hard-uncal-front-bowl
    backend: tabletop
    seed: 0
    max_steps: 500
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
      id: hard-uncal-front-bowl
      title: 'One uncalibrated camera: cylinder into the bowl'
      goal: 'Put the green cylinder into the white bowl. There is one camera (`front`) and its projection matrix is NOT given: work out where things are from the images, for example by moving the gripper (whose position you know) and seeing where it appears.'
      objects:
      - name: item_1
        kind: cyl
        color: green
        pos:
        - 0.16
        - -0.12
        size:
        - 0.02
        - 0.03
      - name: item_2
        kind: box
        color: red
        pos:
        - -0.12
        - -0.1
        size:
        - 0.02
        - 0.02
        - 0.02
      - name: item_3
        kind: box
        color: blue
        pos:
        - 0.02
        - 0.12
        size:
        - 0.02
        - 0.02
        - 0.02
      - name: item_4
        kind: cyl
        color: yellow
        pos:
        - -0.22
        - 0.08
        size:
        - 0.02
        - 0.03
      fixtures:
      - name: bowl
        kind: bowl
        pos:
        - -0.14
        - 0.14
        radius: 0.06
        color: white
      goals:
      - type: in
        obj: item_1
        container: bowl
      max_steps: 500
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

# One uncalibrated camera: cylinder into the bowl

A two-finger parallel gripper hangs over a table (a floating gripper, no arm; it cannot rotate). x points to the right as seen from the front of the table, y away from the front (toward the back of the table), z up; the table top is at z = 0 and spans x from -0.35 to 0.35 and y from -0.30 to 0.30. The gripper's tool point can reach x in [-0.34, 0.34], y in [-0.27, 0.28], z in [0.012, 0.35]. The fingers close along the y axis; the fully open gap is 10 cm.

**Goal:** Put the green cylinder into the white bowl. There is one camera (`front`) and its projection matrix is NOT given: work out where things are from the images, for example by moving the gripper (whose position you know) and seeing where it appears.

## What you can observe

`robo observe` does **not** give object or goal positions. It returns only `hand_pos`, `gripper_open`, `constraint_violations` and saves one image per camera (`front`; 320 x 320 pixels), printing the paths. Open the images to see the scene. `hand_pos` is the gripper's tool point (between the fingertips) in metres, `gripper_open` is 0 (closed) to 1 (10 cm gap) (there is no contact sensor that says what the fingers hold: a finger opening above zero after closing means something is between them, and the images show what), and `constraint_violations` lists broken rules (any entry means the task has failed).

There is only one camera, and `robo info` does NOT give its projection matrix (only its image size and field of view). You know where the gripper is from `hand_pos`, so you can relate image positions to world positions by moving the gripper and looking where it appears.

## Gripper

The grip value is a finger position target, not a hold command: +1 = fully closed, -1 = fully open (10 cm gap), and values in between give a partial opening (finger gap = 10 cm * (1 - G) / 2, so 0 = half open, a 5 cm gap). There is no separate "hold" value: to keep holding an object, keep sending +1 (the fingers then squeeze it). `robo move-to X Y Z --grip G` applies G on every step of the move, starting with the first, so `--grip 1` closes the fingers at the start of the move (they take about 15 steps to close fully); without `--grip`, the last grip value is kept. `robo grip G --steps N` holds the hand still for N steps while applying G. The same values apply to the GRIP argument of `robo act`.

## Task rules

- Step budget: 500 simulated steps (one step = 20 ms). Wall-clock limit: 20 minutes.
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
