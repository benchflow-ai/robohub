---
schema_version: '1.3'
task:
  name: robouse/libero-push-block-to-pad-vision
  description: Push the block onto the green pad
metadata:
  author_name: benchflow
  source_benchmark: LIBERO (Liu et al. 2023), adapted
  source_task: push the plate to the front of the stove
  source_suite: libero_goal
  suite: vision
  category: pushing
  difficulty: easy
  tags:
  - libero
  - push
  base_suite: libero-style
  base_task: libero-push-block-to-pad
  robouse:
    id: libero-push-block-to-pad-vision
    backend: tabletop
    seed: 0
    max_steps: 500
    camera: front
    skills: true
    success_mode: final
    scenario:
      id: libero-push-block-to-pad
      title: Push the block onto the green pad
      goal: Push the orange block across the table until it sits on the green pad (its centre inside the pad).
      objects:
      - name: orange_block
        kind: box
        color: orange
        pos:
        - -0.14
        - -0.06
        size:
        - 0.02
        - 0.02
        - 0.02
      fixtures:
      - name: green_pad
        kind: pad
        pos:
        - 0.12
        - -0.06
        size:
        - 0.05
        - 0.05
        color: green
      goals:
      - type: on_pad
        obj: orange_block
        pad: green_pad
      max_steps: 500
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
oracle:
  env:
    ROBOUSE_ORACLE_TOKEN: ${ROBOUSE_ORACLE_TOKEN:-}
---

You are controlling a simulated robot. Read the task below, then solve it by running the `robo` command in your shell (start with `robo info` and `robo observe`). Keep going until the task is done, then call `robo done` once. Do not stop to ask questions; there is no human to answer.

# Push the block onto the green pad

## Vision variant

This is the vision version of the task. `robo observe` does **not** give object or goal positions: you get only the robot's own state (`hand_pos`, `gripper_open`, `holding`, `examples`, `grid`, `constraint_violations`) and two camera images (`front`, `top`), saved on every `robo observe` and printed as paths. Open the images to see the scene. `robo info` lists each camera with a 3x4 `projection` matrix P: for a world point (x, y, z), `[u*w, v*w, w] = P @ [x, y, z, 1]` gives its pixel (u, v) in that camera's saved image (u to the right, v down, origin at the top-left). With two cameras you can triangulate a point you see in both, or intersect a pixel's ray with a known height. Any field names in the description below that are not in your observation are hidden in this variant.

A two-finger parallel gripper hangs over a table (a floating gripper, no arm). The camera looks at the table from the front: x points to the right, y away from the camera (toward the back of the table), z up; the table top is at z = 0. The fingers close along the y axis.

**Goal:** Push the orange block across the table until it sits on the green pad (its centre inside the pad).

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
