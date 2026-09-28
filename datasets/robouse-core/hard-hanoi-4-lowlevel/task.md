---
schema_version: '1.3'
task:
  name: robouse/hard-hanoi-4-lowlevel
  description: Tower of Hanoi, four discs (low-level control)
metadata:
  author_name: benchflow
  source_benchmark: BenchFlow robo-use L-family (Tower of Hanoi), extended to four discs
  source_task: L-family long-horizon sequencing
  source_suite: hard
  suite: hard
  category: sequencing
  difficulty: hard
  tags:
  - hanoi
  - long-horizon
  - no-skills
  - max-repeat-10
  - tight-budget
  robouse:
    id: hard-hanoi-4-lowlevel
    backend: tabletop
    seed: 0
    max_steps: 2900
    camera: front
    skills: false
    success_mode: final
    max_repeat: 10
    scenario:
      id: hard-hanoi-4-lowlevel
      title: Tower of Hanoi, four discs (low-level control)
      goal: 'Move the tower of four square discs (flat plates of four sizes: red 8.4 cm, yellow 7 cm, green 5.6 cm, blue 4.2 cm wide, each 2 cm thick) from the left grey pad (pad_a) to the right grey pad (pad_c), using the middle pad (pad_b) as a buffer. Move one disc at a time, only ever the top disc of a pile (never pull a disc out from under another), and never leave a larger disc resting on a smaller one (both checked at every step and reported in `constraint_violations`). At the end the four discs must be stacked on pad_c, largest at the bottom, each centred on the one below within 2 cm. The minimum solution is 15 moves.'
      objects:
      - name: large_disc
        kind: box
        color: red
        pos:
        - -0.2
        - 0.04
        size:
        - 0.042
        - 0.042
        - 0.01
        z: 0.01
      - name: medium_disc
        kind: box
        color: yellow
        pos:
        - -0.2
        - 0.04
        size:
        - 0.035
        - 0.035
        - 0.01
        z: 0.0305
      - name: small_disc
        kind: box
        color: green
        pos:
        - -0.2
        - 0.04
        size:
        - 0.028
        - 0.028
        - 0.01
        z: 0.051000000000000004
      - name: tiny_disc
        kind: box
        color: blue
        pos:
        - -0.2
        - 0.04
        size:
        - 0.021
        - 0.021
        - 0.01
        z: 0.0715
      fixtures:
      - name: pad_a
        kind: pad
        pos:
        - -0.2
        - 0.04
        size:
        - 0.055
        - 0.055
        color: grey
        label: A
      - name: pad_b
        kind: pad
        pos:
        - 0.0
        - 0.04
        size:
        - 0.055
        - 0.055
        color: grey
        label: B
      - name: pad_c
        kind: pad
        pos:
        - 0.2
        - 0.04
        size:
        - 0.055
        - 0.055
        color: grey
        label: C
      constraints:
      - type: size_order
        objects:
        - large_disc
        - medium_disc
        - small_disc
        - tiny_disc
      - type: top_only
        objects:
        - large_disc
        - medium_disc
        - small_disc
        - tiny_disc
      goals:
      - type: stack
        order:
        - large_disc
        - medium_disc
        - small_disc
        - tiny_disc
        pad: pad_c
        tol: 0.02
      max_steps: 2900
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
---

You are controlling a simulated robot. Read the task below, then solve it by running the `robo` command in your shell (start with `robo info` and `robo observe`). Keep going until the task is done, then call `robo done` once. Do not stop to ask questions; there is no human to answer.

# Tower of Hanoi, four discs (low-level control)

A two-finger parallel gripper hangs over a table (a floating gripper, no arm; it cannot rotate). x points to the right as seen from the front of the table, y away from the front (toward the back of the table), z up; the table top is at z = 0 and spans x from -0.35 to 0.35 and y from -0.30 to 0.30. The gripper's tool point can reach x in [-0.34, 0.34], y in [-0.27, 0.28], z in [0.012, 0.35]. The fingers close along the y axis; the fully open gap is 10 cm.

**Goal:** Move the tower of four square discs (flat plates of four sizes: red 8.4 cm, yellow 7 cm, green 5.6 cm, blue 4.2 cm wide, each 2 cm thick) from the left grey pad (pad_a) to the right grey pad (pad_c), using the middle pad (pad_b) as a buffer. Move one disc at a time, only ever the top disc of a pile (never pull a disc out from under another), and never leave a larger disc resting on a smaller one (both checked at every step and reported in `constraint_violations`). At the end the four discs must be stacked on pad_c, largest at the bottom, each centred on the one below within 2 cm. The minimum solution is 15 moves.

## What `robo observe` shows

- `hand_pos`: the gripper's tool point (between the fingertips), metres. `gripper_open`: 0 = closed, 1 = fully open (10 cm gap). `holding`: names of objects currently touching the fingers.
- `objects`: every movable object by name, with `kind`, `color`, `pos` (centre, metres), `size` (full x/y/z extent in its own frame), `yaw_deg`, `half_height` (current half extent along z), `tilt_deg` (0 = upright).
- `fixtures`: static things by name (pads, bowls, trays, plates, sockets, the drawer cabinet) with their positions and sizes. Pads are flat coloured squares on the table; bowls and trays have a `floor_z` and a `rim_z`.
- `constraint_violations`: rules the server checks at every step; any entry here means the task has failed.

## Gripper

The grip value is a finger position target, not a hold command: +1 = fully closed, -1 = fully open (10 cm gap), and values in between give a partial opening (finger gap = 10 cm * (1 - G) / 2, so 0 = half open, a 5 cm gap). There is no separate "hold" value: to keep holding an object, keep sending +1 (the fingers then squeeze it). `robo move-to X Y Z --grip G` applies G on every step of the move, starting with the first, so `--grip 1` closes the fingers at the start of the move (they take about 15 steps to close fully); without `--grip`, the last grip value is kept. `robo grip G --steps N` holds the hand still for N steps while applying G. The same values apply to the GRIP argument of `robo act`.

## Task rules

- Skills are disabled on this task: `robo move-to` and `robo grip` return an error. Use `robo act DX DY DZ GRIP` only (DX/DY/DZ move the tool point by about 1 cm per unit per step; GRIP +1 closes, -1 opens, values in between give a partial opening: finger gap = 10 cm * (1 - GRIP) / 2).
- `robo act --repeat N` is limited to N <= 10 on this task (larger values are cut to 10).
- Step budget: 2900 simulated steps (one step = 20 ms). Wall-clock limit: 30 minutes.
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
