---
schema_version: '1.3'
task:
  name: robouse/menagerie-panda-pattern-apprentice
  description: Infer a grid rule from examples and place three tokens accordingly (dual Panda).
metadata:
  author_name: benchflow
  source_benchmark: robo-use (BenchFlow)
  source_task: robo-use D04 Pattern Apprentice (development split)
  family: D04
  suite: menagerie
  category: manipulation
  difficulty: hard
  tags:
  - mujoco
  - menagerie
  - panda
  - dual-panda
  - rule-inference
  - arc-style
  - protected-objects
  robouse:
    id: menagerie-panda-pattern-apprentice
    backend: menagerie
    env: menagerie-panda-pattern-apprentice
    seed: 5
    max_steps: 2000
    camera: workspace
    cameras:
    - workspace
    - overhead
    - overview
    skills: true
    success_mode: final
agent:
  timeout_sec: 3600
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

# Pattern Apprentice (dual Panda)

Two Franka Emika Panda arms (7 joints each, with a parallel two-finger hand whose fingers open to 8 cm; the hand is about 20 cm wide along the finger axis) are mounted side by side at the back edge of a 1.6 m x 0.94 m desk, both reaching forward toward you. The "left" arm is the one at negative x (on the left in the front camera view), the "right" arm the one at positive x. This is a MuJoCo physics simulation with the MuJoCo Menagerie robot models: every object is a free rigid body that moves only through contact and friction (nothing is attached or teleported), so a loose grip, a fast swing or a collision can drop or knock things over. World frame in metres: +x to the right, +y toward the robots (the back of the desk), +z up; the desk top is z = 0.

**Goal:** A 5 x 5 grid of 4 cm cells is marked on a board in the middle of the desk. Three tokens with coloured crowns start in the bowls: `red_circle` and `green_triangle` in the left bowl, `blue_square` in the right bowl (each 6 cm tall with a 2.6 cm base disc). `examples` shows how a hidden rule maps an input arrangement of the three labels on the grid to an output arrangement; `query` is a new input arrangement. Work out the rule, apply it to `query`, and place each token upright on the cell where the rule sends its label. Two violet cylinders just outside the grid must not be disturbed.

**Success:** Each token's base centre is within 10 mm of its answer cell's centre, rests on the board (within 4 mm of z = 0.0102), is tilted at most 10 degrees, released and at rest; and neither violet cylinder moved more than 5 mm or tilted more than 10 degrees at any time. Success is judged by the episode server from the physical state after you call `robo done` and the robots have held still for about 10 steps (0.5 s). "Released" means neither arm touches the object; "at rest" means it moves slower than 1 cm/s.

**Goal fields in `robo observe`:** `examples` (list of {input, output}; each maps a label to [x, y] grid coordinates), `query`, `grid_size` (5), `cell_size` (0.04 m) and `cell_0_0_center`: cell [x, y] is centred at cell_0_0_center + [x * cell_size, y * cell_size, 0] (grid x along world +x, grid y along world +y). The rule, which you must infer, may reflect the grid left-right, turn it by quarter turns and shift it by up to one cell along each axis.

**Controls.** One arm moves at a time. `robo act ARM DX DY DZ GRIP` first selects the arm (ARM -1 = left, +1 = right, 0 = keep the currently selected arm; the other arm holds still), then moves that arm's commanded gripper target by DX, DY, DZ times 2 cm per step along world x, y, z (each in [-1, 1], so 0.25 = 5 mm); GRIP 0 keeps the fingers as they are, any positive value closes them, and a negative value -f opens them to fraction f of full width (-1 fully open, -0.4 = 40 % open). One step is 50 ms. `robo move-to X Y Z` and `robo grip G` drive the currently selected arm (select one with e.g. `robo act -1 0 0 0 0`). Inverse kinematics turns the target into joint commands; the grippers always point straight down.

**Observation.** `robo observe` reports, for each arm (prefixed `left_` / `right_`; in this task the selected arm's fields also appear without prefix, with `selected_arm`), `hand_pos` (the measured point between the fingertips), `hand_target` (where the arm is being driven; it can run up to 5 cm ahead of the hand, which then catches up), `gripper_open` (0 closed .. 1 fully open) and `touching` (task objects in contact with that arm); for every task object `<name>_pos` (the centre of the object's base, where it touches the surface below), `<name>_tilt_deg` (tilt from upright) and `<name>_yaw_deg`; plus the goal fields listed above. `robo observe --image` saves a picture from the front camera (`--camera overhead` looks straight down with +y at the top of the picture; `--camera overview` is a far corner view).

The Panda hand is about 20 cm wide along the finger axis (world y at yaw 0); open fingers sweep past neighbouring cells.

The step budget is 2000 steps (100 s of simulated time).

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
