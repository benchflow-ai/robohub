---
schema_version: '1.3'
task:
  name: robouse/menagerie-panda-two-bowls-to-board
  description: Place eight chess pieces from two bowls on assigned squares with two arms at once (dual Panda).
metadata:
  author_name: benchflow
  source_benchmark: robo-use (BenchFlow)
  source_task: robotics-tasks-20260917/dual-panda-two-bowls-to-board (robo-use L01 Opening Night, reduced)
  family: L01
  suite: menagerie
  category: manipulation
  difficulty: hard
  tags:
  - mujoco
  - menagerie
  - panda
  - dual-panda
  - bimanual
  - long-horizon
  - chess
  robouse:
    id: menagerie-panda-two-bowls-to-board
    backend: menagerie
    env: menagerie-panda-two-bowls-to-board
    seed: 0
    max_steps: 3000
    camera: workspace
    cameras:
    - workspace
    - overhead
    - overview
    skills: false
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

# Two Bowls to a Board (dual Panda, bimanual)

Two Franka Emika Panda arms (7 joints each, with a parallel two-finger hand whose fingers open to 8 cm; the hand is about 20 cm wide along the finger axis) are mounted side by side at the back edge of a 1.6 m x 0.94 m desk, both reaching forward toward you. The "left" arm is the one at negative x (on the left in the front camera view), the "right" arm the one at positive x. This is a MuJoCo physics simulation with the MuJoCo Menagerie robot models: every object is a free rigid body that moves only through contact and friction (nothing is attached or teleported), so a loose grip, a fast swing or a collision can drop or knock things over. World frame in metres: +x to the right, +y toward the robots (the back of the desk), +z up; the desk top is z = 0.

**Goal:** Eight chess pieces start in the two bowls: four light pieces in the left bowl (two pawns, a rook, a bishop) and four dark ones in the right bowl, 5.2 cm apart. Every piece has its own target square on the chessboard, outlined in gold (`<piece>_target`). Place all eight upright on their squares. The two arms share one clock, so the two halves of the job can run at the same time.

**Success:** Every piece's base centre is within 12 mm of its own target, its base rests on the board (within 4 mm of z = 0.0102), it is tilted at most 10 degrees, released and at rest. Success is judged by the episode server from the physical state after you call `robo done` and the robots have held still for about 10 steps (0.5 s). "Released" means neither arm touches the object; "at rest" means it moves slower than 1 cm/s.

**Goal fields in `robo observe`:** `<piece>_target` for each piece (square centre on the board top) and `square_size` (0.04 m). Pieces: base disc 2.6 cm across, a 1.8 cm stem, pawns 6 cm tall, rooks 6.1 cm, bishops 6.9 cm.

**Controls.** Both arms move at the same time on one clock. `robo act L_DX L_DY L_DZ L_GRIP R_DX R_DY R_DZ R_GRIP` gives each arm its own command in the same step: DX, DY, DZ move that arm's commanded gripper target by 2 cm per unit along world x, y, z (each in [-1, 1], so 0.25 = 5 mm); GRIP 0 keeps the fingers as they are, any positive value closes them, and a negative value -f opens them to fraction f of full width (-1 fully open). One step is 50 ms. `robo move-to` and `robo grip` are not available in this task, and the `DX DY DZ GRIP` form in the general instructions below does not apply: every `robo act` needs all 8 numbers. Inverse kinematics turns each target into joint commands; the grippers always point straight down.

**Observation.** `robo observe` reports, for each arm (prefixed `left_` / `right_`), `hand_pos` (the measured point between the fingertips), `hand_target` (where the arm is being driven; it can run up to 5 cm ahead of the hand, which then catches up), `gripper_open` (0 closed .. 1 fully open) and `touching` (task objects in contact with that arm); for every task object `<name>_pos` (the centre of the object's base, where it touches the surface below), `<name>_tilt_deg` (tilt from upright) and `<name>_yaw_deg`; plus the goal fields listed above. `robo observe --image` saves a picture from the front camera (`--camera overhead` looks straight down with +y at the top of the picture; `--camera overview` is a far corner view).

The step budget is 3000 steps (150 s of simulated time).

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
