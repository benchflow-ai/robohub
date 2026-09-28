---
schema_version: '1.3'
task:
  name: robouse/menagerie-panda-clutter-retrieval
  description: Lift a target cylinder out of a ring of protected neighbours into a tray (dual Panda).
metadata:
  author_name: benchflow
  source_benchmark: robo-use (BenchFlow)
  source_task: robotics-tasks-20260917/panda-clutter-retrieval (robo-use D02)
  family: D02
  suite: menagerie
  category: manipulation
  difficulty: hard
  tags:
  - mujoco
  - menagerie
  - panda
  - dual-panda
  - clutter
  - protected-objects
  robouse:
    id: menagerie-panda-clutter-retrieval
    backend: menagerie
    env: menagerie-panda-clutter-retrieval
    seed: 0
    max_steps: 800
    camera: workspace
    cameras:
    - workspace
    - overhead
    - overview
    skills: true
    success_mode: final
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

# Clutter Retrieval (dual Panda)

Two Franka Emika Panda arms (7 joints each, with a parallel two-finger hand whose fingers open to 8 cm; the hand is about 20 cm wide along the finger axis) are mounted side by side at the back edge of a 1.6 m x 0.94 m desk, both reaching forward toward you. The "left" arm is the one at negative x (on the left in the front camera view), the "right" arm the one at positive x. This is a MuJoCo physics simulation with the MuJoCo Menagerie robot models: every object is a free rigid body that moves only through contact and friction (nothing is attached or teleported), so a loose grip, a fast swing or a collision can drop or knock things over. World frame in metres: +x to the right, +y toward the robots (the back of the desk), +z up; the desk top is z = 0.

**Goal:** In the left bowl a red cylinder (`target`, 2.4 cm across, 2.8 cm tall) stands in a ring of four violet cylinders of the same size (`neighbour_0` .. `neighbour_3`, 4.5 cm from it, centre to centre). Lift the red cylinder out without disturbing the violet ones and set it upright in the yellow tray in the middle of the desk.

**Success:** The target's base centre is within 15 mm of `tray_center`, it rests on the tray floor (within 4 mm of z = 0.004), is tilted at most 12 degrees, released and at rest; and at no time during the episode did any violet cylinder move more than 8 mm sideways or tilt more than 15 degrees. Success is judged by the episode server from the physical state after you call `robo done` and the robots have held still for about 10 steps (0.5 s). "Released" means neither arm touches the object; "at rest" means it moves slower than 1 cm/s.

**Goal fields in `robo observe`:** `tray_center` (tray floor centre) and `tray_inner_half_size` (0.028 m; the tray has a 6 mm rim).

**Controls.** You drive the left arm only (the other arm stays parked). `robo act DX DY DZ GRIP` moves the arm's commanded gripper target by DX, DY, DZ times 2 cm per step along world x, y, z (each in [-1, 1], so 0.25 = 5 mm); GRIP 0 keeps the fingers as they are, any positive value closes them, and a negative value -f opens them to fraction f of full width (-1 fully open, -0.4 = 40 % open). One step is 50 ms of simulated time. Inverse kinematics turns the target into joint commands; the gripper always points straight down. `robo move-to X Y Z` and `robo grip G` drive the same arm (`move-to` stops within `--tol` metres of the point, default 0.01).

**Observation.** `robo observe` reports, for each arm you drive, `hand_pos` (the measured point between the fingertips), `hand_target` (where the arm is being driven; it can run up to 5 cm ahead of the hand, which then catches up), `gripper_open` (0 closed .. 1 fully open) and `touching` (task objects in contact with that arm); for every task object `<name>_pos` (the centre of the object's base, where it touches the surface below), `<name>_tilt_deg` (tilt from upright) and `<name>_yaw_deg`; plus the goal fields listed above. `robo observe --image` saves a picture from the front camera (`--camera overhead` looks straight down with +y at the top of the picture; `--camera overview` is a far corner view).

The step budget is 800 steps (40 s of simulated time).

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
