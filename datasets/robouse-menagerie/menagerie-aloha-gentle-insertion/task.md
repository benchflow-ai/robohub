---
schema_version: '1.3'
task:
  name: robouse/menagerie-aloha-gentle-insertion
  description: Seat a peg in a socket under a 6 N contact-force limit (ALOHA 2).
metadata:
  author_name: benchflow
  source_benchmark: robo-use (BenchFlow)
  source_task: robotics-tasks-20260917/aloha2-gentle-insertion (robo-use D03)
  family: D03
  suite: menagerie
  category: manipulation
  difficulty: hard
  tags:
  - mujoco
  - menagerie
  - aloha
  - aloha2
  - insertion
  - force-limit
  robouse:
    id: menagerie-aloha-gentle-insertion
    backend: menagerie
    env: menagerie-aloha-gentle-insertion
    seed: 0
    max_steps: 1200
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
oracle:
  env:
    ROBOUSE_ORACLE_TOKEN: ${ROBOUSE_ORACLE_TOKEN:-}
---

You are controlling a simulated robot. Read the task below, then solve it by running the `robo` command in your shell (start with `robo info` and `robo observe`). Keep going until the task is done, then call `robo done` once. Do not stop to ask questions; there is no human to answer.

# Gentle Insertion (ALOHA 2)

An ALOHA 2 bimanual rig: two ViperX 300 arms (6 joints each, with parallel-jaw grippers that open to about 7 cm) are mounted side by side at the back edge of a 1.6 m x 0.94 m desk, both reaching forward toward you. The "left" arm is the one at negative x (on the left in the front camera view), the "right" arm the one at positive x. This is a MuJoCo physics simulation with the MuJoCo Menagerie robot models: every object is a free rigid body that moves only through contact and friction (nothing is attached or teleported), so a loose grip, a fast swing or a collision can drop or knock things over. World frame in metres: +x to the right, +y toward the robots (the back of the desk), +z up; the desk top is z = 0.

**Goal:** An orange peg (a rod 1.6 cm across and 7.5 cm long with a dark collar at its top end) stands in the left bowl. Seat it upright in the square socket in the middle of the desk (aperture 2.3 cm square, 3 cm deep) without ramming it: the contact force between the peg and the socket is monitored during the whole episode.

**Success:** The peg's base is inside the aperture and at least 80 % of the way down (base z <= 0.011), tilted at most 8 degrees, released and at rest; and the contact force between the peg and the socket never exceeded 6 N at any time. Success is judged by the episode server from the physical state after you call `robo done` and the robots have held still for about 10 steps (0.5 s). "Released" means neither arm touches the object; "at rest" means it moves slower than 1 cm/s.

**Goal fields in `robo observe`:** `socket_center` (aperture centre at the socket's top face), `socket_aperture_half_size`, `socket_floor_z`, `socket_top_z`, `peg_radius`, `peg_length` and `contact_force_limit_n`.

**Controls.** You drive the left arm only (the other arm stays parked). `robo act DX DY DZ GRIP` moves the arm's commanded gripper target by DX, DY, DZ times 2 cm per step along world x, y, z (each in [-1, 1], so 0.25 = 5 mm); GRIP 0 keeps the fingers as they are, any positive value closes them, and a negative value -f opens them to fraction f of full width (-1 fully open, -0.4 = 40 % open). One step is 50 ms of simulated time. Inverse kinematics turns the target into joint commands; the gripper always points straight down. `robo move-to X Y Z` and `robo grip G` drive the same arm (`move-to` stops within `--tol` metres of the point, default 0.01).

**Observation.** `robo observe` reports, for each arm you drive, `hand_pos` (the measured point between the fingertips), `hand_target` (where the arm is being driven; it can run up to 5 cm ahead of the hand, which then catches up), `gripper_open` (0 closed .. 1 fully open) and `touching` (task objects in contact with that arm); for every task object `<name>_pos` (the centre of the object's base, where it touches the surface below), `<name>_tilt_deg` (tilt from upright) and `<name>_yaw_deg`; plus the goal fields listed above. `robo observe --image` saves a picture from the front camera (`--camera overhead` looks straight down with +y at the top of the picture; `--camera overview` is a far corner view).

The step budget is 1200 steps (60 s of simulated time).

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
