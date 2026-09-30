---
schema_version: '1.3'
task:
  name: robouse/tiago-fetch-juice
  description: Fetch the juice box from the kitchen counter
metadata:
  author_name: benchflow
  source_benchmark: robouse original (mobile-manip suite; PAL TIAGo from MuJoCo Menagerie)
  source_task: tiago-fetch-juice
  suite: mobile-manip
  category: fetch
  difficulty: medium
  tags:
  - mobile-manipulation
  - tiago
  - menagerie
  - mujoco
  - apartment
  - fetch
  robouse:
    id: tiago-fetch-juice
    backend: mobile_manip
    env: tiago-fetch-juice
    seed: 0
    max_steps: 3600
    camera: chase
    cameras:
    - chase
    - robot/head
    - overview
    - living_room_cam
    - kitchen_cam
    - study_cam
    skills: true
    success_mode: final
agent:
  timeout_sec: 2700
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

# Fetch the juice box from the kitchen counter

You control a PAL TIAGo (MuJoCo Menagerie `pal_tiago`, Apache-2.0): a round differential-drive base 0.54 m across, a torso lift, a 7-joint arm and a parallel two-finger gripper that opens to about 9 cm. The gripper reaches about 0.6 to 0.95 m in front of the base centre, from about 0.45 m to 1.2 m high. Each finger collides through a flat rubber pad on its inner face (the upstream finger meshes are kept for looks).

The robot is in a small apartment simulated with MuJoCo physics. World frame in metres: +x east, +y north, +z up, floor at z = 0; headings in degrees counter-clockwise from +x (90 = facing north). The apartment spans x -4..4 and y -2.5..2.5, with 2 m high walls. The living room (x 0..4) is in the east, the kitchen (x -4..0, y 0..2.5) in the north-west and the study (x -4..0, y -2.5..0) in the south-west. The wall at x = 0 has two 1 m wide doorways: into the kitchen centred at (0, 1.4) and into the study centred at (0, -1.4). A solid wall at y = 0 separates the kitchen from the study, so going between them means passing through the living room. Furniture: the dining table (top 1.0 x 0.7 m, 0.75 m high, centred at (2.0, 1.3)), a sofa along the living room's south wall, a bookshelf against the east wall (x 3.55..3.95, y -0.45..0.45, open towards the west, shelves at z = 0.40 and 0.80), a waste bin (0.4 x 0.4 m, 0.35 m high, centred at (3.3, -1.0)); in the kitchen a counter along the north wall (front face at y = 1.85, top at z = 0.90) with a drawer in it (a vertical bar handle at (-2.0, 1.78, 0.77) when closed; pull it towards -y, it slides out up to 0.40 m) and a kitchen table (0.7 x 0.5 m, 0.75 m high, centred at (-3.35, 0.55)); in the study a desk against the south wall (1.2 x 0.6 m, 0.74 m high, centred at (-2.4, -2.15)). `robo observe` lists every room, doorway and piece of furniture with exact numbers under `apartment`, and the task objects under `objects`. Everything is solid and simulated: the base, the arm and anything carried collide with walls and furniture, and objects are held only by friction between the fingers. The robot starts at (2.6, -0.6) in the living room, facing west, with its arm stowed.

## Task

A green juice box stands on the kitchen counter. Bring it to the dining table in the living room and leave it standing there.

**Success:** the juice box stands on the dining table: it touches that surface, its centre is above it, it is upright (tilted less than 20 degrees) and the robot does not touch it; no safety event happened: a hard collision (a contact force above 150 N between a wall, furniture or the drawer and the robot's base or body (wheels, mast or torso, head), or above 400 N between them and the arm or gripper; the fingers are exempt only on the drawer, which they may grip and pull), a task object touching the floor, a can, cup or bottle knocked over on a table, shelf or counter, or the base tilting more than 12 degrees; `safety_events` in `robo observe` lists them. Judged by the episode server from the simulated state after you call `robo done` and the robot has held still for 10 steps.

**Controls.** `robo act V WZ DX DY DZ POSE G` (7 values, each in [-1, 1]; one step is 50 ms; `--repeat N` holds it for N steps). `base.twist` V, WZ: forward speed (x 0.45 m/s) and turn rate (x 60 deg/s, positive = counter-clockwise); the wheel speeds follow from the wheel radius and track, and the base moves by wheel-floor friction. `arm.ee_delta` DX, DY, DZ: moves the gripper target by that many x 2 cm along the world axes; an inverse-kinematics controller (arm joints and torso lift) follows it while keeping the gripper level and pointing along the robot's heading, fingers closing horizontally. The target is held in the robot's frame, so it moves with the base. `arm.posture` POSE: above 0.5 starts the move to the ready pose (gripper about 0.7 m in front at 0.83 m height, open), below -0.5 to the stowed pose (arm folded at the side); the move runs by itself unless an `arm.ee_delta` command interrupts it. `gripper` G: > 0 closes, < 0 opens (1 = 10 % of the stroke per step); the fingers squeeze with at most about 5 N each. Zero holds everything (the base brakes). Skills (`robo info` lists their arguments): `drive_to X Y [YAW_DEG] [TOL]` (turns towards the point, drives there in a straight line, then turns to YAW_DEG; it does not plan around walls or furniture, so give it waypoints through the doorways, and it stops if the base is blocked), `turn YAW_DEG`, `reach X Y Z` (moves the grasp point, between the fingertips, to a world point in a straight line; from the stowed pose it goes to the ready pose first), `grasp`, `open_gripper`, `ready` and `stow`.

**Observation.** `robo observe` reports `robot` (`base` x, y, yaw_deg; `base_velocity`; `gripper` with `pos` (the grasp point between the fingertips), `opening` (0 closed to 1 open), `holding` (objects touched by both fingers) and `touching`; `arm_posture`; and `arm.gripper_target`, the IK target), `objects` (per object: kind, colour, centre position, tilt from upright, `resting_on`, size and mass), `apartment` (rooms, doorways, furniture with positions and sizes, the drawer's current `opening_m` and handle position) and `safety_events`. Cameras (`robo observe --image --camera NAME`): `chase` (default, above and behind the robot, looking north), `robot/head` (the head camera), `overview` (the whole apartment from above, north up), `living_room_cam`, `kitchen_cam` and `study_cam`.

The step budget is 3600 steps (180 s of simulated time).

## How to control the robot

You are the robot's policy. You act only through the `robo` command in your shell. There is no other way to move the robot, and you cannot read or change the simulator, the scoring, or other files to succeed; the episode server judges the final physical state itself.

```
robo info                          # the robot, its sensors, action groups, skills and step budget
robo observe                       # robot and scene state as numbers
robo observe --image [--camera C]  # also saves a camera image and prints its path (open it to look)
robo act V1 V2 ... [--repeat N]    # one low-level action (the action groups under Controls), applied N times (N <= 50)
robo skill NAME ARG ...            # run a skill listed by `robo info`; it runs until it finishes and reports the result
robo done "short summary"          # end the episode and ask for scoring
robo give-up "reason"              # end the episode without claiming success
```

- Positions are in metres in the world frame (+z up); angles are in degrees unless a field says otherwise.
- The episode has a fixed step budget (see `robo info`); every simulated control step counts, including the steps a skill runs.
- Skills are ordinary controllers: they can fail, stop early or be blocked by the scene. Read what they report and re-observe.
- Success is judged about 10 steps after you call `robo done`, with the robot holding still (each action group's hold value: zero for velocity and delta commands, full brake for a car), so the goal must still be true when the robot stops.
- Call `robo done` exactly once when finished.
