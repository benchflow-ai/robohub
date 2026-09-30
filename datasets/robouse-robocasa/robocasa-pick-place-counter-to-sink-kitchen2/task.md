---
schema_version: '1.3'
task:
  name: robouse/robocasa-pick-place-counter-to-sink-kitchen2
  description: Counter to sink in RoboCasa kitchen layout 2 (Franka Panda on a mobile base).
metadata:
  author_name: benchflow
  source_benchmark: RoboCasa (RoboCasa365, robosuite 1.5)
  source_task: PickPlaceCounterToSink
  suite: robocasa
  category: manipulation
  difficulty: medium
  tags:
  - mujoco
  - robocasa
  - kitchen
  - panda
  - mobile-manipulator
  - pick-place
  - sink
  robouse:
    id: robocasa-pick-place-counter-to-sink-kitchen2
    backend: robocasa
    env: robocasa-pick-place-counter-to-sink-kitchen2
    seed: 0
    max_steps: 800
    camera: robot0_agentview_left
    cameras:
    - robot0_agentview_left
    - robot0_agentview_right
    - robot0_agentview_center
    - robot0_eye_in_hand
    - robot0_frontview
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

# Counter to sink (RoboCasa kitchen 2)

A household kitchen from RoboCasa (kitchen layout 2, style 2): counters, cabinets, drawers, a sink and appliances, simulated in MuJoCo. The robot is a Franka Panda arm on a mobile base (RoboCasa's PandaOmron), parked where RoboCasa's task set-up puts it, facing the relevant fixture; the base and torso do not move in this task, only the arm and its two-finger gripper (fingers open to 8 cm). World frame in metres, z up; `robot_forward` in `robo observe` is the horizontal direction the robot faces. The arm's reach from the parked base is limited (about 0.7 m forward of `robot_base_pos` at counter height, less low down or far to the side), and the arm and hand can collide with the counter and fixtures.

**Goal:** Pick up the object the instruction names from the counter and put it in the sink.

**Success:** RoboCasa's own check for PickPlaceCounterToSink: the object lies inside the sink basin and the gripper is at least 25 cm from it (the object is released). Success is judged by the episode server after you call `robo done` and the robot has held still for about 10 steps.

**Task fields in `robo observe`:** `object_pos` (centre of the object's bounding box), `object_size` (its bounding-box size [x, y, z] in m), `object_category`, `sink_basin_center`, `sink_basin_half_size` and `sink_basin_bottom_z`.

**Controls.** `robo act DX DY DZ GRIP` moves the gripper's commanded target by DX, DY, DZ times 2 cm per step along the WORLD x, y, z axes (each in [-1, 1]); the gripper keeps its orientation (pointing straight down). GRIP 0 keeps the fingers as they are, any positive value closes them, and a negative value -f opens them to fraction f (-1 fully open). One step is 50 ms (RoboCasa's 20 Hz control rate). The arm follows the target through RoboCasa's operational-space controller; the target stays within 5 cm of the hand. `robo move-to X Y Z` and `robo grip G` work as described below.

**Observation.** `robo observe` reports `hand_pos` (the point between the fingertips), `hand_target` (the commanded target), `gripper_open` (0 closed .. 1 open), `robot_base_pos`, `robot_forward`, `task_instruction` (RoboCasa's own language instruction) and the task fields listed above. `robo observe --image` saves a picture from the robot's shoulder camera (`--camera robot0_agentview_left`, `robot0_agentview_right`, `robot0_eye_in_hand` and `robot0_frontview` are also available).

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
