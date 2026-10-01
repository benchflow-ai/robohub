---
schema_version: '1.3'
task:
  name: robouse/shadow-cube-face-up
  description: 'Cube: red face up (reorient by regrasp)'
metadata:
  author_name: benchflow
  source_benchmark: robouse original (dexhand suite; Shadow Dexterous Hand E3M5 (right) from MuJoCo Menagerie)
  source_task: shadow-cube-face-up
  suite: dexhand
  category: dexterous-manipulation
  difficulty: hard
  tags:
  - dexterous-hand
  - shadow
  - menagerie
  - mujoco
  robouse:
    id: shadow-cube-face-up
    backend: dexhand
    env: shadow-cube-face-up
    seed: 0
    max_steps: 1200
    camera: front
    cameras:
    - front
    - side
    - top
    - back
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

# Cube: red face up (reorient by regrasp)

You control a right Shadow Dexterous Hand (MuJoCo Menagerie model `shadow_hand`, Apache-2.0: 24 joints driven by 20 position actuators; the two distal joints of each finger are coupled by a tendon, as on the real hand, and are driven together by one actuator `..J0` whose target is their sum). Like a hand in a lab setup that is carried by a robot arm or gantry, it hangs from a 6-axis positioner above a table: three linear axes move the wrist in x, y and z and three rotary axes turn it in yaw, pitch and roll. The hand is not fixed to anything else and has no other support. World frame in metres: +x forward (away from the positioner's home), +y to the left, +z up; the floor is at z = 0 and the table top is at z = 0.40, spanning x from -0.35 to 0.55 and y from -0.35 to 0.35. At roll = pitch = yaw = 0 the palm faces down and the fingers point along +x. PITCH tips the fingers down (+) or up (-); ROLL turns the hand about its own long axis (+90 makes the palm face +y, 180 makes it face up); YAW turns it about the vertical (counter-clockwise seen from above is +). `hand.palm_pos` is a point about 3.5 cm in front of the palm, roughly where a grasped object sits. Objects are real rigid bodies with rubber-like friction: a grasp holds only as long as the fingers' contact forces and friction hold it, and anything dropped falls. An object that falls to the floor fails the task.

## Task

A 5.6 cm cube (90 g) with six coloured faces (red, orange, green, blue, white, yellow) stands on the table at about (0.14, 0.0) with its white face up. Turn it so that its **red** face points up and leave it standing on the table. This is reorientation by regrasp: pick the cube up, turn it in the air with the hand, put it down and let go (or anything else that works physically); the red face's direction is in `objects.cube.face_normals`.

**Success:** the cube rests on the table, is not touched by the hand, and its red face's normal is within 20 degrees of straight up; the cube must never fall off the table. Judged by the episode server from the simulated state after you call `robo done` and the hand has held still (zero action) for 10 steps.

**Controls.** `robo act` takes 26 numbers, each in [-1, 1], applied for one 40 ms step (`--repeat N` holds them for N steps): first the wrist group `wrist.pose_delta` = DX DY DZ DROLL DPITCH DYAW, which moves the positioner's target by x 1 cm along world x, y, z and by x 4 degrees on its roll, pitch and yaw joints (rotations about the wrist); then the hand group `hand.joints` = WRJ2 WRJ1 THJ5 THJ4 THJ3 THJ2 THJ1 FFJ4 FFJ3 FFJ0 MFJ4 MFJ3 MFJ0 RFJ4 RFJ3 RFJ0 LFJ5 LFJ4 LFJ3 LFJ0 (20 values), each added to that actuator's position target as a fraction of its range (x 10 %; for the flexion joints + curls the finger). All zeros holds the current targets. The positioner is a stiff joint-space PD controller with force limits (200-300 N, 40 N m), so pushing into the table or an object stalls it instead of passing through; its target never runs more than 3 cm / 15 degrees ahead of the measured pose. The hand's actuators are the upstream position servos, so a finger stopped by an object keeps squeezing with a force that grows with the remaining target error. Skills (`robo info` lists arguments): `move_to X Y Z [SPEED]` and `move_by DX DY DZ [SPEED]` (straight line of the palm point, orientation kept, SPEED 0.01-0.25 m/s; reports `blocked` if stopped, `remaining_m` if it ran out of steps), `rotate ROLL PITCH YAW [SPEED]` (absolute positioner angles in degrees, turning about the palm point, SPEED 5-100 deg/s), `turn_about X Y DEG [SPEED]` (swing the hand about a vertical line, yawing with it, SPEED 5-90 deg/s; reports `remaining_deg` if it ran out of steps), `grasp power|pinch|tripod` (close those fingers until they stall or are fully closed; reports which fingers touch what), `open`, `pose NAME` (named postures: `open`, `flat`, `relaxed`, `point`, `fist`, `power_ready`, `pinch_ready`, `tripod_ready`), `set_joint NAME DEG` and `wait [SECONDS]`. Skills are plain controllers: a grasp can slip or miss, and nothing is attached to the hand.

**Observation.** `robo observe` reports `hand` (`wrist_pos` and `wrist_rpy_deg` of the positioner flange, their targets, `palm_pos`, `palm_normal`, `finger_dir`, `fingertips` positions, `joints` names with `joint_pos_norm` and `joint_target_norm` in [-1, 1] over each actuator's range, and `contacts` per finger), `objects` (position and what each touches, plus per-object details), `table`, `landmarks` (fixed things in the scene), `progress` and `safety_events`. Cameras: `front` (default), `side`, `top` and `back`.

The step budget is 1200 steps (48 s of simulated time).

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
