---
schema_version: '1.3'
task:
  name: robouse/so101-open-drawer
  description: 'SO-101 arm: open the drawer'
metadata:
  author_name: benchflow
  source_benchmark: robouse original (so101 suite; SO-101 follower arm from MuJoCo Menagerie)
  source_task: so101-open-drawer
  suite: so101
  category: articulated
  difficulty: medium
  tags:
  - so101
  - menagerie
  - mujoco
  - small-arm
  robouse:
    id: so101-open-drawer
    backend: so101
    env: so101-open-drawer
    seed: 0
    max_steps: 600
    camera: front
    cameras:
    - front
    - top
    - wrist
    skills: true
    success_mode: final
    obs_mode: state
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

# SO-101 arm: open the drawer

You control a SO-101 follower arm in simulation (MuJoCo 3.3.0; the MuJoCo Menagerie model `robotstudio_so101` of The Robot Studio's SO-101, Apache-2.0), clamped to the far edge of a small hobby desk. It is the simulated twin of Robo Use's real SO-101 (the `real` suite): the same joints, units, limits, speed limit, skills and frame. Positions are in the arm's base frame, in metres: +x forward (away from the base), +y left, +z up; the desk top is z = 0 and spans x from -0.07 to 0.28 and y from -0.225 to 0.225 (45 x 35 cm). Joints are in degrees with 0 at the middle of each joint's range (LeRobot's calibration); the gripper goes from 0 (closed) to 100 (fully open). With the gripper pointing straight down, the tool point reaches from about 10 cm in front of the base (14 cm when 5 cm above the desk) to about 32 cm; closer in `move_to` refuses, farther out it tilts the gripper. Cameras: `front` (from the operator's side of the desk), `top` (straight down) and `wrist` (the SO-101's wrist camera mount); `robo observe --image --camera NAME` saves one.

## Task

A small drawer unit stands on the desk with its front toward the arm (`fixtures.drawer`): a drawer 6 cm wide with a brass knob (a 9 mm stem 2.2 cm long with a 17 mm head; `knob_pos` is a point on the stem). The drawer slides out toward the arm (-x), at most 6 cm. Open it at least 4 cm.

**Success:** the drawer is open at least 4 cm (`fixtures.drawer.opening_m`). Judged by the episode server from the simulated state after you call `robo done` and the arm has held still for 10 control steps (1 s).

**Controls.** `robo act` takes 6 numbers: a change of each joint's target this step, in the order shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll (degrees, at most 3 per step: the 30 deg/s speed limit at 10 Hz), then the gripper (0-100 units, at most 6 per step). Zeros hold the arm. Targets stay inside the joint limits: shoulder_pan [-108, 108], shoulder_lift [-98, 98], elbow_flex [-94.8, 94.8], wrist_flex [-93, 93], wrist_roll [-155.2, 160.8] deg, gripper [2, 98]. `--repeat N` applies the same change N times. Skills (`robo info` lists them with their arguments): `move_to X Y Z [SPEED]` (the tool point to a point in the base frame by inverse kinematics; SPEED is a fraction of the speed limit), `grip VALUE` (open with 60, close with 2), `move_joints SHOULDER_PAN SHOULDER_LIFT ELBOW_FLEX WRIST_FLEX WRIST_ROLL [SPEED]` (interpolated move to a joint pose, degrees), `home` (reach-ready pose) and `park` (rest pose). `move_to` points the gripper straight down wherever the arm can (and says by how much it tilted where it cannot), moves along a straight line in joint space, then corrects the measured error up to twice; it refuses points the arm cannot reach and points less than 4 mm above the desk. The arm has five joints, so `move_to` keeps the wrist roll: the gripper's yaw turns with the base; turn the wrist roll (`robo act` or `move_joints`) to square the jaws to an object. Every skill counts its control steps against the budget.

**Observation.** `robo observe` reports `joint_pos` (measured joints by name, degrees) and `gripper` (measured, 0-100), and: `tool_pos` (the tool point between the fingertips; LeRobot's gripper frame), `tool_axis` (the direction the fingers point), `jaw_axis` (the direction from the fixed jaw toward the moving jaw: closing, the moving jaw pushes an object onto the fixed jaw), `jaw_tips` (the inner faces of the fixed and the moving jaw at their tips), `gripper_opening_m` (the distance between them), `target` (the current joint targets) and `holding` (the object both jaws touch). `objects` gives each object's `pos` (its centre unless `pos_is` says otherwise), `yaw_deg`, `tilt_deg`, size and colour; `fixtures` the drawer, button, zone or marker; `desk` the desk's extent.

The step budget is 600 control steps (60 s of simulated time).

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
