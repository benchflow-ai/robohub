---
schema_version: '1.3'
task:
  name: robouse/robosuite-stack-ur5e
  description: Put the red cube on top of the green cube.
metadata:
  author_name: benchflow
  source_benchmark: robosuite (Zhu et al.)
  source_task: Stack
  suite: robosuite
  category: manipulation
  difficulty: medium
  tags:
  - ur5e
  - mujoco
  - robosuite
  - single-arm
  robots:
  - UR5e
  success_rule: robosuite Stack._check_success()
  success_mode_reason: 'A placement must last: success is judged after `robo done` and a 10-step settle, with the robot holding still, so the object has to stay where it was put (and robosuite also requires the gripper to have let go and moved away).'
  reference_solution: 'scripted end-effector controller: pick the red cube, place it on the green cube, release, retreat'
  robouse:
    id: robosuite-stack-ur5e
    backend: robosuite
    env: Stack
    robots: UR5e
    controller: BASIC
    seed: 1
    max_steps: 400
    camera: agentview
    cameras:
    - agentview
    - robot0_eye_in_hand
    image_size: 320
    skills: true
    success_mode: final
    obs_mode: state
agent:
  timeout_sec: 1200
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

# Stack the cubes (UR5e)

A Universal Robots UR5e (6-DoF) with a Robotiq 2F-85 gripper works at a table; its base is at the -x end, facing +x. The scene is robosuite's `Stack` environment (MuJoCo).

**Goal:** Put the red cube (cubeA, 4 cm) on top of the green cube (cubeB, 5 cm) and let go. robosuite's Stack check: the red cube is lifted off the table, touching the green cube, and no longer held by the gripper.

Call `robo done` when finished; the check is made after the robot then holds still for 10 steps, so the result must last. 

## This robot

- The action has 7 numbers, not 4: `robo act DX DY DZ DROLL DPITCH DYAW GRIP`, each in [-1, 1]: robosuite's OSC_POSE end-effector command plus the gripper, 20 steps per second, in the world frame. DX/DY/DZ = 1.0 asks for a 5 cm move (held at 1.0 the hand travels about 1.1 cm per step); DROLL/DPITCH/DYAW rotate the hand about the world x/y/z axes (1.0 asks for 0.5 rad; leave them at 0 unless you need to turn the hand, e.g. DYAW to line the fingers up with an object); GRIP +1 closes, -1 opens, 0 keeps the fingers as they are (they move over about 5 steps).
- `robo move-to X Y Z [--grip G]` and `robo grip G` are available; they move only the position and keep the hand's orientation (it can drift slowly under load; correct it with DROLL/DPITCH/DYAW).
- World frame, metres: +x points away from the robot's base, +y to the robot's left, +z up.
- `robo observe` fields: `hand_pos` (the gripper's grasp point between the fingertips, metres), `hand_quat` (hand orientation quaternion x, y, z, w; about [1, 0, 0, 0] when the gripper points straight down), `gripper_open` (distance between the two finger pads in metres: largest when open, smallest when closed on nothing, in between when holding something), `gripper_yaw_deg` (direction of the line through the two fingertips in the table plane, degrees from +x, in [-90, 90); the fingers close along this line); `cubea_pos`, `cubea_quat`, `cubea_yaw_deg` (red cube A), `cubeb_pos`, `cubeb_quat`, `cubeb_yaw_deg` (green cube B); `table_height`. Every `<object>_quat` (x, y, z, w) comes with `<object>_yaw_deg`, the object's heading about the vertical axis (its local x axis, degrees from +x).
- `robo observe --image` saves the `agentview` camera; add `--camera robot0_eye_in_hand` for the wrist camera. Images are 320x320.
- The step budget is 400 steps.
- Open the gripper after placing and move the hand up before `robo done`.

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
