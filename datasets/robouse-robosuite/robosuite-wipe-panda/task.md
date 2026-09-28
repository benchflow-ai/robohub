---
schema_version: '1.3'
task:
  name: robouse/robosuite-wipe-panda
  description: Wipe a line of dirt off the table with a sponge-like tool.
metadata:
  author_name: benchflow
  source_benchmark: robosuite (Zhu et al.)
  source_task: Wipe
  suite: robosuite
  category: contact
  difficulty: hard
  tags:
  - panda
  - mujoco
  - robosuite
  - single-arm
  robots:
  - Panda
  success_rule: robosuite Wipe._check_success()
  success_mode_reason: Wiping is monotone (a wiped marker never comes back) and robosuite's Wipe environment itself terminates the episode at the first step where every marker is wiped.
  reference_solution: 'scripted end-effector controller: press the tool onto the table, then sweep to the nearest remaining marker, repeatedly'
  robouse:
    id: robosuite-wipe-panda
    backend: robosuite
    env: Wipe
    robots: Panda
    controller: BASIC
    seed: 0
    max_steps: 600
    camera: agentview
    cameras:
    - agentview
    - robot0_eye_in_hand
    image_size: 320
    skills: true
    success_mode: first
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
---

You are controlling a simulated robot. Read the task below, then solve it by running the `robo` command in your shell (start with `robo info` and `robo observe`). Keep going until the task is done, then call `robo done` once. Do not stop to ask questions; there is no human to answer.

# Wipe the dirt off the table

A Franka Emika Panda (7-DoF) with its parallel-jaw gripper works at a table; its base is at the -x end, facing +x. The scene is robosuite's `Wipe` environment (MuJoCo).

**Goal:** Wipe all the dirt (100 small markers along a line on the table) off the table with the wiping tool mounted on the arm. A marker counts as wiped when the tool's bottom face passes over it while pressed flat on the table. robosuite's Wipe check: every marker is wiped. The episode ends by itself when the last marker goes.

The episode ends as solved the moment the check first passes (you do not need to call `robo done` then). 

## This robot

- The action has 6 numbers: `robo act DX DY DZ DROLL DPITCH DYAW`, each in [-1, 1]: robosuite's OSC_POSE end-effector command, 20 steps per second, in the world frame. DX/DY/DZ = 1.0 asks for a 5 cm move (held at 1.0 the hand travels about 1.1 cm per step); DROLL/DPITCH/DYAW rotate the hand about the world x/y/z axes (1.0 asks for 0.5 rad).
- `robo move-to X Y Z` is available (it keeps the tool's orientation); `robo grip` is not.
- World frame, metres: +x points away from the robot's base, +y to the robot's left, +z up.
- `robo observe` fields: `hand_pos` (the tool's reference point, a few millimetres above the middle of its face, metres), `hand_quat` (x, y, z, w); `markers_left`, `dirt_markers_xy` (x, y of every marker still on the table), `proportion_wiped`, `wipe_centroid` and `wipe_radius` (centre and spread of the remaining dirt), `hand_force_N` (force on the tool, newtons; it rises when the tool presses on the table); `table_height`.
- `robo observe --image` saves the `agentview` camera; add `--camera robot0_eye_in_hand` for the wrist camera. Images are 320x320.
- The step budget is 600 steps.
- The action has 6 numbers (there is no gripper). Keep the tool flat (pointing straight down) and pressed lightly onto the table while sweeping; wiping does not count while any other part of the arm touches the table or a joint is at its limit. `move_to` is available; `grip` is not.

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
