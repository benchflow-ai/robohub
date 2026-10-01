---
schema_version: '1.3'
task:
  name: robouse/metaworld-hammer-vision
  description: Pick up the hammer and use it to hammer the nail into the wall.
metadata:
  author_name: benchflow
  source_benchmark: Meta-World MT50 (Farama, v3)
  source_task: hammer-v3
  suite: vision
  category: manipulation
  difficulty: hard
  tags:
  - sawyer
  - mujoco
  - single-arm
  base_suite: metaworld
  base_task: metaworld-hammer
  robouse:
    id: metaworld-hammer-vision
    backend: metaworld
    env: hammer-v3
    seed: 0
    max_steps: 500
    camera: corner
    skills: true
    success_mode: first
    obs_mode: vision
    cameras:
    - corner
    - topview
    visible_fields:
    - hand_pos
    - gripper_open
agent:
  timeout_sec: 900
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

# Hammer

## Vision variant

This is the vision version of the task. `robo observe` does **not** give object or goal positions: you get only the robot's own state (`hand_pos`, `gripper_open`) and two camera images (`corner`, `topview`), saved on every `robo observe` and printed as paths. Open the images to see the scene. `robo info` lists each camera with a 3x4 `projection` matrix P: for a world point (x, y, z), `[u*w, v*w, w] = P @ [x, y, z, 1]` gives its pixel (u, v) in that camera's saved image (u to the right, v down, origin at the top-left). With two cameras you can triangulate a point you see in both, or intersect a pixel's ray with a known height. Any field names in the description below that are not in your observation are hidden in this variant.

A Sawyer robot arm with a two-finger gripper stands at a table (adapted from Meta-World, `hammer-v3`).

**Goal:** Pick up the hammer and use it to hammer the nail into the wall.

In `robo observe`: `hand_pos` is the gripper position, `gripper_open` its opening (1 = open), `obj1_pos`/`obj1_quat` and `obj2_pos` are the task objects (obj1 = hammer handle; goal = nail), and `goal_pos` is the target. The full 39-number observation vector is also given as `obs_vector`.

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
