---
schema_version: '1.3'
task:
  name: robouse/metaworld-bin-picking
  description: Pick the cube out of the left bin and place it in the right bin.
metadata:
  author_name: benchflow
  source_benchmark: Meta-World MT50 (Farama, v3)
  source_task: bin-picking-v3
  suite: metaworld
  category: manipulation
  difficulty: hard
  tags:
  - sawyer
  - mujoco
  - single-arm
  robouse:
    id: metaworld-bin-picking
    backend: metaworld
    env: bin-picking-v3
    seed: 0
    max_steps: 500
    camera: corner
    skills: true
    success_mode: first
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
---

You are controlling a simulated robot. Read the task below, then solve it by running the `robo` command in your shell (start with `robo info` and `robo observe`). Keep going until the task is done, then call `robo done` once. Do not stop to ask questions; there is no human to answer.

# Bin Picking

A Sawyer robot arm with a two-finger gripper stands at a table (adapted from Meta-World, `bin-picking-v3`).

**Goal:** Pick the cube out of the left bin and place it in the right bin.

In `robo observe`: `hand_pos` is the gripper position, `gripper_open` its opening (1 = open), `obj1_pos`/`obj1_quat` and `obj2_pos` are the task objects (obj1 = cube; goal = target in the right bin), and `goal_pos` is the target. The full 39-number observation vector is also given as `obs_vector`.

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
