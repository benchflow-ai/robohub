---
schema_version: '1.3'
task:
  name: robouse/cubepick-reach-layout-2
  description: CubePick reach, layout 2
metadata:
  author_name: benchflow
  source_benchmark: inspect-robots cubepick-reach (robocurve/inspect-robots, MIT)
  source_task: cubepick-reach/layout-2
  suite: cubepick
  category: reach
  difficulty: easy
  tags:
  - mock-world
  - inspect-robots
  robouse:
    id: cubepick-reach-layout-2
    backend: cubepick
    env: cubepick-reach-layout-2
    seed: 2
    max_steps: 80
    camera: top
    cameras:
    - top
    skills: false
    success_mode: first
    frame_every: 1
    control_dt: 0.1
agent:
  timeout_sec: 600
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

# CubePick: reach the cube (layout 2)

A point end effector moves in the unit square (x from 0 to 1, y from 0 to 1). It starts at (0.1, 0.1); a cube lies somewhere in the far quadrant (both coordinates between 0.6 and 0.9). This is inspect-robots' built-in `cubepick-reach` benchmark, layout 2.

## Task

Reach the cube.

**Success:** the episode is scored as solved the first time the end effector is within 0.05 of the cube (upstream's `success_at_end` with termination on success). Calling `robo done` earlier ends the episode unsolved.

**Controls.** `robo act DX DY` moves the end effector by (DX, DY), each clipped to +-0.1; positions stay inside the unit square. `--repeat N` applies the same move N times.

**Observation.** `robo observe` reports `eef_pos`, `cube_pos` and their `distance`. `robo observe --image` saves the top camera: the cube is the green square, the end effector the red one; +x points right and +y points down in the image.

The step budget is 80 steps.

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
