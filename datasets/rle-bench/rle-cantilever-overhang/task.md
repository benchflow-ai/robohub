---
schema_version: '1.3'
task:
  name: robouse/rle-cantilever-overhang
  description: Stack identical blocks so they reach as far past the table edge as possible (RLE-Bench tabletop reasoning).
metadata:
  author_name: benchflow
  source_benchmark: RLE-Bench (RLE-Bench/RLE-Bench, commit 34a73aa)
  source_task: task03 CantileverOverhang
  suite: rle-bench
  category: manipulation
  difficulty: hard
  tags:
  - mujoco
  - robosuite
  - panda
  - mobile-manipulator
  - vision
  - stacking
  - physical-reasoning
  - balance
  rle_bench:
    family: 'task03 (interactive control: tabletop reasoning)'
    task: CantileverOverhang
    seed: RLE-Bench's evaluation seed for this task
    scoring: RLE-Bench quality >= 0.5 passes (quality recorded in result.json)
  robouse:
    id: rle-cantilever-overhang
    backend: rlebench
    env: rle-cantilever-overhang
    seed: 0
    max_steps: 50000
    obs_mode: vision
    visible_fields:
    - joint_pos
    - joint_vel
    - eef_pos
    - eef_quat
    - gripper_pos
    - base_pos
    - base_quat
    - eef_base_pos
    - eef_base_quat
    camera: robot0_agentview_left
    cameras:
    - robot0_agentview_left
    - robot0_agentview_right
    - robot0_eye_in_hand
    skills: true
    success_mode: final
    frame_every: 10
agent:
  timeout_sec: 32400
verifier:
  service: simulator
  user: root
  timeout_sec: 600
sandbox:
  cpus: 1
  memory_mb: 2048
  build_timeout_sec: 3600
oracle:
  env:
    ROBOUSE_ORACLE_TOKEN: ${ROBOUSE_ORACLE_TOKEN:-}
---

You are controlling a simulated robot. Read the task below, then solve it by running the `robo` command in your shell (start with `robo info` and `robo observe`). Keep going until the task is done, then call `robo done` once. Do not stop to ask questions; there is no human to answer.

# Cantilever overhang (RLE-Bench)

Use the robot to extend a stack of identical blocks past the table edge. Build a stable stack extending beyond any table edge. Leave it supported by the table without support from the robot. Retract the arm before calling `robo done`.

A PandaOmron robot (a Franka Panda arm with a two-finger gripper on a mobile base with a torso lift) stands at a 0.7 x 1.6 m table whose top is at z = 0.9 m, spanning x in [-0.35, 0.35] and y in [-0.8, 0.8] around the world origin. Control runs at 20 Hz.

**Scoring.** RLE-Bench scores the settled horizontal overhang of the stack beyond the table edge (blocks resting on the table or on other blocks, none touching the robot, after the scene settles for 5 s) relative to the best overhang the blocks allow (the harmonic stack). This task passes when that fraction is at least 0.5, which one block alone cannot reach. The scene is submitted however the episode ends: `robo done`, running out of steps or time, or stopping without `robo done`; `robo give-up` scores 0.

You have **50,000 control steps** in one attempt. Observe and experiment until satisfied, then call `robo done`.

**Controls.** `robo act` takes 12 numbers in [-1, 1] (RLE-Bench's layout): arm translation in the ROBOT BASE frame (3, scaled by 0.05 m), arm axis-angle rotation in the base frame (3, scaled by 0.5 rad), gripper (+1 closes, -1 opens; the value is absolute, so 0 is half-way), base forward/lateral/yaw velocity (3), torso velocity (1) and mode (-1 arm, +1 base). Neutral hold: `robo act 0 0 0 0 0 0 -1 0 0 0 0 -1`. `robo skill move` instead tracks a world-frame tool pose through the same controller while the base and torso hold still. `robo skill reset` restores the same initial scene, discards progress and costs one step.

**Observation.** `robo observe` gives RGB images from three cameras, `robot0_agentview_left` and `robot0_agentview_right` (mounted on the mobile base) and `robot0_eye_in_hand` (on the wrist), 512 x 512, and the robot's proprioception: `joint_pos`, `joint_vel`, `eef_pos`, `eef_quat`, `gripper_pos`, `base_pos`, `base_quat`, `eef_base_pos` and `eef_base_quat`. Positions are in metres; `eef_base_*` is relative to the robot base, the other poses are world-frame. `robo info` gives each camera's intrinsics. No depth, masses, contact readings or object poses are given: infer object locations and physical properties by observing and interacting with the scene.

## How to control the robot

You are the robot's policy. You act only through the `robo` command in your shell. There is no other way to move the robot, and you cannot read or change the simulator, the scoring, or other files to succeed; the episode server judges the final scene itself.

```
robo info                                   # action layout, skills, cameras (intrinsics), step budget
robo observe                                # robot proprioception, and one image per camera (paths printed; open them to look)
robo observe --camera C                     # one camera only: robot0_agentview_left, robot0_agentview_right, robot0_eye_in_hand
robo act A1 ... A12       [--repeat N]   # one control step (applied N times, N <= 50)
robo skill move X Y Z QX QY QZ QW [G] [S]   # track a world-frame tool pose for S steps (1-200, default 40); gripper G (+1 closes, -1 opens, default +1)
robo skill reset                            # restore the same initial scene (costs one step)
robo done "short summary"                   # end the episode and submit the scene
robo give-up "reason"                       # end the episode without submitting (scores 0)
```

- Positions are in metres in the world frame, z up; quaternions are unit x, y, z, w.
- Every control step counts against the budget, including the steps `move` runs; observing is free.
- Skills are ordinary controllers: collisions and reach limits can stop the arm short. Use intermediate waypoints and re-observe.
