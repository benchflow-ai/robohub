---
schema_version: '1.3'
task:
  name: robouse/rle-hidden-com-box-1
  description: Find which quadrant of sealed box 1 holds the hidden ballast by handling it (RLE-Bench HiddenCOM).
metadata:
  author_name: benchflow
  source_benchmark: RLE-Bench (RLE-Bench/RLE-Bench, commit 34a73aa)
  source_task: task03 HiddenCOM, box 1 of 3
  suite: rle-bench
  category: manipulation
  difficulty: medium
  tags:
  - mujoco
  - robosuite
  - panda
  - vision
  - hidden-mass
  - physical-reasoning
  - active-perception
  rle_bench:
    family: 'task03 (interactive control: tabletop reasoning)'
    task: HiddenCOM
    box: 1
    scoring: 1 if the submitted quadrant is correct (RLE-Bench scores the fraction of its three boxes)
  robouse:
    id: rle-hidden-com-box-1
    backend: rlebench
    env: rle-hidden-com-box-1
    seed: 0
    max_steps: 12000
    obs_mode: vision
    visible_fields:
    - joint_pos
    - joint_vel
    - eef_pos
    - eef_quat
    - gripper_pos
    - submitted
    camera: workspace
    cameras:
    - workspace
    - closeup
    - top
    skills: true
    success_mode: final
    frame_every: 5
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
oracle:
  env:
    ROBOUSE_ORACLE_TOKEN: ${ROBOUSE_ORACLE_TOKEN:-}
---

You are controlling a simulated robot. Read the task below, then solve it by running the `robo` command in your shell (start with `robo info` and `robo observe`). Keep going until the task is done, then call `robo done` once. Do not stop to ask questions; there is no human to answer.

# Hidden centre of mass, box 1 of 3 (RLE-Bench)

Use the Panda robot to identify the hidden ballast quadrant of a sealed box. Identify the quadrant by the A/B/C/D markings on the box. RLE-Bench runs three such boxes, which look identical but have different masses, ballast placements and inertias; this task is box 1. Determine the answer correctly using as few control steps as possible. You cannot open the box or reset the experiment.

A fixed Franka Panda arm with a two-finger gripper stands at a table (top at z = 0.75 m). In front of it, on a 4 mm mat, lies a sealed 0.15 x 0.15 x 0.05 m box, initially centred at (0, 0, 0.779) m. Its rigid centred handle runs along x: an 80 x 16 x 12 mm bar at (0, 0, 0.860) m with 50 mm clearance below. The lid is marked with the letters A, B, C and D, one per quadrant. Inside, a hidden ballast sits in one quadrant; the box's mass and the ballast's mass and placement are not given. You may push, lift or rotate the box on the clear tabletop. You cannot open it. Control runs at 20 Hz.

**Scoring.** This task passes when the quadrant you submit is the one holding the ballast. RLE-Bench's score for its three boxes is the fraction answered correctly, so the mean over the three box tasks is RLE-Bench's score.

You have **12,000 control steps**. The episode ends when they are used up, so submit before then; without a submission the task scores 0.

**Controls.** `robo act` takes 7 numbers in [-1, 1]: three world-aligned end-effector position deltas (scaled by 0.05 m), three axis-angle deltas (scaled by 0.5 rad) and the gripper (+1 closes, -1 opens). `robo skill move` tracks a world-frame tool pose through the same controller. `robo skill submit A` (or B, C, D) commits your answer: once, final, with no correctness feedback and no step cost. Then call `robo done`.

**Observation.** `robo observe` gives RGB images from three cameras, `workspace`, `closeup` and `top`, 512 x 512 (the top camera looks straight down from (0.03, 0, 1.71) m with a 43 degree vertical field of view), and `joint_pos`, `joint_vel`, `eef_pos`, `eef_quat` (x, y, z, w), `gripper_pos` and `submitted`. No force/torque, object-pose, mass or contact readings are given.

## How to control the robot

You are the robot's policy. You act only through the `robo` command in your shell. There is no other way to move the robot, and you cannot read or change the simulator, the scoring, or other files to succeed; the episode server judges the final scene itself.

```
robo info                                   # action layout, skills, cameras (intrinsics), step budget
robo observe                                # robot proprioception, and one image per camera (paths printed; open them to look)
robo observe --camera C                     # one camera only: workspace, closeup, top
robo act A1 ... A7        [--repeat N]   # one control step (applied N times, N <= 50)
robo skill move X Y Z QX QY QZ QW [G] [S]   # track a world-frame tool pose for S steps (1-200, default 40); gripper G (+1 closes, -1 opens, default +1)
robo skill submit A                         # commit the quadrant (A, B, C or D): final, no feedback
robo done "short summary"                   # end the episode
robo give-up "reason"                       # end the episode without submitting (scores 0)
```

- Positions are in metres in the world frame, z up; quaternions are unit x, y, z, w.
- Every control step counts against the budget, including the steps `move` runs; observing is free.
- Skills are ordinary controllers: collisions and reach limits can stop the arm short. Use intermediate waypoints and re-observe.
