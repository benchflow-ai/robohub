---
schema_version: '1.3'
task:
  name: robouse/ms-pick-moving-cube-vision
  description: Grasp the red cube while it slides across the table and hold it at the goal point, tracking it only in camera images.
metadata:
  author_name: benchflow
  source_benchmark: ManiSkill3 3.0.1 (Hillbot / UC San Diego, haosulab/ManiSkill, Apache-2.0); SAPIEN ray-traced rendering
  source_task: PickCube-v1 with a conveyor, camera-only observations
  suite: gpu-maniskill
  category: dynamic
  difficulty: hard
  hard_because: 'dynamic object under partial observability: the cube moves 2.5 mm per control step and is only visible in images (plus depth readings, each of which costs a step), so position and velocity must be estimated from successive frames'
  language_instruction: Grasp the red cube while it slides across the table and hold it at the goal point, tracking it only in camera images.
  tags:
  - gpu-required
  - maniskill
  - camera-only
  - dynamic
  reference_solution: the visual-servo policy on the privileged state (oracle token)
  gpu: required (NVIDIA RTX, remote worker)
  robouse:
    id: ms-pick-moving-cube-vision
    backend: gpu
    sim: maniskill
    env: pick-moving-cube-vision
    seed: 0
    max_steps: 150
    skills: true
    success_mode: final
    obs_mode: vision
    frame_every: 1
    linger_s: 5
    ready_timeout_s: 900
    rpc_timeout_s: 900
    visible_fields:
    - tcp
    - qpos
    - gripper
    - goal_pos
    cameras:
    - front
    - top
agent:
  timeout_sec: 2400
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

# ManiSkill: pick a moving cube, cameras only

A 4 cm red cube slides across the table at a constant 5 cm/s, as if on a conveyor belt: it moves 2.5 mm every control step (0.05 s of simulated time), in a straight line along the y axis, until it is grasped or lifted. When it passes y = 0.3 m (or -0.3 m) it is carried off the end of the belt, out of reach, and the task can no longer be solved. Simulated time only passes when the robot acts: each `robo act` step is 0.05 s, and a `robo skill` takes as long as its motion does. Grasp the cube and bring it to the goal point (`goal_pos` in `robo observe`, in the air above the table), holding it there with the arm still. The task is solved when the cube's centre is within 2.5 cm of the goal point and the robot is static.

This is a camera-only task: `robo observe` returns only the robot's own state (TCP pose, joint angles, gripper). Every observation saves images from the cameras listed in `robo info` (calibrated: each has a 3x4 projection matrix P with [u*w, v*w, w] = P [x, y, z, 1] in image pixels, u to the right, v down). The cameras are RGB-D: `robo skill depth CAMERA U V` returns the world point seen at pixel (U, V) of that camera (with about 2 mm of sensor noise); it costs one step like every skill.

The step budget is 150 steps: every `robo act` step and every `robo skill` call counts as one.

## Scene and frames

World frame: x points away from the robot across the table, y to the robot's left, z up; the table top is z = 0 and the robot base sits at (-0.615, 0, 0). Positions are in metres. Poses in `robo observe` give `pos`, `quat_wxyz` (a unit quaternion w, x, y, z) and `rpy_deg` (roll, pitch, yaw in degrees: R = Rz(yaw) Ry(pitch) Rx(roll)).

The gripper's tool point (TCP) sits between the fingertips. In the TCP frame, +z points out between the fingers (the approach direction) and +y is the axis along which the fingers close. Pointing straight down with the fingers closing along world y is roll 180, pitch 0, yaw 0 (quaternion 0 1 0 0); turning that about the vertical is a change of yaw.

## How to control the robot

You are the robot's policy. You act only through the `robo` command in your shell; you cannot read or change the simulator, the scoring, or other files to succeed. The episode server checks the task's physical success condition on the final simulated state itself.

```
robo info                                      # action space, skills, cameras, step budget
robo observe                                   # TCP pose, joint angles, gripper, and the task's object state
robo observe --image [--camera C]              # also saves a ray-traced image (cameras: scene, front, top, wrist) and prints its path
robo skill move X Y Z                          # motion-planned move of the TCP to (X, Y, Z), keeping its orientation
robo skill move X Y Z ROLL PITCH YAW           # ... to a full pose, orientation in degrees
robo skill move X Y Z QW QX QY QZ              # ... orientation as a quaternion
robo skill move ... speed=0.3                  # any move, slowed down (speed 0.05-1, default 1)
robo skill gripper open|close                  # open or close the fingers
robo skill gripper -0.6                        # finger target in [-1, 1] (-1 closed, +1 open); a loose grip lets a held object pivot
robo skill wait [N]                            # hold still for N control steps (default 10)
robo act DX DY DZ DRX DRY DRZ GRIP [--repeat N]   # low-level: one 0.05 s control step (see below)
robo done "short summary"                      # end the episode and ask for scoring
robo give-up "reason"                          # end the episode without claiming success
```

- `move` plans a path for the arm itself (straight-line screw motion first, then a sampling planner) but does not avoid the objects on the table: moving through them pushes them. It reports the TCP position it reached and the remaining position and orientation error. Contact (a held object touching something) can stop the arm short of the target.
- `act` is ManiSkill's end-effector delta-pose controller: DX, DY, DZ move the TCP by up to 0.1 m per step (value x 0.1 m), DRX, DRY, DRZ rotate it by up to 0.1 rad per step about world-aligned axes, and GRIP is the absolute finger target (+1 open, -1 closed). Values are clipped to [-1, 1].
- Success is judged after you call `robo done`, with the robot holding still for 10 control steps: the condition must hold at that moment. `robo give-up`, running out of steps or time, or stopping without `robo done` all score 0. Call `robo done` exactly once.
