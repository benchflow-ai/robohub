---
schema_version: '1.3'
task:
  name: robouse/ms-plug-charger
  description: Pick up the charger and plug its two prongs fully into the wall receptacle.
metadata:
  author_name: benchflow
  source_benchmark: ManiSkill3 3.0.1 (Hillbot / UC San Diego, haosulab/ManiSkill, Apache-2.0); SAPIEN ray-traced rendering
  source_task: PlugCharger-v1
  suite: gpu-maniskill
  category: contact-rich-insertion
  difficulty: hard
  hard_because: 'very tight tolerance: 0.5 mm clearance around each prong; success needs the charger within 5 mm and 0.2 rad of the fully inserted pose; poses randomized per seed'
  language_instruction: Pick up the charger and plug its two prongs fully into the wall receptacle.
  tags:
  - gpu-required
  - maniskill
  - tight-tolerance
  reference_solution: ManiSkill's motion-planning solution for PlugCharger ported to the skills (angled grasp, align 5 cm in front of the socket, refine, insert)
  gpu: required (NVIDIA RTX, remote worker)
  robouse:
    id: ms-plug-charger
    backend: gpu
    sim: maniskill
    env: plug-charger
    seed: 0
    max_steps: 40
    skills: true
    success_mode: final
    obs_mode: state
    frame_every: 1
    linger_s: 5
    ready_timeout_s: 900
    rpc_timeout_s: 900
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

# ManiSkill: plug a charger into its socket

Pick up the charger (a small block with two metal prongs) and plug it into the receptacle on the wall: both prongs must go into the receptacle's two holes. The holes leave 0.5 mm of clearance around each prong. The task is solved when the charger is within 5 mm and 0.2 rad of the fully inserted pose.

`robo observe` gives `charger_pose` (the charger's frame: +x points along the prongs, the prongs stick out at +x), `receptacle_pose` and `goal_pose` (where `charger_pose` must end up when fully plugged in).

The step budget is 40 steps: every `robo act` step and every `robo skill` call counts as one.

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
