---
schema_version: '1.3'
task:
  name: robouse/gs-pour-water
  description: Pick up the glass of water and pour at least 80 % of the water into the bowl, spilling at most 10 %.
metadata:
  author_name: benchflow
  source_benchmark: 'Genesis 1.4.2 (Genesis-Embodied-AI/Genesis, Apache-2.0): SPH liquid coupled to rigid bodies, CUDA backend'
  source_task: ''
  suite: gpu-genesis
  category: fluids
  difficulty: hard
  hard_because: 'fluid (about 2,000 SPH particles): the pour point, tilt angle and tilt rate decide how much lands in the bowl; the liquid is only visible in images and its state is not in the observation; glass and bowl positions randomized per seed'
  language_instruction: Pick up the glass of water and pour at least 80 % of the water into the bowl, spilling at most 10 %.
  tags:
  - gpu-required
  - genesis
  - fluid
  reference_solution: 'scripted: top grasp of the glass near its rim, carry it beside the bowl with the rim over the bowl''s near wall, tilt in four steps to 130 degrees with pauses, set it back upright'
  gpu: required (NVIDIA RTX, remote worker)
  robouse:
    id: gs-pour-water
    backend: gpu
    sim: genesis
    env: pour-water
    seed: 0
    max_steps: 60
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

# Genesis: pour water into a bowl

A glass (5 cm square inside, 10 cm tall, transparent) full of water stands on the floor, and a grey bowl (13 cm square inside, 6 cm deep) stands nearby. Pick up the glass and pour the water into the bowl. The task is solved when at least 80 % of the water (by particle count) is inside the bowl and at most 10 % has been spilled (anywhere that is neither the bowl nor the glass).

`robo observe` gives the glass pose (its base centre) and the bowl's base centre, with their inner widths, heights and wall thicknesses. The water itself is not in the observation; look at the camera images.

The step budget is 60 steps: every `robo act` step and every `robo skill` call counts as one.

## Scene and frames

World frame: the robot base is at the origin, x points forward from the robot, y to its left, z up; everything stands on the floor (z = 0). Positions are in metres. Poses in `robo observe` give `pos`, `quat_wxyz` and `rpy_deg` (roll, pitch, yaw in degrees: R = Rz(yaw) Ry(pitch) Rx(roll)).

The gripper's tool point (TCP) sits between the fingertips; its +z axis points out between the fingers and the fingers close along its y axis. Pointing straight down with the fingers closing along world y is roll 180, pitch 0, yaw 0. Rotating from there about the world x axis (decreasing roll from 180) tips whatever the hand holds sideways towards +y.

## How to control the robot

You are the robot's policy. You act only through the `robo` command in your shell; you cannot read or change the simulator, the scoring, or other files to succeed. The episode server measures the liquid itself on the final simulated state.

```
robo info                                  # action space, skills, cameras, step budget
robo observe                               # TCP pose, gripper, glass and bowl poses and sizes
robo observe --image [--camera C]          # also saves an image (cameras: scene, front, top) and prints its path
robo skill move X Y Z                      # straight-line move of the TCP to (X, Y, Z), keeping its orientation
robo skill move X Y Z ROLL PITCH YAW       # ... to a full pose (degrees); the orientation is interpolated on the way
robo skill move ... speed=0.5              # slower (0.1) or faster (up to 2) than the default 0.25 m/s and 45 deg/s
robo skill gripper open|close              # open or close the fingers
robo skill wait [N]                        # hold still for N control steps of 0.05 s (default 10)
robo act DX DY DZ DRX DRY DRZ GRIP [--repeat N]   # low-level: one 0.05 s step; TCP moves up to 1 cm and turns up to 3 degrees per unit (world axes); GRIP +1 open, -1 closed
robo done "short summary"                  # end the episode and ask for scoring
robo give-up "reason"                      # end the episode without claiming success
```

- The liquid is simulated: it sloshes when the glass accelerates and pours when the glass tips past the point where the surface reaches the rim. It keeps flowing while you wait. Liquid on the floor stays there.
- Grasp model: closing the fingers around the glass (the TCP inside the glass's footprint, between its floor and rim) holds it rigidly until the fingers open.
- Success is judged after you call `robo done`, with the robot holding still for 10 control steps: the condition must hold at that moment. `robo give-up`, running out of steps or time, or stopping without `robo done` all score 0. Call `robo done` exactly once.
