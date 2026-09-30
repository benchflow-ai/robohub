---
schema_version: '1.3'
task:
  name: robouse/il-factory-nut-thread-noisy
  description: Thread the held M16 nut onto the bolt when the measured bolt position is off by a few millimetres.
metadata:
  author_name: benchflow
  source_benchmark: 'NVIDIA Isaac Lab 2.3.0 (isaac-sim/IsaacLab, BSD-3-Clause) on Isaac Sim 5.1 (NVIDIA Isaac Sim license): the Factory assembly environments (Narang et al., RSS 2022), PhysX GPU dynamics, RTX rendering'
  source_task: Isaac-Factory-NutThread-Direct-v0 with 3 mm observation noise
  suite: gpu-isaaclab
  category: contact-rich-assembly
  difficulty: hard
  hard_because: 'noisy sensors plus contact-rich screwing: the reported bolt position has a fixed error of about 3 mm (per seed), more than the 2.5 mm centring tolerance, so the nut must be centred by feel or from images before it will thread; hundreds of small steps'
  language_instruction: Thread the held M16 nut onto the bolt when the measured bolt position is off by a few millimetres.
  tags:
  - gpu-required
  - isaaclab
  - noisy-sensors
  - contact-rich
  - long-horizon
  reference_solution: 'scripted with the true bolt pose (privileged): align, then press down while turning clockwise'
  gpu: required (NVIDIA RTX, remote worker)
  robouse:
    id: il-factory-nut-thread-noisy
    backend: gpu
    sim: isaaclab
    env: factory-nut-thread-noisy
    seed: 0
    max_steps: 600
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
oracle:
  env:
    ROBOUSE_ORACLE_TOKEN: ${ROBOUSE_ORACLE_TOKEN:-}
---

You are controlling a simulated robot. Read the task below, then solve it by running the `robo` command in your shell (start with `robo info` and `robo observe`). Keep going until the task is done, then call `robo done` once. Do not stop to ask questions; there is no human to answer.

# Isaac Lab Factory: thread a nut with a noisy bolt estimate

The robot holds an M16 nut just above an M16 bolt (thread pitch 2 mm). Screw the nut onto the bolt: centre it, then turn it clockwise (RZ, which in this task only turns clockwise: RZ = +1 is the fastest turn, RZ = -1 no turn) while pressing down. The task is solved (Factory's condition) when the nut's bottom face is within 2.5 mm of the bolt's axis, has come down to within 0.75 mm (0.375 pitch) above a point 1.5 pitches below the top of the thread, and the hand's yaw has turned past zero (the hand starts at about +105 degrees and turns clockwise).

The measured position of the bolt (`fixed_part_position_estimate`) is off by a few millimetres in x and y (a fixed calibration error, the same all episode). Use the cameras and what happens when the nut touches the bolt to find it.

The step budget is 600 steps: every `robo act` step and every `robo skill` call counts as one.

## Scene and frames

World frame: metres, z up. The fixed part (hole, gear base or bolt) is bolted to the table; the robot already holds the other part a few centimetres above it, as Factory starts every episode. `held_part_base` is the pose of the held part's base (the peg's bottom end, the gear's hub, the nut's bottom face); `fixed_part_position_estimate` is where the fixed part's top centre is measured to be.

## How to control the robot

You are the robot's policy. You act only through the `robo` command in your shell; you cannot read or change the simulator, the scoring, or other files to succeed. The episode server checks Factory's own success condition on the final simulated state.

```
robo info                                   # action space, skills, cameras, step budget
robo observe                                # fingertip pose, held part pose, fixed part estimate and dimensions
robo observe --image [--camera C]           # also saves an RTX image (cameras: scene, close, side) and prints its path
robo act DX DY DZ RX RY RZ [--repeat N]     # one 1/15 s control step of Factory's task-space impedance controller
robo skill hold [N]                         # zero command for N steps (default 15)
robo done "short summary"                   # end the episode and ask for scoring
robo give-up "reason"                       # end the episode without claiming success
```

- `act` is Factory's action: DX, DY, DZ in [-1, 1] set a position target up to 2 cm from the current fingertip position; RX, RY, RZ in [-1, 1] rotate the target by up to 0.097 rad. Roll and pitch are held level (the gripper keeps pointing down), so only RZ turns the hand. The target is clamped to within 5 cm of the fixed part, and actions are smoothed (an exponential moving average), so the hand responds over several steps. The controller is compliant: pushing against a surface builds force rather than moving the part.
- Success is judged after you call `robo done`, with the robot holding still for 10 control steps. `robo give-up`, running out of steps or time, or stopping without `robo done` all score 0. Call `robo done` exactly once.
