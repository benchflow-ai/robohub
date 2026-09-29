---
schema_version: '1.3'
task:
  name: robouse/il-factory-peg-insert
  description: Insert the held 8 mm peg into the hole in the fixed socket, all the way down.
metadata:
  author_name: benchflow
  source_benchmark: 'NVIDIA Isaac Lab 2.3.0 (isaac-sim/IsaacLab, BSD-3-Clause) on Isaac Sim 5.1 (NVIDIA Isaac Sim license): the Factory assembly environments (Narang et al., RSS 2022), PhysX GPU dynamics, RTX rendering'
  source_task: Isaac-Factory-PegInsert-Direct-v0
  suite: gpu-isaaclab
  category: contact-rich-assembly
  difficulty: hard
  hard_because: 'tight tolerance: 8.0 mm peg in an 8.1 mm hole (0.05 mm clearance per side); success needs the peg centred within 2.5 mm and seated to within 1 mm of the bottom (4 % of the 25 mm socket depth); compliant control with smoothed actions'
  language_instruction: Insert the held 8 mm peg into the hole in the fixed socket, all the way down.
  tags:
  - gpu-required
  - isaaclab
  - tight-tolerance
  - isaac-sim
  reference_solution: 'scripted: align the peg''s base over the hole from the privileged fixed-part pose, then descend with small lateral corrections'
  gpu: required (NVIDIA RTX, remote worker)
  robouse:
    id: il-factory-peg-insert
    backend: gpu
    sim: isaaclab
    env: factory-peg-insert
    seed: 0
    max_steps: 300
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

# Isaac Lab Factory: insert an 8 mm peg

The robot holds an 8 mm peg a few centimetres above a socket with an 8.1 mm hole (25 mm deep). Insert the peg into the hole and push it all the way down. The task is solved when the peg's base is within 2.5 mm of the hole's axis and within 1 mm of the bottom of the hole.

The step budget is 300 steps: every `robo act` step and every `robo skill` call counts as one.

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
