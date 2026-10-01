---
schema_version: '1.3'
task:
  name: robouse/pg-go1-corridor-push
  description: Walk the Go1 along a 0.9 m wide zig-zag corridor to its end while random shoves push it sideways, without touching the walls.
metadata:
  author_name: benchflow
  source_benchmark: 'MuJoCo Playground 0.2.0 (Google DeepMind, google-deepmind/mujoco_playground, Apache-2.0): Unitree robot models and joystick environments; walking policies trained for this track with Brax PPO on the GPU worker; MuJoCo Warp physics'
  source_task: Go1JoystickFlatTerrain policy with Playground's velocity-kick perturbation enabled
  suite: gpu-playground
  category: locomotion-navigation
  difficulty: hard
  hard_because: 'perturbations mid-episode: every 2-4 s a shove of 0.5-1.5 m/s knocks the robot off course in a random direction; the corridor leaves 13 cm on each side of the footprint, so the agent must keep re-centring; three legs with two 90-degree turns'
  language_instruction: Walk the Go1 along a 0.9 m wide zig-zag corridor to its end while random shoves push it sideways, without touching the walls.
  tags:
  - gpu-required
  - playground
  - quadruped
  - perturbation
  - narrow
  reference_solution: A* through the corridor on the privileged map and a heading controller at reduced speed, re-planned from the robot's current position after every command
  gpu: required (NVIDIA RTX, remote worker)
  robouse:
    id: pg-go1-corridor-push
    backend: gpu
    sim: playground
    env: go1-corridor-push
    seed: 0
    max_steps: 100
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

# Playground: Go1 down a zig-zag corridor while being shoved

You control a Unitree Go1 quadruped (about 0.6 m long; footprint radius 0.32 m). Walk it through the corridor to the goal at its far end (within 0.4 m of `goal`) without touching the walls and without falling. The corridor is 0.9 m wide and has two 90-degree turns. Every few seconds a random shove pushes the robot (the policy keeps it on its feet, but it gets knocked off course).

`robo observe` lists the wall segments (boxes) and the goal.

The step budget is 100 steps: every `robo act` step and every `robo skill` call counts as one.

## Scene and frames

World frame: x and y on the floor, z up, metres. The robot starts at the origin facing +x (yaw 0). `yaw_deg` is the heading measured counter-clockwise from +x.

## How to control the robot

You are the robot's high-level policy. The robot already knows how to walk: a learned locomotion policy (trained with reinforcement learning in MuJoCo Playground) turns your velocity command into leg motion 50 times a second. You act only through the `robo` command in your shell; you cannot read or change the simulator, the scoring, or other files to succeed.

```
robo info                              # command limits, skills, cameras, step budget
robo observe                           # base pose and velocity, fallen flag, contacts, and the task's map
robo observe --image [--camera C]      # also saves an image (cameras: chase, overview, head) and prints its path
robo act VX VY WZ                      # hold a body-frame velocity command for 0.5 s: VX forward m/s, VY left m/s, WZ turn rate rad/s (counter-clockwise)
robo act VX VY WZ --repeat N           # the same command for N x 0.5 s (N steps of the budget)
robo skill walk VX VY WZ SECONDS       # hold a command for up to 5 s; stops early if the robot touches something or falls (one step)
robo skill stand [SECONDS]             # stand still (zero command), default 1 s
robo done "short summary"              # end the episode and ask for scoring
robo give-up "reason"                  # end the episode without claiming success
```

- The robot does not track commands perfectly: it accelerates over a few tenths of a second, drifts sideways, and turns a little while walking straight. Re-observe and correct.
- Obstacles and walls are boxes given by their centre and half sizes in x and y. Collision is checked on the robot's footprint, a circle of `robot_radius_m` around its base: any overlap with an obstacle counts as a touch, and any touch fails the task (it is counted in `obstacle_contacts`; the episode goes on). If the robot falls (`fallen: true`) it cannot get up again.
- Success is judged after you call `robo done`: the condition must hold at that moment. `robo give-up`, running out of steps or time, or stopping without `robo done` all score 0. Call `robo done` exactly once.
