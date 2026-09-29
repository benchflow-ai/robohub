---
schema_version: '1.3'
task:
  name: robouse/pg-go1-find-ball
  description: Find the red ball hidden somewhere in a walled room using the robot's cameras, and stop next to it.
metadata:
  author_name: benchflow
  source_benchmark: 'MuJoCo Playground 0.2.0 (Google DeepMind, google-deepmind/mujoco_playground, Apache-2.0): Unitree robot models and joystick environments; walking policies trained for this track with Brax PPO on the GPU worker; MuJoCo Warp physics'
  source_task: Go1JoystickFlatTerrain policy; scene built for this track
  suite: gpu-playground
  category: locomotion-navigation
  difficulty: hard
  hard_because: 'partial observability: no map and no ball position; the ball sits behind one of four interior walls, so the agent must explore with the head camera, avoid walls it can only see, and stop within 0.6 m of the ball'
  language_instruction: Find the red ball hidden somewhere in a walled room using the robot's cameras, and stop next to it.
  tags:
  - gpu-required
  - playground
  - quadruped
  - camera-only
  - exploration
  reference_solution: A* to the ball on the privileged map (oracle token) and the heading controller
  gpu: required (NVIDIA RTX, remote worker)
  robouse:
    id: pg-go1-find-ball
    backend: gpu
    sim: playground
    env: go1-find-ball
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
---

You are controlling a simulated robot. Read the task below, then solve it by running the `robo` command in your shell (start with `robo info` and `robo observe`). Keep going until the task is done, then call `robo done` once. Do not stop to ask questions; there is no human to answer.

# Playground: Go1 finds the red ball (camera only)

You control a Unitree Go1 quadruped (about 0.6 m long; footprint radius 0.32 m) in an 8 x 8 m room (walls at x = +-4 and y = +-4) with four low interior walls. A red ball (30 cm across) lies somewhere in the room, hidden behind one of the interior walls. Find it and stop with the robot's base within 0.6 m of the ball's centre, without touching any wall and without falling.

`robo observe` gives only the robot's own state: no map and no ball position. Use the cameras: `head` looks forward from the robot, `chase` follows it from behind, `overview` looks down on the room at a steep angle (`robo observe --image --camera overview`).

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
