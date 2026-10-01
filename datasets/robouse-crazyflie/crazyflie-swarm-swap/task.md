---
schema_version: '1.3'
task:
  name: robouse/crazyflie-swarm-swap
  description: Swap two drones
metadata:
  author_name: benchflow
  source_benchmark: robouse original (crazyflie suite; Bitcraze Crazyflie 2 from MuJoCo Menagerie)
  source_task: crazyflie-swarm-swap
  suite: crazyflie
  category: multi-robot
  difficulty: hard
  tags:
  - aerial
  - quadrotor
  - crazyflie
  - menagerie
  - mujoco
  - swarm
  robouse:
    id: crazyflie-swarm-swap
    backend: crazyflie
    env: crazyflie-swarm-swap
    seed: 0
    max_steps: 2000
    camera: swarm
    cameras:
    - swarm
    - cf0/chase
    - cf1/chase
    - cf2/chase
    - overview
    - east_room
    skills: true
    success_mode: final
agent:
  timeout_sec: 1800
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

# Swap two drones

You fly three Bitcraze Crazyflie 2 nano-quadrotors (cf0, cf1, cf2) (MuJoCo Menagerie model; 27 g and 9 cm across each) in an indoor flight arena. World frame in metres: +x east, +y north, +z up, floor at z = 0. The arena is 8 m x 6 m (x from -4 to 4, y from -3 to 3) and 3 m high, closed by walls and a ceiling. A full-height partition wall at x = 1.5 separates the west hall from the east room; the only way through it is a 0.36 m x 0.36 m window centred at (1.5, 1.8, 1.0). The west hall has a `home` pad, a teal pad `A`, three square racing gates (orange frames, 0.5 m openings) and three swarm pads `S0`, `S1`, `S2`; the east room has a yellow pad `B` and a 0.9 m high shelf with a white landing platform (0.3 m square) on top. Pads are 0.3 m squares. Every position is listed under `arena` in `robo observe`. Touching anything other than the floor, a pad or the platform is a crash and fails the task; so does a collision between drones.

## Task

Three drones rest on pads: cf0 on `S0`, cf1 on `S1`, cf2 on `S2`. Swap cf0 and cf2: cf0 must end on `S2` and cf2 on `S0`, both landed; cf1 must end on `S1` (it may move in between).

**Success:** cf0 rests on `S2`, cf1 on `S1` and cf2 on `S0`; any crash, or two drones ever closer than 0.2 m (centre to centre), fails the task. Judged by the episode server from the simulated state after you call `robo done` and the drones have held still for 10 steps. A drone "rests on" a pad or the platform when it touches it, moves slower than 0.1 m/s and is tilted less than 20 degrees.

**Controls.** `robo act VX0 VY0 VZ0 VX1 VY1 VZ1 VX2 VY2 VZ2` sets the velocity setpoints of drones cf0, cf1 and cf2 for the next step(s), each in [-1, 1] (x 1 m/s along world x, y, z); headings stay fixed. One step is 50 ms; `--repeat N` holds the setpoints for N steps. A drone whose three values are all zero holds its position; resting on a surface with VZ <= 0 its motors stay off. Skills: `takeoff_all [Z]`, `goto ID X Y Z [TOL]` (one drone flies in a straight line while the others hold position), `goto_all X0 Y0 Z0 X1 Y1 Z1 X2 Y2 Z2 [TOL]` (all at once, straight lines, no collision avoidance), `land ID`, `land_all` and `hover [SECONDS]`.

**Observation.** `robo observe` reports `drones` (per drone: `id`, `pos`, `vel`, `yaw_deg`, `roll_deg`, `pitch_deg`, `landed_on`, `motors_on`), `arena` and `safety_events`. Cameras: `swarm` (default, the swarm pads from the south), `cf0/chase`, `cf1/chase`, `cf2/chase`, `overview` (the west hall) and `east_room`.

The step budget is 2000 steps (100 s of flight).

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
