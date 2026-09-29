---
schema_version: '1.3'
task:
  name: robouse/crazyflie-waypoints
  description: Waypoint tour
metadata:
  author_name: benchflow
  source_benchmark: robouse original (crazyflie suite; Bitcraze Crazyflie 2 from MuJoCo Menagerie)
  source_task: crazyflie-waypoints
  suite: crazyflie
  category: navigation
  difficulty: easy
  tags:
  - aerial
  - quadrotor
  - crazyflie
  - menagerie
  - mujoco
  robouse:
    id: crazyflie-waypoints
    backend: crazyflie
    env: crazyflie-waypoints
    seed: 0
    max_steps: 1400
    camera: cf0/chase
    cameras:
    - cf0/chase
    - cf0/fpv
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
---

You are controlling a simulated robot. Read the task below, then solve it by running the `robo` command in your shell (start with `robo info` and `robo observe`). Keep going until the task is done, then call `robo done` once. Do not stop to ask questions; there is no human to answer.

# Waypoint tour

You fly a Bitcraze Crazyflie 2 nano-quadrotor (MuJoCo Menagerie model; 27 g and 9 cm across each) in an indoor flight arena. World frame in metres: +x east, +y north, +z up, floor at z = 0. The arena is 8 m x 6 m (x from -4 to 4, y from -3 to 3) and 3 m high, closed by walls and a ceiling. A full-height partition wall at x = 1.5 separates the west hall from the east room; the only way through it is a 0.36 m x 0.36 m window centred at (1.5, 1.8, 1.0). The west hall has a `home` pad, a teal pad `A`, three square racing gates (orange frames, 0.5 m openings) and three swarm pads `S0`, `S1`, `S2`; the east room has a yellow pad `B` and a 0.9 m high shelf with a white landing platform (0.3 m square) on top. Pads are 0.3 m squares. Every position is listed under `arena` in `robo observe`. Touching anything other than the floor, a pad or the platform is a crash and fails the task.

## Task

Take off from the `home` pad, fly through these four waypoints in this order: (-3.0, -1.0, 1.0), (-1.0, -2.0, 1.5), (0.5, 0.5, 0.6), (-2.0, 1.5, 2.0), then land on pad `A` at (-1.5, -1.2).

**Success:** each waypoint was reached in order (the drone's centre within 0.10 m of it), and the drone rests on pad `A` with no crash; `progress.waypoints_reached` counts the waypoints reached so far. Judged by the episode server from the simulated state after you call `robo done` and the drones have held still for 10 steps. A drone "rests on" a pad or the platform when it touches it, moves slower than 0.1 m/s and is tilted less than 20 degrees.

**Controls.** `robo act VX VY VZ YAW_RATE` sets the velocity setpoint for the next step(s), each in [-1, 1]: VX, VY, VZ along world x, y, z (x 1 m/s) and YAW_RATE (x 90 deg/s, counter-clockwise seen from above). One step is 50 ms; `--repeat N` holds a setpoint for N steps. An onboard cascaded controller (velocity -> attitude -> thrust and body moments) tracks the setpoint. An all-zero action holds the current position; resting on a surface with VZ <= 0 the motors stay off, so take off with a positive VZ. Skills (`robo info` lists them with their arguments): `takeoff [Z]`, `goto X Y Z [TOL]` (straight line, stops at the point; it does not avoid obstacles), `turn YAW_DEG`, `land` (straight down onto whatever is below, then motors off) and `hover [SECONDS]`.

**Observation.** `robo observe` reports `drones` (one entry: `pos`, `vel`, `yaw_deg`, `roll_deg`, `pitch_deg`, `landed_on` (the pad or surface it rests on, else null) and `motors_on`), `arena`, `progress` where the task counts something, and `safety_events`. Cameras: `cf0/chase` (default, behind the drone), `cf0/fpv` (forward), `overview` (the west hall) and `east_room`.

The step budget is 1400 steps (70 s of flight).

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
