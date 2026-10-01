---
schema_version: '1.3'
task:
  name: robouse/driving-ramp-merge
  description: Merge from an on-ramp
metadata:
  author_name: benchflow
  source_benchmark: robouse original (driving suite; MetaDrive 0.4.3)
  source_task: driving-ramp-merge
  suite: driving
  category: traffic
  difficulty: hard
  tags:
  - driving
  - wheeled
  - metadrive
  - merge
  - on-ramp
  - traffic
  simulator: MetaDrive 0.4.3 (Apache-2.0)
  robouse:
    id: driving-ramp-merge
    backend: driving
    env: driving-ramp-merge
    seed: 0
    max_steps: 800
    camera: topdown
    cameras:
    - topdown
    - route_map
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

# Merge from an on-ramp

You drive a passenger car (MetaDrive's default sedan: 4.5 m long, 1.85 m wide, Bullet vehicle dynamics, no engine force above 80 km/h) on a road map built by the MetaDrive driving simulator. Map frame in metres: `x`, `y` on the ground, `heading_deg` 0 along +x and counter-clockwise positive; the top-down camera shows +y up. Traffic drives on the right. The two directions of a road are separated by a solid yellow centre line (drawn yellow in the top-down view), which you must never touch or cross. Lanes of your direction are numbered from the left: lane 0 is the one next to the yellow centre line. Roads are named by MetaDrive node pairs (e.g. `>>>->1S0_0_`). The navigation route is fixed: `robo observe` gives, under `route`, the distance travelled and remaining, the next roads (distance to their start, lane count and shape: straight, or a curve with its radius and turn angle) and the destination. Road users within 60 m are listed under `vehicles` and road objects (cones, warning triangles, barriers) under `obstacles`, with their position in your car's frame, their distance along your route and their lane. Other drivers follow MetaDrive's car-following model: they brake for what is ahead in their lane, but they do not yield to you at junctions and do not make room when you cut in. Simulated time advances only when you act; the scene is paused while you think.

## Task

You start on an on-ramp. It joins the right side of a three-lane highway as a short acceleration lane (lane 3 of 4, about 38 m long, then it ends). A long platoon of vehicles drives in the highway's right lane (lane 2) at about 50 km/h. Merge into lane 2 through a safe gap (or behind the platoon; you may stop at the end of the ramp to wait) before the acceleration lane ends, then drive to the destination and stop there. The route is 220 m long (7 roads); the destination is at (350.6, 3.5), at the end of road `2r0_2_->3S0_0_`. You start at (133.3, -8.2), heading 0 deg, in lane 0 of 1.

**Success:** you must bring the car to a stop inside the destination zone: from 5 m before the end of the route's final road to 5 m past it, in any lane of that road (`route.in_destination_zone`; `route.remaining_m` is the distance to the end of the final road, so stop with it between -5 and 5). The final road ends where the map ends, at `remaining_m` = 0: driving on past its end leaves the road (an out-of-road event), so in practice stop before `remaining_m` reaches 0. After you call `robo done` the car brakes fully for 10 steps; then it must be at rest (below 2 km/h) inside the zone, and at no time may it have collided with a vehicle or object, touched the road edge or sidewalk, left the road or touched a solid lane line (MetaDrive's out-of-road rule), or cut off another driver (a vehicle behind you in your lane closing in with less than a 1 s time gap, because you cut in front of it or braked hard in front of it). Violations are listed under `events` as they happen, and any event fails the task. Judged by the episode server from the simulated state.

**Controls.** `robo act STEER THROTTLE [--repeat N]` applies MetaDrive's own car action for N steps of 0.1 s: STEER in [-1, 1] sets the front-wheel angle (x 40 deg, positive = left); THROTTLE in [-1, 1] is engine force when positive and brake force when negative (full braking stops the car from 50 km/h in about 11 m); 0 coasts. During the 10-step settle after `robo done` the wheels are straightened and the brakes are fully on (STEER 0, THROTTLE -1). Skills (`robo info` lists their arguments): `follow_route SPEED_KMH METERS` keeps the current lane along the route at a set speed (lane-centred steering plus cruise control) for METERS; it does NOT brake for vehicles or obstacles and it does not slow for curves, but it reports the nearest vehicle or object ahead in the lane; if the lane ends (an acceleration lane) it keeps going straight off its end. `change_lane left|right [SPEED_KMH]` steers into the adjacent lane and centres in it; it does not check for traffic or solid lines. `stop [METERS]` brakes to a standstill while keeping the lane, spread over about METERS (0 = brake hard). `wait SECONDS` stays stopped with the brakes on.

**Observation.** `robo observe` reports `ego` (`pos`, `heading_deg`, `speed_kmh`, last `steering` and `throttle`, size), `lane` (`road`, `index` (0 = leftmost), `count`, `lateral_offset_m` from the lane centre (+ = left), `width_m`, `station_m` along the road, `road_length_m`, `shape`, `left_line` / `right_line` types: broken lines may be crossed, solid ones may not), `route` (`travelled_m`, `remaining_m`, `total_m`, `route_completion`, `next_roads`, `destination`, `in_destination_zone`, and `give_way` where a give-way rule applies), `vehicles` (per vehicle: `id`, `kind`, `rel_ahead_m` / `rel_left_m` in your car's frame, `distance_m`, `speed_kmh`, `heading_rel_deg`, its `road` and `lane`, and for vehicles on your route `route_ahead_m` (negative = behind), `gap_m` (bumper to bumper) and `in_my_lane`; `broken_down` for a stopped car with hazard lights), `obstacles` (same position fields) and `events`. Cameras: `topdown` (default; follows the car, about 96 m across) and `route_map` (the whole route); `robo observe --image [--camera route_map]` saves one.

The step budget is 800 steps (80 s of driving).

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
