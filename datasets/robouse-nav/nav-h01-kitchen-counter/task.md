---
schema_version: '1.3'
task:
  name: robouse/nav-h01-kitchen-counter
  description: Find the kitchen counter
metadata:
  author_name: benchflow
  source_benchmark: robouse original (nav suite; procedurally generated homes, protocol of Dimensional's Can Jev Nav?)
  source_task: nav-h01-kitchen-counter
  suite: nav
  category: object-goal navigation
  difficulty: easy
  tags:
  - navigation
  - mobile-base
  - spatial-language
  geodesic_m: 6.36
  detour: 0.94
  robouse:
    id: nav-h01-kitchen-counter
    backend: nav
    env: nav-h01-kitchen-counter
    seed: 0
    max_steps: 1200
    camera: local
    cameras:
    - local
    skills: true
    success_mode: final
    frame_every: 2
    nav:
      scene:
        bounds:
        - -0.5
        - -0.5
        - 9.02561753780617
        - 9.000371587211436
        walls:
        - - 4.263
          - 0
          - 8.676
          - 0.15
        - - 4.263
          - 8.5
          - 8.676
          - 0.15
        - - 0
          - 4.25
          - 0.15
          - 8.65
        - - 8.526
          - 4.25
          - 0.15
          - 8.65
        - - 4.769
          - 5.291
          - 0.15
          - 1.537
        - - 4.769
          - 7.838
          - 0.15
          - 1.324
        - - 1.252
          - 4.523
          - 2.504
          - 0.15
        - - 4.203
          - 4.523
          - 1.133
          - 0.15
        - - 4.769
          - 1.339
          - 0.15
          - 2.677
        - - 4.769
          - 4.08
          - 0.15
          - 0.885
        - - 5.581
          - 4.523
          - 1.624
          - 0.15
        - - 8.052
          - 4.523
          - 0.947
          - 0.15
        objects:
        - id: plant_0
          label: plant
          box:
          - 0.32
          - 0.667
          - 0.45
          - 0.45
          room: bedroom
        - id: wardrobe_1
          label: wardrobe
          box:
          - 4.374
          - 1.323
          - 0.6
          - 1.2
          room: bedroom
        - id: nightstand_2
          label: nightstand
          box:
          - 2.542
          - 0.32
          - 0.5
          - 0.45
          room: bedroom
        - id: trash_can_3
          label: trash can
          box:
          - 1.843
          - 4.228
          - 0.4
          - 0.4
          room: bedroom
        - id: armchair_4
          label: armchair
          box:
          - 0.52
          - 5.204
          - 0.85
          - 0.85
          room: living
        - id: bookshelf_5
          label: bookshelf
          box:
          - 3.316
          - 7.905
          - 0.35
          - 1.0
          room: living
        - id: wardrobe_6
          label: wardrobe
          box:
          - 5.464
          - 1.147
          - 1.2
          - 0.6
          room: bedroom
        - id: plant_7
          label: plant
          box:
          - 8.206
          - 0.739
          - 0.45
          - 0.45
          room: bedroom
        - id: kitchen_counter_8
          label: kitchen counter
          box:
          - 8.131
          - 6.88
          - 0.6
          - 2.0
          room: kitchen
        - id: fridge_9
          label: fridge
          box:
          - 5.467
          - 4.968
          - 0.8
          - 0.7
          room: kitchen
        - id: oven_10
          label: oven
          box:
          - 5.775
          - 8.055
          - 0.65
          - 0.7
          room: kitchen
        rooms:
        - kind: bedroom
          box:
          - 2.3845763697044235
          - 2.2612694692901654
          - 4.769152739408847
          - 4.522538938580331
        - kind: living
          box:
          - 2.3845763697044235
          - 6.5114552628958835
          - 4.769152739408847
          - 3.9778326486311055
        - kind: bedroom
          box:
          - 6.6473851386075085
          - 2.2612694692901654
          - 3.7564647983973227
          - 4.522538938580331
        - kind: kitchen
          box:
          - 6.6473851386075085
          - 6.5114552628958835
          - 3.7564647983973227
          - 3.9778326486311055
        seed: 1001
      target:
        id: kitchen_counter_8
        label: kitchen counter
        box:
        - 8.131
        - 6.88
        - 0.6
        - 2.0
        room: kitchen
      spawn:
      - 1.011
      - 8.126
      spawn_yaw_deg: 180
      geodesic_m: 6.36
      success_radius: 1.0
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

# Find the kitchen counter

You control a mobile robot base (0.25 m radius) in a home: rooms joined by doorways, with furniture along the walls. You do not get a map or world coordinates: `robo observe` describes the scene from the robot's point of view.

## Task

Drive to the kitchen counter and stop next to it.

**Success:** after you call `robo done`, the robot is within 1.0 m of the kitchen counter's footprint and can see it (a straight line from the robot to the nearest point of the kitchen counter crosses no wall or furniture). Judged by the episode server after the robot has held still for 10 steps. Scored as in Dimensional's study: success, SPL (success weighted by the shortest path over the path driven), SoftSPL, collisions, time to target and path smoothness are all recorded.

**Controls.** `robo act VX VY WZ` sets the body-frame velocity for the next step(s): VX forward (m/s, up to +-0.5), VY to the left (m/s, up to +-0.3) and WZ counter-clockwise (deg/s, up to +-46); one step is 0.1 s and `--repeat N` holds it for N steps. Moves into walls or furniture slide along them or stop. Skills: `drive X Y YAW [SECONDS]` (Dimensional's discrete command: X forward|none|backward, Y left|none|right, YAW turn_left|none|turn_right, at 0.5 m/s and 0.8 rad/s, held for SECONDS, default 0.5), `turn DEG` (in place, counter-clockwise positive) and `forward METRES` (straight ahead; stops when blocked).

**Observation.** `robo observe` returns a WorldState: `goal` (label, `bearing` as one of ahead, ahead_left, left, behind_left, behind, behind_right, right, ahead_right, `bearing_deg`, `distance` as touching < 0.5 m, near < 1.5 m, mid < 4 m or far, `distance_m`, `visible`, `arrived`), `objects` (furniture the robot can see within 6 m, with bearing and distance), `way_to_target` (whether the straight line toward the goal is `blocked`, `blocked_by` what, `clear_m`, and `open_sides`), `free_space` (clear distance and open / narrow / blocked along the eight bearings) and `robot` (motion, last command, what happened over the last 8 s: `moved_m`, `turned_deg`, `target_closer_m`, `pattern` stuck / progressing / not_progressing, and `collisions`). `robo observe --image` saves an egocentric top-down view of what the robot can see (ahead is up; the target is red).

The step budget is 1200 steps (120 s).

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
