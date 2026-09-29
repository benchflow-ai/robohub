---
schema_version: '1.3'
task:
  name: robouse/nav-h07-armchair
  description: Find the armchair
metadata:
  author_name: benchflow
  source_benchmark: robouse original (nav suite; procedurally generated homes, protocol of Dimensional's Can Jev Nav?)
  source_task: nav-h07-armchair
  suite: nav
  category: object-goal navigation
  difficulty: medium
  tags:
  - navigation
  - mobile-base
  - spatial-language
  geodesic_m: 9.94
  detour: 1.47
  robouse:
    id: nav-h07-armchair
    backend: nav
    env: nav-h07-armchair
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
        - 9.809789122813328
        - 9.179994502968002
        walls:
        - - 4.655
          - 0
          - 9.46
          - 0.15
        - - 4.655
          - 8.68
          - 9.46
          - 0.15
        - - 0
          - 4.34
          - 0.15
          - 8.83
        - - 9.31
          - 4.34
          - 0.15
          - 8.83
        - - 5.749
          - 4.442
          - 1.816
          - 0.15
        - - 8.528
          - 4.442
          - 1.563
          - 0.15
        - - 4.841
          - 5.026
          - 0.15
          - 1.168
        - - 4.841
          - 7.662
          - 0.15
          - 2.036
        - - 4.841
          - 0.334
          - 0.15
          - 0.668
        - - 4.841
          - 3.14
          - 0.15
          - 2.604
        - - 2.42
          - 4.442
          - 4.991
          - 0.15
        objects:
        - id: desk_0
          label: desk
          box:
          - 1.559
          - 4.022
          - 1.3
          - 0.65
          room: office
        - id: trash_can_1
          label: trash can
          box:
          - 0.295
          - 0.762
          - 0.4
          - 0.4
          room: office
        - id: bed_2
          label: bed
          box:
          - 0.895
          - 5.914
          - 1.6
          - 2.0
          room: bedroom
        - id: trash_can_3
          label: trash can
          box:
          - 3.628
          - 4.737
          - 0.4
          - 0.4
          room: bedroom
        - id: plant_4
          label: plant
          box:
          - 5.825
          - 4.122
          - 0.45
          - 0.45
          room: bathroom
        - id: bathtub_5
          label: bathtub
          box:
          - 8.815
          - 2.744
          - 0.8
          - 1.7
          room: bathroom
        - id: trash_can_6
          label: trash can
          box:
          - 8.572
          - 0.295
          - 0.4
          - 0.4
          room: bathroom
        - id: tv_stand_7
          label: tv stand
          box:
          - 8.835
          - 5.337
          - 0.45
          - 1.6
          room: living
        - id: armchair_8
          label: armchair
          box:
          - 6.961
          - 8.16
          - 0.85
          - 0.85
          room: living
        rooms:
        - kind: office
          box:
          - 2.4204355467477905
          - 2.2209065326892174
          - 4.840871093495581
          - 4.441813065378435
        - kind: bedroom
          box:
          - 2.4204355467477905
          - 6.5609037841732185
          - 4.840871093495581
          - 4.238181437589567
        - kind: bathroom
          box:
          - 7.075330108154454
          - 2.2209065326892174
          - 4.468918029317747
          - 4.441813065378435
        - kind: living
          box:
          - 7.075330108154454
          - 6.5609037841732185
          - 4.468918029317747
          - 4.238181437589567
        seed: 1007
      target:
        id: armchair_8
        label: armchair
        box:
        - 6.961
        - 8.16
        - 0.85
        - 0.85
        room: living
      spawn:
      - 1.778
      - 3.073
      spawn_yaw_deg: 270
      geodesic_m: 9.94
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
---

You are controlling a simulated robot. Read the task below, then solve it by running the `robo` command in your shell (start with `robo info` and `robo observe`). Keep going until the task is done, then call `robo done` once. Do not stop to ask questions; there is no human to answer.

# Find the armchair

You control a mobile robot base (0.25 m radius) in a home: rooms joined by doorways, with furniture along the walls. You do not get a map or world coordinates: `robo observe` describes the scene from the robot's point of view.

## Task

Drive to the armchair and stop next to it.

**Success:** after you call `robo done`, the robot is within 1.0 m of the armchair's footprint and can see it (a straight line from the robot to the nearest point of the armchair crosses no wall or furniture). Judged by the episode server after the robot has held still for 10 steps. Scored as in Dimensional's study: success, SPL (success weighted by the shortest path over the path driven), SoftSPL, collisions, time to target and path smoothness are all recorded.

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
