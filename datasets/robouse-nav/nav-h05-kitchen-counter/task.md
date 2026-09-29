---
schema_version: '1.3'
task:
  name: robouse/nav-h05-kitchen-counter
  description: Find the kitchen counter
metadata:
  author_name: benchflow
  source_benchmark: robouse original (nav suite; procedurally generated homes, protocol of Dimensional's Can Jev Nav?)
  source_task: nav-h05-kitchen-counter
  suite: nav
  category: object-goal navigation
  difficulty: medium
  tags:
  - navigation
  - mobile-base
  - spatial-language
  geodesic_m: 6.11
  detour: 1.65
  robouse:
    id: nav-h05-kitchen-counter
    backend: nav
    env: nav-h05-kitchen-counter
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
        - 9.439607457752171
        - 12.887725941995836
        walls:
        - - 4.47
          - 0
          - 9.09
          - 0.15
        - - 4.47
          - 12.388
          - 9.09
          - 0.15
        - - 0
          - 6.194
          - 0.15
          - 12.538
        - - 8.94
          - 6.194
          - 0.15
          - 12.538
        - - 5.42
          - 3.775
          - 1.198
          - 0.15
        - - 8.015
          - 3.775
          - 1.849
          - 0.15
        - - 4.821
          - 0.879
          - 0.15
          - 1.759
        - - 4.821
          - 3.338
          - 0.15
          - 0.874
        - - 1.032
          - 8.295
          - 2.064
          - 0.15
        - - 3.992
          - 8.295
          - 1.659
          - 0.15
        - - 1.346
          - 3.775
          - 2.692
          - 0.15
        - - 4.213
          - 3.775
          - 1.216
          - 0.15
        - - 4.821
          - 8.685
          - 0.15
          - 0.78
        - - 4.821
          - 11.232
          - 0.15
          - 2.311
        - - 5.24
          - 8.295
          - 0.837
          - 0.15
        - - 7.793
          - 8.295
          - 2.293
          - 0.15
        - - 4.821
          - 6.035
          - 0.15
          - 4.67
        objects:
        - id: fridge_0
          label: fridge
          box:
          - 0.445
          - 1.296
          - 0.7
          - 0.8
          room: kitchen
        - id: kitchen_counter_1
          label: kitchen counter
          box:
          - 2.715
          - 0.395
          - 2.0
          - 0.6
          room: kitchen
        - id: oven_2
          label: oven
          box:
          - 1.263
          - 3.33
          - 0.65
          - 0.7
          room: kitchen
        - id: coffee_table_3
          label: coffee table
          box:
          - 1.578
          - 4.42
          - 0.6
          - 1.1
          room: living
        - id: plant_4
          label: plant
          box:
          - 0.32
          - 5.455
          - 0.45
          - 0.45
          room: living
        - id: bookshelf_5
          label: bookshelf
          box:
          - 4.226
          - 6.668
          - 1.0
          - 0.35
          room: living
        - id: nightstand_6
          label: nightstand
          box:
          - 3.906
          - 12.068
          - 0.5
          - 0.45
          room: bedroom
        - id: bed_7
          label: bed
          box:
          - 0.895
          - 10.481
          - 1.6
          - 2.0
          room: bedroom
        - id: wardrobe_8
          label: wardrobe
          box:
          - 4.126
          - 10.928
          - 1.2
          - 0.6
          room: bedroom
        - id: plant_9
          label: plant
          box:
          - 1.044
          - 8.615
          - 0.45
          - 0.45
          room: bedroom
        - id: bed_10
          label: bed
          box:
          - 7.874
          - 1.095
          - 1.6
          - 2.0
          room: bedroom
        - id: plant_11
          label: plant
          box:
          - 5.141
          - 0.991
          - 0.45
          - 0.45
          room: bedroom
        - id: trash_can_12
          label: trash can
          box:
          - 7.979
          - 3.48
          - 0.4
          - 0.4
          room: bedroom
        - id: nightstand_13
          label: nightstand
          box:
          - 6.135
          - 0.345
          - 0.45
          - 0.5
          room: bedroom
        - id: trash_can_14
          label: trash can
          box:
          - 8.404
          - 8.0
          - 0.4
          - 0.4
          room: bathroom
        - id: toilet_15
          label: toilet
          box:
          - 8.495
          - 5.987
          - 0.7
          - 0.45
          room: bathroom
        - id: plant_16
          label: plant
          box:
          - 8.06
          - 4.095
          - 0.45
          - 0.45
          room: bathroom
        - id: bathtub_17
          label: bathtub
          box:
          - 5.766
          - 5.858
          - 1.7
          - 0.8
          room: bathroom
        - id: trash_can_18
          label: trash can
          box:
          - 8.645
          - 11.17
          - 0.4
          - 0.4
          room: office
        - id: desk_19
          label: desk
          box:
          - 6.418
          - 11.968
          - 1.3
          - 0.65
          room: office
        - id: office_chair_20
          label: office chair
          box:
          - 8.545
          - 9.51
          - 0.6
          - 0.6
          room: office
        rooms:
        - kind: kitchen
          box:
          - 2.4105782614049667
          - 1.8873870843149263
          - 4.821156522809933
          - 3.7747741686298526
        - kind: living
          box:
          - 2.4105782614049667
          - 6.0348423132881335
          - 4.821156522809933
          - 4.520136289316563
        - kind: bedroom
          box:
          - 2.4105782614049667
          - 10.341318199971125
          - 4.821156522809933
          - 4.092815484049421
        - kind: bedroom
          box:
          - 6.880381990281052
          - 1.8873870843149263
          - 4.118450934942238
          - 3.7747741686298526
        - kind: bathroom
          box:
          - 6.880381990281052
          - 6.0348423132881335
          - 4.118450934942238
          - 4.520136289316563
        - kind: office
          box:
          - 6.880381990281052
          - 10.341318199971125
          - 4.118450934942238
          - 4.092815484049421
        seed: 1005
      target:
        id: kitchen_counter_1
        label: kitchen counter
        box:
        - 2.715
        - 0.395
        - 2.0
        - 0.6
        room: kitchen
      spawn:
      - 0.97
      - 4.212
      spawn_yaw_deg: 90
      geodesic_m: 6.11
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
