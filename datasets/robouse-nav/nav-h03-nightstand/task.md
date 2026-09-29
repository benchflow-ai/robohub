---
schema_version: '1.3'
task:
  name: robouse/nav-h03-nightstand
  description: Find the nightstand
metadata:
  author_name: benchflow
  source_benchmark: robouse original (nav suite; procedurally generated homes, protocol of Dimensional's Can Jev Nav?)
  source_task: nav-h03-nightstand
  suite: nav
  category: object-goal navigation
  difficulty: easy
  tags:
  - navigation
  - mobile-base
  - spatial-language
  geodesic_m: 5.59
  detour: 1.19
  robouse:
    id: nav-h03-nightstand
    backend: nav
    env: nav-h03-nightstand
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
        - 8.704535881785258
        - 12.069140581014821
        walls:
        - - 4.102
          - 0
          - 8.355
          - 0.15
        - - 4.102
          - 11.569
          - 8.355
          - 0.15
        - - 0
          - 5.785
          - 0.15
          - 11.719
        - - 8.205
          - 5.785
          - 0.15
          - 11.719
        - - 4.406
          - 8.783
          - 0.15
          - 1.958
        - - 4.406
          - 11.191
          - 0.15
          - 0.757
        - - 0.894
          - 3.848
          - 1.788
          - 0.15
        - - 3.678
          - 3.848
          - 1.456
          - 0.15
        - - 5.224
          - 7.803
          - 1.637
          - 0.15
        - - 7.65
          - 7.803
          - 1.11
          - 0.15
        - - 0.975
          - 7.803
          - 1.95
          - 0.15
        - - 3.732
          - 7.803
          - 1.348
          - 0.15
        - - 4.406
          - 0.722
          - 0.15
          - 1.444
        - - 4.406
          - 3.198
          - 0.15
          - 1.3
        - - 4.406
          - 5.826
          - 0.15
          - 4.105
        - - 6.305
          - 3.848
          - 3.949
          - 0.15
        objects:
        - id: trash_can_0
          label: trash can
          box:
          - 3.562
          - 0.295
          - 0.4
          - 0.4
          room: bedroom
        - id: plant_1
          label: plant
          box:
          - 0.32
          - 3.268
          - 0.45
          - 0.45
          room: bedroom
        - id: plant_2
          label: plant
          box:
          - 4.086
          - 4.769
          - 0.45
          - 0.45
          room: bedroom
        - id: bed_3
          label: bed
          box:
          - 1.095
          - 5.904
          - 2.0
          - 1.6
          room: bedroom
        - id: trash_can_4
          label: trash can
          box:
          - 4.111
          - 6.449
          - 0.4
          - 0.4
          room: bedroom
        - id: nightstand_5
          label: nightstand
          box:
          - 0.721
          - 7.458
          - 0.45
          - 0.5
          room: bedroom
        - id: bathtub_6
          label: bathtub
          box:
          - 0.945
          - 9.215
          - 1.7
          - 0.8
          room: bathroom
        - id: toilet_7
          label: toilet
          box:
          - 2.834
          - 11.249
          - 0.7
          - 0.45
          room: bathroom
        - id: washing_machine_8
          label: washing machine
          box:
          - 3.986
          - 8.657
          - 0.65
          - 0.65
          room: bathroom
        - id: dining_table_9
          label: dining table
          box:
          - 7.257
          - 3.303
          - 1.6
          - 0.9
          room: kitchen
        - id: fridge_10
          label: fridge
          box:
          - 5.809
          - 0.495
          - 0.7
          - 0.8
          room: kitchen
        - id: plant_11
          label: plant
          box:
          - 5.502
          - 3.528
          - 0.45
          - 0.45
          room: kitchen
        - id: trash_can_12
          label: trash can
          box:
          - 7.91
          - 1.026
          - 0.4
          - 0.4
          room: kitchen
        - id: bookshelf_13
          label: bookshelf
          box:
          - 7.935
          - 6.81
          - 0.35
          - 1.0
          room: living
        - id: armchair_14
          label: armchair
          box:
          - 4.926
          - 5.29
          - 0.85
          - 0.85
          room: living
        - id: filing_cabinet_15
          label: filing cabinet
          box:
          - 7.86
          - 9.378
          - 0.5
          - 0.6
          room: office
        - id: trash_can_16
          label: trash can
          box:
          - 6.724
          - 11.274
          - 0.4
          - 0.4
          room: office
        rooms:
        - kind: bedroom
          box:
          - 2.2029488200246647
          - 1.924202265685088
          - 4.4058976400493295
          - 3.848404531370176
        - kind: bedroom
          box:
          - 2.2029488200246647
          - 5.825883884301495
          - 4.4058976400493295
          - 3.954958705862638
        - kind: bathroom
          box:
          - 2.2029488200246647
          - 9.686251909123818
          - 4.4058976400493295
          - 3.7657773437820072
        - kind: kitchen
          box:
          - 6.305216760917293
          - 1.924202265685088
          - 3.7986382417359286
          - 3.848404531370176
        - kind: living
          box:
          - 6.305216760917293
          - 5.825883884301495
          - 3.7986382417359286
          - 3.954958705862638
        - kind: office
          box:
          - 6.305216760917293
          - 9.686251909123818
          - 3.7986382417359286
          - 3.7657773437820072
        seed: 1003
      target:
        id: nightstand_5
        label: nightstand
        box:
        - 0.721
        - 7.458
        - 0.45
        - 0.5
        room: bedroom
      spawn:
      - 5.642
      - 9.124
      spawn_yaw_deg: 90
      geodesic_m: 5.59
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

# Find the nightstand

You control a mobile robot base (0.25 m radius) in a home: rooms joined by doorways, with furniture along the walls. You do not get a map or world coordinates: `robo observe` describes the scene from the robot's point of view.

## Task

Drive to the nightstand and stop next to it.

**Success:** after you call `robo done`, the robot is within 1.0 m of the nightstand's footprint and can see it (a straight line from the robot to the nearest point of the nightstand crosses no wall or furniture). Judged by the episode server after the robot has held still for 10 steps. Scored as in Dimensional's study: success, SPL (success weighted by the shortest path over the path driven), SoftSPL, collisions, time to target and path smoothness are all recorded.

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
