---
schema_version: '1.3'
task:
  name: robouse/nav-h04-wardrobe
  description: Find the wardrobe
metadata:
  author_name: benchflow
  source_benchmark: robouse original (nav suite; procedurally generated homes, protocol of Dimensional's Can Jev Nav?)
  source_task: nav-h04-wardrobe
  suite: nav
  category: object-goal navigation
  difficulty: easy
  tags:
  - navigation
  - mobile-base
  - spatial-language
  geodesic_m: 5.37
  detour: 0.9
  robouse:
    id: nav-h04-wardrobe
    backend: nav
    env: nav-h04-wardrobe
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
        - 8.785164405859241
        - 12.684866825830962
        walls:
        - - 4.143
          - 0
          - 8.435
          - 0.15
        - - 4.143
          - 12.185
          - 8.435
          - 0.15
        - - 0
          - 6.092
          - 0.15
          - 12.335
        - - 8.285
          - 6.092
          - 0.15
          - 12.335
        - - 4.324
          - 3.956
          - 1.465
          - 0.15
        - - 7.248
          - 3.956
          - 2.075
          - 0.15
        - - 0.783
          - 8.092
          - 1.565
          - 0.15
        - - 3.104
          - 8.092
          - 0.974
          - 0.15
        - - 3.591
          - 8.418
          - 0.15
          - 0.652
        - - 3.591
          - 10.936
          - 0.15
          - 2.497
        - - 3.591
          - 1.057
          - 0.15
          - 2.113
        - - 3.591
          - 3.507
          - 0.15
          - 0.896
        - - 3.591
          - 4.549
          - 0.15
          - 1.187
        - - 3.591
          - 7.116
          - 0.15
          - 1.952
        - - 1.004
          - 3.956
          - 2.009
          - 0.15
        - - 3.268
          - 3.956
          - 0.646
          - 0.15
        - - 4.526
          - 8.092
          - 1.869
          - 0.15
        - - 7.471
          - 8.092
          - 1.629
          - 0.15
        objects:
        - id: trash_can_0
          label: trash can
          box:
          - 0.295
          - 1.776
          - 0.4
          - 0.4
          room: office
        - id: plant_1
          label: plant
          box:
          - 0.828
          - 0.32
          - 0.45
          - 0.45
          room: office
        - id: filing_cabinet_2
          label: filing cabinet
          box:
          - 3.157
          - 0.345
          - 0.6
          - 0.5
          room: office
        - id: wardrobe_3
          label: wardrobe
          box:
          - 0.395
          - 7.058
          - 0.6
          - 1.2
          room: bedroom
        - id: plant_4
          label: plant
          box:
          - 0.32
          - 5.016
          - 0.45
          - 0.45
          room: bedroom
        - id: bed_5
          label: bed
          box:
          - 1.74
          - 11.29
          - 2.0
          - 1.6
          room: bedroom
        - id: plant_6
          label: plant
          box:
          - 0.32
          - 9.595
          - 0.45
          - 0.45
          room: bedroom
        - id: sink_7
          label: sink
          box:
          - 3.986
          - 0.524
          - 0.6
          - 0.5
          room: bathroom
        - id: bathtub_8
          label: bathtub
          box:
          - 7.79
          - 2.634
          - 0.8
          - 1.7
          room: bathroom
        - id: coffee_table_9
          label: coffee table
          box:
          - 3.986
          - 7.355
          - 0.6
          - 1.1
          room: living
        - id: bookshelf_10
          label: bookshelf
          box:
          - 6.976
          - 4.551
          - 0.35
          - 1.0
          room: living
        - id: oven_11
          label: oven
          box:
          - 7.84
          - 11.403
          - 0.7
          - 0.65
          room: kitchen
        - id: fridge_12
          label: fridge
          box:
          - 7.473
          - 8.587
          - 0.7
          - 0.8
          room: kitchen
        - id: plant_13
          label: plant
          box:
          - 3.911
          - 10.573
          - 0.45
          - 0.45
          room: kitchen
        - id: trash_can_14
          label: trash can
          box:
          - 4.617
          - 11.89
          - 0.4
          - 0.4
          room: kitchen
        rooms:
        - kind: office
          box:
          - 1.7957037387676282
          - 1.977785785385442
          - 3.5914074775352565
          - 3.955571570770884
        - kind: bedroom
          box:
          - 1.7957037387676282
          - 6.0239767054204885
          - 3.5914074775352565
          - 4.136810269299209
        - kind: bedroom
          box:
          - 1.7957037387676282
          - 10.138624332950528
          - 3.5914074775352565
          - 4.092484985760869
        - kind: bathroom
          box:
          - 5.938285941697249
          - 1.977785785385442
          - 4.693756928323985
          - 3.955571570770884
        - kind: living
          box:
          - 5.938285941697249
          - 6.0239767054204885
          - 4.693756928323985
          - 4.136810269299209
        - kind: kitchen
          box:
          - 5.938285941697249
          - 10.138624332950528
          - 4.693756928323985
          - 4.092484985760869
        seed: 1004
      target:
        id: wardrobe_3
        label: wardrobe
        box:
        - 0.395
        - 7.058
        - 0.6
        - 1.2
        room: bedroom
      spawn:
      - 6.537
      - 4.961
      spawn_yaw_deg: 270
      geodesic_m: 5.37
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

# Find the wardrobe

You control a mobile robot base (0.25 m radius) in a home: rooms joined by doorways, with furniture along the walls. You do not get a map or world coordinates: `robo observe` describes the scene from the robot's point of view.

## Task

Drive to the wardrobe and stop next to it.

**Success:** after you call `robo done`, the robot is within 1.0 m of the wardrobe's footprint and can see it (a straight line from the robot to the nearest point of the wardrobe crosses no wall or furniture). Judged by the episode server after the robot has held still for 10 steps. Scored as in Dimensional's study: success, SPL (success weighted by the shortest path over the path driven), SoftSPL, collisions, time to target and path smoothness are all recorded.

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
