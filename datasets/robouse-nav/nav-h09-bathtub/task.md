---
schema_version: '1.3'
task:
  name: robouse/nav-h09-bathtub
  description: Find the bathtub
metadata:
  author_name: benchflow
  source_benchmark: robouse original (nav suite; procedurally generated homes, protocol of Dimensional's Can Jev Nav?)
  source_task: nav-h09-bathtub
  suite: nav
  category: object-goal navigation
  difficulty: easy
  tags:
  - navigation
  - mobile-base
  - spatial-language
  geodesic_m: 12.32
  detour: 1.09
  robouse:
    id: nav-h09-bathtub
    backend: nav
    env: nav-h09-bathtub
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
        - 13.843484164458046
        - 8.778349407868475
        walls:
        - - 6.672
          - 0
          - 13.493
          - 0.15
        - - 6.672
          - 8.278
          - 13.493
          - 0.15
        - - 0
          - 4.139
          - 0.15
          - 8.428
        - - 13.343
          - 4.139
          - 0.15
          - 8.428
        - - 3.421
          - 4.395
          - 0.15
          - 1.051
        - - 3.421
          - 7.054
          - 0.15
          - 2.448
        - - 3.421
          - 0.73
          - 0.15
          - 1.46
        - - 3.421
          - 3.126
          - 0.15
          - 1.487
        - - 8.613
          - 0.379
          - 0.15
          - 0.758
        - - 8.613
          - 2.84
          - 0.15
          - 2.059
        - - 8.613
          - 4.186
          - 0.15
          - 0.633
        - - 8.613
          - 6.891
          - 0.15
          - 2.774
        - - 9.923
          - 3.87
          - 2.619
          - 0.15
        - - 12.886
          - 3.87
          - 0.915
          - 0.15
        - - 6.017
          - 3.87
          - 5.342
          - 0.15
        - - 1.711
          - 3.87
          - 3.571
          - 0.15
        objects:
        - id: bed_0
          label: bed
          box:
          - 0.895
          - 1.722
          - 1.6
          - 2.0
          room: bedroom
        - id: nightstand_1
          label: nightstand
          box:
          - 1.555
          - 3.525
          - 0.45
          - 0.5
          room: bedroom
        - id: trash_can_2
          label: trash can
          box:
          - 0.324
          - 3.575
          - 0.4
          - 0.4
          room: bedroom
        - id: coffee_table_3
          label: coffee table
          box:
          - 0.645
          - 7.564
          - 1.1
          - 0.6
          room: living
        - id: sofa_4
          label: sofa
          box:
          - 1.145
          - 5.391
          - 2.1
          - 0.9
          room: living
        - id: office_chair_5
          label: office chair
          box:
          - 8.218
          - 2.732
          - 0.6
          - 0.6
          room: office
        - id: trash_can_6
          label: trash can
          box:
          - 4.938
          - 0.295
          - 0.4
          - 0.4
          room: office
        - id: desk_7
          label: desk
          box:
          - 5.976
          - 3.45
          - 1.3
          - 0.65
          room: office
        - id: plant_8
          label: plant
          box:
          - 3.741
          - 0.58
          - 0.45
          - 0.45
          room: office
        - id: trash_can_9
          label: trash can
          box:
          - 6.957
          - 7.983
          - 0.4
          - 0.4
          room: bedroom
        - id: wardrobe_10
          label: wardrobe
          box:
          - 4.116
          - 7.336
          - 1.2
          - 0.6
          room: bedroom
        - id: nightstand_11
          label: nightstand
          box:
          - 6.618
          - 4.215
          - 0.45
          - 0.5
          room: bedroom
        - id: washing_machine_12
          label: washing machine
          box:
          - 9.574
          - 3.45
          - 0.65
          - 0.65
          room: bathroom
        - id: bathtub_13
          label: bathtub
          box:
          - 12.848
          - 1.387
          - 0.8
          - 1.7
          room: bathroom
        - id: oven_14
          label: oven
          box:
          - 12.898
          - 5.223
          - 0.7
          - 0.65
          room: kitchen
        - id: trash_can_15
          label: trash can
          box:
          - 13.048
          - 7.509
          - 0.4
          - 0.4
          room: kitchen
        rooms:
        - kind: bedroom
          box:
          - 1.710712028582588
          - 1.9349234174471373
          - 3.421424057165176
          - 3.8698468348942745
        - kind: living
          box:
          - 1.710712028582588
          - 6.074098121381375
          - 3.421424057165176
          - 4.408502572974201
        - kind: office
          box:
          - 6.01723503729019
          - 1.9349234174471373
          - 5.191621960250028
          - 3.8698468348942745
        - kind: bedroom
          box:
          - 6.01723503729019
          - 6.074098121381375
          - 5.191621960250028
          - 4.408502572974201
        - kind: bathroom
          box:
          - 10.978265090936624
          - 1.9349234174471373
          - 4.730438147042841
          - 3.8698468348942745
        - kind: kitchen
          box:
          - 10.978265090936624
          - 6.074098121381375
          - 4.730438147042841
          - 4.408502572974201
        seed: 1009
      target:
        id: bathtub_13
        label: bathtub
        box:
        - 12.848
        - 1.387
        - 0.8
        - 1.7
        room: bathroom
      spawn:
      - 2.849
      - 7.578
      spawn_yaw_deg: 180
      geodesic_m: 12.32
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

# Find the bathtub

You control a mobile robot base (0.25 m radius) in a home: rooms joined by doorways, with furniture along the walls. You do not get a map or world coordinates: `robo observe` describes the scene from the robot's point of view.

## Task

Drive to the bathtub and stop next to it.

**Success:** after you call `robo done`, the robot is within 1.0 m of the bathtub's footprint and can see it (a straight line from the robot to the nearest point of the bathtub crosses no wall or furniture). Judged by the episode server after the robot has held still for 10 steps. Scored as in Dimensional's study: success, SPL (success weighted by the shortest path over the path driven), SoftSPL, collisions, time to target and path smoothness are all recorded.

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
