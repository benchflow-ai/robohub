---
schema_version: '1.3'
task:
  name: robouse/nav-h02-bookshelf
  description: Find the bookshelf
metadata:
  author_name: benchflow
  source_benchmark: robouse original (nav suite; procedurally generated homes, protocol of Dimensional's Can Jev Nav?)
  source_task: nav-h02-bookshelf
  suite: nav
  category: object-goal navigation
  difficulty: hard
  tags:
  - navigation
  - mobile-base
  - spatial-language
  geodesic_m: 15.01
  detour: 1.81
  robouse:
    id: nav-h02-bookshelf
    backend: nav
    env: nav-h02-bookshelf
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
        - 9.177586038651377
        - 11.482916088578374
        walls:
        - - 4.339
          - 0
          - 8.828
          - 0.15
        - - 4.339
          - 10.983
          - 8.828
          - 0.15
        - - 0
          - 5.491
          - 0.15
          - 11.133
        - - 8.678
          - 5.491
          - 0.15
          - 11.133
        - - 0.353
          - 3.588
          - 0.706
          - 0.15
        - - 3.462
          - 3.588
          - 3.114
          - 0.15
        - - 5.019
          - 4.242
          - 0.15
          - 1.306
        - - 5.019
          - 6.385
          - 0.15
          - 1.163
        - - 5.019
          - 7.814
          - 0.15
          - 1.695
        - - 5.019
          - 10.291
          - 0.15
          - 1.383
        - - 5.871
          - 6.966
          - 1.704
          - 0.15
        - - 8.197
          - 6.966
          - 0.962
          - 0.15
        - - 2.509
          - 6.966
          - 5.169
          - 0.15
        - - 5.373
          - 3.588
          - 0.709
          - 0.15
        - - 7.75
          - 3.588
          - 1.855
          - 0.15
        - - 5.019
          - 1.794
          - 0.15
          - 3.738
        objects:
        - id: tv_stand_0
          label: tv stand
          box:
          - 4.699
          - 2.584
          - 0.45
          - 1.6
          room: living
        - id: bookshelf_1
          label: bookshelf
          box:
          - 2.883
          - 0.27
          - 1.0
          - 0.35
          room: living
        - id: coffee_table_2
          label: coffee table
          box:
          - 0.395
          - 0.841
          - 0.6
          - 1.1
          room: living
        - id: armchair_3
          label: armchair
          box:
          - 2.918
          - 3.068
          - 0.85
          - 0.85
          room: living
        - id: kitchen_counter_4
          label: kitchen counter
          box:
          - 0.888
          - 5.871
          - 0.6
          - 2.0
          room: kitchen
        - id: dining_table_5
          label: dining table
          box:
          - 2.978
          - 6.421
          - 1.6
          - 0.9
          room: kitchen
        - id: trash_can_6
          label: trash can
          box:
          - 4.724
          - 4.145
          - 0.4
          - 0.4
          room: kitchen
        - id: bed_7
          label: bed
          box:
          - 1.095
          - 9.502
          - 2.0
          - 1.6
          room: bedroom
        - id: nightstand_8
          label: nightstand
          box:
          - 4.674
          - 7.571
          - 0.5
          - 0.45
          room: bedroom
        - id: trash_can_9
          label: trash can
          box:
          - 2.398
          - 7.261
          - 0.4
          - 0.4
          room: bedroom
        - id: plant_10
          label: plant
          box:
          - 0.431
          - 7.286
          - 0.45
          - 0.45
          room: bedroom
        - id: plant_11
          label: plant
          box:
          - 6.304
          - 0.32
          - 0.45
          - 0.45
          room: bathroom
        - id: sink_12
          label: sink
          box:
          - 8.283
          - 1.556
          - 0.6
          - 0.5
          room: bathroom
        - id: nightstand_13
          label: nightstand
          box:
          - 8.333
          - 4.584
          - 0.5
          - 0.45
          room: bedroom
        - id: desk_14
          label: desk
          box:
          - 8.258
          - 9.975
          - 0.65
          - 1.3
          room: office
        - id: plant_15
          label: plant
          box:
          - 5.403
          - 7.286
          - 0.45
          - 0.45
          room: office
        rooms:
        - kind: living
          box:
          - 2.5093729130838116
          - 1.794211916914362
          - 5.018745826167623
          - 3.588423833828724
        - kind: kitchen
          box:
          - 2.5093729130838116
          - 5.2773269900366895
          - 5.018745826167623
          - 3.377806312415932
        - kind: bedroom
          box:
          - 2.5093729130838116
          - 8.974573117411515
          - 5.018745826167623
          - 4.0166859423337185
        - kind: bathroom
          box:
          - 6.8481659324095006
          - 1.794211916914362
          - 3.658840212483754
          - 3.588423833828724
        - kind: bedroom
          box:
          - 6.8481659324095006
          - 5.2773269900366895
          - 3.658840212483754
          - 3.377806312415932
        - kind: office
          box:
          - 6.8481659324095006
          - 8.974573117411515
          - 3.658840212483754
          - 4.0166859423337185
        seed: 1002
      target:
        id: bookshelf_1
        label: bookshelf
        box:
        - 2.883
        - 0.27
        - 1.0
        - 0.35
        room: living
      spawn:
      - 2.613
      - 9.077
      spawn_yaw_deg: 180
      geodesic_m: 15.01
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

# Find the bookshelf

You control a mobile robot base (0.25 m radius) in a home: rooms joined by doorways, with furniture along the walls. You do not get a map or world coordinates: `robo observe` describes the scene from the robot's point of view.

## Task

Drive to the bookshelf and stop next to it.

**Success:** after you call `robo done`, the robot is within 1.0 m of the bookshelf's footprint and can see it (a straight line from the robot to the nearest point of the bookshelf crosses no wall or furniture). Judged by the episode server after the robot has held still for 10 steps. Scored as in Dimensional's study: success, SPL (success weighted by the shortest path over the path driven), SoftSPL, collisions, time to target and path smoothness are all recorded.

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
