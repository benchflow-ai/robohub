---
schema_version: '1.3'
task:
  name: robouse/nav-h00-armchair
  description: Find the armchair
metadata:
  author_name: benchflow
  source_benchmark: robouse original (nav suite; procedurally generated homes, protocol of Dimensional's Can Jev Nav?)
  source_task: nav-h00-armchair
  suite: nav
  category: object-goal navigation
  difficulty: easy
  tags:
  - navigation
  - mobile-base
  - spatial-language
  geodesic_m: 9.14
  detour: 1.25
  robouse:
    id: nav-h00-armchair
    backend: nav
    env: nav-h00-armchair
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
        - 8.68413729427132
        - 11.99778679378661
        walls:
        - - 4.092
          - 0
          - 8.334
          - 0.15
        - - 4.092
          - 11.498
          - 8.334
          - 0.15
        - - 0
          - 5.749
          - 0.15
          - 11.648
        - - 8.184
          - 5.749
          - 0.15
          - 11.648
        - - 4.606
          - 0.743
          - 0.15
          - 1.485
        - - 4.606
          - 3.187
          - 0.15
          - 1.015
        - - 4.606
          - 4.651
          - 0.15
          - 1.913
        - - 4.606
          - 7.083
          - 0.15
          - 0.932
        - - 5.389
          - 7.549
          - 1.566
          - 0.15
        - - 7.655
          - 7.549
          - 1.058
          - 0.15
        - - 4.606
          - 7.965
          - 0.15
          - 0.832
        - - 4.606
          - 10.502
          - 0.15
          - 1.992
        - - 5.325
          - 3.694
          - 1.439
          - 0.15
        - - 7.713
          - 3.694
          - 0.942
          - 0.15
        - - 2.303
          - 3.694
          - 4.756
          - 0.15
        - - 0.91
          - 7.549
          - 1.82
          - 0.15
        - - 3.777
          - 7.549
          - 1.657
          - 0.15
        objects:
        - id: plant_0
          label: plant
          box:
          - 1.091
          - 3.374
          - 0.45
          - 0.45
          room: bathroom
        - id: washing_machine_1
          label: washing machine
          box:
          - 1.292
          - 0.42
          - 0.65
          - 0.65
          room: bathroom
        - id: toilet_2
          label: toilet
          box:
          - 0.32
          - 2.273
          - 0.45
          - 0.7
          room: bathroom
        - id: bookshelf_3
          label: bookshelf
          box:
          - 2.756
          - 3.964
          - 1.0
          - 0.35
          room: living
        - id: armchair_4
          label: armchair
          box:
          - 0.52
          - 6.146
          - 0.85
          - 0.85
          room: living
        - id: wardrobe_5
          label: wardrobe
          box:
          - 0.695
          - 10.475
          - 1.2
          - 0.6
          room: bedroom
        - id: nightstand_6
          label: nightstand
          box:
          - 0.713
          - 7.894
          - 0.45
          - 0.5
          room: bedroom
        - id: trash_can_7
          label: trash can
          box:
          - 0.295
          - 9.106
          - 0.4
          - 0.4
          room: bedroom
        - id: filing_cabinet_8
          label: filing cabinet
          box:
          - 5.249
          - 0.395
          - 0.5
          - 0.6
          room: office
        - id: plant_9
          label: plant
          box:
          - 6.539
          - 0.32
          - 0.45
          - 0.45
          room: office
        - id: trash_can_10
          label: trash can
          box:
          - 7.889
          - 5.919
          - 0.4
          - 0.4
          room: kitchen
        - id: fridge_11
          label: fridge
          box:
          - 5.178
          - 4.139
          - 0.8
          - 0.7
          room: kitchen
        - id: trash_can_12
          label: trash can
          box:
          - 7.889
          - 8.744
          - 0.4
          - 0.4
          room: bedroom
        - id: bed_13
          label: bed
          box:
          - 7.073
          - 10.603
          - 2.0
          - 1.6
          room: bedroom
        - id: plant_14
          label: plant
          box:
          - 5.302
          - 11.178
          - 0.45
          - 0.45
          room: bedroom
        rooms:
        - kind: bathroom
          box:
          - 2.302843003603325
          - 1.8470793578331017
          - 4.60568600720665
          - 3.6941587156662035
        - kind: living
          box:
          - 2.302843003603325
          - 5.621694135696792
          - 4.60568600720665
          - 3.8550708400611784
        - kind: bedroom
          box:
          - 2.302843003603325
          - 9.523508174756996
          - 4.60568600720665
          - 3.9485572380592275
        - kind: office
          box:
          - 6.394911650738985
          - 1.8470793578331017
          - 3.57845128706467
          - 3.6941587156662035
        - kind: kitchen
          box:
          - 6.394911650738985
          - 5.621694135696792
          - 3.57845128706467
          - 3.8550708400611784
        - kind: bedroom
          box:
          - 6.394911650738985
          - 9.523508174756996
          - 3.57845128706467
          - 3.9485572380592275
        seed: 1000
      target:
        id: armchair_4
        label: armchair
        box:
        - 0.52
        - 6.146
        - 0.85
        - 0.85
        room: living
      spawn:
      - 6.009
      - 0.562
      spawn_yaw_deg: 180
      geodesic_m: 9.14
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
