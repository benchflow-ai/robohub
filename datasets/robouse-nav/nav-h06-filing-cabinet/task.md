---
schema_version: '1.3'
task:
  name: robouse/nav-h06-filing-cabinet
  description: Find the filing cabinet
metadata:
  author_name: benchflow
  source_benchmark: robouse original (nav suite; procedurally generated homes, protocol of Dimensional's Can Jev Nav?)
  source_task: nav-h06-filing-cabinet
  suite: nav
  category: object-goal navigation
  difficulty: medium
  tags:
  - navigation
  - mobile-base
  - spatial-language
  geodesic_m: 3.9
  detour: 1.63
  robouse:
    id: nav-h06-filing-cabinet
    backend: nav
    env: nav-h06-filing-cabinet
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
        - 11.96853369704894
        - 8.631774807367428
        walls:
        - - 5.734
          - 0
          - 11.619
          - 0.15
        - - 5.734
          - 8.132
          - 11.619
          - 0.15
        - - 0
          - 4.066
          - 0.15
          - 8.282
        - - 11.469
          - 4.066
          - 0.15
          - 8.282
        - - 7.699
          - 4.684
          - 0.15
          - 1.35
        - - 7.699
          - 7.283
          - 0.15
          - 1.698
        - - 3.501
          - 0.302
          - 0.15
          - 0.605
        - - 3.501
          - 2.761
          - 0.15
          - 2.495
        - - 7.699
          - 0.583
          - 0.15
          - 1.166
        - - 7.699
          - 3.045
          - 0.15
          - 1.928
        - - 3.501
          - 4.966
          - 0.15
          - 1.915
        - - 3.501
          - 7.626
          - 0.15
          - 1.012
        - - 4.075
          - 4.009
          - 1.149
          - 0.15
        - - 6.626
          - 4.009
          - 2.147
          - 0.15
        - - 0.784
          - 4.009
          - 1.568
          - 0.15
        - - 3.125
          - 4.009
          - 0.752
          - 0.15
        - - 8.005
          - 4.009
          - 0.612
          - 0.15
        - - 10.398
          - 4.009
          - 2.141
          - 0.15
        objects:
        - id: tv_stand_0
          label: tv stand
          box:
          - 0.895
          - 1.004
          - 1.6
          - 0.45
          room: living
        - id: plant_1
          label: plant
          box:
          - 0.565
          - 3.689
          - 0.45
          - 0.45
          room: living
        - id: trash_can_2
          label: trash can
          box:
          - 0.295
          - 2.163
          - 0.4
          - 0.4
          room: living
        - id: plant_3
          label: plant
          box:
          - 0.32
          - 7.299
          - 0.45
          - 0.45
          room: bedroom
        - id: wardrobe_4
          label: wardrobe
          box:
          - 0.695
          - 5.687
          - 1.2
          - 0.6
          room: bedroom
        - id: nightstand_5
          label: nightstand
          box:
          - 0.769
          - 4.329
          - 0.5
          - 0.45
          room: bedroom
        - id: trash_can_6
          label: trash can
          box:
          - 6.133
          - 0.295
          - 0.4
          - 0.4
          room: kitchen
        - id: plant_7
          label: plant
          box:
          - 4.95
          - 0.32
          - 0.45
          - 0.45
          room: kitchen
        - id: kitchen_counter_8
          label: kitchen counter
          box:
          - 4.596
          - 2.64
          - 2.0
          - 0.6
          room: kitchen
        - id: fridge_9
          label: fridge
          box:
          - 6.869
          - 3.564
          - 0.8
          - 0.7
          room: kitchen
        - id: plant_10
          label: plant
          box:
          - 6.378
          - 7.812
          - 0.45
          - 0.45
          room: office
        - id: filing_cabinet_11
          label: filing cabinet
          box:
          - 3.846
          - 4.499
          - 0.5
          - 0.6
          room: office
        - id: office_chair_12
          label: office chair
          box:
          - 6.587
          - 4.404
          - 0.6
          - 0.6
          room: office
        - id: bed_13
          label: bed
          box:
          - 10.574
          - 1.179
          - 1.6
          - 2.0
          room: bedroom
        - id: nightstand_14
          label: nightstand
          box:
          - 10.198
          - 3.664
          - 0.45
          - 0.5
          room: bedroom
        - id: sink_15
          label: sink
          box:
          - 8.556
          - 7.737
          - 0.5
          - 0.6
          room: bathroom
        - id: washing_machine_16
          label: washing machine
          box:
          - 10.383
          - 4.429
          - 0.65
          - 0.65
          room: bathroom
        - id: trash_can_17
          label: trash can
          box:
          - 11.174
          - 6.281
          - 0.4
          - 0.4
          room: bathroom
        rooms:
        - kind: living
          box:
          - 1.7502598777843787
          - 2.0043425599169744
          - 3.5005197555687575
          - 4.008685119833949
        - kind: bedroom
          box:
          - 1.7502598777843787
          - 6.0702299636006884
          - 3.5005197555687575
          - 4.123089687533479
        - kind: kitchen
          box:
          - 5.599877859076166
          - 2.0043425599169744
          - 4.198716207014817
          - 4.008685119833949
        - kind: office
          box:
          - 5.599877859076166
          - 6.0702299636006884
          - 4.198716207014817
          - 4.123089687533479
        - kind: bedroom
          box:
          - 9.583884829816258
          - 2.0043425599169744
          - 3.7692977344653658
          - 4.008685119833949
        - kind: bathroom
          box:
          - 9.583884829816258
          - 6.0702299636006884
          - 3.7692977344653658
          - 4.123089687533479
        seed: 1006
      target:
        id: filing_cabinet_11
        label: filing cabinet
        box:
        - 3.846
        - 4.499
        - 0.5
        - 0.6
        room: office
      spawn:
      - 0.972
      - 4.816
      spawn_yaw_deg: 270
      geodesic_m: 3.9
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

# Find the filing cabinet

You control a mobile robot base (0.25 m radius) in a home: rooms joined by doorways, with furniture along the walls. You do not get a map or world coordinates: `robo observe` describes the scene from the robot's point of view.

## Task

Drive to the filing cabinet and stop next to it.

**Success:** after you call `robo done`, the robot is within 1.0 m of the filing cabinet's footprint and can see it (a straight line from the robot to the nearest point of the filing cabinet crosses no wall or furniture). Judged by the episode server after the robot has held still for 10 steps. Scored as in Dimensional's study: success, SPL (success weighted by the shortest path over the path driven), SoftSPL, collisions, time to target and path smoothness are all recorded.

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
