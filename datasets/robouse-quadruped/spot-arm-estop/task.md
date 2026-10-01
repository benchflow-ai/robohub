---
schema_version: '1.3'
task:
  name: robouse/spot-arm-estop
  description: Press the emergency stop
metadata:
  author_name: benchflow
  source_benchmark: robouse original (quadruped suite; Boston Dynamics Spot with arm from MuJoCo Menagerie)
  source_task: spot-arm-estop
  suite: quadruped
  category: manipulation
  difficulty: medium
  tags:
  - quadruped
  - legged
  - boston_dynamics_spot
  - menagerie
  - mujoco
  - arm
  robouse:
    id: spot-arm-estop
    backend: quadruped
    env: spot-arm-estop
    seed: 0
    max_steps: 900
    camera: chase
    cameras:
    - chase
    - head
    - overview
    - west_hall
    - east_room
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

# Press the emergency stop

You control a Boston Dynamics Spot quadruped with its 6-joint arm and gripper (MuJoCo Menagerie model, 56 kg, 1.1 m long, base about 0.47 m above the floor when standing) in a small indoor plant. World frame in metres: +x east, +y north, +z up, floor at z = 0; headings are in degrees, 0 = facing +x (east), 90 = facing +y (north). The plant is 12 m x 8 m (x from -6 to 6, y from -4 to 4), closed by 1 m walls. A partition wall at x = 1 separates the west hall from the east plant room; its only opening is a 1.1 m wide doorway (y from 1.5 to 2.6) at the east end of a corridor (walls at y = 1.3 and y = 2.9, from x = -2.2 to the partition; open at its west end). The west hall has the blue dock (0.9 m square at (-4.8, -2.8)), a row of crates (0.6 m cubes, 0.8 m high, at x = -3.2 from y = -3.95 to -0.95) and one more crate at (-1.9, -1.1), a green mat at (-1.2, -2.8), and a pipe with a valve and an inspection tag on the west wall. The east room has a pump (radius 0.3 m, 0.6 m high, at (4.2, -2.6)) with an asset tag, an instrument board with a pressure gauge 1.4 m up on the east wall, a yellow panel with a red emergency-stop button and a green reset button on the east wall, and a 0.2 m high mezzanine platform in the north-east corner (x 4.4-6, y 1.6-4) with a control cabinet, reached by a ramp (x 2.8-4.4, y 2.4-3.4) that rises eastwards. Every fixed position is listed under `facility` in `robo observe`. The robot stands on a free-floating base: only its feet touching the floor move it, so it can slip, trip over obstacles or fall; a fall (base tilted more than 50 degrees, or the body touching the floor, the ramp or the platform) is recorded in `safety_events` and fails the task.

## Task

Spot with its arm stands in the east room at (2.0, -1.0), facing east. On the east wall a yellow panel carries a red emergency-stop button (cap face centre (5.83, -1.2, 0.9)) and a green reset button ((5.83, -0.85, 0.9)); both caps are 9 cm across, face west and travel 3 cm into the panel against a spring. Walk up to the panel and push the red button's cap in with the tip of the (closed) gripper.

**Success:** the red button's cap was pushed in by more than 15 mm at some time in the episode (`progress.buttons_pressed`), at the end the robot is standing still (not stepping, base slower than 0.1 m/s and turning slower than 11 deg/s), and it never fell. Judged by the episode server from the simulated state after you call `robo done` and the robot has held still (zero action) for 10 steps; no new inspections or checkpoints are recorded during those 10 steps.

**Controls.** `robo act VX VY WZ DHEIGHT DPITCH DX DY DZ` sets the action for the next step(s). VX and VY (m/s, up to +-0.5 and +-0.4) and WZ (deg/s, up to +-46, counter-clockwise seen from above) are the body-frame velocity command (x forward, y left) for the onboard trot controller (gait clock, foot placement from the commanded and measured velocity, leg inverse kinematics, attitude feedback and joint servos); all three zero means stop stepping and stand. DHEIGHT (m, +-0.02 per step) and DPITCH (deg, +-5 per step) change the body-height and nose-up pitch setpoints (height from -45 % to +5 % of the standing height, pitch +-20 degrees; a non-zero velocity command raises a lowered body to at least 85 % of the standing height); zero keeps them. DX DY DZ (m, +-0.03 per step) move the gripper-tip target in the body frame (x forward, y left, z up); zero holds it. The arm controller (inverse kinematics and joint position servos) keeps the jaw pointing forward and the gripper closed; the tip reaches about 1 m from the shoulder, which sits on top of the body 0.29 m ahead of its centre. One step is 50 ms; `--repeat N` holds an action for N steps. Skills (`robo info` lists them with their arguments): `walk_to X Y [TOL] [SPEED]` (turns towards the point and trots along the straight line, then stops; it does not avoid obstacles and reports when it is blocked), `turn YAW_DEG`, `stand` (stop, standing height, level body), `sit` (stop and lie down at the lowest body height; a velocity command raises the body again) and `look_at X Y Z [SECONDS]` (turn to face the point, pitch the body so the head camera points at height Z within +-20 degrees, hold still for SECONDS, at most 3, and report where the point falls in the head image). Arm skills: `reach X Y Z [TOL]` (moves the gripper tip in a straight line to a world point while the legs stand; it stops at contact or when out of reach) and `stow`.

**Observation.** `robo observe` reports `robot` (`pos` of the base centre, `yaw_deg`, `roll_deg`, `pitch_up_deg`, `vel_body`, `yaw_rate_deg_s`, `gait` = standing / walking / sitting, `body_pose_setpoint`, `standing_height_m`, `feet_in_contact`, the `head_camera` position and forward direction, and `fallen`), `arm` (gripper `tip_pos` in the world and body frames, the tip target and the joint angles), `facility` (every fixed landmark), `objects` (this task's movable objects and floor markings, if any), `progress` (`inspected_tags`, and the task's counters) and `safety_events`. Cameras: `chase` (default; follows the robot from above and behind-left), `head` (forward-facing on the front of the base, 70 degree field of view, square image), `overview`, `west_hall` and `east_room`.

The step budget is 900 steps (45 s).

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
