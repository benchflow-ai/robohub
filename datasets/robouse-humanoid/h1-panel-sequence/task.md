---
schema_version: '1.3'
task:
  name: robouse/h1-panel-sequence
  description: Press the panel buttons in order
metadata:
  author_name: benchflow
  source_benchmark: robouse original (humanoid suite; Unitree H1 from MuJoCo Menagerie)
  source_task: h1-panel-sequence
  suite: humanoid
  category: interaction
  difficulty: medium
  tags:
  - humanoid
  - h1
  - fixed-base
  - manipulation
  - menagerie
  - mujoco
  embodiment: unitree_h1
  robouse:
    id: h1-panel-sequence
    backend: humanoid
    env: h1-panel-sequence
    seed: 0
    max_steps: 600
    camera: front
    cameras:
    - front
    - top
    - side
    - head
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

# Press the panel buttons in order

You control a Unitree H1 humanoid (1.8 m; MuJoCo Menagerie `unitree_h1`, BSD-3-Clause). It has no hands: each 4-joint arm ends in a round fist (6.6 cm ball). The robot is mounted on a support stand (its pelvis is welded to the stand): it does not balance or walk, its legs hold a standing pose and only the arms move. A counter stands in front of it. World frame in metres: +x points forward from the robot, +y to the robot's left, +z up; the floor is at z = 0 and the origin is on the floor below the robot's pelvis. A vertical control panel stands on the counter facing the robot, 43 cm in front of it. It has five push buttons (5 cm wide, protruding 1.5 cm): green and blue on the left, yellow and white on the right, and a red emergency-stop button in the middle. A button counts as pressed when pushed in 8 mm; it springs back when released. All positions and sizes are listed under `scene` and `objects` in `robo observe`. Each hand's control point is the centre of its fist. The arms have four joints, so the forearm's angle follows from where the fist is; the forearms also touch things. Objects are pushed, pressed or squeezed between the two fists or forearms.

## Task

Press the buttons in this order: yellow, green, white, blue. Do not press the red emergency-stop button.

**Success:** the presses recorded by the panel are exactly yellow, green, white, blue in that order (no other, repeated or missing press); pressing red is a safety event and fails the task; `progress.pressed_so_far` lists the presses; and no safety event. Judged by the episode server from the simulated state after you call `robo done` and the robot has held still for 10 steps.

**Controls.** `robo act LDX LDY LDZ RDX RDY RDZ`, each in [-1, 1]. LDX LDY LDZ and RDX RDY RDZ move the goal point of the left and right hand by up to 2 cm per step along world x, y, z; an arm controller (inverse kinematics and joint servos) moves the hand after its goal, stops when the hand is blocked and keeps the goal within reach. Zero keeps the goal where it is, so an all-zero action holds the robot still. One step is 40 ms; `--repeat N` applies an action N times. Skills (`robo info` lists them with their arguments): `reach SIDE X Y Z [TOL]` (move one hand's control point in a straight line to a point; reports whether it arrived, was blocked or hit the edge of its reach, and what the hand touches), `reach_both XL YL ZL XR YR ZR [TOL]` (both hands at once, for bimanual holds), `home` (both hands back to their start points), `wait [STEPS]`. Skills do not plan around obstacles.

**Observation.** `robo observe` reports `robot.hands.left` / `robot.hands.right` (`pos`: the control point, `goal`: its current goal, `touching`: what the hand touches); `objects` (each free object's centre `pos`, `tilt_deg` from upright, `touched_by_robot`, and its size); `scene` (the table and every fixture and target zone, with positions); `progress` where the task tracks something; and `safety_events`. Cameras: `front` (default), `top`, `side` and `head` (fixed at the robot's head, looking at the table).

The step budget is 600 steps (24 s of robot time).

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
