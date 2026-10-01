---
schema_version: '1.3'
task:
  name: robouse/apollo-carry-tray
  description: Carry the tray with the cup to the mat
metadata:
  author_name: benchflow
  source_benchmark: robouse original (humanoid suite; Apptronik Apollo from MuJoCo Menagerie)
  source_task: apollo-carry-tray
  suite: humanoid
  category: bimanual
  difficulty: hard
  tags:
  - humanoid
  - apollo
  - fixed-base
  - manipulation
  - menagerie
  - mujoco
  - bimanual
  embodiment: apptronik_apollo
  robouse:
    id: apollo-carry-tray
    backend: humanoid
    env: apollo-carry-tray
    seed: 0
    max_steps: 700
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

# Carry the tray with the cup to the mat

You control an Apptronik Apollo humanoid (1.73 m; MuJoCo Menagerie `apptronik_apollo`, Apache-2.0). Its hands are rigid plates (about 8 cm wide, 17 cm long and 4 cm thick; the modelled fingers do not move). The robot is mounted on a support stand (its pelvis is welded to the stand): it does not balance or walk, its legs hold a standing pose and its torso is pitched 14 degrees forward over a table. The arms and the neck move. World frame in metres: +x points forward from the robot, +y to the robot's left, +z up; the floor is at z = 0 and the origin is on the floor below the robot's pelvis. A tray (20 x 32 cm, with 7 cm high sides) stands on the table in front of the robot with a loose cup (6 cm wide, 9 cm tall) standing in it. A blue mat lies further forward on the table. All positions and sizes are listed under `scene` and `objects` in `robo observe`. Knocking an object onto the floor is a safety event and fails the task. Each hand's control point is the centre of its hand plate. The arm controller keeps both hands pointing forward (+x) with the palms facing each other; the forearms are a little wider than the plates, so a box squeezed between the hands also touches the forearms. Each hand reaches only a little past the robot's midline (y = 0).

## Task

Carry the tray, with the cup still standing in it, onto the blue mat.

**Success:** the whole tray lies on the mat, flat (tilt at most 5 degrees) and resting on the table, the cup stands upright (tilt at most 10 degrees), and the robot touches neither; the cup touching anything other than the tray at any time (e.g. falling out) is a safety event and fails the task; and no safety event. Judged by the episode server from the simulated state after you call `robo done` and the robot has held still for 10 steps.

**Controls.** `robo act LDX LDY LDZ RDX RDY RDZ HYAW HPITCH`, each in [-1, 1]. LDX LDY LDZ and RDX RDY RDZ move the goal point of the left and right hand by up to 2 cm per step along world x, y, z; an arm controller (inverse kinematics and joint servos) moves the hand after its goal, stops when the hand is blocked and keeps the goal within reach. Zero keeps the goal where it is, so an all-zero action holds the robot still. HYAW turns the head left (+) or right (-) and HPITCH tilts it down (+) or up (-), by up to 3 degrees per step; the `head` camera moves with the head. One step is 40 ms; `--repeat N` applies an action N times. Skills (`robo info` lists them with their arguments): `reach SIDE X Y Z [TOL]` (move one hand's control point in a straight line to a point; reports whether it arrived, was blocked or hit the edge of its reach, and what the hand touches), `reach_both XL YL ZL XR YR ZR [TOL]` (both hands at once, for bimanual holds), `home` (both hands back to their start points), `wait [STEPS]`, `look_at X Y Z` (turn the head camera toward a point). Skills do not plan around obstacles.

**Observation.** `robo observe` reports `robot.hands.left` / `robot.hands.right` (`pos`: the control point, `goal`: its current goal, `touching`: what the hand touches) and `robot.head` (neck yaw and pitch); `objects` (each free object's centre `pos`, `tilt_deg` from upright, `touched_by_robot`, and its size); `scene` (the table and every fixture and target zone, with positions); `progress` where the task tracks something; and `safety_events`. Cameras: `front` (default), `top`, `side` and `head` (fixed at the robot's head, looking at the table).

The step budget is 700 steps (28 s of robot time).

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
