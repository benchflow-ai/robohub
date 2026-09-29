---
schema_version: '1.3'
task:
  name: robouse/g1-drawer-stow
  description: Stow the soap in the drawer
metadata:
  author_name: benchflow
  source_benchmark: robouse original (humanoid suite; Unitree G1 with Dex3-1 hands from MuJoCo Menagerie)
  source_task: g1-drawer-stow
  suite: humanoid
  category: articulated
  difficulty: hard
  tags:
  - humanoid
  - g1
  - fixed-base
  - manipulation
  - menagerie
  - mujoco
  embodiment: unitree_g1
  robouse:
    id: g1-drawer-stow
    backend: humanoid
    env: g1-drawer-stow
    seed: 0
    max_steps: 1200
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
---

You are controlling a simulated robot. Read the task below, then solve it by running the `robo` command in your shell (start with `robo info` and `robo observe`). Keep going until the task is done, then call `robo done` once. Do not stop to ask questions; there is no human to answer.

# Stow the soap in the drawer

You control a Unitree G1 humanoid (1.32 m; MuJoCo Menagerie `unitree_g1`, BSD-3-Clause) with Dex3-1 three-finger hands (a thumb and two fingers per hand). The robot is mounted on a support stand (its pelvis is welded to the stand): it does not balance or walk, its legs hold a standing pose and its waist is bent 26 degrees forward over a table in front of it. Only the arms and fingers move. World frame in metres: +x points forward from the robot, +y to the robot's left, +z up; the floor is at z = 0 and the origin is on the floor below the robot's pelvis. A small white cabinet (26 cm wide, 10 cm tall) stands on the table in front of the robot. Its drawer slides out toward the robot (up to 16 cm) and has a tall vertical bar handle 5 cm in front of the drawer face that rises 11 cm above the cabinet top. A bar of soap in a light-blue box (4.4 x 4.4 x 6 cm) lies on the table at the left. Only the part of the drawer that is pulled out in front of the cabinet is open from above; the drawer front is as tall as the cabinet. All positions and sizes are listed under `scene` and `objects` in `robo observe`. Knocking an object onto the floor is a safety event and fails the task. Each hand's control point is its grasp point, between the thumb and the fingers. The arm controller keeps the hand level with the fingers pointing forward (+x) and the palm facing the robot's midline; the two fingers are on the outer side of the control point and reach 8 cm in front of it, the thumb is on the inner side. An object whose centre is at the control point sits between thumb and fingers (the open hand is about 6 cm wide inside); closing the fingers wraps them around it (a friction grasp). The hand cannot tilt, so objects are approached from behind (-x) or from above, and each hand reaches only about 10 cm past the robot's midline (y = 0).

## Task

Put the soap into the drawer and close the drawer.

**Success:** the soap's centre is inside the drawer's tray, the drawer is closed (open at most 1.5 cm), and the robot does not touch the soap; `progress` shows the drawer opening and whether the soap is in the drawer; and no safety event. Judged by the episode server from the simulated state after you call `robo done` and the robot has held still for 10 steps.

**Controls.** `robo act LDX LDY LDZ RDX RDY RDZ LGRIP RGRIP`, each in [-1, 1]. LDX LDY LDZ and RDX RDY RDZ move the goal point of the left and right hand by up to 2 cm per step along world x, y, z; an arm controller (inverse kinematics and joint servos) moves the hand after its goal, stops when the hand is blocked and keeps the goal within reach. Zero keeps the goal where it is, so an all-zero action holds the robot still. LGRIP and RGRIP close (+1) or open (-1) the fingers of that hand by 0.2 of their range per step (0 keeps the current finger command); the fingers stop when they press on something, with the Dex3's own joint torque limits. One step is 40 ms; `--repeat N` applies an action N times. Skills (`robo info` lists them with their arguments): `reach SIDE X Y Z [TOL]` (move one hand's control point in a straight line to a point; reports whether it arrived, was blocked or hit the edge of its reach, and what the hand touches), `reach_both XL YL ZL XR YR ZR [TOL]` (both hands at once, for bimanual holds), `home` (both hands back to their start points), `wait [STEPS]`, `grasp SIDE` (close the fingers until they stop; reports what the hand touches and whether an object is held between thumb and fingers), `release SIDE` (open the fingers). Skills do not plan around obstacles.

**Observation.** `robo observe` reports `robot.hands.left` / `robot.hands.right` (`pos`: the control point, `goal`: its current goal, `touching`: what the hand touches, `grip` (0 open .. 1 closed) and `holding` (an object between thumb and fingers)); `objects` (each free object's centre `pos`, `tilt_deg` from upright, `touched_by_robot`, and its size); `scene` (the table and every fixture and target zone, with positions); `progress` where the task tracks something; and `safety_events`. Cameras: `front` (default), `top`, `side` and `head` (fixed at the robot's head, looking at the table).

The step budget is 1200 steps (48 s of robot time).

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
