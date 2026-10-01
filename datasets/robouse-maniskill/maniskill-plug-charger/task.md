---
schema_version: '1.3'
task:
  name: robouse/maniskill-plug-charger
  description: Plug the charger into the receptacle (ManiSkill3 PlugCharger-v1, Franka Panda, CPU physics).
metadata:
  author_name: benchflow
  source_benchmark: ManiSkill3 (Hao Su Lab / Hillbot; mani-skill 3.0.1, SAPIEN 3.0.3)
  source_task: PlugCharger-v1
  suite: maniskill
  category: manipulation
  difficulty: hard
  tags:
  - sapien
  - maniskill
  - panda
  - tabletop
  - insertion
  - precision
  success_rule: ManiSkill PlugCharger-v1 evaluate()['success']
  reference_solution: 'scripted end-effector controller: grasp the back of the base with the hand tilted back, align the pins with the slots and push in along the slot axis while re-centring'
  robouse:
    id: maniskill-plug-charger
    backend: maniskill
    env: maniskill-plug-charger
    seed: 1
    max_steps: 600
    camera: render_camera
    cameras:
    - render_camera
    - base_camera
    - hand_camera
    skills: true
    success_mode: final
    obs_mode: state
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

# Plug the charger into the receptacle (ManiSkill3 PlugCharger-v1)

A Franka Emika Panda arm (7 joints, two-finger parallel gripper that opens to 8 cm) stands at a wooden table in ManiSkill3's `PlugCharger-v1` task, simulated with SAPIEN 3 (PhysX). The robot's base is at x = -0.615, y = 0 and faces +x, so +x points away from the robot and +y to its left. The table top is the plane z = 0.

**Goal:** Pick up the charger (a small white block with two metal pins) and plug its pins into the wall receptacle that stands on the table.

**Success:** ManiSkill's own PlugCharger check: the charger's pose is within 5 mm and 0.2 rad of the fully plugged-in pose (`goal_pos`, `goal_quat`). The pin slots leave only 0.5 mm of clearance. Success is judged by the episode server after you call `robo done` and the robot has held still for about 10 steps, so the result must last.

**Task fields in `robo observe`:** `charger_pos` (the point where the pins leave the base, on the pins' centre line), `charger_quat`, `charger_yaw_deg`, `charger_plug_axis` (unit vector along the pins, pointing out of the base), `charger_base_pos` (centre of the 4 x 3 x 2.4 cm base block), `charger_base_half_size`, `goal_pos` and `goal_quat` (where `charger_pos` and the charger's orientation must end up), `goal_plug_axis` (the pin direction at the goal, pointing into the receptacle), `receptacle_pos`, `receptacle_quat`, `receptacle_yaw_deg` and `grasped` (the charger).

**Controls.** The action has 7 numbers, not 4: `robo act DX DY DZ DROLL DPITCH DYAW GRIP`, each in [-1, 1], in the WORLD frame. DX/DY/DZ move the gripper's commanded position by 2 cm per unit per step. DROLL/DPITCH/DYAW turn its commanded orientation by 0.1 rad (5.7 degrees) per unit per step about the world x, y and z axes, around the point between the fingertips; leave them at 0 unless you need to turn the hand (for example DYAW to line the fingers up with an object's faces). GRIP: any positive value closes the fingers, a negative value -f opens them to fraction f (-1 fully open), 0 keeps them as they are (they take about 5 steps to move). One step is 50 ms (ManiSkill's 20 Hz control rate). The arm follows the commanded pose through inverse kinematics and ManiSkill's joint position controller; the commanded position stays within 5 cm of the measured hand (and the orientation within 0.35 rad), so pushing against something stalls instead of winding up. Some orientations are out of the arm's reach (the last wrist joint turns about 166 degrees each way): turning the fingers by 90 or 180 degrees about the vertical gives the same grasp on a cube, so prefer the smaller turn. `robo move-to X Y Z [--grip G]` and `robo grip G` also work; they move only the position and keep the hand's orientation, and `move-to` stops within 1 cm of the point unless you pass a tighter `--tol` (e.g. `--tol 0.002`).

**Observation.** `robo observe` reports `hand_pos` (the point between the fingertips, metres), `hand_quat` (hand orientation quaternion w, x, y, z), `hand_down_axis` (unit vector the fingers point along; [0, 0, -1] is straight down), `finger_axis` (unit vector along which the fingers close), `hand_target_pos` and `hand_target_quat` (the commanded pose the arm is moving to), `gripper_open` (0 closed .. 1 fully open) and `gripper_width` (distance between the fingers, metres), plus the task fields below. Every `<object>_quat` is (w, x, y, z); `<object>_yaw_deg` is the object's heading about the vertical (its local x axis, degrees from +x). `grasped`, where present, is ManiSkill's own grasp test (both fingers pressing on the object). `robo observe --image` saves a 384x384 picture from the `render_camera` view; `--camera base_camera`, `--camera hand_camera` (on the wrist) gives other views.

The step budget is 600 steps (30 s of simulated time).

## How to control the robot

You are the robot's policy. You act only through the `robo` command in your shell. There is no other way to move the robot, and you cannot read or change the simulator, the scoring, or other files to succeed; the episode server judges the final physical state itself.

```
robo info                         # action space, available skills, step budget
robo observe                      # robot and object state as numbers
robo observe --image              # also saves a camera image and prints its path (open it to look)
robo act DX DY DZ GRIP [--repeat N]   # low-level action, applied N times (N <= 50)
robo move-to X Y Z [--grip G]     # skill: move the gripper toward a point (if enabled for this task)
robo grip G [--steps N]           # skill: hold position and set the gripper (+1 close, -1 open)
robo done "short summary"         # end the episode and ask for scoring
robo give-up "reason"             # end the episode without claiming success
```

- Positions are in metres in the world frame (x, y on the table plane, z up).
- The episode has a fixed step budget (see `robo info`); every simulated step counts, including skills.
- Unless the task says otherwise, success is judged about 10 steps after you call `robo done`, with the robot holding still, so the goal must still be true when the robot stops.
- Work in small steps and re-observe after each motion. Call `robo done` exactly once when finished.
