---
schema_version: '1.3'
task:
  name: robouse/kitchen-hinge-cabinet
  description: Open the hinged wall cabinet (its right-hand door).
metadata:
  author_name: benchflow
  source_benchmark: Gymnasium-Robotics Franka Kitchen (Farama; Relay Policy Learning / D4RL kitchen)
  source_task: FrankaKitchen-v1 tasks_to_complete=[hinge cabinet]
  suite: kitchen
  category: manipulation
  difficulty: medium
  tags:
  - franka
  - mujoco
  - single-arm
  - articulated
  - kitchen
  robouse:
    id: kitchen-hinge-cabinet
    backend: kitchen
    env: kitchen-hinge-cabinet
    seed: 0
    max_steps: 500
    camera: front
    skills: true
    success_mode: final
agent:
  timeout_sec: 900
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

# Kitchen: Open the hinged wall cabinet (its right-hand door)

A 7-joint Franka Panda arm with a two-finger parallel gripper stands in front of a kitchen (adapted from Gymnasium-Robotics `FrankaKitchen-v1`, the Relay Policy Learning / D4RL kitchen). World frame in metres: the robot's base is at (0, 0, 1.8); +y points from the robot into the kitchen, +x to the robot's right, +z up. The stove top is at about z = 1.6, the knob panel at y = 0.64 and the wall cabinets at z = 2.6. In front of the robot are: a microwave (left), the stove with four burners and a kettle, the oven knob panel with a light switch, and two wall cabinets above (hinged on the left, sliding on the right).

**Goal:** Open the hinged wall cabinet (its right-hand door).

Success is the upstream Franka Kitchen completion check for the subtask (`hinge cabinet`): the subtask's joint values must be within 0.3 (Euclidean norm) of its goal values, judged on the physical state after you call `robo done` and the robot has held still for about 10 steps. Everything must still be in place at that moment (a door that swings shut again, or a kettle that falls over, does not count).

- hinge cabinet: `subtasks.hinge_cabinet.joints` is the angle of the left door and of the right door (rad; 0 = closed, the right door opens to positive angles, limit 1.57); the goal is [0, 1.45], so the right door must be open past about 1.15 rad while the left door stays shut.

The hinged cabinet is the wall cabinet on the left, with two doors. Its right-hand door hinges on a vertical axis at `hinge_cabinet_hinge_pos` (the door's right edge) and swings toward the robot; `hinge_handle_pos` is the middle of that door's vertical bar handle.

In `robo observe`: `hand_pos` is the grasp centre between the finger pads (the fingertips reach about 5 cm further along +y; the fingers are about 2 cm thick and open to 8 cm), `hand_target` the commanded position, `hand_roll` / `hand_roll_target` the measured and commanded roll (rad), `gripper_open` (0 closed .. 1 open) and `gripper_width` (m), `arm_joints` the seven arm joint angles (rad). Landmarks (world positions, m): `microwave_handle_pos`, `microwave_hinge_pos`, `kettle_handle_pos`, `kettle_pos`, `kettle_quat` (w, x, y, z), `kettle_goal_pos`, `bottom_burner_knob_pos`, `top_burner_knob_pos`, `light_switch_pos`, `slide_handle_pos`, `hinge_handle_pos`, `hinge_cabinet_hinge_pos`. Under `subtasks`, each of the seven kitchen subtasks lists its current joint values (`joints`), its `goal`, `distance` (the norm of joints - goal), `complete` (distance < 0.3) and `required` (whether this task asks for it); `goals_complete` counts the required ones that are complete now.

**This task uses a 5-number action.** `robo act DX DY DZ DROLL GRIP [--repeat N]`, each in [-1, 1], one step = 80 ms: DX DY DZ move the gripper's commanded position by up to 3 cm per unit along world x, y, z; DROLL turns the gripper about its pointing axis by up to 0.1 rad (5.7 degrees) per unit (positive = clockwise as seen from the robot, the top of the gripper moving toward +x); GRIP +1 closes the fingers, -1 opens them. The backend turns this into joint motions with inverse kinematics; the commanded position never runs more than 6 cm ahead of the gripper, so pushing against furniture stalls rather than winding up. The gripper always points straight ahead (+y), horizontal. At roll 0 its two fingers are side by side along x (they close left-right, suited to vertical bars); at roll 1.57 (90 degrees) they are above and below each other (they close top-bottom, suited to horizontal bars). The roll is limited to +-1.6 rad. `robo move-to X Y Z [--grip G]` moves `hand_pos` to a point in a straight line at the current roll, and `robo grip G` holds position and sets the fingers; the `DX DY DZ GRIP` form in the general instructions below does not apply here (use 5 numbers). The arm cannot pass through furniture, and a move can stall if the arm or fingers touch something; check `hand_pos` after each move.

The step budget is 500 steps.

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
