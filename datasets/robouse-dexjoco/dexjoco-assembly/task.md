---
schema_version: '1.3'
task:
  name: robouse/dexjoco-assembly
  description: With two dexterous hands, insert an 8 mm peg into a socket.
metadata:
  author_name: benchflow
  source_benchmark: DexJoCo (brave-eai/dexjoco, MIT; arXiv 2605.16257)
  source_task: bimanual_assembly
  source_demo: DexJoCo/DexJoCo-Datasets-Raw/dexjoco_raw_datasets/bimanual_assembly/assembly_demo_12_2026-03-19_19-30-52_804150
  suite: dexjoco
  category: manipulation
  difficulty: hard
  tags:
  - mujoco
  - dexterous
  - allegro
  - panda
  - bimanual
  - insertion
  - precision
  robouse:
    id: dexjoco-assembly
    backend: dexjoco
    env: bimanual_assembly
    seed: 0
    max_steps: 3000
    camera: back
    cameras:
    - back
    - handcam_rgb_right
    - handcam_rgb_left
    skills: true
    success_mode: first
    max_repeat: 50
    init_state:
    - -0.442948
    - -0.4
    - 1.5170135
    - 0.0
    - -0.9999969
    - 0.0
    - -0.0025
    - -0.442948
    - 0.4
    - 1.5170135
    - 0.0
    - -0.9999969
    - 0.0
    - -0.0025
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.263
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.263
    - 0.0
    - 0.0
    - 0.0
    - -0.2655997
    - 0.1732069
    - 0.9467634
    - 0.9983931
    - 0.0
    - 0.0
    - 0.0566674
    - -0.250095
    - -0.228174
    - 0.9667634
    - 0.7040682
    - -0.0654825
    - 0.7040682
    - 0.0654825
    - 0.0267634
agent:
  timeout_sec: 2400
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

# Peg-in-Hole Assembly (DexJoCo)

Two Franka Emika Panda arms stand side by side at a table (table top about 0.93-1.0 m above the floor, raised by a random amount per scene), each with a four-fingered Allegro robot hand (16 joints) mounted on its flange instead of a gripper. The hands are about 25 cm long and 10 cm wide; their fingers are index, middle, ring and thumb. World frame in metres, z up.

**Goal:** A round peg (8 mm across) and a socket block with a matching hole lie on the table. Pick them up and insert the peg fully into the socket's hole.

**Success:** The peg touches the bottom of the socket's hole continuously for 30 steps (0.6 s). The episode ends as solved the moment this condition is met; `robo done` before that scores 0.

**Task fields in `robo observe`:** `peg_*` and `socket_*` (poses of the two parts).

**Controls.** `robo act` takes 44 numbers: the right arm's 22 (`R_DX .. R_F15`), then the left arm's 22 (`L_DX .. L_F15`). Per arm: `DX DY DZ` move the commanded hand pose by 2 cm per unit along world x, y, z (up to +-8 units = 16 cm per step); `RX RY RZ` rotate the commanded hand orientation by 0.2 rad per unit about the world x, y, z axes (up to +-3); `F0 .. F15` change the commanded finger joint angles by 0.3 rad per unit (up to +-6): F0-F3 index, F4-F7 middle, F8-F11 ring, F12-F15 thumb. In the left hand, DexJoCo orders the three fingers the other way round: L_F0-L_F3 drive the joints DexJoCo names ring, L_F4-L_F7 middle, L_F8-L_F11 index (thumb stays L_F12-L_F15); take a wrist-camera picture to see which physical finger moves. For index, middle and ring, joint 0 spreads the finger sideways and joints 1-3 curl it (positive = flex toward the palm); for the thumb, joint 0 swings it across the palm (opposition) and joints 1-3 rotate and curl it. Commands are targets: the arm's operational-space controller and the finger position servos track them, so the hand lags a fast command and stops where contact blocks it. `robo info` lists each joint's range (the commanded angle is clipped to it). One step is 20 ms of simulated time. Actions are deltas, so `--repeat N` applies the same change N times. `robo move-to X Y Z` moves the the *active* arm's; the right arm is active at the start and any `robo act` that moves only one arm makes that arm active (`active_arm` in `robo observe`) hand (the point `hand_pos`) toward a point without changing its orientation or fingers, and `robo grip G` sets the the *active* arm's; the right arm is active at the start and any `robo act` that moves only one arm makes that arm active (`active_arm` in `robo observe`) hand to a generic power grasp (G > 0 closes to fraction G of it, G < 0 opens all fingers, 0 keeps them); the grasp preset does not suit every object, so finger-level `robo act` control is often needed. The `DX DY DZ GRIP` form in the general instructions below does not apply: every `robo act` needs all 44 numbers.

**Observation.** `robo observe` reports for each arm, prefixed `right_` / `left_`: `hand_pos` and `hand_quat_wxyz` (measured flange position and orientation, quaternion w, x, y, z), `hand_target_pos` and `hand_target_quat_wxyz` (the commanded pose), `finger_joints` and `finger_joint_targets` (16 measured and commanded joint angles in radians, in F0..F15 order); for objects, `<name>_pos` (body origin), `<name>_quat_wxyz`, `<name>_tilt_deg` (angle of the body's z axis from vertical) and `<name>_yaw_deg`; plus the task fields listed above. `robo observe --image` saves a picture from the back camera; `--camera` picks another (`back`, `handcam_rgb_right`, `handcam_rgb_left`). The hand's orientation matters: at the start the palm faces down; look at a picture before you reach.

The step budget is 3000 steps (60 s of simulated time). A human teleoperator needed 483 steps for this scene.

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
