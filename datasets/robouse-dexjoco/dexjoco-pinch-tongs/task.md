---
schema_version: '1.3'
task:
  name: robouse/dexjoco-pinch-tongs
  description: Pick up kitchen tongs, hold them up and squeeze them shut three times.
metadata:
  author_name: benchflow
  source_benchmark: DexJoCo (brave-eai/dexjoco, MIT; arXiv 2605.16257)
  source_task: pinch_tongs
  source_demo: DexJoCo/DexJoCo-Datasets-Raw/dexjoco_raw_datasets/pinch_tongs/table_tongs_demo_10_2026-03-12_16-30-04_442884
  suite: dexjoco
  category: manipulation
  difficulty: hard
  tags:
  - mujoco
  - dexterous
  - allegro
  - panda
  - tool-use
  - articulated
  - single-arm
  robouse:
    id: dexjoco-pinch-tongs
    backend: dexjoco
    env: pinch_tongs
    seed: 0
    max_steps: 2500
    camera: front
    cameras:
    - front
    - top0
    - left
    - right
    - handcam_rgb
    skills: true
    success_mode: first
    max_repeat: 50
    init_state:
    - -0.442948
    - -0.0
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
    - -0.3253343
    - -0.2292902
    - 1.0046566
    - 0.2705981
    - 0.2705981
    - 0.6532815
    - 0.6532815
    - 0.0496566
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

# Pinch the Tongs (DexJoCo)

A Franka Emika Panda arm stands at a table (table top about 0.93-1.0 m above the floor, raised by a random amount per scene) with a four-fingered Allegro robot hand (16 joints) mounted on its flange instead of a gripper. The hand is about 25 cm long and 10 cm wide; its fingers are index, middle, ring and thumb. World frame in metres, z up.

**Goal:** A pair of kitchen tongs lies on the table. Pick them up, hold them at least 10 cm above the table, and squeeze them shut three times.

**Success:** The tongs are at or above `lift_height_z` and `pinch_count` is at least 3, continuously for 30 steps (0.6 s). A pinch counts when `tongs_opening` (hinge angle, rad) first rises above 0.1 (open) and then drops to -0.07 or below (squeezed shut). The episode ends as solved the moment this condition is met; `robo done` before that scores 0.

**Task fields in `robo observe`:** `tongs_*` (tongs pose), `tongs_opening` (rad), `pinch_count` and `lift_height_z`.

**Controls.** `robo act` takes 22 numbers (`DX DY DZ RX RY RZ F0 .. F15`). Per arm: `DX DY DZ` move the commanded hand pose by 2 cm per unit along world x, y, z (up to +-8 units = 16 cm per step); `RX RY RZ` rotate the commanded hand orientation by 0.2 rad per unit about the world x, y, z axes (up to +-3); `F0 .. F15` change the commanded finger joint angles by 0.3 rad per unit (up to +-6): F0-F3 index, F4-F7 middle, F8-F11 ring, F12-F15 thumb. For index, middle and ring, joint 0 spreads the finger sideways and joints 1-3 curl it (positive = flex toward the palm); for the thumb, joint 0 swings it across the palm (opposition) and joints 1-3 rotate and curl it. Commands are targets: the arm's operational-space controller and the finger position servos track them, so the hand lags a fast command and stops where contact blocks it. `robo info` lists each joint's range (the commanded angle is clipped to it). One step is 20 ms of simulated time. Actions are deltas, so `--repeat N` applies the same change N times. `robo move-to X Y Z` moves the hand (the point `hand_pos`) toward a point without changing its orientation or fingers, and `robo grip G` sets the hand to a generic power grasp (G > 0 closes to fraction G of it, G < 0 opens all fingers, 0 keeps them); the grasp preset does not suit every object, so finger-level `robo act` control is often needed. The `DX DY DZ GRIP` form in the general instructions below does not apply: every `robo act` needs all 22 numbers.

**Observation.** `robo observe` reports for the arm: `hand_pos` and `hand_quat_wxyz` (measured flange position and orientation, quaternion w, x, y, z), `hand_target_pos` and `hand_target_quat_wxyz` (the commanded pose), `finger_joints` and `finger_joint_targets` (16 measured and commanded joint angles in radians, in F0..F15 order); for objects, `<name>_pos` (body origin), `<name>_quat_wxyz`, `<name>_tilt_deg` (angle of the body's z axis from vertical) and `<name>_yaw_deg`; plus the task fields listed above. `robo observe --image` saves a picture from the front camera; `--camera` picks another (`front`, `top0` (overhead), `left`, `right`, `handcam_rgb` (wrist)). The hand's orientation matters: at the start the palm faces down; look at a picture before you reach.

The step budget is 2500 steps (50 s of simulated time). A human teleoperator needed 375 steps for this scene.

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
