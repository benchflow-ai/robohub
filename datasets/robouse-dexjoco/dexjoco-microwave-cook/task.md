---
schema_version: '1.3'
task:
  name: robouse/dexjoco-microwave-cook
  description: With two dexterous hands, open a microwave, put a hot dog in, close it and press start.
metadata:
  author_name: benchflow
  source_benchmark: DexJoCo (brave-eai/dexjoco, MIT; arXiv 2605.16257)
  source_task: bimanual_microwave_cook
  source_demo: DexJoCo/DexJoCo-Datasets-Raw/dexjoco_raw_datasets/bimanual_microwave_cook/microwave_cook_demo_10_2026-02-09_17-09-31_847162
  suite: dexjoco
  category: manipulation
  difficulty: hard
  tags:
  - mujoco
  - dexterous
  - allegro
  - panda
  - bimanual
  - articulated
  - long-horizon
  - appliance
  robouse:
    id: dexjoco-microwave-cook
    backend: dexjoco
    env: bimanual_microwave_cook
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
    - -0.442943
    - -0.4000001
    - 1.517013
    - 1.6e-06
    - -0.9999998
    - 7.0e-07
    - -0.0006542
    - -0.442943
    - 0.4000001
    - 1.517013
    - 3.0e-06
    - -0.9999998
    - 1.5e-06
    - -0.0006542
    - 3.95e-05
    - 0.0363198
    - 0.0117282
    - 0.002239
    - 3.93e-05
    - 0.0363199
    - 0.0117282
    - 0.002239
    - 3.91e-05
    - 0.0363199
    - 0.0117282
    - 0.002239
    - 0.3135996
    - -8.78e-05
    - -2.41e-05
    - -8.1e-06
    - 3.95e-05
    - 0.0363198
    - 0.0117282
    - 0.002239
    - 3.93e-05
    - 0.0363199
    - 0.0117282
    - 0.002239
    - 3.91e-05
    - 0.0363198
    - 0.0117282
    - 0.002239
    - 0.313637
    - -8.78e-05
    - -2.4e-05
    - -8.1e-06
    - -0.3126657
    - -0.3658445
    - 0.9986442
    - 0.7151768
    - 0.0
    - 0.0
    - 0.6989436
    - -0.05
    - 0.12
    - 1.1336442
    - 0.7071068
    - 0.0
    - 0.0
    - -0.7071068
    - 0.0336442
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

# Cook a Hot Dog in the Microwave (DexJoCo)

Two Franka Emika Panda arms stand side by side at a table (table top about 0.93-1.0 m above the floor, raised by a random amount per scene), each with a four-fingered Allegro robot hand (16 joints) mounted on its flange instead of a gripper. The hands are about 25 cm long and 10 cm wide; their fingers are index, middle, ring and thumb. World frame in metres, z up.

**Goal:** A microwave oven and a hot dog stand on the table. Open the microwave door, put the hot dog inside, close the door, and press the start button.

**Success:** The hot dog is inside the microwave's cavity, the door is closed (`microwave_door_angle` within 0.01 rad of 0) and the start button is being touched, all at the same time. The episode ends as solved the moment this condition is met; `robo done` before that scores 0.

**Task fields in `robo observe`:** `hot_dog_*`, `microwave_*` (oven body pose), `microwave_door_angle` (rad; 0 = closed) and `start_button` (button position).

**Controls.** `robo act` takes 44 numbers: the right arm's 22 (`R_DX .. R_F15`), then the left arm's 22 (`L_DX .. L_F15`). Per arm: `DX DY DZ` move the commanded hand pose by 2 cm per unit along world x, y, z (up to +-8 units = 16 cm per step); `RX RY RZ` rotate the commanded hand orientation by 0.2 rad per unit about the world x, y, z axes (up to +-3); `F0 .. F15` change the commanded finger joint angles by 0.3 rad per unit (up to +-6): F0-F3 index, F4-F7 middle, F8-F11 ring, F12-F15 thumb. In the left hand, DexJoCo orders the three fingers the other way round: L_F0-L_F3 drive the joints DexJoCo names ring, L_F4-L_F7 middle, L_F8-L_F11 index (thumb stays L_F12-L_F15); take a wrist-camera picture to see which physical finger moves. For index, middle and ring, joint 0 spreads the finger sideways and joints 1-3 curl it (positive = flex toward the palm); for the thumb, joint 0 swings it across the palm (opposition) and joints 1-3 rotate and curl it. Commands are targets: the arm's operational-space controller and the finger position servos track them, so the hand lags a fast command and stops where contact blocks it. `robo info` lists each joint's range (the commanded angle is clipped to it). One step is 20 ms of simulated time. Actions are deltas, so `--repeat N` applies the same change N times. `robo move-to X Y Z` moves the the *active* arm's; the right arm is active at the start and any `robo act` that moves only one arm makes that arm active (`active_arm` in `robo observe`) hand (the point `hand_pos`) toward a point without changing its orientation or fingers, and `robo grip G` sets the the *active* arm's; the right arm is active at the start and any `robo act` that moves only one arm makes that arm active (`active_arm` in `robo observe`) hand to a generic power grasp (G > 0 closes to fraction G of it, G < 0 opens all fingers, 0 keeps them); the grasp preset does not suit every object, so finger-level `robo act` control is often needed. The `DX DY DZ GRIP` form in the general instructions below does not apply: every `robo act` needs all 44 numbers.

**Observation.** `robo observe` reports for each arm, prefixed `right_` / `left_`: `hand_pos` and `hand_quat_wxyz` (measured flange position and orientation, quaternion w, x, y, z), `hand_target_pos` and `hand_target_quat_wxyz` (the commanded pose), `finger_joints` and `finger_joint_targets` (16 measured and commanded joint angles in radians, in F0..F15 order); for objects, `<name>_pos` (body origin), `<name>_quat_wxyz`, `<name>_tilt_deg` (angle of the body's z axis from vertical) and `<name>_yaw_deg`; plus the task fields listed above. `robo observe --image` saves a picture from the back camera; `--camera` picks another (`back`, `handcam_rgb_right`, `handcam_rgb_left`). The hand's orientation matters: at the start the palm faces down; look at a picture before you reach.

The step budget is 3000 steps (60 s of simulated time). A human teleoperator needed 693 steps for this scene.

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
