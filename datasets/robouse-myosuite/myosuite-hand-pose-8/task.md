---
schema_version: '1.3'
task:
  name: robouse/myosuite-hand-pose-8
  description: 'Hand shape: sign pose 8'
metadata:
  author_name: benchflow
  source_benchmark: MyoSuite (MyoSuite 2.12.2, MyoHub)
  source_task: myoHandPose8Fixed-v0
  suite: myosuite
  category: posture
  difficulty: medium
  tags:
  - myosuite
  - musculoskeletal
  - muscles
  - pose
  simulator: MyoSuite 2.12.2 (Apache-2.0), MuJoCo 3.6.0
  robouse:
    id: myosuite-hand-pose-8
    backend: myosuite
    env: myosuite-hand-pose-8
    seed: 0
    max_steps: 500
    camera: view
    cameras:
    - view
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

# Hand shape: sign pose 8

You control MyoHand, MyoSuite's musculoskeletal model of a right human hand and forearm (MuJoCo physics), only through its 39 muscles. The forearm is fixed in space. 23 joints move: the wrist (`pro_sup` pronation/supination, `deviation`, `flexion`), the thumb (`cmc_abduction`, `cmc_flexion`, `mp_flexion`, `ip_flexion`) and the four fingers, numbered 2 = index, 3 = middle, 4 = ring, 5 = little, each with `mcpN_flexion` and `mcpN_abduction` (knuckle), `pmN_flexion` (middle joint) and `mdN_flexion` (end joint). Joint angles are in radians; `robo info` lists each joint's range under `joint_ranges_rad`. The muscles are the forearm muscles of the wrist and fingers (e.g. `ECRL`, `FCU`, `FDS2`..`FDS5`, `FDP2`..`FDP5`, `EDC2`..`EDC5`, `FPL`, `EPL`, `APL`) and the small hand muscles (`OP`, and per finger `RI`, `LU_RB`, `UI_UB` interossei/lumbricals). A muscle can only pull, many muscles cross several joints, and the fingers collide with each other (`contacts`), so moving one joint tends to move others. World frame in metres, +z up; the fingers point roughly along -y.

## Task

Form hand shape 8 of the ten American Sign Language hand shapes MyoSuite defines (the env's ASL_qpos table). The hand starts open (all joints 0). Some target shapes press fingers together, so an exact match of every joint is neither needed nor always possible. Bring the joints to the target angles in `target_pose_rad` (MyoSuite env `myoHandPose8Fixed-v0`). The target is `pro_sup` = 0.000, `deviation` = 0.000, `flexion` = 0.000, `cmc_abduction` = 0.428, `cmc_flexion` = 0.223, `mp_flexion` = -0.785, `ip_flexion` = -1.309, `mcp2_flexion` = 0.646, `mcp2_abduction` = -0.007, `pm2_flexion` = 0.128, `md2_flexion` = 0.195, `mcp3_flexion` = 1.390, `mcp3_abduction` = 0.000, `pm3_flexion` = 1.084, `md3_flexion` = 0.573, `mcp4_flexion` = 0.668, `mcp4_abduction` = -0.021, `pm4_flexion` = 0.000, `md4_flexion` = 0.063, `mcp5_flexion` = 0.432, `mcp5_abduction` = -0.068, `pm5_flexion` = 0.189, `md5_flexion` = 0.149 rad. At the start the pose error norm is 2.68 rad.

**Success:** MyoSuite's own `solved` check for this env: the Euclidean norm of the joint-angle error over all 23 joints (target minus current, `pose_error_norm`) must be below 0.7 rad (`threshold`, the env's pose_thd), judged by the episode server from the simulated state after you call `robo done` and the 10-step settle. Simulated time advances only when you act (one step = 20 ms).

**Controls.** `robo act E1 E2 ... [--repeat N]` sets the excitation (neural drive) of every muscle, a number in [0, 1], in the order `robo info` lists them (the action names), and holds it for N steps of 20 ms. A muscle's activation follows its excitation with MuJoCo's first-order activation dynamics (about 10 ms rising, 40 ms falling); its force also depends on its length and speed. The excitations stay as you last sent them until you act again, and they are also what the muscles hold during the 10-step settle after `robo done`. Skills (`robo info` lists them): `robo skill set_joint_targets JOINT=RAD [JOINT=RAD ...] [steps=60] [ramp=20]` runs a muscle-space controller: every step it computes the joint torques of a PD law towards the targets (MuJoCo inverse dynamics), solves a bounded least-squares problem for the muscle activations in [0, 1] that best produce those torques, and sends the matching excitations as that step's action (the steps count against the budget like `robo act`). Joints you do not name keep their previous targets (at the start: the start pose). The targets move linearly from the current pose over RAMP steps; the skill stops after STEPS steps (at most 300) or once the joints are still, and reports the remaining joint-target error and the task's error. `robo skill hold [steps=20]` sets every target to the current angle and keeps the joints there with the same controller. The controller cannot do the impossible: joints coupled by shared muscles or blocked by contact can stop short of their targets, so check the result and adjust.

**Observation.** `robo observe` reports `target_pose_rad` (the goal angles), `pose_error_norm` and `threshold`; `joint_angles_rad`, `joint_velocities_rad_s` and `joint_targets_rad` (the controller's current targets) per joint; `muscle_activations` (one per muscle, in action order); `contacts` (pairs of bones or objects touching); `time_s`; and `obs_vector`, MyoSuite's own observation vector for this env (its observation keys, in order: qpos, qvel, pose_err, act; not printed by plain `robo observe`, shown with `robo observe --json`). `robo observe --image` saves a picture from a fixed camera (muscle paths are not drawn).

The step budget is 500 steps (10 s of simulated time).

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
