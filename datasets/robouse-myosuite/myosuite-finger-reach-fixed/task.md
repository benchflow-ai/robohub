---
schema_version: '1.3'
task:
  name: robouse/myosuite-finger-reach-fixed
  description: Index fingertip to a fixed point
metadata:
  author_name: benchflow
  source_benchmark: MyoSuite (MyoSuite 2.12.2, MyoHub)
  source_task: myoFingerReachFixed-v0
  suite: myosuite
  category: reaching
  difficulty: easy
  tags:
  - myosuite
  - musculoskeletal
  - muscles
  - reach
  simulator: MyoSuite 2.12.2 (Apache-2.0), MuJoCo 3.6.0
  robouse:
    id: myosuite-finger-reach-fixed
    backend: myosuite
    env: myosuite-finger-reach-fixed
    seed: 0
    max_steps: 300
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
---

You are controlling a simulated robot. Read the task below, then solve it by running the `robo` command in your shell (start with `robo info` and `robo observe`). Keep going until the task is done, then call `robo done` once. Do not stop to ask questions; there is no human to answer.

# Index fingertip to a fixed point

You control MyoFinger, MyoSuite's musculoskeletal model of a single index finger (MuJoCo physics), only through its 5 muscles: `extn` (extensor), `adabR` and `adabL` (sideways, abduction/adduction), `mflx` and `dflx` (flexors). The finger's base is fixed; it has 4 joints: `IFadb` (sideways, -0.44 to 0.44 rad), `IFmcp` (knuckle flexion, -0.44 to 1.05 rad), `IFpip` and `IFdip` (middle and end joint flexion, 0 to 1.05 rad). Angles are in radians. The finger starts straight (all joints 0), pointing along +x; flexing bends it towards -z. World frame in metres, +z up. A muscle can only pull, and the flexors cross several joints.

## Task

Move the fingertip to the target position (MyoSuite env `myoFingerReachFixed-v0`): `IFtip` (index tip) to [0.2, 0.05, 0.2] (world frame, m). The targets are also drawn as coloured spheres. At the start the error norm is 0.132 m.

**Success:** MyoSuite's own `solved` check for this env: the Euclidean norm of all fingertip position errors stacked together (`reach_error_norm`, 3 numbers) must be below 0.0125 m (12.5 mm per fingertip, `threshold`), judged by the episode server from the simulated state after you call `robo done` and the 10-step settle. Simulated time advances only when you act (one step = 20 ms).

**Controls.** `robo act E1 E2 ... [--repeat N]` sets the excitation (neural drive) of every muscle, a number in [0, 1], in the order `robo info` lists them (the action names), and holds it for N steps of 20 ms. A muscle's activation follows its excitation with MuJoCo's first-order activation dynamics (about 10 ms rising, 40 ms falling); its force also depends on its length and speed. The excitations stay as you last sent them until you act again, and they are also what the muscles hold during the 10-step settle after `robo done`. Skills (`robo info` lists them): `robo skill set_joint_targets JOINT=RAD [JOINT=RAD ...] [steps=60] [ramp=20]` runs a muscle-space controller: every step it computes the joint torques of a PD law towards the targets (MuJoCo inverse dynamics), solves a bounded least-squares problem for the muscle activations in [0, 1] that best produce those torques, and sends the matching excitations as that step's action (the steps count against the budget like `robo act`). Joints you do not name keep their previous targets (at the start: the start pose). The targets move linearly from the current pose over RAMP steps; the skill stops after STEPS steps (at most 300) or once the joints are still, and reports the remaining joint-target error and the task's error. `robo skill hold [steps=20]` sets every target to the current angle and keeps the joints there with the same controller. The controller cannot do the impossible: joints coupled by shared muscles or blocked by contact can stop short of their targets, so check the result and adjust. `robo skill reach_tips TIP=X,Y,Z [TIP=X,Y,Z ...] [steps=60] [ramp=20]` (positions in metres, no spaces inside a position) first solves inverse kinematics for the named fingertips within the joint ranges, starting from the current pose, then runs `set_joint_targets` with the result; it also reports the kinematic residual per fingertip.

**Observation.** `robo observe` reports `fingertips` and `fingertip_targets` (positions, m), `reach_error_norm` and `threshold`; `joint_angles_rad`, `joint_velocities_rad_s` and `joint_targets_rad` (the controller's current targets) per joint; `muscle_activations` (one per muscle, in action order); `contacts` (pairs of bones or objects touching); `time_s`; and `obs_vector`, MyoSuite's own observation vector for this env (its observation keys, in order: qpos, qvel, tip_pos, reach_err, act; not printed by plain `robo observe`, shown with `robo observe --json`). `robo observe --image` saves a picture from a fixed camera (muscle paths are not drawn).

The step budget is 300 steps (6 s of simulated time).

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
