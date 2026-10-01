---
schema_version: '1.3'
task:
  name: robouse/capx-two-arm-handover-s3
  description: One Franka arm picks up a hammer and hands it to the other. Code as policy with CaP-X's privileged API.
metadata:
  author_name: benchflow
  source_benchmark: CaP-X (capgym/cap-x, commit 53e9966)
  source_task: two_arm_handover (capx/envs/tasks/franka/two_arm_handover.py)
  suite: capx
  category: code-as-policy
  difficulty: medium
  tags:
  - mujoco
  - robosuite
  - panda
  - code-as-policy
  - bimanual
  - handover
  capx:
    tier: 'privileged API (the API of CaP-Bench''s S1 tier), multi-turn: not one of CaP-Bench''s eight tiers'
    api:
    - close_gripper_arm0
    - close_gripper_arm1
    - get_hammer_pose
    - goto_pose_arm0
    - goto_pose_arm1
    - open_gripper_arm0
    - open_gripper_arm1
    layout_seed: 3
  robouse:
    id: capx-two-arm-handover-s3
    backend: capx
    env: two_arm_handover
    seed: 3
    max_steps: 5000
    camera: agentview
    cameras:
    - agentview
    skills: true
    success_mode: final
    frame_every: 8
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

# Hand over the hammer (CaP-X, privileged API)

This is a code-as-policy task from CaP-X: you control the robot by writing Python code that calls the task's API. The task text and the API documentation below are CaP-X's own.

## Task (CaP-X's prompt)

```text
You are controlling a two-arm Franka Emika robot system with API described below.
Goal: Arm 0 should pick up the hammer, lift it, and hand it over to Arm 1. Arm 1 should then grasp the hammer handle (not hammer head). The z-value of hammer must be between 0.15 and 0.20 during the handover to count as a success.

Coordinate system:
- All pose functions accept positions in robot0's base frame (same coordinate system as returned/used by get_object_pose/goto_pose_arm*).
- The table surface is not necessarily at z=0.
- The coordinate axis follows these conventions:
  - Z-axis: up (positive) and down (negative)
  - X-axis: right (positive) and left (negative)
  - Y-axis: forward (positive) and backward (negative)

Environment details:
- Arm 0 (left) and Arm 1 (right) are positioned on opposite sides of the table.
- The hammer handle length is randomized between 0.15m and 0.25m.
- The hammer initially lies flat on the table, aligned along the Y-axis, with the handle toward +Y and the hammer head toward -Y

Critical information:
- Handover must occur near the midpoint of the initial gripper positions. Reason about the best handover hammer orientation that allows collision-free transfer to Arm 1.
- The table does not span the entire region between the two arms. If the hammer falls in the central gap it will drop to the floor, making the task UNRECOVERABLE
- AVOID COLLISIONS. You must decompose movements into rotation and translation components, moving stepwise. Reason about the optimal sequence of waypoints to safely avoid collisions.
- Arm links have volume, and intermediate motions are not collision-checked. Keep a minimum 8cm buffer between grippers

Reference quaternions:
- Arm0 gripper facing down opening along X-axis: [0, 0.707, 0.707, 0].
- Arm0 gripper facing down opening along Y-axis: [0, 1, 0, 0].
- Arm1 gripper facing down opening along Y-axis: [0, 0, 1, 0].

Arm gripper approximate initial starting positions (robot0 frame). These values are rough and may vary by a few centimeters each trial:
- Arm0: x = 0.44. y = 0.0
- Arm1: x = 1.18. y = 0.0
```

**Success:** CaP-X's own check for this task: the second arm holds the hammer by its handle, the first arm holds nothing, and the hammer is more than 10 cm above the table. It is judged after you call `robo done` and the robot has held still for about 10 control steps.

## API (CaP-X's documentation)

```text
get_hammer_pose() -> tuple[numpy.ndarray, numpy.ndarray]
  Doc:
    Get the pose of the middle of the hammer handle. 
    The quaternion output may be unreliable.
    
    Returns:
        position: (3,) XYZ in meters.
        quaternion_wxyz: (4,) WXYZ unit quaternion.

goto_pose_arm0(position: numpy.ndarray, quaternion_wxyz: numpy.ndarray, z_approach: float = 0.0) -> None
  Doc:
    Go to pose using Inverse Kinematics for Arm 0 (robot0).
    Position and quaternion are in robot0's base frame.
    There is no need to call a second goto_pose_arm0 with the same position and quaternion_wxyz after calling it with z_approach.
    Args:
        position: (3,) XYZ in meters, in robot0's base frame.
        quaternion_wxyz: (4,) WXYZ unit quaternion.
        z_approach: (float) Z-axis distance offset for goto_pose insertion approach motion. Will first arrive at position + z_approach meters in Z-axis before moving to the requested pose. Useful for more precise grasp approaches. Default is 0.0.
    Returns:
        None

goto_pose_arm1(position: numpy.ndarray, quaternion_wxyz: numpy.ndarray, z_approach: float = 0.0) -> None
  Doc:
    Go to pose using Inverse Kinematics for Arm 1 (robot1).
    Position and quaternion are in robot0's base frame (same as returned by get_hammer_pose).
    The function automatically transforms coordinates from robot0's base frame to robot1's base frame.
    There is no need to call a second goto_pose_arm1 with the same position and quaternion_wxyz after calling it with z_approach.
    Args:
        position: (3,) XYZ in meters, in robot0's base frame (will be transformed to robot1's base frame).
        quaternion_wxyz: (4,) WXYZ unit quaternion.
        z_approach: (float) Z-axis distance offset for goto_pose insertion approach motion. Will first arrive at position + z_approach meters in Z-axis before moving to the requested pose. Useful for more precise grasp approaches. Default is 0.0.
    Returns:
        None

open_gripper_arm0() -> None
  Doc:
    Open gripper fully for Arm 0 (robot0).
    
    Args:
        None

open_gripper_arm1() -> None
  Doc:
    Open gripper fully for Arm 1 (robot1).
    
    Args:
        None

close_gripper_arm0() -> None
  Doc:
    Close gripper fully for Arm 0 (robot0).
    
    Args:
        None

close_gripper_arm1() -> None
  Doc:
    Close gripper fully for Arm 1 (robot1).
    
    Args:
        None
```

## How you run code here

Each API function is a skill of the robot: `robo skill NAME ARG ...` runs it, with one JSON value per argument (a number, a list for an array, `true`/`false`, a string) or `name=VALUE` for a keyword argument, for example `robo skill goto_pose "[0.5, 0.0, 0.2]" "[0, 0, 1, 0]" z_approach=0.1`. It prints what the function returns and anything it printed. A call that takes more than 40 control steps returns `running: True` after 40 of them; `robo skill capx_continue` goes on with it, 40 steps at a time, until it returns. To write ordinary Python instead, save this module as `capx_api.py` in your working folder and `from capx_api import *`: it defines every API function of the task under CaP-X's name and signature, returns numpy arrays (a tuple when a function returns several values), and goes on with long calls by itself, and raises `RuntimeError` when a call fails.

```python
"""CaP-X's API as Python functions, over the robo CLI (standard library and numpy only).

Each API function of the task (`robo info` lists them under skills) becomes a function of this module with CaP-X's name
and arguments: arrays and numbers go to `robo skill NAME ...` as JSON, and what the function returns comes back as
numpy arrays (a tuple when it returns several values). Anything the API prints is printed here too.

    from capx_api import *
    pos, quat = sample_grasp_pose("red cube")
    goto_pose(pos, quat, z_approach=0.1)
"""

import json
import subprocess

import numpy as np


def _enc(v):
    if hasattr(v, "tolist"):
        v = v.tolist()
    return json.dumps(v)


def _dec(v):
    if isinstance(v, dict):
        return {k: _dec(x) for k, x in v.items()}
    if isinstance(v, list):
        if all(isinstance(x, (int, float)) and not isinstance(x, bool) for x in v) and v:
            return np.asarray(v, dtype=np.float64)
        return tuple(_dec(x) for x in v)
    return v


def _skill(name, argv):
    out = subprocess.run(["robo", "skill", name, *argv, "--json"], capture_output=True, text=True)
    r = json.loads(out.stdout or '{"ok": false, "error": "no reply from robo"}')
    if not r.get("ok"):
        raise RuntimeError(f"{name}: {r.get('error')}")
    return r["result"]


def call(name, *args, **kwargs):
    argv = [_enc(a) for a in args] + [f"{k}={_enc(v)}" for k, v in kwargs.items()]
    res = _skill(name, argv)
    while res.get("running"):  # a long call runs in parts of a few dozen control steps
        res = _skill("capx_continue", [])
    if res.get("stdout"):
        print(res["stdout"], end="")
    if "error" in res:
        raise RuntimeError(f"{name}: {res['error']}")
    if res.get("stopped"):
        raise RuntimeError(f"{name}: stopped ({res['stopped']})")
    return _dec(res.get("return"))


def _functions():
    out = subprocess.run(["robo", "info", "--json"], capture_output=True, text=True)
    return [n for n in json.loads(out.stdout)["result"]["skills"] if n != "capx_continue"]


__all__ = []
for _name in _functions():
    globals()[_name] = (lambda n: lambda *a, **k: call(n, *a, **k))(_name)
    __all__.append(_name)
```

- You may run code as many times as you like and look at what each call returns and prints; the robot keeps its state between calls. `robo observe` gives the robot's joint positions and gripper opening, and `robo observe --image` saves a picture from the scene camera. The API is the only way to move the robot (`robo act 0` only holds it still for one step).
- Poses are in the robot's base frame, in metres, with quaternions as w, x, y, z, as the API documentation says.
- Python 3 with numpy is installed; install other packages with pip if your code needs them.
- The episode has a budget of **5,000 control steps** (CaP-X's horizon for this task); every step an API call takes counts. When it is used up the episode ends.
- Call `robo done` exactly once when finished; `robo give-up "reason"` ends without claiming success.

This task is one fixed scene layout (seed 3); CaP-X samples a new layout for every trial.
