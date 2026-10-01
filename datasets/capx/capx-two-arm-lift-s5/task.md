---
schema_version: '1.3'
task:
  name: robouse/capx-two-arm-lift-s5
  description: Two Franka arms on opposite sides of the table grasp a pot's two handles and lift it together. Code as policy with CaP-X's privileged API.
metadata:
  author_name: benchflow
  source_benchmark: CaP-X (capgym/cap-x, commit 53e9966)
  source_task: two_arm_lift (capx/envs/tasks/franka/two_arm_lift.py)
  suite: capx
  category: code-as-policy
  difficulty: medium
  tags:
  - mujoco
  - robosuite
  - panda
  - code-as-policy
  - bimanual
  - lift
  capx:
    tier: 'privileged API (the API of CaP-Bench''s S1 tier), multi-turn: not one of CaP-Bench''s eight tiers'
    api:
    - close_gripper_arm0
    - close_gripper_arm1
    - get_arm0_gripper_pose
    - get_arm1_gripper_pose
    - get_handle0_pos
    - get_handle1_pos
    - goto_pose_arm0
    - goto_pose_arm1
    - goto_pose_both
    - open_gripper_arm0
    - open_gripper_arm1
    layout_seed: 5
  robouse:
    id: capx-two-arm-lift-s5
    backend: capx
    env: two_arm_lift
    seed: 5
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

# Lift the pot with two arms (CaP-X, privileged API)

This is a code-as-policy task from CaP-X: you control the robot by writing Python code that calls the task's API. The task text and the API documentation below are CaP-X's own.

## Task (CaP-X's prompt)

```text
You are controlling a two-arm Franka Emika robot system on opposite sides of the table with API described below.
Goal: The two arms should coordinate to lift a pot. Arm 0 should grasp handle 0, and Arm 1 should grasp handle 1. Then both arms should lift the pot up at the same height.

Guidance:
- The pot is in the center of the table and has two handles on opposite sides.
- You may want to slightly lift both arms first to avoid occluding the pot and handles.
- Use `get_handle0_pos()` to get the bounding box center position of handle 0 (returns a single 3D array).
- Use `get_handle1_pos()` to get the bounding box center position of handle 1 (returns a single 3D array).
- Avoid using top-down grasps, i.e. avoid using quaternion wxyz [0, 0, 1, 0] for the approach.
- Sideways grasps are preferred with the y-axis of the gripper aligned with the world z-axis.
- Coordinate system: All pose functions accept/return positions in robot0's base frame.
```

**Success:** CaP-X's own check for this task: the pot's bottom is more than 10 cm above the table. It is judged after you call `robo done` and the robot has held still for about 10 control steps.

## API (CaP-X's documentation)

```text
get_handle0_pos() -> numpy.ndarray
  Doc:
    Get the bounding box center position of handle 0 using vision detection.
    
    Args:
        None
    Returns:
        bbox_center: (3,) XYZ position of bounding box center in world coordinates

get_handle1_pos() -> numpy.ndarray
  Doc:
    Get the bounding box center position of handle 1 using vision detection.
    
    Args:
        None
    Returns:
        bbox_center: (3,) XYZ position of bounding box center in world coordinates

get_arm0_gripper_pose() -> tuple[numpy.ndarray, numpy.ndarray]
  Doc:
    Get the pose of the gripper for arm 0.
    
    Args:
        None
    Returns:
        position: (3,) XYZ position in meters.
        quaternion_wxyz: (4,) WXYZ unit quaternion.

get_arm1_gripper_pose() -> tuple[numpy.ndarray, numpy.ndarray]
  Doc:
    Get the pose of the gripper for arm 1.
    
    Args:
        None
    Returns:
        position: (3,) XYZ position in meters.
        quaternion_wxyz: (4,) WXYZ unit quaternion.

goto_pose_arm0(position: numpy.ndarray, quaternion_wxyz: numpy.ndarray, z_approach: float = 0.0) -> None
  Doc:
    Go to pose using Inverse Kinematics for Arm 0 (robot0)
    Args:
        position: (3,) XYZ position in meters for arm 0.
        quaternion_wxyz: (4,) WXYZ unit quaternion for arm 0.
        z_approach: (float) Z-axis distance offset for goto_pose insertion approach motion. Will first arrive at position + z_approach meters in Z-axis before moving to the requested pose. Useful for more precise grasp approaches. Default is 0.0.
    Returns:
        None

open_gripper_arm0() -> None
  Doc:
    Open gripper fully for Arm 0 (robot0).
    Args:
        None
    Returns:
        None

close_gripper_arm0() -> None
  Doc:
    Close gripper fully for Arm 0 (robot0).
    Args:
        None
    Returns:
        None

goto_pose_arm1(position: numpy.ndarray, quaternion_wxyz: numpy.ndarray, z_approach: float = 0.0) -> None
  Doc:
    Go to pose using Inverse Kinematics for Arm 1 (robot1).
    Args:
        position: (3,) XYZ position in meters for arm 1.
        quaternion_wxyz: (4,) WXYZ unit quaternion for arm 1.
        z_approach: (float) Z-axis distance offset for goto_pose insertion approach motion. Will first arrive at position + z_approach meters in Z-axis before moving to the requested pose. Useful for more precise grasp approaches. Default is 0.0.
    Returns:
        None

open_gripper_arm1() -> None
  Doc:
    Open gripper fully for Arm 1 (robot1).
    Args:
        None
    Returns:
        None

close_gripper_arm1() -> None
  Doc:
    Close gripper fully for Arm 1 (robot1).
    Args:
        None
    Returns:
        None

goto_pose_both(position0: numpy.ndarray, quaternion_wxyz0: numpy.ndarray, position1: numpy.ndarray, quaternion_wxyz1: numpy.ndarray, z_approach: float = 0.0) -> None
  Doc:
    Go to pose using Inverse Kinematics for moving both arms simultaneously. Positions and quaternions are in robot0's base frame.
    Args:
        position0: (3,) XYZ position in meters for arm 0.
        quaternion_wxyz0: (4,) WXYZ unit quaternion for arm 0.
        position1: (3,) XYZ position in meters for arm 1.
        quaternion_wxyz1: (4,) WXYZ unit quaternion for arm 1.
        z_approach: (float) Z-axis distance offset for goto_pose insertion approach motion. Will first arrive at position + z_approach meters in Z-axis before moving to the requested pose. Useful for more precise grasp approaches. Default is 0.0.
    Returns:
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

This task is one fixed scene layout (seed 5); CaP-X samples a new layout for every trial.
