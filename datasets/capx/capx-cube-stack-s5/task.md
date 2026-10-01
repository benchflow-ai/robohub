---
schema_version: '1.3'
task:
  name: robouse/capx-cube-stack-s5
  description: Stack the red cube on the green cube and release it. Code as policy with CaP-X's privileged API.
metadata:
  author_name: benchflow
  source_benchmark: CaP-X (capgym/cap-x, commit 53e9966)
  source_task: cube_stack (capx/envs/tasks/franka/franka_pick_place.py)
  suite: capx
  category: code-as-policy
  difficulty: medium
  tags:
  - mujoco
  - robosuite
  - panda
  - code-as-policy
  - stack
  - single-arm
  capx:
    tier: 'privileged API (the API of CaP-Bench''s S1 tier), multi-turn: not one of CaP-Bench''s eight tiers'
    api:
    - close_gripper
    - get_object_pose
    - goto_pose
    - open_gripper
    - sample_grasp_pose
    layout_seed: 5
  robouse:
    id: capx-cube-stack-s5
    backend: capx
    env: cube_stack
    seed: 5
    max_steps: 1500
    camera: robot0_robotview
    cameras:
    - robot0_robotview
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

# Stack the cubes (CaP-X, privileged API)

This is a code-as-policy task from CaP-X: you control the robot by writing Python code that calls the task's API. The task text and the API documentation below are CaP-X's own.

## Task (CaP-X's prompt)

```text
You are controlling a Franka Emika robot with the API described below.
Goal: Pick up the red cube and gently stack it on top of the green cube, then release it.

Key rules:
- The extent from get_object_pose(..., return_bbox_extent=True) is the FULL side length. Use extent[2]/2 for half-height.
- For placement orientation, reuse the grasp quaternion from sample_grasp_pose. Do NOT use the quaternion from get_object_pose (it is unreliable for orientation).
- Always use z_approach=0.1 when approaching an object for grasping or placing.
- After grasping, lift the cube to a safe height (at least +0.2m in Z) before moving laterally to the placement location.
- The stacking height formula is: place_z = green_center_z + green_extent[2]/2 + red_extent[2]/2
- Nothing should be dropped from a height. Always approach with z_approach for controlled descent.
```

**Success:** CaP-X's own check for this task: the red cube rests on the green cube, lifted more than 4 cm above the table, and is no longer grasped. It is judged after you call `robo done` and the robot has held still for about 10 control steps.

## API (CaP-X's documentation)

```text
get_object_pose(object_name: str, return_bbox_extent: bool = False) -> tuple[numpy.ndarray, numpy.ndarray, numpy.ndarray | None]
  Doc:
    Get the pose of an object in the environment from a natural language description.
    The quaternion from get_object_pose may be unreliable, so disregard it and use the grasp pose quaternion OR (0, 0, 1, 0) wxyz as the gripper down orientation if using this for placement position.
    
    Args:
        object_name: The name of the object to get the pose of.
    
    Returns:
        position: (3,) XYZ in meters.
        quaternion_wxyz: (4,) WXYZ unit quaternion.
        bbox_extent: (3,) object extent in meters of x, y, z axes respectively in the world frame (full side length, not half-length extent). If return_bbox_extent is False, returns None.

sample_grasp_pose(object_name: str) -> tuple[numpy.ndarray, numpy.ndarray]
  Doc:
    Sample a grasp pose for an object in the environment from a natural language description.
    Do use the grasp sample quaternion from sample_grasp_pose.
    
    Args:
        object_name: The name of the object to sample a grasp pose for.
    
    Returns:
        position: (3,) XYZ in meters.
        quaternion_wxyz: (4,) WXYZ unit quaternion.

goto_pose(position: numpy.ndarray, quaternion_wxyz: numpy.ndarray, z_approach: float = 0.0) -> None
  Doc:
    Go to pose using Inverse Kinematics.
    There is no need to call a second goto_pose with the same position and quaternion_wxyz after calling it with z_approach.
    Args:
        position: (3,) XYZ in meters.
        quaternion_wxyz: (4,) WXYZ unit quaternion.
        z_approach: (float) Z-axis distance offset for goto_pose insertion approach motion. Will first arrive at position + z_approach meters in Z-axis before moving to the requested pose. Useful for more precise grasp approaches. Default is 0.0.
    Returns:
        None

open_gripper() -> None
  Doc:
    Open gripper fully.
    
    Args:
        None

close_gripper() -> None
  Doc:
    Close gripper fully.
    
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
- The episode has a budget of **1,500 control steps** (CaP-X's horizon for this task); every step an API call takes counts. When it is used up the episode ends.
- Call `robo done` exactly once when finished; `robo give-up "reason"` ends without claiming success.

This task is one fixed scene layout (seed 5); CaP-X samples a new layout for every trial.
