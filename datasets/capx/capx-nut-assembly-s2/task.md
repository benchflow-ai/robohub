---
schema_version: '1.3'
task:
  name: robouse/capx-nut-assembly-s2
  description: Grasp the square nut by its handle and put it onto the square peg. Code as policy with CaP-X's privileged API.
metadata:
  author_name: benchflow
  source_benchmark: CaP-X (capgym/cap-x, commit 53e9966)
  source_task: nut_assembly (capx/envs/tasks/franka/franka_nut_assembly.py)
  suite: capx
  category: code-as-policy
  difficulty: medium
  tags:
  - mujoco
  - robosuite
  - panda
  - code-as-policy
  - assembly
  - single-arm
  - precision
  capx:
    tier: 'privileged API (the API of CaP-Bench''s S1 tier), multi-turn: not one of CaP-Bench''s eight tiers'
    api:
    - close_gripper
    - get_object_pose
    - goto_home_joint_position
    - goto_pose
    - open_gripper
    - sample_grasp_pose
    layout_seed: 2
  robouse:
    id: capx-nut-assembly-s2
    backend: capx
    env: nut_assembly
    seed: 2
    max_steps: 1000
    camera: birdview
    cameras:
    - birdview
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

# Nut on the peg (CaP-X, privileged API)

This is a code-as-policy task from CaP-X: you control the robot by writing Python code that calls the task's API. The task text and the API documentation below are CaP-X's own.

## Task (CaP-X's prompt)

```text
You are controlling a Franka Emika robot with API described below.
Goal: grasp and insert the `brown square nut` onto the `brown square block`.
Note that you would grasp the nut by its handle. You can try language query `extruded handle of the brown square nut` to get a good grasp at the handle.
The brown square nut and the extruded handle of the brown square nut are part of the same rigid body.
The grasp pose query for 'extruded handle of the brown square nut' returns an end-effector pose expressed in world frame, located on the handle region.
The nut's object center pose obtained via 'white hollow center of the brown square nut' and the handle grasp pose obtained via 'extruded handle of the brown square nut' have a fixed rigid transform, which must be applied correctly when inserting the nut onto the peg.
If you want to use numpy, or scipy for spatial transformations, you need to import it explicitly.
```

**Success:** CaP-X's own check for this task: the square nut is on the square peg and the gripper has moved away from it. It is judged after you call `robo done` and the robot has held still for about 10 control steps.

## API (CaP-X's documentation)

```text
get_object_pose(object_name: str) -> tuple[numpy.ndarray, numpy.ndarray]
  Doc:
    Get the pose of an object in the environment from a natural language description.
    
    Args:
        object_name: The name of the object to get the pose of.
    
    Returns:
        position: (3,) XYZ in meters.
        quaternion_wxyz: (4,) WXYZ unit quaternion.

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

goto_home_joint_position() -> None
  Doc:
    Return the arm to its reset joint configuration with high manipulability

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
- The episode has a budget of **1,000 control steps** (CaP-X's horizon for this task); every step an API call takes counts. When it is used up the episode ends.
- Call `robo done` exactly once when finished; `robo give-up "reason"` ends without claiming success.

This task is one fixed scene layout (seed 2); CaP-X samples a new layout for every trial.
