---
schema_version: '1.3'
task:
  name: robouse/capx-libero-object-swap-9
  description: Pick the orange juice and place it in the basket. LIBERO-PRO, code as policy with CaP-X's privileged API.
metadata:
  author_name: benchflow
  source_benchmark: CaP-X (capgym/cap-x, commit 53e9966) on LIBERO-PRO (uynitsuj/LIBERO-PRO, commit 5368540)
  source_task: libero_object_swap task 9 (pick_up_the_orange_juice_and_place_it_in_the_basket)
  suite: capx-libero
  category: code-as-policy
  difficulty: medium
  tags:
  - mujoco
  - libero
  - libero-pro
  - panda
  - code-as-policy
  - pick-place
  capx:
    tier: 'privileged API (the API of CaP-Bench''s S1 tier), multi-turn: not one of CaP-Bench''s eight tiers'
    api:
    - close_gripper
    - get_all_object_poses
    - get_object_pose
    - get_observation
    - goto_pose
    - open_gripper
    - sample_grasp_pose
    initial_state: 0
  robouse:
    id: capx-libero-object-swap-9
    backend: capx_libero
    env: libero:libero_object_swap:9
    seed: 1
    max_steps: 4000
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

# Pick the orange juice and place it in the basket (CaP-X LIBERO-PRO, privileged API)

This is a code-as-policy task from CaP-X on LIBERO-PRO (libero_object_swap, task 9): you control the robot by writing Python code that calls the task's API. The task text and the API documentation below are CaP-X's own.

## Task (CaP-X's prompt)

```text
You are controlling a Franka Emika robot with API described below.
Goal: Pick the orange juice and place it in the basket
After this code executes, you will be able to get a new observation of the environment and write new code to complete the task if needed.
```

**Success:** LIBERO's own check for this task, the goal of its BDDL file: `(In orange_juice_1 basket_1_contain_region)`. It is judged after you call `robo done` and the robot has held still for about 10 control steps.

## API (CaP-X's documentation)

```text
get_observation() -> dict[str, typing.Any]
  Doc:
    Get the observation of the environment.
    Returns:
        observation:
            A dictionary containing the observation of the environment.
            The dictionary contains the following keys:
            - ["agentview"]["images"]["rgb"]: Current color camera image as a numpy array of shape (H, W, 3), dtype uint8.
            - ["agentview"]["images"]["depth"]: Current depth camera image as a numpy array of shape (H, W), dtype float32.
            - ["agentview"]["intrinsics"]: Camera intrinsic matrix as a numpy array of shape (3, 3), dtype float64.
            - ["agentview"]["pose_mat"]: Camera extrinsic matrix as a numpy array of shape (4, 4), dtype float64.
            - ["robot0_eye_in_hand"]["images"]["rgb"]: Current wrist camera image as a numpy array of shape (H, W, 3), dtype uint8.
            - ["robot0_eye_in_hand"]["images"]["depth"]: Current wrist camera depth image as a numpy array of shape (H, W), dtype float32.
            - ["robot0_eye_in_hand"]["intrinsics"]: Wrist camera intrinsic matrix as a numpy array of shape (3, 3), dtype float64.
            - ["robot0_eye_in_hand"]["pose_mat"]: Wrist camera extrinsic matrix as a numpy array of shape (4, 4), dtype float64.
            - ["robot_cartesian_pos"]: Current end-effector (panda_hand) pose in the robot/world frame as a numpy array of shape (8,), dtype float64. The first 3 elements are the robot's end-effector XYZ, the next 4 elements are the quaternion wxyz, and the last element is the gripper position normalized, 0 (closed) to 1 (open).
            - ["robot_joint_pos"]: Current joint positions as a numpy array of shape (7,), dtype float64. The last element is the gripper position normalized, 0 (closed) to 1 (open).

get_object_pose(object_name: str) -> tuple[numpy.ndarray, numpy.ndarray]
  Doc:
    Get the pose of an object in the environment from a natural language description.
    The quaternion from get_object_pose may be unreliable, so disregard it and use the grasp pose quaternion OR (0, 0, 1, 0) wxyz as the gripper down orientation if using this for placement position.
    
    Args:
        object_name: The name of the object to get the pose of, in underscore separated lowercase words.
    
    Returns:
        position: (3,) XYZ in meters.
        quaternion_wxyz: (4,) WXYZ unit quaternion.

get_all_object_poses() -> dict[str, tuple[numpy.ndarray, numpy.ndarray]]
  Doc:
    Get the poses of all objects in the scene (both movable and fixed).
    
    Returns:
        Dictionary mapping object name to a tuple of:
            position: (3,) XYZ in meters.
            quaternion_wxyz: (4,) WXYZ unit quaternion.

sample_grasp_pose(object_name: str) -> None
  Doc:
    Sample a grasp pose for an object in the environment from a natural language description.
    Do use the grasp sample quaternion from sample_grasp_pose.
    
    Args:
        object_name: The name of the object to sample a grasp pose for, in underscore separated lowercase words.
    
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

CaP-X's LIBERO API also has `goto_pose_interactive_cartesian`, which takes a Python function as its target; a function cannot be passed through `robo`, so it is not offered here.

## How you run code here

Each API function is a skill of the robot: `robo skill NAME ARG ...` runs it, with one JSON value per argument (a number, a list for an array, `true`/`false`, a string) or `name=VALUE` for a keyword argument, for example `robo skill goto_pose "[0.5, 0.0, 0.2]" "[0, 1, 0, 0]" z_approach=0.1`. It prints what the function returns and anything it printed. A call that takes more than 40 control steps returns `running: True` after 40 of them; `robo skill capx_continue` goes on with it, 40 steps at a time, until it returns. To write ordinary Python instead, save this module as `capx_api.py` in your working folder and `from capx_api import *`: it defines every API function of the task under CaP-X's name and signature, returns numpy arrays (a tuple when a function returns several values; dictionaries stay dictionaries), and goes on with long calls by itself, and raises `RuntimeError` when a call fails.

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
- Poses are in the robot's base frame, in metres, with quaternions as w, x, y, z. Object names are LIBERO's, lowercase with underscores (`get_all_object_poses()` lists them).
- Python 3 with numpy is installed; install other packages with pip if your code needs them.
- The episode has a budget of **4,000 control steps** (CaP-X's LIBERO horizon); every step an API call takes counts. When it is used up the episode ends.
- Call `robo done` exactly once when finished; `robo give-up "reason"` ends without claiming success.

The scene is LIBERO's first initial state for this task (CaP-X's trial 1).
