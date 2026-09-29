"""Reference solutions for the RoboCasa tasks. They drive the robot only through the robo socket, reading the task's
public observation fields (object, receptacle and handle positions)."""
from __future__ import annotations

import numpy as np

from robouse.agent_cli import _send

STEP_M = .02


class Finished(Exception):
    pass


def obs() -> dict:
    r = _send({"op": "observe"})
    if not r.get("ok"):
        raise Finished(r.get("error"))
    return r["result"]["state"]


def act(d, g: float) -> dict:
    r = _send({"op": "act", "action": [float(x) for x in np.clip(d, -1, 1)] + [float(g)], "repeat": 1})
    if not r.get("ok"):
        raise Finished(r.get("error"))
    if "episode" in r["result"]:
        raise Finished(r["result"]["episode"])
    return r["result"]["state"]


def goto(p, g: float, tol: float = .006, n: int = 150, speed: float = .5) -> dict:
    """Move the commanded target to p at up to `speed` x 2 cm per step, then wait for the hand to arrive."""
    st = obs()
    p = np.asarray(p, dtype=float)
    for _ in range(n):
        d = p - np.asarray(st["hand_target"])
        if np.linalg.norm(d) < 1e-4 and np.linalg.norm(p - np.asarray(st["hand_pos"])) < tol:
            break
        u = d / STEP_M
        m = float(np.max(np.abs(u)))
        st = act(u * (speed / m) if m > speed else u, g)
    return st


def hold(steps: int, g: float) -> dict:
    st = None
    for _ in range(steps):
        st = act([0, 0, 0], g)
    return st


def close_drawer() -> None:
    st = obs()
    hdl = np.asarray(st["drawer_handle_pos"])
    n = np.asarray(st["drawer_front_normal"])
    goto(hdl + n * .06 + [0, 0, .08], 1.0)
    goto(hdl + n * .03, 1.0)
    for _ in range(250):  # push the drawer front along the drawer's axis, following the handle as it moves
        st = obs()
        if st["drawer_open_fraction"] < .003:
            break
        tgt = np.asarray(st["drawer_handle_pos"]) - n * .03
        tgt[2] = hdl[2]
        act(np.clip((tgt - np.asarray(st["hand_target"])) / STEP_M, -.4, .4), 1.0)
    h = np.asarray(obs()["hand_pos"])
    goto(h + n * .12 + [0, 0, .05], 1.0)


def _dbg(msg):
    import os
    if os.environ.get("ORACLE_DEBUG"):
        st = obs()
        print("DBG", msg, {k: st[k] for k in ("hand_pos", "hand_target", "object_pos", "gripper_open") if k in st}, flush=True)


def pick_place() -> None:
    st = obs()
    o = np.asarray(st["object_pos"])
    hz = float(st["object_size"][2]) / 2
    goto(o + [0, 0, hz + .10], -1.0)
    _dbg("above")
    goto(o + [0, 0, max(0.0, hz - .03)], -1.0, tol=.004, speed=.3)
    _dbg("at grasp")
    hold(15, 1.0)
    h = np.asarray(obs()["hand_pos"])
    goto(h + [0, 0, .15], 1.0)
    _dbg("lifted")
    st = obs()
    c = np.asarray(st["sink_basin_center"])
    drop = float(st["sink_basin_bottom_z"]) + .12
    h = np.asarray(st["hand_pos"])
    goto([c[0], c[1], h[2]], 1.0)
    goto([c[0], c[1], drop], 1.0)
    _dbg("over target")
    hold(15, -1.0)
    h = np.asarray(obs()["hand_pos"])
    goto(h + [0, 0, .2], -1.0)


def run(env: str) -> None:
    from .robocasa import TASKS

    kind = TASKS[env]["env"]
    try:
        {"CloseDrawer": close_drawer, "PickPlaceCounterToSink": pick_place}[kind]()
        hold(5, 0.0)
        _send({"op": "done", "text": "oracle finished"})
    except Finished:
        return
