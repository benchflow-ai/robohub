"""Reference solutions for the RoboCasa tasks. They drive the robot only through the robo socket, reading the task's
public observation fields (object, receptacle, handle, lever and knob positions)."""
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


def _v(x) -> np.ndarray:
    return np.asarray(x, dtype=float)


def _rot(axis, ang: float) -> np.ndarray:
    axis = _v(axis) / np.linalg.norm(axis)
    K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + np.sin(ang) * K + (1 - np.cos(ang)) * K @ K


def _lift_away(dz: float = .1, g: float = 1.0) -> None:
    goto(_v(obs()["hand_pos"]) + [0, 0, dz], g)


# ---- drawers ------------------------------------------------------------------------------------------------------

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
    goto(h + [0, 0, .08], 1.0)          # lift clear of the handle first, so the fingers do not pull the drawer back out
    goto(h + n * .12 + [0, 0, .10], 1.0)


# ---- pick and place -----------------------------------------------------------------------------------------------

def _pick() -> float:
    """Grasp the task object from above; returns its half height."""
    st = obs()
    o = np.asarray(st["object_pos"])
    hz = float(st["object_size"][2]) / 2
    goto(o + [0, 0, hz + .10], -1.0)
    goto(o + [0, 0, max(0.0, hz - .03)], -1.0, tol=.004, speed=.3)
    hold(15, 1.0)
    return hz


def pick_place() -> None:
    """Counter to sink."""
    _pick()
    h = np.asarray(obs()["hand_pos"])
    goto(h + [0, 0, .15], 1.0)
    st = obs()
    c = np.asarray(st["sink_basin_center"])
    drop = float(st["sink_basin_bottom_z"]) + .12
    h = np.asarray(st["hand_pos"])
    goto([c[0], c[1], h[2]], 1.0)
    goto([c[0], c[1], drop], 1.0)
    hold(15, -1.0)
    h = np.asarray(obs()["hand_pos"])
    goto(h + [0, 0, .2], -1.0)


def pick_place_cabinet() -> None:
    """Counter to cabinet: grasp, back off toward the robot, lift above the shelf, slide in through the open front,
    set the object down, and withdraw the same way (RoboCasa also wants the gripper 25 cm away from the object)."""
    hz = _pick()
    st = obs()
    c, half, n = _v(st["cabinet_interior_center"]), _v(st["cabinet_interior_half_size"]), _v(st["cabinet_front_normal"])
    bot = float(st["cabinet_interior_bottom_z"])
    h, o = _v(st["hand_pos"]), _v(st["object_pos"])
    grip_off = h[2] - o[2]                        # hand height above the object centre while holding it
    carry_z = bot + hz + grip_off + .05           # object bottom 5 cm above the shelf
    out = c + n * (half[1] + .15)                 # 15 cm in front of the cabinet opening
    goto([h[0], h[1], h[2] + .05], 1.0)
    goto([out[0], out[1], max(h[2] + .05, carry_z)], 1.0)
    goto([out[0], out[1], carry_z], 1.0)
    inside = c + n * (half[1] * .2)
    goto([inside[0], inside[1], carry_z], 1.0, speed=.4)
    goto([inside[0], inside[1], carry_z - .035], 1.0, speed=.3)
    hold(15, -1.0)
    h = _v(obs()["hand_pos"])
    goto([out[0], out[1], h[2]], -1.0, speed=.4)
    far = out + n * .1
    goto([far[0], far[1], h[2] - .25], -1.0)


# ---- levers, lids, knobs -------------------------------------------------------------------------------------------

def turn_on_toaster() -> None:
    """Press the toaster's lever down with the closed fingertips until the toaster reports it is on."""
    st = obs()
    p, size = _v(st["toaster_lever_pos"]), _v(st["toaster_lever_size"])
    top = p[2] + size[2] / 2
    goto([p[0], p[1], top + .10], 1.0)
    goto([p[0], p[1], top + .015], 1.0, speed=.3)
    for _ in range(60):
        if act([0, 0, -.25], 1.0)["toaster_on"]:
            break
    hold(3, 1.0)
    _lift_away(.12)


def close_kettle_lid() -> None:
    """Sweep a closed fingertip along the arc the lid's edge travels about its hinge, from behind the open lid to
    closed, then press it shut."""
    st = obs()
    h0, ax, c = _v(st["kettle_lid_hinge_pos"]), _v(st["kettle_lid_hinge_axis"]), _v(st["kettle_lid_pos"])
    v = c - h0
    toward = _v(st["kettle_pos"]) - h0            # the lid closes over the kettle body
    toward[2] = 0
    toward /= np.linalg.norm(toward)
    u, rho = v / np.linalg.norm(v), 1.4 * np.linalg.norm(v)
    sgn = 1 if (_rot(ax, .3) @ u) @ toward > (_rot(ax, -.3) @ u) @ toward else -1
    phi = float(st["kettle_lid_open"]) * np.pi / 2

    def pt(a: float, back: float) -> np.ndarray:
        d = _rot(ax, sgn * a) @ u
        tan = _rot(ax, sgn * (a + .05)) @ u - d
        return h0 + d * rho - tan / np.linalg.norm(tan) * back

    p = pt(0, .035)
    goto(p + [0, 0, .06], 1.0)
    goto(p, 1.0, speed=.3)
    for k in range(1, 31):
        goto(pt(phi * k / 30, .02), 1.0, speed=.25, n=10)
        if obs()["kettle_lid_open"] < .01:
            break
    for _ in range(20):
        if act([0, 0, -.2], 1.0)["kettle_lid_open"] < .005:
            break
    _lift_away(.08)


def turn_off_faucet() -> None:
    """Push the faucet handle's tip about its hinge in the closing direction with a closed fingertip."""
    for _ in range(3):
        st = obs()
        if not st["water_on"] and st["faucet_handle_angle"] < .3:
            break
        h0, ax = _v(st["faucet_handle_hinge_pos"]), _v(st["faucet_handle_hinge_axis"])
        c = _v(st["faucet_handle_pos"])
        v = c - h0
        tip = h0 + v * 1.25 if np.linalg.norm(v) > .02 else c
        t = -np.cross(ax, tip - h0)
        t /= np.linalg.norm(t)
        start = tip - t * .04
        goto(start + [0, 0, .08], 1.0)
        goto(start, 1.0, speed=.3)
        goto(tip + t * .04, 1.0, speed=.2, n=60)
        h = _v(obs()["hand_pos"])
        goto(h - t * .03 + [0, 0, .08], 1.0)


def _turn_knob(sign: int) -> None:
    """Push the end of the knob's ridge sideways with a closed fingertip (short pushes, re-aimed each time)."""
    for _ in range(5):
        st = obs()
        ang = float(st["stove_knob_angle"])
        if (sign > 0 and ang > .6) or (sign < 0 and ang < .2):
            break
        c, ax, u = _v(st["stove_knob_pos"]), _v(st["stove_knob_axis"]), _v(st["stove_knob_ridge_dir"])
        r = float(st["stove_knob_ridge_half_length"]) * .7
        base = _v(st["robot_base_pos"])
        e = min([c + u * r, c - u * r], key=lambda q: np.linalg.norm((q - base)[:2]))
        t = np.cross(ax, e - c) * sign
        t /= np.linalg.norm(t)
        z = float(st["stove_knob_top_z"]) + .002
        a = e - t * .03
        goto([a[0], a[1], z + .05], 1.0)
        goto([a[0], a[1], z], 1.0, speed=.3)
        b = e + t * .012
        goto([b[0], b[1], z], 1.0, speed=.15, n=50)
        _lift_away(.05)
    _lift_away(.06)


def turn_on_stove() -> None:
    _turn_knob(+1)


def turn_off_stove() -> None:
    _turn_knob(-1)


SOLVERS = {"CloseDrawer": close_drawer, "PickPlaceCounterToSink": pick_place, "PickPlaceCounterToCabinet": pick_place_cabinet,
           "TurnOnToaster": turn_on_toaster, "CloseElectricKettleLid": close_kettle_lid, "TurnOffSinkFaucet": turn_off_faucet,
           "TurnOnStove": turn_on_stove, "TurnOffStove": turn_off_stove}


def run(env: str) -> None:
    from .robocasa import TASKS

    try:
        SOLVERS[TASKS[env]["env"]]()
        hold(5, 0.0)
        _send({"op": "done", "text": "oracle finished"})
    except Finished:
        return
