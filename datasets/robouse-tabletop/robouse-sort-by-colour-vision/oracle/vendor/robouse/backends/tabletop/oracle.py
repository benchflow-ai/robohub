"""Tabletop reference solutions: scripted pick-and-place, drawer and push plans through the episode socket."""

from __future__ import annotations

import numpy as np

from .scenarios import SCENARIOS
from .scene import DR_TRAVEL, HAND_HI, HAND_LO

# --------------------------------------------------------------------------------------------------


class _Robot:
    def __init__(self):
        from robouse.agent_cli import _send

        self._send = _send
        self.finished = False

    def req(self, r: dict) -> dict:
        if self.finished:
            raise _Finished()
        resp = self._send(r)
        if not resp.get("ok"):
            raise RuntimeError(f"{r.get('op')}: {resp.get('error')}")
        res = resp.get("result") or {}
        if isinstance(res, dict) and "episode" in res:
            self.finished = True
            raise _Finished()
        return res

    def obs(self) -> dict:
        return self.req({"op": "observe"})["state"]

    def move(self, p, g=None, tol=0.004, tries=3) -> dict:
        p = [float(np.clip(p[i], HAND_LO[i], HAND_HI[i])) for i in range(3)]
        res = {}
        for _ in range(tries):
            res = self.req({"op": "move_to", "pos": p, "grip": g, "max_steps": 150, "tol": tol})
            if res.get("reached"):
                break
        return res

    def grip(self, g, steps=12) -> None:
        self.req({"op": "grip", "value": float(g), "steps": steps})

    def act(self, a, repeat=1) -> dict:
        return self.req({"op": "act", "action": [float(x) for x in a], "repeat": int(repeat)})


class _Finished(Exception):
    pass


def target_of(st: dict, tgt: dict, xy_z: bool = False) -> tuple[np.ndarray, float]:
    """(xy target for the object's centre, surface z it should rest on). With `xy_z`, an explicit `xy` target may
    carry its own surface height `z`."""
    fx = st.get("fixtures", {})
    off = np.array(tgt.get("offset", [0.0, 0.0]), float)
    if "pad" in tgt:
        return np.array(fx[tgt["pad"]]["pos"][:2]) + off, fx[tgt["pad"]]["pos"][2]
    if "fixture" in tgt:
        f = fx[tgt["fixture"]]
        if f["kind"] == "plate":
            return np.array(f["pos"][:2]) + off, f["top_z"]
        if f["kind"] == "drawer":
            return np.array(f["interior_center"][:2]) + off, f["interior_center"][2]
        if f["kind"] == "socket":
            return np.array(f["pos"][:2]) + off, 0.0
        return np.array(f["pos"][:2]) + off, f["floor_z"]
    if "on" in tgt:
        b = st["objects"][tgt["on"]]
        return np.array(b["pos"][:2]) + off, b["pos"][2] + b["half_height"]
    if "cell" in tgt:
        r, c = tgt["cell"]
        return np.array(st["grid"]["cell_centers"][r][c][:2]), 0.0
    if "xy" in tgt:
        return np.array(tgt["xy"], float), (tgt.get("z", 0.0) if xy_z else 0.0)
    raise ValueError(tgt)


def _pick_place(rb: _Robot, sc: dict, obj: str, tgt: dict, opt: dict) -> None:
    carry = opt.get("carry_z", 0.15)
    g_open = opt.get("open", -0.3)
    g_rel = opt.get("release", g_open)
    st = rb.obs()
    o = st["objects"][obj]
    p = np.array(o.get("handle_pos", o["pos"]), float)
    grasp_z = max(p[2] + opt.get("grasp_dz", 0.0), 0.012)
    rb.move([p[0], p[1], carry], g_open, tol=0.006)
    rb.move([p[0], p[1], grasp_z + 0.03], g_open, tol=0.004)
    rb.move([p[0], p[1], grasp_z], g_open, tol=0.003)
    rb.grip(1.0, 12)
    rb.move([p[0], p[1], carry], 1.0, tol=0.006)
    st = rb.obs()
    o = st["objects"][obj]
    hand = np.array(st["hand_pos"])
    offset = np.array(o["pos"]) - hand
    xy, surf = target_of(st, tgt)
    want_z = surf + o["half_height"] + opt.get("drop", 0.004)
    hx, hy = xy - offset[:2]
    hz = want_z - offset[2]
    rb.move([hx, hy, carry], 1.0, tol=0.003)
    if opt.get("slow"):
        # lower gently: at most 0.2 cm per step, stop when the object touches down
        for _ in range(200):
            st = rb.obs()
            oz = st["objects"][obj]["pos"][2]
            if oz - (surf + o["half_height"]) < 0.0015:
                break
            dz = max(-0.2, min(-0.05, -(oz - surf - o["half_height"]) * 20))
            rb.act([0, 0, dz, 1.0], repeat=2)
    else:
        rb.move([hx, hy, hz + 0.03], 1.0, tol=0.004)
        rb.move([hx, hy, hz], 1.0, tol=0.003)
    rb.grip(g_rel, 10)
    h = rb.obs()["hand_pos"]
    rb.move([h[0], h[1], carry], g_rel, tol=0.01)


def _open_drawer(rb: _Robot, name: str, amount: float = DR_TRAVEL) -> None:
    f = rb.obs()["fixtures"][name]
    hx, hy, hz = f["handle_pos"]
    closed_y = hy + f["opening"]
    rb.move([hx, hy, 0.13], 0.2, tol=0.006)
    rb.move([hx, hy, hz], 0.2, tol=0.003)
    rb.grip(1.0, 12)
    rb.move([hx, closed_y - amount - 0.004, hz], 1.0, tol=0.003)
    rb.grip(0.2, 10)
    rb.move([hx, closed_y - amount - 0.01, 0.13], 0.2, tol=0.01)


def _close_drawer(rb: _Robot, name: str) -> None:
    f = rb.obs()["fixtures"][name]
    hx, hy, hz = f["handle_pos"]
    closed_y = hy + f["opening"]
    rb.move([hx, hy - 0.04, 0.13], 1.0, tol=0.006)
    rb.move([hx, hy - 0.04, hz], 1.0, tol=0.004)
    rb.move([hx, closed_y - 0.017 + 0.004, hz], 1.0, tol=0.003)
    rb.move([hx, closed_y - 0.06, hz], 1.0, tol=0.01)
    rb.move([hx, closed_y - 0.06, 0.13], -0.3, tol=0.01)


def _push(rb: _Robot, obj: str, target, opt: dict) -> None:
    for _ in range(opt.get("passes", 2)):
        st = rb.obs()
        o = st["objects"][obj]
        p = np.array(o["pos"][:2])
        t = np.array(target, float)
        if np.linalg.norm(t - p) < 0.012:
            break
        u = (t - p) / np.linalg.norm(t - p)
        half = max(o["size"][0], o["size"][1]) / 2
        start = p - u * (half + 0.012 + 0.025)
        end = t - u * (half + 0.012)
        z = opt.get("z", 0.02)
        rb.move([start[0], start[1], 0.1], 1.0, tol=0.006)
        rb.move([start[0], start[1], z], 1.0, tol=0.004)
        # push in short segments so the block stays in front of the fingers
        n = max(1, int(np.linalg.norm(end - start) / 0.03))
        for i in range(1, n + 1):
            q = start + (end - start) * i / n
            rb.move([q[0], q[1], z], 1.0, tol=0.004, tries=1)
        h = rb.obs()["hand_pos"]
        rb.move([h[0] - u[0] * 0.03, h[1] - u[1] * 0.03, z], 1.0, tol=0.01, tries=1)
        rb.move([h[0] - u[0] * 0.03, h[1] - u[1] * 0.03, 0.12], 1.0, tol=0.01)


def oracle_main(env: str) -> None:
    sc = SCENARIOS[env]
    rb = _Robot()
    try:
        for step in sc.get("oracle", []):
            op = step[0]
            if op == "pp":
                _pick_place(rb, sc, step[1], step[2], step[3] if len(step) > 3 else {})
            elif op == "open_drawer":
                _open_drawer(rb, step[1])
            elif op == "close_drawer":
                _close_drawer(rb, step[1])
            elif op == "push":
                _push(rb, step[1], step[2], step[3] if len(step) > 3 else {})
            elif op == "give_up":
                rb.req({"op": "give_up", "text": step[1]})
                return
            else:
                raise ValueError(op)
        h = rb.obs()["hand_pos"]
        rb.move([h[0], h[1], max(h[2], 0.15)], None, tol=0.01, tries=1)
        rb.req({"op": "done", "text": "oracle finished"})
    except _Finished:
        return


# --------------------------------------------------------------------------------------------------
# Scenarios
