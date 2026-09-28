"""Reference solutions for the menagerie tasks. They drive the robot only through the robo socket.

The waypoint structure (approach, descend, close, lift, carry, seat with the measured grasp offset, staged
release, retreat) follows robo-use `oracle.py` and the robotics-tasks-20260917 reference controllers
(`refkit.py` + per-task policies), re-expressed on the backend's Cartesian action: each step moves the
commanded gripper target toward the next waypoint at a bounded speed, then waits for the measured gripper
point to arrive.
"""
from __future__ import annotations

import numpy as np

from robouse.agent_cli import _send

DT = .05
STEP_M = .02


class Finished(Exception):
    pass


class Bot:
    def __init__(self):
        r = _send({"op": "info"})
        if not r.get("ok"):
            raise Finished(r.get("error"))
        names = r["result"]["action"]["names"]
        self.names = names
        self.mode = "both" if names[0].startswith("L_") else "switch" if names[0] == "ARM" else "single"
        self.yaw = any("DYAW" in n for n in names)
        self.state = self.obs()
        self.single_arm = self.state.get("arm")

    # ---- plumbing -------------------------------------------------------------------------------------------
    def obs(self) -> dict:
        r = _send({"op": "observe"})
        if not r.get("ok"):
            raise Finished(r.get("error"))
        self.state = r["result"]["state"]
        return self.state

    def key(self, side: str, field: str) -> str:
        return field if self.mode == "single" else f"{side}_{field}"

    def hand(self, side: str) -> np.ndarray:
        return np.asarray(self.state[self.key(side, "hand_pos")], dtype=float)

    def target(self, side: str) -> np.ndarray:
        return np.asarray(self.state[self.key(side, "hand_target")], dtype=float)

    def pos(self, name: str) -> np.ndarray:
        return np.asarray(self.state[f"{name}_pos"], dtype=float)

    def touching(self, side: str) -> list:
        return list(self.state.get(self.key(side, "touching"), []))

    def act(self, cmds: dict, repeat: int = 1) -> None:
        """cmds: side -> (d[3] in units, dyaw units, grip)."""
        def per(c):
            d, dyaw, g = c
            v = [float(x) for x in np.clip(d, -1, 1)]
            if self.yaw:
                v.append(float(np.clip(dyaw, -1, 1)))
            return v + [float(g)]

        if self.mode == "both":
            vec = per(cmds.get("left", ([0, 0, 0], 0, 0))) + per(cmds.get("right", ([0, 0, 0], 0, 0)))
        elif self.mode == "switch":
            (side, c), = cmds.items()
            vec = [-1.0 if side == "left" else 1.0] + per(c)
        else:
            (side, c), = cmds.items()
            vec = per(c)
        r = _send({"op": "act", "action": vec, "repeat": repeat})
        if not r.get("ok"):
            raise Finished(r.get("error"))
        res = r["result"]
        self.state = res["state"]
        if "episode" in res:
            raise Finished(res["episode"])

    # ---- motions --------------------------------------------------------------------------------------------
    def goto(self, goals: dict, grip: dict | float = 0.0, *, speed: float = .25, tol: float = .004, yaw: dict | None = None,
             dwell: int = 2, max_steps: int = 250) -> bool:
        """Move each arm's target toward goals[side] at `speed` m/s; stop when every hand is within tol."""
        grips = grip if isinstance(grip, dict) else {s: grip for s in goals}
        yaws = yaw or {}
        ok = 0
        for _ in range(max_steps):
            cmds, done = {}, True
            for side, goal in goals.items():
                goal = np.asarray(goal, dtype=float)
                t = self.target(side)
                d = goal - t
                n = float(np.linalg.norm(d))
                cap = speed * DT
                if n > cap:
                    d = d * (cap / n)
                dyaw = 0.0
                if side in yaws and self.yaw:
                    cur = float(self.state.get(self.key(side, "hand_yaw_deg"), 0.0))
                    dyaw = float(np.clip((yaws[side] - cur) / 10.0, -1, 1))
                    if abs(yaws[side] - cur) > .5:
                        done = False
                cmds[side] = (d / STEP_M, dyaw, grips.get(side, 0.0))
                if n > 1e-4 or float(np.linalg.norm(goal - self.hand(side))) > tol:
                    done = False
            if self.mode != "both" and len(cmds) > 1:
                raise ValueError("only one arm moves per step in this layout")
            ok = ok + 1 if done else 0
            if ok >= dwell:
                return True
            self.act(cmds)
        return False

    def hold(self, side: str, steps: int, grip: float = 0.0) -> None:
        for _ in range(steps):
            self.act({side: ([0, 0, 0], 0, grip)})

    def done(self, text: str = "oracle finished") -> None:
        _send({"op": "done", "text": text})


def pick_place(bot: Bot, side: str, obj: str, dest, rest_z: float, *, grasp_h: float, hover: float = .16,
               approach_open: float = -1.0, release=(-.8,), place_clear: float = .0015, seat_speed: float = .05,
               descend_speed: float = .08, yaw: float | None = None, carry_speed: float = .25) -> None:
    """Single-arm pick and place with measured-offset placement (robo-use `measured_placement_targets`)."""
    yw = {side: yaw} if yaw is not None else None
    p = bot.pos(obj)
    bot.goto({side: [p[0], p[1], hover]}, approach_open, speed=carry_speed, tol=.006, yaw=yw)
    bot.obs()
    p = bot.pos(obj)
    bot.goto({side: [p[0], p[1], p[2] + grasp_h]}, approach_open, speed=descend_speed, tol=.0025, yaw=yw)
    bot.hold(side, 22, grip=1.0)
    h = bot.hand(side)
    bot.goto({side: [h[0], h[1], hover]}, 1.0, speed=.12, tol=.006, yaw=yw)
    dest = np.asarray(dest, dtype=float)
    bot.goto({side: [dest[0], dest[1], hover]}, 1.0, speed=carry_speed, tol=.005, yaw=yw)
    for _ in range(3):  # re-measure the held offset as the tool settles
        bot.obs()
        off = np.clip(bot.pos(obj) - bot.hand(side), -.08, .08)
        goal = np.array([dest[0] - off[0], dest[1] - off[1], rest_z - off[2] + place_clear])
        bot.goto({side: goal}, 1.0, speed=seat_speed, tol=.0015, yaw=yw, dwell=3, max_steps=200)
    for g in release:
        bot.hold(side, 16, grip=g)
    h = bot.hand(side)
    bot.goto({side: [h[0], h[1], hover]}, release[-1], speed=.15, tol=.01, yaw=yw)


def _landing(bot: Bot, env: str):
    s = bot.state
    side = s["arm"]
    name = "pawn" if "pawn_pos" in s else "block"
    grasp = .052 if name == "pawn" else .038
    pick_place(bot, side, name, s["pad_center"], s["pad_center"][2], grasp_h=grasp, release=(-.8,))


def _clutter(bot: Bot, env: str):
    s = bot.state
    side = s["arm"]
    pick_place(bot, side, "target", s["tray_center"], s["tray_center"][2],
               grasp_h=.024 if s["robot"] == "aloha" else .020, approach_open=-.6 if s["robot"] == "aloha" else -.42,
               descend_speed=.07, release=(-.9,), place_clear=.002)


def _insertion(bot: Bot, env: str):
    s = bot.state
    side = s["arm"]
    sock = np.asarray(s["socket_center"], dtype=float)
    floor, top = s["socket_floor_z"], s["socket_top_z"]
    hover = .20
    p = bot.pos("peg")
    bot.goto({side: [p[0], p[1], hover]}, -1.0, speed=.25, tol=.008)
    bot.obs()
    p = bot.pos("peg")
    bot.goto({side: [p[0], p[1], p[2] + .070]}, -1.0, speed=.07, tol=.003)
    bot.hold(side, 24, grip=1.0)
    h = bot.hand(side)
    bot.goto({side: [h[0], h[1], hover]}, 1.0, speed=.10, tol=.008)

    def aligned(z):
        bot.obs()
        off = np.clip(bot.pos("peg") - bot.hand(side), -.1, .1)
        return np.array([sock[0] - off[0], sock[1] - off[1], z - off[2]])

    bot.goto({side: aligned(hover - .04)}, 1.0, speed=.2, tol=.004)
    for _ in range(3):
        bot.goto({side: aligned(top + .008)}, 1.0, speed=.04, tol=.0012, dwell=4)
    bot.goto({side: aligned(floor + .004)}, 1.0, speed=.015, tol=.0015, dwell=4, max_steps=300)
    bot.hold(side, 22, grip=-.7)
    h = bot.hand(side)
    bot.goto({side: [h[0], h[1], hover - .04]}, -.85, speed=.10, tol=.01)


def _two_bowls(bot: Bot, env: str):
    """Both arms run their own pick-and-place programme on the same clock (robotics-tasks D_policy)."""
    s = bot.state
    pieces = [k[:-7] for k in s if k.endswith("_target")]
    plans = {"left": [p for p in pieces if p.startswith("light")], "right": [p for p in pieces if p.startswith("dark")]}
    progs = {side: _piece_program(bot, side, plans[side]) for side in plans}
    while progs:
        cmds = {}
        for side, prog in list(progs.items()):
            try:
                cmds[side] = next(prog)
            except StopIteration:
                del progs[side]
        if cmds:
            bot.act(cmds)


def _piece_program(bot: Bot, side: str, pieces: list):
    """Generator of per-step commands for one arm: approach, narrow, descend, close, lift, carry, seat, release."""
    hover, clear = .17, .0015

    def move_to(goal_fn, grip, speed, tol, dwell=3, max_steps=220):
        ok = 0
        for _ in range(max_steps):
            goal = np.asarray(goal_fn(), dtype=float)
            t = bot.target(side)
            d = goal - t
            n = float(np.linalg.norm(d))
            if n > speed * DT:
                d = d * (speed * DT / n)
            ok = ok + 1 if (n < 1e-4 and np.linalg.norm(goal - bot.hand(side)) <= tol) else 0
            if ok >= dwell:
                return
            yield (d / STEP_M, 0.0, grip)

    def hold(n, grip):
        for _ in range(n):
            yield ([0, 0, 0], 0.0, grip)

    for p in pieces:
        target = np.asarray(bot.state[f"{p}_target"], dtype=float)
        src = bot.pos(p)
        yield from move_to(lambda: [bot.pos(p)[0], bot.pos(p)[1], hover], -1.0, .30, .006)
        yield from hold(5, -.5)
        src = bot.pos(p)
        yield from move_to(lambda: [src[0], src[1], src[2] + .030], -.5, .08, .0025)
        yield from hold(22, 1.0)
        h = bot.hand(side).copy()
        yield from move_to(lambda: [h[0], h[1], hover], 1.0, .12, .006)
        yield from move_to(lambda: [target[0], target[1], hover], 1.0, .30, .005)

        def seat():
            off = np.clip(bot.pos(p) - bot.hand(side), -.05, .05)
            return [target[0] - off[0], target[1] - off[1], target[2] - off[2] + clear]

        yield from move_to(seat, 1.0, .05, .002, dwell=3, max_steps=150)
        yield from hold(14, -.35)
        yield from hold(16, -.55)
        yield from move_to(lambda: [target[0], target[1], hover], -.55, .15, .008)
    yield from move_to(lambda: [bot.hand(side)[0], -.12, .25], -1.0, .3, .02)


def _dbg(bot, msg):
    import os
    if os.environ.get("ORACLE_DEBUG"):
        st = bot.obs()
        print("DBG", msg, st.get("time_s"), {k: v for k, v in st.items() if k.endswith(("_pos", "_tilt_deg", "_yaw_deg")) and not k.startswith(("left", "right"))}, flush=True)


def park(bot: Bot, side: str, grip: float = -1.0) -> None:
    x = -.36 if side == "left" else .36
    bot.goto({side: [x, -.12, .25]}, grip, speed=.3, tol=.02)


def _keyed_insertion(bot: Bot, env: str):
    """Regrasp: pick the peg by its flat shaft faces (which transmit a twist), turn it square to the socket, set it
    down on the board, re-pick it by the handle top so the fingers stay above the socket, then insert slowly."""
    s = bot.state
    side = s["arm"]
    sock = np.asarray(s["socket_center"], dtype=float)
    top = float(s["socket_top_z"])
    hover = .17
    stage = np.array([.10 if side == "right" else -.10, sock[1] - .10, sock[2]])
    psi = s_yaw(float(bot.state["peg_yaw_deg"]))
    p = bot.pos("peg")
    bot.goto({side: [p[0], p[1], hover]}, -1.0, speed=.25, tol=.006, yaw={side: psi})
    bot.obs()
    p = bot.pos("peg")
    bot.goto({side: [p[0], p[1], p[2] + .032]}, -1.0, speed=.06, tol=.002, yaw={side: psi})
    bot.hold(side, 22, grip=1.0)
    h = bot.hand(side)
    bot.goto({side: [h[0], h[1], .12]}, 1.0, speed=.10, tol=.006, yaw={side: psi})
    bot.goto({side: [stage[0], stage[1], .12]}, 1.0, speed=.2, tol=.006, yaw={side: 0.0})
    for _ in range(3):
        bot.obs()
        cur = float(bot.state["hand_yaw_deg"])
        bot.goto({side: bot.target(side)}, 1.0, yaw={side: cur - s_yaw(float(bot.state["peg_yaw_deg"]))}, speed=.05, tol=.01,
                 max_steps=30)
    _dbg(bot, "turned")
    for _ in range(2):
        bot.obs()
        off = np.clip(bot.pos("peg") - bot.hand(side), -.08, .08)
        bot.goto({side: [stage[0] - off[0], stage[1] - off[1], stage[2] - off[2] + .0015]}, 1.0, speed=.04, tol=.0015, dwell=3)
    bot.hold(side, 16, grip=-1.0)
    h = bot.hand(side)
    bot.goto({side: [h[0], h[1], hover]}, -1.0, speed=.12, tol=.006)
    _dbg(bot, "staged")
    bot.obs()
    p = bot.pos("peg")
    bot.goto({side: [p[0], p[1], hover]}, -1.0, speed=.2, tol=.006, yaw={side: 0.0})
    bot.goto({side: [p[0], p[1], p[2] + .078]}, -1.0, speed=.06, tol=.002, yaw={side: 0.0})
    bot.hold(side, 22, grip=1.0)
    h = bot.hand(side)
    bot.goto({side: [h[0], h[1], hover]}, 1.0, speed=.08, tol=.006)
    _dbg(bot, "regrasped")

    def aligned(z):
        bot.obs()
        off = np.clip(bot.pos("peg") - bot.hand(side), -.12, .12)
        return np.array([sock[0] - off[0], sock[1] - off[1], z - off[2]])

    bot.goto({side: aligned(top + .05)}, 1.0, speed=.15, tol=.004)
    _dbg(bot, "above socket")
    for _ in range(4):
        bot.goto({side: aligned(top + .006)}, 1.0, speed=.03, tol=.0008, dwell=4)
    _dbg(bot, "at socket top")
    bot.goto({side: aligned(sock[2] + .001)}, 1.0, speed=.012, tol=.0012, dwell=4, max_steps=300)
    bot.hold(side, 22, grip=-.7)
    h = bot.hand(side)
    bot.goto({side: [h[0], h[1], hover]}, -.85, speed=.10, tol=.01)


def s_yaw(y: float) -> float:
    """Wrap a yaw to (-90, 90] (the keyed shaft looks the same after a half turn)."""
    y = (y + 90.0) % 180.0 - 90.0
    return y


def _pattern(bot: Bot, env: str):
    from .scenes.ru_patterns import consistent_answers

    s = bot.state
    answers = consistent_answers(s["examples"], s["query"])
    answer = answers[0]
    c00 = np.asarray(s["cell_0_0_center"], dtype=float)
    size = float(s["cell_size"])
    for label in ("red_circle", "green_triangle", "blue_square"):
        bot.obs()
        p = bot.pos(label)
        side = "left" if p[0] < 0 else "right"
        x, y = answer[label]
        dest = c00 + np.array([x * size, y * size, 0.0])
        grasp = .058 if (bot.state["robot"] == "aloha" and label == "green_triangle") else .052
        pick_place(bot, side, label, dest, dest[2], grasp_h=grasp, approach_open=-1.0, release=(-.35, -.55),
                   hover=.16)
        park(bot, side, -.55)


def _fragile_kit(bot: Bot, env: str):
    s = bot.obs()
    labels = s["vial_labels"]  # privileged (the reference solution reads the ground truth instead of the image)
    station = np.asarray(s["inspection_station"], dtype=float)
    for v in ("vial_0", "vial_2", "vial_1"):
        bot.obs()
        p = bot.pos(v)
        side = "left" if p[0] < 0 else "right"
        slot = np.asarray(bot.state[f"{labels[v]}_compartment_center"], dtype=float)
        # pick
        bot.goto({side: [p[0], p[1], .16]}, -1.0, speed=.25, tol=.006)
        bot.obs()
        p = bot.pos(v)
        bot.goto({side: [p[0], p[1], p[2] + .045]}, -1.0, speed=.06, tol=.0025)
        bot.hold(side, 22, grip=1.0)
        h = bot.hand(side)
        bot.goto({side: [h[0], h[1], .16]}, 1.0, speed=.10, tol=.006)
        _dbg(bot, f"{v} lifted")
        # show the underside to the inspection camera
        bot.obs()
        off = bot.pos(v) - bot.hand(side)
        bot.goto({side: [station[0] - off[0], station[1] - off[1], .16]}, 1.0, speed=.20, tol=.005)
        bot.goto({side: [station[0] - off[0], station[1] - off[1], .10 - off[2]]}, 1.0, speed=.08, tol=.004, dwell=6)
        bot.goto({side: [station[0] - off[0], station[1] - off[1], .16]}, 1.0, speed=.10, tol=.006)
        # pack
        _dbg(bot, f"{v} inspected")
        bot.goto({side: [slot[0], slot[1], .16]}, 1.0, speed=.20, tol=.005)
        _dbg(bot, f"{v} over slot")
        for _ in range(3):
            bot.obs()
            off = np.clip(bot.pos(v) - bot.hand(side), -.08, .08)
            bot.goto({side: [slot[0] - off[0], slot[1] - off[1], slot[2] - off[2] + .002]}, 1.0, speed=.04, tol=.0015, dwell=3)
        bot.hold(side, 16, grip=-.6)
        h = bot.hand(side)
        bot.goto({side: [h[0], h[1], .16]}, -.6, speed=.12, tol=.01)
        park(bot, side)


def _route_around(bot: Bot, env: str):
    s = bot.state
    side = s["arm"]
    src, dst = np.asarray(s["source_pad_center"]), np.asarray(s["delivery_pad_center"])
    col = np.asarray(s["column_center_xy"])
    hover = .16
    p = bot.pos("payload")
    bot.goto({side: [p[0], p[1], hover]}, -1.0, speed=.25, tol=.006)
    bot.obs()
    p = bot.pos("payload")
    bot.goto({side: [p[0], p[1], p[2] + .052]}, -1.0, speed=.08, tol=.0025)
    bot.hold(side, 22, grip=1.0)
    h = bot.hand(side)
    bot.goto({side: [h[0], h[1], hover]}, 1.0, speed=.12, tol=.006)
    # pass behind the column (between it and the robot), where neither the payload nor the arm can sweep it
    y_pass = col[1] + .20  # the Panda hand is about 20 cm wide along its finger axis
    bot.goto({side: [h[0], y_pass, hover]}, 1.0, speed=.2, tol=.01)
    bot.goto({side: [dst[0], y_pass, hover]}, 1.0, speed=.2, tol=.01)
    bot.goto({side: [dst[0], dst[1], hover]}, 1.0, speed=.2, tol=.005)
    for _ in range(3):
        bot.obs()
        off = np.clip(bot.pos("payload") - bot.hand(side), -.08, .08)
        bot.goto({side: [dst[0] - off[0], dst[1] - off[1], dst[2] - off[2] + .0015]}, 1.0, speed=.05, tol=.0015, dwell=3)
    bot.hold(side, 16, grip=-.8)
    h = bot.hand(side)
    bot.goto({side: [h[0], h[1], hover]}, -.8, speed=.15, tol=.01)
    bot.goto({side: [h[0], y_pass, hover]}, -.8, speed=.2, tol=.01)


def _fragile_disposal(bot: Bot, env: str):
    s = bot.state
    side = s["arm"]
    binc = np.asarray(s["bin_center"])
    sq = np.asarray(s["square_center"])
    # the blank may be dropped into the bin
    p = bot.pos("blank")
    bot.goto({side: [p[0], p[1], .16]}, -1.0, speed=.25, tol=.006)
    bot.obs()
    p = bot.pos("blank")
    bot.goto({side: [p[0], p[1], p[2] + .052]}, -1.0, speed=.08, tol=.0025)
    bot.hold(side, 22, grip=1.0)
    h = bot.hand(side)
    bot.goto({side: [h[0], h[1], .18]}, 1.0, speed=.12, tol=.006)
    bot.goto({side: [binc[0], binc[1], .18]}, 1.0, speed=.2, tol=.006)
    bot.goto({side: [binc[0], binc[1], .13]}, 1.0, speed=.1, tol=.006)
    bot.hold(side, 16, grip=-1.0)
    bot.goto({side: [binc[0], binc[1], .18]}, -1.0, speed=.15, tol=.01)
    # the vial is set down gently on the gold square
    bot.obs()
    pick_place(bot, side, "vial", sq, sq[2], grasp_h=.064, release=(-.6,), seat_speed=.03, hover=.16)


ORACLES = {"landing": _landing, "clutter": _clutter, "insertion": _insertion, "two_bowls": _two_bowls,
           "keyed_insertion": _keyed_insertion, "pattern": _pattern, "fragile_kit": _fragile_kit,
           "route_around": _route_around, "fragile_disposal": _fragile_disposal}


def run(env: str) -> None:
    from ..menagerie import TASKS

    fam = TASKS[env]["family"]
    try:
        bot = Bot()
        ORACLES[fam](bot, env)
        bot.done()
    except Finished:
        return
