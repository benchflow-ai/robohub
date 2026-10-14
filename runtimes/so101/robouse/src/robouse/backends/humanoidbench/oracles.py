"""Reference solutions for the humanoidbench suite. They drive the robot only through the episode socket (robo skills and
actions, public observation), like an agent. Each is a function of an `Oracle` client (backends/embodied.py)."""

from __future__ import annotations

import math

from ..embodied import Oracle


def stand(o: Oracle) -> None:
    # HumanoidBench stand: stay upright and still; the return passes the success bar after about 820 steps
    while True:
        o.skill("stand", 100)


def walk(o: Oracle) -> None:
    # walk forward (+x) at full speed, holding the heading at 0 so the forward speed counts along +x
    while True:
        st = o.state()["robot"]
        e = -math.radians(st["heading_deg"])
        o.act([1.0, 0.0, max(-1.0, min(1.0, 2.0 * e))] + [0.0] * 6, 5)


def maze(o: Oracle) -> None:
    sc = o.state()["scene"]
    for x, y in sc["checkpoints"]:
        o.skill("walk_to", x, y, 999, 0.2)
    while True:
        o.skill("stand", 100)


def _follow(o: Oracle, path, speed=0.9, heading=None, lookahead=0.6, until=None, max_steps=2000, chunk=2):
    """Pure pursuit along a polyline of floor points with `robo act` (velocity commands). heading: hold this world heading
    (radians) and move sideways as needed; None: face the direction of travel."""
    import numpy as np

    P = np.asarray(path, float)
    seg = 0
    for _ in range(max_steps // chunk):
        st = o.state()
        r = st["robot"]
        if until is not None and until(st):
            return st
        p = np.asarray(r["pelvis"][:2], float)
        yaw = math.radians(r["heading_deg"])
        # advance along the path: nearest point on the current or later segments
        while seg < len(P) - 2 and np.linalg.norm(p - P[seg + 1]) < lookahead:
            seg += 1
        a, b = P[seg], P[seg + 1]
        ab = b - a
        t = float(np.clip(np.dot(p - a, ab) / max(np.dot(ab, ab), 1e-9), 0, 1))
        near = a + t * ab
        # look-ahead point
        remain = lookahead
        q, k = near, seg
        while True:
            nxt = P[k + 1]
            d = float(np.linalg.norm(nxt - q))
            if d >= remain or k >= len(P) - 2:
                q = q + (nxt - q) * min(1.0, remain / max(d, 1e-9))
                break
            remain -= d
            q, k = nxt, k + 1
        if seg >= len(P) - 2 and np.linalg.norm(p - P[-1]) < 0.15:
            return st
        v = q - p
        v = v / max(np.linalg.norm(v), 1e-9) * speed
        c, s = math.cos(yaw), math.sin(yaw)
        vb = np.array([c * v[0] + s * v[1], -s * v[0] + c * v[1]])
        want = heading if heading is not None else math.atan2(v[1], v[0])
        eh = (want - yaw + math.pi) % (2 * math.pi) - math.pi
        fwd, left = float(vb[0]), float(vb[1])
        if abs(left) > 0.5:  # keep the direction when the sideways speed saturates
            fwd, left = fwd * 0.5 / abs(left), math.copysign(0.5, left)
        o.act([max(-1.0, min(1.0, fwd)), left, max(-1.0, min(1.0, 2.0 * eh))] + [0.0] * 6, chunk)
    return o.state()


def pole(o: Oracle) -> None:
    # diagonal lanes through the pole forest: poles stand on a 1 m x 1.5 m staggered grid, so the lines y = +-0.75 (x - x0)
    # pass through the middle of a gap in every row
    path = [(0.0, 0.0), (1.0, 0.0), (4.0, 2.25), (10.0, -2.25), (16.0, 2.25), (22.0, -2.25)]
    _hands_to_body(o, (0.1, 0.3, 0.95), (0.1, -0.3, 0.95))  # hands in at the hips, clear of the poles
    _follow(o, path, speed=1.0)
    while True:
        o.skill("stand", 100)


def _hands_to_body(o: Oracle, left, right, steps=25) -> None:
    """Move both hand goals to points in the heading frame with `robo act` hand deltas (2 cm per step)."""
    import numpy as np

    for _ in range(steps):
        h = o.state()["robot"]["hands"]
        dl = (np.asarray(left) - np.asarray(h["left"]["goal_body"])) / 0.02
        dr = (np.asarray(right) - np.asarray(h["right"]["goal_body"])) / 0.02
        if max(np.abs(dl).max(), np.abs(dr).max()) < 0.05:
            return
        o.act([0.0, 0.0, 0.0] + list(np.clip(dl, -1, 1)) + list(np.clip(dr, -1, 1)), 1)


def _body(r, p):
    """World point -> heading frame of the robot state r."""
    yaw = math.radians(r["heading_deg"])
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = p[0] - r["pelvis"][0], p[1] - r["pelvis"][1]
    return [c * dx + s * dy, -s * dx + c * dy, p[2]]


def _hand_track(o: Oracle, targets, until, max_steps=300, fwd=None, speed=1.0):
    """Each step: move hand goals toward world points targets[side](state) (re-read every step) at up to 2 cm per step
    (x speed); optional fwd(state) -> (FORWARD, LEFT, TURN). Stops when until(state) or after max_steps."""
    import numpy as np

    st = o.state()
    for _ in range(max_steps):
        if until(st):
            return st
        r = st["robot"]
        a = [0.0] * 9
        if fwd is not None:
            a[0:3] = list(fwd(st))
        for side, fn in targets.items():
            k = 3 if side == "left" else 6
            g = np.asarray(_body(r, fn(st)), float) - np.asarray(r["hands"][side]["goal_body"], float)
            a[k : k + 3] = list(np.clip(g / 0.02, -speed, speed))
        o.act(a, 1)
        st = o.state()
    return st


def reach(o: Oracle) -> None:
    import numpy as np

    g = np.asarray(o.state()["scene"]["target"], float)
    # stand facing the target with it 0.35 m ahead of the pelvis and 0.2 m to the left (in front of the left shoulder)
    h = math.atan2(g[1], g[0]) if np.linalg.norm(g[:2]) > 0.5 else 0.0
    c, s = math.cos(h), math.sin(h)
    spot = g[:2] - np.array([c * 0.35 - s * 0.2, s * 0.35 + c * 0.2])
    o.skill("walk_to", round(float(spot[0]), 3), round(float(spot[1]), 3), round(math.degrees(h), 1), 0.05)
    o.skill("reach", "left", *[round(float(v), 3) for v in g], 0.02)  # the hand then keeps holding the target
    while True:
        o.skill("stand", 100)


def push(o: Oracle) -> None:
    """Clamp the box between the two hands and slide it to the goal: both hands go beside the box, then their goals move
    4 cm inside the box's faces (a squeeze) and, together, toward the goal (re-planned every step from the box's position)."""
    import numpy as np

    sc = o.state()["scene"]
    box0, goal = np.asarray(sc["box"]["pos"], float), np.asarray(sc["goal"], float)
    mid = (box0[:2] + goal[:2]) / 2
    o.skill("walk_to", 0.2, round(float(mid[1]), 3), 0, 0.04)  # 0.2 m from the table edge, centred on the push
    z = 1.03  # a little above the box's centre (box: 0.2 m cube resting on the 0.95 m table top)

    def beside(sgn, off):
        return lambda st: [st["scene"]["box"]["pos"][0], st["scene"]["box"]["pos"][1] + sgn * off, z]

    def near(st):
        return all(
            np.linalg.norm(np.asarray(st["robot"]["hands"][sd]["pos"]) - np.asarray(f(st))) < 0.04
            for sd, f in (("left", beside(1, 0.19)), ("right", beside(-1, 0.19)))
        )

    _hand_track(o, {"left": beside(1, 0.19), "right": beside(-1, 0.19)}, near, 100)

    def carry(sgn):
        def f(st):
            b = np.asarray(st["scene"]["box"]["pos"], float)
            d = goal[:2] - b[:2]
            n = float(np.linalg.norm(d))
            c = b[:2] + d * min(1.0, 0.05 / max(n, 1e-6))
            return [c[0], c[1] + sgn * (0.1 + 0.02 - 0.04), z]

        return f

    _hand_track(o, {"left": carry(1), "right": carry(-1)}, lambda st: st["progress"]["solved"], 400, speed=0.5)


def sit_simple(o: Oracle) -> None:
    # the sit-down motion moves the pelvis about 0.2 m back, so start it 0.25 m in front of the seat's centre (the start spot)
    ch = o.state()["scene"]["chair"]["pos"]
    o.skill("walk_to", round(ch[0] + 0.25, 3), round(ch[1], 3), 0, 0.05)
    o.skill("sit_down")
    while True:
        o.skill("stand", 100)


ORACLES = {
    "humanoidbench-stand": stand,
    "humanoidbench-walk": walk,
    "humanoidbench-maze": maze,
    "humanoidbench-pole": pole,
    "humanoidbench-sit-simple": sit_simple,
    "humanoidbench-reach": reach,
    "humanoidbench-push": push,
}
