"""Reference solutions for the ManiSkill3 tasks. They drive the robot only through the episode socket (the same `observe`
and `act` requests `robo` sends) and read only the public observation: scripted grasp-and-place controllers in the spirit
of ManiSkill's own motion-planning solutions (which need mplib, not available for linux/arm64), closed-loop on the
observed object poses.

Every solution servos the commanded hand pose (`hand_target_pos`, `hand_target_quat`) toward a goal pose with the
7-number action [DX, DY, DZ, DROLL, DPITCH, DYAW, GRIP] and waits until the measured hand (`hand_pos`, `hand_quat`) is
there. Quaternions are (w, x, y, z), ManiSkill's convention.
"""

from __future__ import annotations

import math

import numpy as np

from ..embodied import EpisodeOver, Oracle

STEP_M, STEP_RAD = 0.02, 0.1


# ---- rotations --------------------------------------------------------------------------------------------------------


def qmat(q) -> np.ndarray:
    w, x, y, z = [float(v) for v in q]
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def rotvec(R) -> np.ndarray:
    c = np.clip((np.trace(R) - 1) / 2, -1, 1)
    a = float(np.arccos(c))
    if a < 1e-9:
        return np.zeros(3)
    if a > np.pi - 1e-4:
        w, V = np.linalg.eigh((R + R.T) / 2)
        return V[:, np.argmax(w)] * a
    return np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / (2 * np.sin(a)) * a


def rot(axis_angle) -> np.ndarray:
    v = np.asarray(axis_angle, dtype=float)
    a = float(np.linalg.norm(v))
    if a < 1e-12:
        return np.eye(3)
    k = v / a
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(a) * K + (1 - np.cos(a)) * K @ K


def down(theta: float) -> np.ndarray:
    """Hand pointing straight down with the finger (closing) axis at heading theta (radians) in the table plane."""
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[-s, c, 0.0], [c, s, 0.0], [0.0, 0.0, -1.0]])


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def heading(v) -> float:
    return math.atan2(float(v[1]), float(v[0]))


def nearest_heading(options, current: float) -> float:
    return min(options, key=lambda t: abs(wrap(t - current)))


def face_heading(yaw_deg: float, current: float) -> float:
    """A finger heading across two opposite faces of a box with heading yaw_deg, the one nearest the current heading."""
    y = math.radians(yaw_deg)
    return nearest_heading([y + k * math.pi / 2 for k in range(-4, 5)], current)


def clear_face_heading(yaw_deg: float, current: float, at, others) -> float:
    """Like face_heading, but prefer the pair of faces whose finger axis points away from nearby objects, so the open
    fingers do not land on a neighbour."""
    y = math.radians(yaw_deg)
    at = np.asarray(at, dtype=float)[:2]
    near = [np.asarray(q, dtype=float)[:2] - at for q in others]
    near = [v for v in near if np.linalg.norm(v) < 0.12]

    def cost(t):
        f = np.array([math.cos(t), math.sin(t)])
        clash = sum(abs(float(f @ v)) / (np.linalg.norm(v) ** 2 + 1e-9) for v in near)
        return clash * 0.1 + abs(wrap(t - current))

    return min([y + k * math.pi / 2 for k in range(-4, 5)], key=cost)


# ---- controller -------------------------------------------------------------------------------------------------------


class Ctl(Oracle):
    def __init__(self):
        super().__init__()
        self.g = 0.0

    def s(self) -> dict:
        return self.state()

    def goto(
        self,
        p,
        R=None,
        grip: float | None = None,
        tol: float = 0.004,
        rtol: float = 0.03,
        max_steps: int = 150,
        speed: float = 1.0,
    ) -> dict:
        """Servo the commanded hand pose to (p, R) and wait until the measured hand is within tol / rtol of it."""
        g = self.g if grip is None else grip
        p = np.asarray(p, dtype=float)
        st = self.s()
        if R is None:
            R = qmat(st["hand_target_quat"])
        for _ in range(max_steps):
            pc, Rc = np.asarray(st["hand_target_pos"]), qmat(st["hand_target_quat"])
            ph, Rh = np.asarray(st["hand_pos"]), qmat(st["hand_quat"])
            if np.linalg.norm(p - ph) < tol and np.linalg.norm(rotvec(R @ Rh.T)) < rtol:
                break
            dp = np.clip((p - pc) / STEP_M, -speed, speed)
            dr = np.clip(rotvec(R @ Rc.T) / STEP_RAD, -1, 1)
            st = self.act(np.r_[dp, dr, g])["state"]
        return st

    def grip(self, g: float, n: int = 12) -> dict:
        self.g = g
        return self.act([0, 0, 0, 0, 0, 0, g], n)["state"]

    def wait(self, n: int) -> dict:
        return self.act([0, 0, 0, 0, 0, 0, self.g], n)["state"]

    def hand_heading(self) -> float:
        return heading(self.s()["finger_axis"])

    def pick(self, pos, theta: float, z_above: float = 0.12, z_grasp: float | None = None, close: float = 1.0) -> dict:
        """Open, come down over pos with finger heading theta, close, lift to z_above."""
        pos = np.asarray(pos, dtype=float)
        R = down(theta)
        self.grip(-1, 6)
        self.goto([pos[0], pos[1], max(z_above, pos[2] + 0.08)], R, tol=0.01)
        self.goto([pos[0], pos[1], pos[2] if z_grasp is None else z_grasp], R, speed=0.5, tol=0.004)
        self.grip(close, 12)
        return self.goto([pos[0], pos[1], z_above], R, tol=0.01)


# ---- task solutions ---------------------------------------------------------------------------------------------------


def pick_cube(o: Ctl) -> None:
    s = o.s()
    o.pick(s["cube_pos"], face_heading(s["cube_yaw_deg"], o.hand_heading()), z_above=0.1)
    s = o.s()
    goal = np.asarray(s["goal_pos"])
    off = np.asarray(s["cube_pos"]) - np.asarray(s["hand_pos"])  # cube centre relative to the hand while held
    o.goto(goal - off, tol=0.006)
    for _ in range(3):  # correct with the observed cube position
        s = o.s()
        err = goal - np.asarray(s["cube_pos"])
        if np.linalg.norm(err) < 0.006:
            break
        o.goto(np.asarray(s["hand_pos"]) + err, tol=0.003)
    o.wait(10)


def place_on(o: Ctl, target_xy, z_release: float, theta: float) -> None:
    R = down(theta)
    o.goto([target_xy[0], target_xy[1], z_release + 0.06], R, tol=0.006)
    o.goto([target_xy[0], target_xy[1], z_release], R, speed=0.4, tol=0.004)
    o.grip(-1, 8)
    o.goto([target_xy[0], target_xy[1], z_release + 0.08], R, tol=0.01)


def stack_cube(o: Ctl) -> None:
    s = o.s()
    th = clear_face_heading(s["red_cube_yaw_deg"], o.hand_heading(), s["red_cube_pos"], [s["green_cube_pos"]])
    o.pick(s["red_cube_pos"], th)
    s = o.s()
    b = np.asarray(s["green_cube_pos"])
    th2 = face_heading(s["green_cube_yaw_deg"], th)
    place_on(o, b[:2], b[2] + 0.041, th2)
    o.wait(10)


def stack_pyramid(o: Ctl) -> None:
    s = o.s()
    a, b = np.asarray(s["red_cube_pos"]), np.asarray(s["green_cube_pos"])
    # put the red cube next to the green one, on the side facing the red cube's start, lined up with the green cube
    yb = math.radians(s["green_cube_yaw_deg"])
    axes = [np.array([math.cos(yb + k * math.pi / 2), math.sin(yb + k * math.pi / 2)]) for k in range(4)]
    c0 = np.asarray(s["blue_cube_pos"])
    d = max(
        axes, key=lambda v: min(np.linalg.norm(b[:2] + v * 0.042 - c0[:2]), 0.08) + 0.01 * float(v @ (a[:2] - b[:2]))
    )
    th = clear_face_heading(s["red_cube_yaw_deg"], o.hand_heading(), a, [b, c0])
    o.pick(a, th)
    dst = b[:2] + d * 0.042
    th2 = face_heading(s["green_cube_yaw_deg"], th)
    place_on(o, dst, 0.021, th2)
    s = o.s()
    a, b, c = (np.asarray(s[k]) for k in ("red_cube_pos", "green_cube_pos", "blue_cube_pos"))
    mid = (a[:2] + b[:2]) / 2
    # the blue cube goes across the pair: fingers along the pair's line so the cube bridges both
    along = heading(a[:2] - b[:2])
    th = clear_face_heading(s["blue_cube_yaw_deg"], o.hand_heading(), c, [a, b])
    o.pick(c, th, z_above=0.14)
    place_on(o, mid, 0.062, nearest_heading([along, along + math.pi / 2, along - math.pi / 2, along + math.pi], th))
    o.wait(10)


def push_cube(o: Ctl, sign: float) -> None:
    """sign +1: push the cube along +x (PushCube); -1: pull it along -x from its far side (PullCube)."""
    s = o.s()
    c, goal = np.asarray(s["cube_pos"]), np.asarray(s["goal_pos"])
    R = down(nearest_heading([math.pi / 2, -math.pi / 2], o.hand_heading()))  # fingers across y: a flat pusher face
    o.grip(1, 6)
    start = c + np.array([-sign * 0.06, 0, 0])
    o.goto([start[0], start[1], 0.08], R, tol=0.01)
    o.goto([start[0], start[1], 0.02], R, tol=0.005, speed=0.5)
    for _ in range(6):
        s = o.s()
        c = np.asarray(s["cube_pos"])
        if np.linalg.norm(c[:2] - goal[:2]) < 0.5 * float(s["goal_radius"]):
            break
        tgt = np.asarray(s["hand_pos"]) + np.r_[goal[:2] - c[:2], 0.0]
        tgt[2] = 0.02
        o.goto(tgt, R, tol=0.004, speed=0.5, max_steps=60)
    o.goto(np.asarray(o.s()["hand_pos"]) + np.array([-sign * 0.02, 0, 0.06]), R, tol=0.01)
    o.wait(10)


def poke_cube(o: Ctl) -> None:
    s = o.s()
    peg, axis_yaw = np.asarray(s["peg_pos"]), math.radians(s["peg_yaw_deg"])
    ax = np.array([math.cos(axis_yaw), math.sin(axis_yaw), 0.0])
    grasp = peg - ax * (s["peg_half_length"] * 0.5)  # grip the peg's tail half; the head end pokes
    o.pick(
        grasp,
        nearest_heading([axis_yaw + math.pi / 2, axis_yaw - math.pi / 2], o.hand_heading()),
        z_above=0.05,
        z_grasp=float(peg[2]),
    )
    s = o.s()
    goal = np.asarray(s["goal_pos"])
    for _ in range(12):
        s = o.s()
        cube, head = np.asarray(s["cube_pos"]), np.asarray(s["peg_head_pos"])
        if np.linalg.norm(cube[:2] - goal[:2]) < 0.015:
            break
        # head behind the cube on the goal line, then drive through toward the goal
        d = goal[:2] - cube[:2]
        d /= np.linalg.norm(d) + 1e-9
        hand = np.asarray(s["hand_pos"])
        want_head = (
            np.r_[cube[:2] - d * 0.022, head[2]]
            if np.linalg.norm(head[:2] - (cube[:2] - d * 0.03)) > 0.015
            else np.r_[head[:2] + (goal[:2] - cube[:2]) * 1.0, head[2]]
        )
        tgt = hand + (want_head - head)
        tgt[2] = float(s["peg_half_width"]) + 0.004 + (hand[2] - head[2])
        o.goto(tgt, tol=0.004, speed=0.4, max_steps=60)
    o.wait(10)


def place_sphere(o: Ctl) -> None:
    s = o.s()
    o.pick(s["sphere_pos"], o.hand_heading(), z_above=0.1, close=1.0)
    s = o.s()
    b = np.asarray(s["bin_pos"])
    off = np.asarray(s["sphere_pos"]) - np.asarray(s["hand_pos"])
    tgt_sphere = np.r_[b[:2], s["bin_floor_top_z"] + s["sphere_radius"] + 0.005]
    R = qmat(s["hand_target_quat"])
    o.goto(np.r_[tgt_sphere[:2] - off[:2], 0.1], R, tol=0.005)
    o.goto(tgt_sphere - off, R, tol=0.003, speed=0.3)
    o.grip(-1, 8)
    o.goto(np.asarray(o.s()["hand_pos"]) + [0, 0, 0.08], R, tol=0.01)
    o.wait(15)


def roll_ball(o: Ctl) -> None:
    s = o.s()
    ball, goal = np.asarray(s["ball_pos"]), np.asarray(s["goal_pos"])
    d = goal[:2] - ball[:2]
    d /= np.linalg.norm(d)
    R = down(
        nearest_heading([heading(d) + math.pi / 2, heading(d) - math.pi / 2], o.hand_heading())
    )  # flat face hits the ball
    o.grip(1, 4)
    back = ball[:2] - d * 0.1
    hand = np.asarray(s["hand_pos"])
    o.goto([hand[0], hand[1], 0.16], R, tol=0.02)  # over the ball, not through it
    o.goto([back[0], back[1], 0.16], R, tol=0.01)
    o.goto([back[0], back[1], 0.035], R, tol=0.004, speed=0.5)
    touch = ball[:2] - d * (float(s["ball_radius"]) + 0.014)  # finger face just short of the ball: push, do not strike
    o.goto([touch[0], touch[1], 0.035], R, tol=0.003, speed=0.3)
    # push through the centre at a steady 0.3 m/s; stop once the ball has left the fingers
    for _ in range(30):
        s = o.act(np.r_[d * 0.75, 0.0, 0.0, 0.0, 0.0, 1.0])["state"]
        gap = (np.asarray(s["ball_pos"]) - np.asarray(s["hand_pos"]))[:2] @ d
        if np.linalg.norm(s["ball_vel"]) > 0.2 and gap > float(s["ball_radius"]) + 0.04:
            break
    o.wait(2)
    o.goto(np.asarray(o.s()["hand_pos"]) + [0, 0, 0.08], R, tol=0.02)
    for _ in range(40):  # let it roll (the episode ends as solved when the ball crosses the goal)
        s = o.wait(5)
        if np.linalg.norm(np.asarray(s["ball_vel"])) < 0.003:
            break


def lift_peg_upright(o: Ctl) -> None:
    s = o.s()
    peg, ax = np.asarray(s["peg_pos"]), np.asarray(s["peg_axis"])
    L = float(s["peg_half_length"])
    th = nearest_heading([heading(ax) + math.pi / 2, heading(ax) - math.pi / 2], o.hand_heading())
    o.pick(peg, th, z_above=0.1, z_grasp=float(peg[2]))  # grip the middle: no torque, no slip
    R0 = down(th)
    o.goto(np.r_[peg[:2], L + 0.1], R0, tol=0.01)
    # turn the hand 90 degrees about the finger axis: the peg (across the fingers) swings upright
    fing = R0[:, 1]
    # the fingers end up pointing back toward the robot (the other way needs the wrist past its joint limit)
    out = np.r_[peg[:2] - np.asarray(s["robot_base_pos"])[:2], 0.0]
    R1 = min((rot(fing * sg * math.pi / 2) @ R0 for sg in (1, -1)), key=lambda Rt: float(Rt[:, 2] @ out))
    st = o.goto(np.r_[peg[:2], L + 0.1], R1, tol=0.03, rtol=0.03, max_steps=80)
    # correct the remaining tilt of the peg itself, then lower until its lower end is just above the table
    for _ in range(4):
        st = o.s()
        pax = np.asarray(st["peg_axis"])
        pax = pax if pax[2] > 0 else -pax
        tilt = np.cross(pax, [0, 0, 1.0])
        if np.linalg.norm(tilt) < 0.01:
            break
        o.goto(np.asarray(st["hand_pos"]), rot(tilt) @ qmat(st["hand_quat"]), tol=0.005, rtol=0.005, max_steps=60)
    st = o.s()
    lower_end = float(st["peg_pos"][2]) - L * abs(float(st["peg_axis"][2]))
    R = qmat(st["hand_target_quat"])
    o.goto(np.asarray(st["hand_pos"]) - [0, 0, lower_end - 0.003], R, tol=0.002, speed=0.3)
    o.grip(-1, 10)
    st = o.s()
    o.goto(np.asarray(st["hand_pos"]) - R[:, 2] * 0.1, R, tol=0.01)
    o.wait(15)


def align_rot(cur_axis, want_axis) -> np.ndarray:
    """Small rotation (axis-angle vector) that turns cur_axis onto want_axis."""
    a, b = np.asarray(cur_axis, float), np.asarray(want_axis, float)
    a, b = a / np.linalg.norm(a), b / np.linalg.norm(b)
    c = np.cross(a, b)
    n = float(np.linalg.norm(c))
    return np.zeros(3) if n < 1e-9 else c / n * math.atan2(n, float(a @ b))


def insert_servo(
    o: Ctl,
    tip_key: str,
    axis_key: str,
    hole,
    hax,
    along_goal: float,
    tol_lat: float,
    tol_along: float,
    n: int,
    speed: float,
) -> float:
    """Move the held object so its tip (state[tip_key]) sits on the line through `hole` along `hax`, at `along_goal`
    along it, with its axis (state[axis_key]) parallel to hax. Returns the remaining lateral error."""
    lat_n = 1.0
    for _ in range(n):
        st = o.s()
        tip, ax = np.asarray(st[tip_key]), np.asarray(st[axis_key])
        rel = tip - hole
        along = float(rel @ hax)
        lat = rel - along * hax
        lat_n = float(np.linalg.norm(lat))
        rv = align_rot(ax, hax)
        if lat_n < tol_lat and abs(along - along_goal) < tol_along and np.linalg.norm(rv) < 0.01:
            break
        err = -lat + (along_goal - along) * hax
        Rh = qmat(st["hand_quat"])
        # rotating about the hand moves the tip: aim the hand so the tip lands on the line after the turn
        hand = np.asarray(st["hand_pos"])
        tip_after = hand + rot(rv) @ (tip - hand)
        o.goto(
            hand + err + (tip - tip_after), rot(rv) @ Rh, tol=min(tol_lat, 0.002), rtol=0.01, speed=speed, max_steps=40
        )
    return lat_n


def peg_insertion_side(o: Ctl) -> None:
    s = o.s()
    peg, ax = np.asarray(s["peg_pos"]), np.asarray(s["peg_axis"])
    L = float(s["peg_half_length"])
    hole, hax = np.asarray(s["hole_pos"]), np.asarray(s["hole_axis"])
    grasp = peg - ax * max(0.05, L / 2 + 0.01)  # the white (tail) half; the orange head goes in
    # grasp with the finger heading that needs the smaller turn once the peg is lined up with the hole
    dth = wrap(heading(hax) - heading(ax))
    h0 = o.hand_heading()
    th = min(
        [heading(ax) + math.pi / 2, heading(ax) - math.pi / 2],
        key=lambda t: abs(wrap(t + dth - h0)) + abs(wrap(t - h0)),
    )
    o.pick(grasp, th, z_above=0.2, z_grasp=float(peg[2]))
    s = o.s()
    o.goto(
        np.asarray(s["hand_pos"]),
        rot([0, 0, wrap(heading(hax) - heading(s["peg_axis"]))]) @ qmat(s["hand_target_quat"]),
        tol=0.01,
        rtol=0.02,
        max_steps=200,
    )
    insert_servo(o, "peg_head_pos", "peg_axis", hole, hax, -(L + 0.04), 0.004, 0.01, 6, 1.0)
    insert_servo(o, "peg_head_pos", "peg_axis", hole, hax, -(L + 0.015), 0.001, 0.003, 12, 0.3)
    for a in np.arange(-L + 0.005, 0.0151, 0.01):  # in, a centimetre at a time, re-centring on the way
        insert_servo(o, "peg_head_pos", "peg_axis", hole, hax, float(a), 0.0015, 0.004, 4, 0.2)
    o.wait(5)


def plug_charger(o: Ctl) -> None:
    s = o.s()
    base, ax = np.asarray(s["charger_base_pos"]), np.asarray(s["charger_plug_axis"])
    goal, gax = np.asarray(s["goal_pos"]), np.asarray(s["goal_plug_axis"])
    dth = wrap(heading(gax) - heading(ax))
    h0 = o.hand_heading()
    th = min(
        [heading(ax) + math.pi / 2, heading(ax) - math.pi / 2],
        key=lambda t: abs(wrap(t + dth - h0)) + abs(wrap(t - h0)),
    )
    # grip the back half of the base, the hand tilted 15 degrees back (palm away from the plug) so it clears the receptacle
    R = down(th)
    fing = R[:, 1]
    tilt = min((rot(fing * sg * math.radians(15)) @ R for sg in (1, -1)), key=lambda Rt: -float(Rt[:, 2] @ ax))
    grasp = base - ax * 0.008
    o.grip(-1, 6)
    o.goto(np.r_[grasp[:2], 0.12], tilt, tol=0.01)
    o.goto(np.r_[grasp[:2], grasp[2] + 0.002], tilt, tol=0.003, speed=0.4)
    o.grip(1, 12)
    s = o.s()
    o.goto(np.r_[np.asarray(s["hand_pos"])[:2], 0.16], tilt, tol=0.01)
    s = o.s()
    o.goto(
        np.asarray(s["hand_pos"]),
        rot([0, 0, wrap(heading(gax) - heading(s["charger_plug_axis"]))]) @ qmat(s["hand_target_quat"]),
        tol=0.01,
        rtol=0.02,
        max_steps=200,
    )
    # the charger's origin is where the pins leave the base; at the goal it sits on the receptacle face
    insert_servo(o, "charger_pos", "charger_plug_axis", goal, gax, -0.05, 0.003, 0.01, 6, 1.0)
    insert_servo(o, "charger_pos", "charger_plug_axis", goal, gax, -0.03, 0.0003, 0.002, 15, 0.15)
    for a in (-0.022, -0.016, -0.012, -0.008, -0.004, 0.0):  # the pins are 16 mm long; enter with 0.5 mm clearance
        insert_servo(o, "charger_pos", "charger_plug_axis", goal, gax, a, 0.0003, 0.001, 6, 0.1)
    o.wait(5)


def pull_cube_tool(o: Ctl) -> None:
    s = o.s()
    tool, tax = np.asarray(s["tool_pos"]), np.asarray(s["tool_handle_axis"])
    grasp = tool + tax * 0.04
    th = nearest_heading([heading(tax) + math.pi / 2, heading(tax) - math.pi / 2], o.hand_heading())
    o.pick(grasp, th, z_above=0.12, z_grasp=float(tool[2]))
    s = o.s()
    cube = np.asarray(s["cube_pos"])
    hand, tool = np.asarray(s["hand_pos"]), np.asarray(s["tool_pos"])
    off = tool - hand  # tool origin relative to the hand while held
    # tool origin such that the hook's inner face is just beyond the cube and the hook spans the cube's y
    want = np.r_[
        cube[0] + s["cube_half_size"] + 0.012 - (s["tool_handle_length"] - s["tool_hook_length"]), cube[1] - 0.055, 0.0
    ]
    R = qmat(s["hand_target_quat"])
    o.goto(np.r_[(want - off)[:2], 0.12], R, tol=0.01)
    o.goto(np.r_[(want - off)[:2], s["tool_height"] / 2 + 0.004 - off[2]], R, tol=0.004, speed=0.4)
    base = np.asarray(s["robot_base_pos"])
    for _ in range(10):
        s = o.s()
        cube = np.asarray(s["cube_pos"])
        if np.linalg.norm(cube[:2] - base[:2]) < 0.5:
            break
        hand = np.asarray(s["hand_pos"])
        o.goto(hand + [-0.06, 0, 0], R, tol=0.006, speed=0.5, max_steps=40)
    o.grip(-1, 6)
    o.goto(np.asarray(o.s()["hand_pos"]) + [0, 0, 0.08], R, tol=0.01)
    o.wait(10)


SOLUTIONS = {
    "PickCube-v1": pick_cube,
    "StackCube-v1": stack_cube,
    "StackPyramid-v1": stack_pyramid,
    "PushCube-v1": lambda o: push_cube(o, 1.0),
    "PullCube-v1": lambda o: push_cube(o, -1.0),
    "PokeCube-v1": poke_cube,
    "PlaceSphere-v1": place_sphere,
    "RollBall-v1": roll_ball,
    "LiftPegUpright-v1": lift_peg_upright,
    "PegInsertionSide-v1": peg_insertion_side,
    "PlugCharger-v1": plug_charger,
    "PullCubeTool-v1": pull_cube_tool,
}


def run(task_id: str) -> None:
    from .backend import TASKS

    fn = SOLUTIONS[TASKS[task_id]["env"]]
    o = Ctl()
    try:
        fn(o)
        o.done("reference solution finished")
    except EpisodeOver:
        return
