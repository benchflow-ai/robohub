"""Reference solutions for the robosuite suite: scripted end-effector controllers.

They talk to the episode server only through the robo socket (`act`, `observe`, `done`) and read only the public
observation fields listed in each task's instruction. Numpy only; robosuite is never imported here.

Each controller closes the loop on the observed hand pose: position error -> dx/dy/dz, and the hand's tilt
away from pointing straight down plus the finger-line yaw error -> droll/dpitch/dyaw (world-frame deltas).
"""
from __future__ import annotations

import numpy as np

ROT_CTL = True  # orientation control on; the Sawyer task turns it off (its OSC stalls when asked to hold the
                # hand vertical over the table, so its reference solution grasps with position control only)
POS_GAIN = 15.0  # action per metre of position error (1.0 asks for 5 cm)
ROT_GAIN = 1.2  # action per 0.5 rad of orientation error


def quat_mat(q) -> np.ndarray:
    x, y, z, w = [float(v) for v in q]
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def wrap90(a: float) -> float:
    """Wrap an angle difference for a line (period 180 deg) into [-90, 90)."""
    return (a + 90.0) % 180.0 - 90.0


def rotvec_to(v_from, v_to) -> np.ndarray:
    """Axis-angle vector rotating unit vector v_from onto v_to."""
    a, b = np.asarray(v_from, float), np.asarray(v_to, float)
    a, b = a / np.linalg.norm(a), b / np.linalg.norm(b)
    ax = np.cross(a, b)
    s, c = np.linalg.norm(ax), float(np.clip(a @ b, -1, 1))
    if s < 1e-9:
        return np.zeros(3)
    return ax / s * np.arctan2(s, c)


class Link:
    """The socket, as an agent sees it. `send` is `robouse.agent_cli._send` (or Episode.handle in tests)."""

    def __init__(self, send=None):
        if send is None:
            from ..agent_cli import _send as send
        self.send = send
        self.finished = False
        r = self.send({"op": "observe"})
        self.st = r["result"]["state"]
        info = self.send({"op": "info"})["result"]
        self.names = info["action"]["names"]
        self.budget = info["max_steps"]
        self.used = info["steps_used"]

    def observe(self) -> dict:
        r = self.send({"op": "observe"})
        if r.get("ok"):
            self.st = r["result"]["state"]
        else:
            self.finished = True
        return self.st

    def act(self, a, repeat: int = 1) -> dict:
        if self.finished:
            return self.st
        r = self.send({"op": "act", "action": [float(x) for x in a], "repeat": int(repeat)})
        if not r.get("ok"):
            self.finished = True
            return self.st
        res = r["result"]
        self.st = res["state"]
        self.used = res.get("steps_used", self.used)
        if "episode" in res:
            self.finished = True
        return self.st

    def done(self, text: str = "reference solution finished") -> None:
        if not self.finished:
            self.send({"op": "done", "text": text})
            self.finished = True


class Arm:
    """One arm's slice of the action and its observation fields."""

    def __init__(self, link: Link, idx: int | None, grip: bool = True):
        self.link, self.idx = link, idx
        self.pre = "" if idx is None else f"robot{idx}_"
        off = 0 if idx is None else 7 * idx if grip else 6 * idx
        self.sl = slice(off, off + 6)
        self.gi = off + 6 if grip else None
        self.grip_cmd = -1.0
        self.down = np.array([0.0, 0.0, -1.0])  # keep the hand pointing straight down (tilt correction)
        self.tilt_ctl = True
        self.rot_ctl = ROT_CTL  # False: position only (see run)
        self.rot_max = 1.0  # cap on rotation commands (large wrist turns are done slowly)
        self.yaw = None  # desired finger-line yaw, degrees (None: keep)

    def pos(self):
        return np.asarray(self.link.st[self.pre + "hand_pos"], float)

    def rot_cmd(self) -> np.ndarray:
        st = self.link.st
        R = quat_mat(st[self.pre + "hand_quat"])
        rv = rotvec_to(R[:, 2], self.down) if self.tilt_ctl else np.zeros(3)
        if self.yaw is not None and (self.pre + "gripper_yaw_deg") in st:
            rv[2] += np.radians(wrap90(self.yaw - st[self.pre + "gripper_yaw_deg"]))
        if not self.rot_ctl:
            return np.zeros(3)
        return np.clip(rv / 0.5 * ROT_GAIN, -self.rot_max, self.rot_max)

    def command(self, target, a: np.ndarray, gain: float = POS_GAIN, vmax: float = 1.0) -> None:
        if target is None:
            a[self.sl.start:self.sl.start + 3] = 0.0
        else:
            a[self.sl.start:self.sl.start + 3] = np.clip((np.asarray(target, float) - self.pos()) * gain, -vmax, vmax)
        a[self.sl.start + 3:self.sl.stop] = self.rot_cmd()
        if self.gi is not None:
            a[self.gi] = self.grip_cmd

    def yaw_err(self) -> float:
        st = self.link.st
        if self.yaw is None or not self.rot_ctl or (self.pre + "gripper_yaw_deg") not in st:
            return 0.0
        return abs(wrap90(self.yaw - st[self.pre + "gripper_yaw_deg"]))


def drive(link: Link, arms_targets: list[tuple[Arm, object]], tol: float = 0.008, max_n: int = 120,
          yaw_tol: float = 4.0, vmax: float = 1.0, min_n: int = 0) -> bool:
    """Move each arm toward its target (None = hold) until all are within tol (and yaw within yaw_tol)."""
    hist = []
    for n in range(max_n):
        if link.finished:
            return False
        hist.append(np.concatenate([arm.pos() for arm, _ in arms_targets]))
        if n >= 15 and np.max(np.abs(hist[-1] - hist[-12])) < 0.001 and all(arm.yaw_err() < 3 * yaw_tol for arm, _ in arms_targets):
            return False  # stalled (blocked by contact): stop pushing
        ok = all(t is None or np.linalg.norm(np.asarray(t) - arm.pos()) < tol for arm, t in arms_targets)
        ok = ok and all(arm.yaw_err() < yaw_tol for arm, _ in arms_targets)
        if ok and n >= min_n:
            return True
        a = np.zeros(len(link.names))
        for arm, t in arms_targets:
            arm.command(t, a, vmax=vmax)
        link.act(a)
    return False


def set_grip(link: Link, arms: list[Arm], value: float, n: int = 10) -> None:
    for arm in arms:
        arm.grip_cmd = value
    for _ in range(n):
        if link.finished:
            return
        a = np.zeros(len(link.names))
        for arm in arms:
            arm.command(None, a)
        link.act(a)


def face_yaw(obj_yaw: float, current: float, period: float = 90.0, offset: float = 0.0) -> float:
    """Finger-line yaw closest to `current` among obj_yaw + offset + k * period."""
    base = obj_yaw + offset
    cands = [base + k * period for k in range(-4, 5)]
    return min(cands, key=lambda c: abs(wrap90(c - current)))


# ---- single-arm primitives -------------------------------------------------------------------------------------
def pick(link: Link, arm: Arm, key: str, grasp_dz: float = 0.0, yaw_offset: float = 0.0, period: float = 90.0,
         hover: float = 0.10, lift_to: float | None = None, grasp_key: str | None = None) -> np.ndarray:
    """Grasp object `key` from above (fingers across its faces) and lift it. Returns hand-minus-object offset."""
    st = link.st
    gk = grasp_key or f"{key}_pos"
    p = np.asarray(st[gk], float)
    arm.yaw = face_yaw(st[f"{key}_yaw_deg"], st["gripper_yaw_deg"], period, yaw_offset)
    arm.grip_cmd = -1.0
    if arm.rot_ctl and arm.yaw_err() > 30:  # big wrist turn: do it first, slowly, where the arm starts
        arm.rot_max = 0.4
        drive(link, [(arm, arm.pos())], tol=0.03, max_n=80, yaw_tol=5.0)
        arm.rot_max = 1.0
    drive(link, [(arm, p + [0, 0, hover])], tol=0.01, max_n=150)
    p = np.asarray(link.st[gk], float)
    drive(link, [(arm, p + [0, 0, grasp_dz])], tol=0.006, max_n=80, vmax=0.6)
    set_grip(link, [arm], 1.0, 12)
    offset = arm.pos() - np.asarray(link.st[f"{key}_pos"], float)
    top = lift_to if lift_to is not None else arm.pos()[2] + 0.15
    drive(link, [(arm, np.r_[arm.pos()[:2], top])], tol=0.015, max_n=80)
    return offset


def place(link: Link, arm: Arm, key: str, obj_target, offset, hover: float = 0.12, carry_z: float | None = None,
          retreat: float = 0.12) -> None:
    """Carry the held object so its centre reaches obj_target, release, and move the hand away."""
    obj_target = np.asarray(obj_target, float)
    hand_goal = obj_target + offset
    z = carry_z if carry_z is not None else max(arm.pos()[2], hand_goal[2] + hover)
    drive(link, [(arm, np.r_[arm.pos()[:2], z])], tol=0.015, max_n=60)
    drive(link, [(arm, np.r_[hand_goal[:2], z])], tol=0.01, max_n=150)
    # re-measure the grasp offset (the object can slip) and descend
    offset = arm.pos() - np.asarray(link.st[f"{key}_pos"], float)
    hand_goal = obj_target + offset
    drive(link, [(arm, hand_goal)], tol=0.008, max_n=100, vmax=0.5)
    set_grip(link, [arm], -1.0, 10)
    drive(link, [(arm, arm.pos() + [0, 0, retreat])], tol=0.015, max_n=60)


def hold(link: Link, arms: list[Arm], n: int = 10) -> None:
    for _ in range(n):
        if link.finished:
            return
        a = np.zeros(len(link.names))
        for arm in arms:
            arm.command(None, a)
        link.act(a)


# ---- tasks -----------------------------------------------------------------------------------------------------
def solve_lift(link: Link) -> None:
    arm = Arm(link, None)
    pick(link, arm, "cube", grasp_dz=0.0)
    hold(link, [arm], 10)


def solve_stack(link: Link) -> None:
    arm = Arm(link, None)
    off = pick(link, arm, "cubea", grasp_dz=0.0)
    b = np.asarray(link.st["cubeb_pos"], float)
    place(link, arm, "cubea", b + [0, 0, 0.025 + 0.02 + 0.006], off)
    hold(link, [arm], 5)


PICKPLACE = {  # object key: (grasp height above the object's centre, finger yaw offset from the object's x axis)
    "can": (0.0, 0.0), "milk": (0.04, 0.0), "bread": (0.0, 90.0), "cereal": (0.03, 90.0),
}


def solve_pickplace(link: Link, key: str) -> None:
    arm = Arm(link, None)
    dz, yoff = PICKPLACE[key]
    off = pick(link, arm, key, grasp_dz=dz, yaw_offset=yoff, period=180.0 if key == "cereal" else 90.0,
               lift_to=1.10)
    tgt = np.asarray(link.st["target_bin_center"], float)
    z_obj = link.st["bin_floor_z"] + (0.07 if key in ("milk", "cereal") else 0.05)
    place(link, arm, key, np.r_[tgt[:2], z_obj], off, carry_z=1.10, retreat=0.15)
    hold(link, [arm], 5)


def solve_nut(link: Link, kind: str) -> None:
    arm = Arm(link, None)
    key = f"{kind}nut"
    st = link.st
    handle = np.asarray(st[f"{key}_handle_pos"], float)
    center = np.asarray(st[f"{key}_pos"], float)
    along = handle[:2] - center[:2]
    # fingers close across the handle: finger line perpendicular to the handle's long axis
    hy = np.degrees(np.arctan2(along[1], along[0]))
    off = pick(link, arm, key, grasp_dz=0.0, yaw_offset=wrap90(hy + 90 - st[f"{key}_yaw_deg"]), period=180.0,
               lift_to=1.0, grasp_key=f"{key}_handle_pos")
    peg = np.asarray(link.st[f"{kind}_peg_pos"], float)
    # turn the nut so its handle points toward the robot (-x) or sideways, never away (+x): the hand holds the
    # handle about 5 cm from the nut's centre, and reaching past the peg stalls the arm. The handle lies on the nut's
    # x axis, so this also leaves a square nut at 0 mod 90 degrees, matching its peg.
    c, h = np.asarray(link.st[f"{key}_pos"], float), np.asarray(link.st[f"{key}_handle_pos"], float)
    hd = np.degrees(np.arctan2(h[1] - c[1], h[0] - c[0]))
    goal = min((180.0, 90.0, -90.0), key=lambda g: abs((g - hd + 180.0) % 360.0 - 180.0))
    turn = (goal - hd + 180.0) % 360.0 - 180.0
    arm.yaw = link.st["gripper_yaw_deg"] + turn
    arm.rot_max = 0.5
    drive(link, [(arm, arm.pos())], tol=0.03, max_n=80, yaw_tol=2.0)
    arm.rot_max = 1.0
    table = link.st["table_height"]
    for attempt in range(3):
        servo_obj(link, arm, key, peg[:2], table + 0.15, kind, tol=0.004, max_n=120)  # nut hovering above the peg
        # lower the nut onto the peg while keeping its centre over the peg
        servo_obj(link, arm, key, peg[:2], table + 0.02, kind, tol=0.006, max_n=150, descend=True)
        if link.st[f"{key}_pos"][2] < table + 0.05:
            break
        drive(link, [(arm, arm.pos() + [0, 0, 0.06])], tol=0.01, max_n=40)  # jammed: lift and try again
    set_grip(link, [arm], -1.0, 10)
    drive(link, [(arm, arm.pos() + [0, 0, 0.12])], tol=0.015, max_n=60)
    hold(link, [arm], 5)


def servo_obj(link: Link, arm: Arm, key: str, xy, z_obj: float, kind: str, tol: float, max_n: int,
              descend: bool = False) -> None:
    """Move the held nut (not the hand) so its centre reaches (xy, z_obj); square nuts also keep yaw = 0 mod 90."""
    for n in range(max_n):
        if link.finished:
            return
        st = link.st
        obj = np.asarray(st[f"{key}_pos"], float)
        goal = np.r_[xy, z_obj]
        if descend:  # only go down while the nut is centred over the peg
            off = np.linalg.norm(obj[:2] - xy)
            goal[2] = z_obj if off < 0.0025 else max(z_obj, obj[2])
        if kind == "square":
            turn = ((0.0 - st[f"{key}_yaw_deg"]) + 45.0) % 90.0 - 45.0
            arm.yaw = st["gripper_yaw_deg"] + turn
        err = goal - obj
        if np.linalg.norm(err[:2]) < 0.002 and abs(err[2]) < tol and arm.yaw_err() < 1.5:
            return
        a = np.zeros(len(link.names))
        arm.command(arm.pos() + err * np.array([2.5, 2.5, 1.0]), a, vmax=0.5)  # stiffer in xy (the hole is tight)
        link.act(a)


def solve_door(link: Link) -> None:
    """Grasp the lever handle from above, push it down to release the latch, then pull the door open toward +y."""
    arm = Arm(link, None)
    h = np.asarray(link.st["handle_pos"], float)
    arm.yaw = 90.0  # finger line along y: the fingers close across the lever bar (its long axis runs along x)
    drive(link, [(arm, h + [0, 0, 0.08])], tol=0.01, max_n=120)
    h = np.asarray(link.st["handle_pos"], float)
    drive(link, [(arm, h + [0, 0, 0.0])], tol=0.006, max_n=60, vmax=0.5)
    set_grip(link, [arm], 1.0, 10)
    # turn the lever: the hand follows the lever's arc. The pivot is on the +x side of the grasp, so at lever angle
    # th the grasp point moves along (sin th, -cos th) in (x, z): first down, then mostly toward +x.
    for _ in range(100):  # the latch releases once the lever has turned past about 1.2 rad
        if link.finished or abs(link.st["handle_qpos"]) > 1.35:
            break
        th = abs(link.st["handle_qpos"])
        a = np.zeros(len(link.names))
        arm.command(arm.pos() + 0.025 * np.array([np.sin(th) + 0.1, 0, -np.cos(th)]), a)
        link.act(a)
    # pull the door open along its arc, keeping the lever turned until the latch has cleared the frame
    for _ in range(150):
        if link.finished or link.st["hinge_qpos"] > 0.38:
            break
        a = np.zeros(len(link.names))
        # keep the lever pressed down until the latch has cleared the frame (it re-catches below ~1.2 rad)
        th = abs(link.st["handle_qpos"])
        press = 0.02 * np.array([np.sin(th), -np.cos(th)]) if link.st["hinge_qpos"] < 0.2 and th < 1.4 else np.zeros(2)
        # pull along the door's arc: tangent to the circle about the hinge (counter-clockwise seen from above)
        r = arm.pos()[:2] - np.asarray(link.st["door_hinge_pos"][:2], float)
        t = np.array([-r[1], r[0]]) / np.linalg.norm(r)
        arm.command(arm.pos() + np.r_[0.03 * t, 0.0] + np.array([press[0], 0, press[1]]), a)
        link.act(a)
    hold(link, [arm], 5)


def solve_wipe(link: Link) -> None:
    """Press the sponge onto the table and sweep it over the remaining dirt markers, nearest first."""
    arm = Arm(link, None, grip=False)
    table = link.st["table_height"]
    # find the contact height: go down until the hand feels the table
    m = np.asarray(link.st["dirt_markers_xy"][0], float)
    drive(link, [(arm, np.r_[m, table + 0.08])], tol=0.01, max_n=120)
    for _ in range(60):
        if link.finished or link.st["hand_force_N"] > 8:
            break
        a = np.zeros(len(link.names))
        arm.command(arm.pos() + [0, 0, -0.01], a, vmax=0.3)
        link.act(a)
    z_contact = arm.pos()[2]
    press = z_contact - 0.005  # a little below the contact height, so the sponge keeps pressing
    skip: dict = {}  # marker -> step until which it is skipped (no progress while aiming at it)
    last_left, since = link.st["markers_left"], 0
    for n in range(900):
        if link.finished:
            return
        st = link.st
        pts = np.asarray(st["dirt_markers_xy"], float).reshape(-1, 2)
        if len(pts) == 0:
            break
        if st["markers_left"] < last_left:
            last_left, since = st["markers_left"], 0
        since += 1
        hp = arm.pos()
        cand = [i for i, p in enumerate(pts) if skip.get(tuple(p), -1) < n] or list(range(len(pts)))
        i = min(cand, key=lambda k: np.linalg.norm(pts[k] - hp[:2]))
        tgt = pts[i]
        if since > 25:  # stuck on this marker: leave it for later
            skip[tuple(tgt)] = n + 60
            since = 0
        d = tgt - hp[:2]
        dist = np.linalg.norm(d)
        u = d / dist if dist > 1e-6 else np.zeros(2)
        goal = hp[:2] + u * min(dist + 0.01, 0.03)  # aim a centimetre past the marker to overcome friction
        a = np.zeros(len(link.names))
        arm.command(np.r_[goal[0], goal[1], press], a, vmax=0.6)
        link.act(a)
    drive(link, [(arm, arm.pos() + [0, 0, 0.05])], tol=0.01, max_n=20)


def solve_two_arm_lift(link: Link) -> None:
    """Each arm grasps its own pot handle from above, then both lift together."""
    a0, a1 = Arm(link, 0), Arm(link, 1)
    st = link.st
    pot = np.asarray(st["pot_pos"], float)
    hs = [np.asarray(st["handle0_pos"], float), np.asarray(st["handle1_pos"], float)]
    for arm, h in ((a0, hs[0]), (a1, hs[1])):
        r = h[:2] - pot[:2]
        # fingers close across the handle bar, which runs perpendicular to the centre-to-handle direction
        arm.yaw = face_yaw(np.degrees(np.arctan2(r[1], r[0])), st[arm.pre + "gripper_yaw_deg"], 180.0)
    drive(link, [(a0, hs[0] + [0, 0, 0.10]), (a1, hs[1] + [0, 0, 0.10])], tol=0.01, max_n=150)
    hs = [np.asarray(link.st["handle0_pos"], float), np.asarray(link.st["handle1_pos"], float)]
    drive(link, [(a0, hs[0]), (a1, hs[1])], tol=0.007, max_n=80, vmax=0.5)
    set_grip(link, [a0, a1], 1.0, 12)
    for _ in range(80):  # lift together: both hands aim at the same height
        if link.finished:
            return
        z = min(a0.pos()[2], a1.pos()[2]) + 0.02
        a = np.zeros(len(link.names))
        a0.command(np.r_[a0.pos()[:2], z], a, vmax=0.5)
        a1.command(np.r_[a1.pos()[:2], z], a, vmax=0.5)
        link.act(a)
        if link.st["pot_pos"][2] > link.st["table_height"] + 0.25:
            break
    hold(link, [a0, a1], 10)


def solve_handover(link: Link) -> None:
    """robot0 picks the hammer up by the butt end of its handle and brings it to the middle; robot1 takes the
    handle next to the head; robot0 lets go and moves away."""
    a0, a1 = Arm(link, 0), Arm(link, 1)

    def across_handle(arm):  # finger line perpendicular to the handle, in the table plane
        ax = np.asarray(link.st["hammer_handle_axis"], float)
        return face_yaw(np.degrees(np.arctan2(ax[1], ax[0])), link.st[arm.pre + "gripper_yaw_deg"], 180.0, 90.0)

    def on_handle(frac):  # point on the handle: frac -1 = butt end, +1 = head end
        st = link.st
        return np.asarray(st["hammer_pos"], float) + np.asarray(st["hammer_handle_axis"], float) * frac * (st["hammer_handle_length"] / 2)

    # each robot takes the part of the handle on its own side: robot0 is at -y, robot1 at +y
    head_side = 1.0 if link.st["hammer_handle_axis"][1] > 0 else -1.0  # +1: the head points toward robot1
    frac0 = -0.3 * head_side  # robot0: near the middle (the head is heavy), on its own side
    butt = lambda: on_handle(frac0)
    a0.yaw = across_handle(a0)
    drive(link, [(a0, butt() + [0, 0, 0.10]), (a1, None)], tol=0.01, max_n=150)
    drive(link, [(a0, butt() + [0, 0, 0.003]), (a1, None)], tol=0.006, max_n=80, vmax=0.5)
    set_grip(link, [a0], 1.0, 12)
    drive(link, [(a0, a0.pos() + [0, 0, 0.2]), (a1, None)], tol=0.015, max_n=60)
    # bring the handle's centre to the middle of the table
    goal = np.array([0.0, -0.08, 1.0])
    for _ in range(2):
        off = a0.pos() - np.asarray(link.st["hammer_pos"], float)
        drive(link, [(a0, goal + off), (a1, None)], tol=0.008, max_n=120)
    # robot1 grasps the handle next to the head, from above
    # robot1: the far end of the handle on its side (next to the head, or the butt end)
    near_head = lambda: on_handle(head_side) - head_side * np.asarray(link.st["hammer_handle_axis"], float) * 0.025
    a1.yaw = across_handle(a1)
    safe = max(a1.pos()[2], near_head()[2] + 0.18)  # climb first, so robot1 never sweeps through the hammer
    drive(link, [(a0, None), (a1, np.r_[a1.pos()[:2], safe])], tol=0.015, max_n=60)
    drive(link, [(a0, None), (a1, np.r_[near_head()[:2], safe])], tol=0.01, max_n=150)
    drive(link, [(a0, None), (a1, near_head() + [0, 0, 0.06])], tol=0.01, max_n=80, vmax=0.5)
    drive(link, [(a0, None), (a1, near_head())], tol=0.006, max_n=80, vmax=0.4)
    set_grip(link, [a1], 1.0, 12)
    # robot0 lets go and moves back toward its own side
    set_grip(link, [a0], -1.0, 12)
    drive(link, [(a0, a0.pos() + [0, -0.12, 0.08]), (a1, a1.pos() + [0, 0.02, 0.03])], tol=0.015, max_n=60)
    hold(link, [a0, a1], 10)


def solve_peg_in_hole(link: Link) -> None:
    """Both arms turn about 45 degrees so the peg's axis and the hole's axis meet in the middle (a 90 degree turn
    for one wrist alone does not reach), then robot0 lines the peg up in front of the hole and pushes it through."""
    a0, a1 = Arm(link, 0, grip=False), Arm(link, 1, grip=False)
    st = link.st
    v0 = np.asarray(st["peg_axis"], float)
    n0 = np.asarray(st["hole_axis"], float)
    c0 = np.asarray(st["hole_center"], float)
    start = np.asarray(st["peg_pos"], float)
    n0 = n0 if (start - c0) @ n0 < 0 else -n0  # hole axis pointing from the peg's side into the plate
    m = (v0 + n0) / np.linalg.norm(v0 + n0)  # the shared axis both will turn to
    hand1 = a1.pos()

    def step(peg_goal, vmax=0.4, hold1=True):
        st = link.st
        a = np.zeros(len(link.names))
        peg = np.asarray(st["peg_pos"], float)
        a0.command(a0.pos() + (np.asarray(peg_goal, float) - peg), a, vmax=vmax)
        n = np.asarray(st["hole_axis"], float)
        n = n if n @ n0 > 0 else -n
        # robot0 aims the peg at the plate's current axis (not the fixed one), so drift of either hand is corrected
        a[a0.sl.start + 3:a0.sl.stop] = np.clip(rotvec_to(st["peg_axis"], m if hold1 else n) / 0.5, -0.6, 0.6)
        a1.command(hand1, a, vmax=vmax)
        a[a1.sl.start + 3:a1.sl.stop] = np.clip(rotvec_to(n, m) / 0.5, -0.6, 0.6)
        link.act(a)

    for _ in range(120):  # 1) turn both, the peg raised a little and pulled back from the plate
        if link.finished or link.st["peg_hole_cos"] > 0.995:
            break
        step(start + [0, 0, 0.05] - m * 0.05)
    # 2) line the peg up on the hole's axis, 15 cm in front of the plate; 3) push it through along the axis
    for depth, tol in ((-0.15, 0.006), (0.0, 0.01)):
        for _ in range(150):
            if link.finished:
                return
            n = np.asarray(link.st["hole_axis"], float)
            n = n if n @ n0 > 0 else -n
            goal = np.asarray(link.st["hole_center"], float) + n * depth
            if np.linalg.norm(goal - np.asarray(link.st["peg_pos"], float)) < tol and link.st["peg_hole_cos"] > 0.98:
                break
            step(goal, hold1=False)
    for _ in range(5):  # hold still with zero deltas (the generic hold would level the hands again)
        link.act(np.zeros(len(link.names)))


SOLVERS = {
    "TwoArmPegInHole": solve_peg_in_hole,
    "TwoArmHandover": solve_handover,
    "TwoArmLift": solve_two_arm_lift,
    "Wipe": solve_wipe,
    "Door": solve_door,
    "Lift": solve_lift,
    "Stack": solve_stack,
    "PickPlaceCan": lambda l: solve_pickplace(l, "can"),
    "PickPlaceMilk": lambda l: solve_pickplace(l, "milk"),
    "PickPlaceBread": lambda l: solve_pickplace(l, "bread"),
    "PickPlaceCereal": lambda l: solve_pickplace(l, "cereal"),
    "NutAssemblySquare": lambda l: solve_nut(l, "square"),
    "NutAssemblyRound": lambda l: solve_nut(l, "round"),
}


def run(env: str, send=None) -> None:
    """env: a robosuite env name, optionally with flags after a colon (`Lift:norot` = position control only)."""
    global ROT_CTL
    name, _, flags = env.partition(":")
    ROT_CTL = "norot" not in flags.split(",")
    link = Link(send)
    SOLVERS[name](link)
    link.done()
