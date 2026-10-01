"""Reference solutions for the mobile-manip tasks. They use only the public interface (`robo observe` fields, `robo act`,
`robo skill`) through an Oracle client, and read object and furniture positions from the observation like an agent."""

from __future__ import annotations

import math

import numpy as np

# waypoints in front of the doorways (x = 0 wall; doorways centred at y = 1.4 and y = -1.4)
KITCHEN_DOOR = {"east": (0.75, 1.4), "west": (-0.75, 1.4)}
STUDY_DOOR = {"east": (0.75, -1.4), "west": (-0.75, -1.4)}


def _st(o) -> dict:
    return o.state()


def _obj(o, name) -> np.ndarray:
    return np.array(_st(o)["objects"][name]["pos"])


def drive(o, *pts, yaw=None, tol=0.05):
    for i, p in enumerate(pts):
        last = i == len(pts) - 1
        args = [p[0], p[1]] + ([yaw] if (last and yaw is not None) else ["none"]) + [f"tol={tol if last else 0.1}"]
        o.skill("drive_to", *args)


def hold(o, n: int, dim: int) -> None:
    o.act([0.0] * dim, n)


# ------------------------------------------------------------------------------------------------------------------
# arm helpers per robot family
# ------------------------------------------------------------------------------------------------------------------


class Arm:
    """IK arms (TIAGo, Google Robot): world-frame gripper motions with the reach skill."""

    def __init__(self, o, key: str):
        self.o, self.key = o, key
        self.dim = 8 if key == "google" else 7

    def vec(self, twist=(0, 0, 0), arm=(0, 0, 0), pose=0, g=0):
        v, wz, vy = twist
        if self.key == "google":
            return [v, vy, wz, *arm, pose, g]
        return [v, wz, *arm, pose, g]

    def reach(self, p):
        return self.o.skill("reach", *[round(float(c), 4) for c in p])

    def back(self, dist: float, speed: float = 0.5):
        """Drive straight backwards `dist` metres (base.twist)."""
        vmax = 0.5 if self.key == "google" else 0.45
        n = max(1, int(round(dist / (speed * vmax * 0.05))))
        self.o.act(self.vec(twist=(-speed, 0, 0)), min(n, 50))
        if n > 50:
            self.o.act(self.vec(twist=(-speed, 0, 0)), n - 50)
        self.o.act(self.vec(), 6)


class StretchArm:
    """Stretch: the arm points to the robot's right; reach aligns the base along its heading, lifts and extends."""

    def __init__(self, o):
        self.o = o
        self.dim = 8

    def vec(self, v=0, wz=0, lift=0, ext=0, wyaw=0, wpitch=0, pose=0, g=0):
        return [v, wz, lift, ext, wyaw, wpitch, pose, g]

    def reach(self, p):
        return self.o.skill("reach", *[round(float(c), 4) for c in p])

    def retract(self, to: float = 0.0):
        e = _st(self.o)["robot"]["arm"]["extension_m"]
        n = int(max(0, e - to) / 0.01) + 3
        for k in range(0, n, 50):
            self.o.act(self.vec(ext=-1), min(50, n - k))

    def lift_by(self, dz: float):
        n = int(abs(dz) / 0.01) + 1
        self.o.act(self.vec(lift=math.copysign(1, dz)), min(n, 50))
        self.o.act(self.vec(), 8)


# ------------------------------------------------------------------------------------------------------------------
# solutions
# ------------------------------------------------------------------------------------------------------------------


def solve(o, env: str, task: dict) -> None:
    fn = {
        "stretch-fetch-cup": stretch_fetch_cup,
        "stretch-open-drawer": stretch_open_drawer,
        "stretch-shelve-two": stretch_shelve_two,
        "tiago-fetch-juice": tiago_fetch_juice,
        "tiago-drawer-stow": tiago_drawer_stow,
        "tiago-bin-cans": tiago_bin_cans,
        "tiago-study-to-kitchen": tiago_study_to_kitchen,
        "google-fetch-can": google_fetch_can,
        "google-open-drawer": google_open_drawer,
        "google-sort-bottle-can": google_sort,
    }[env]
    fn(o)


def _drawer(o) -> dict:
    return _st(o)["apartment"]["furniture"]["drawer"]


# ---- IK arms (TIAGo, Google) --------------------------------------------------------------------------------------
# Pattern: stand 1 m behind the object facing it, ready pose (gripper ~0.7 m ahead, 0.3 m short of the object), bring
# the gripper to a height that clears the surface edge, roll 0.15 m closer, then reach in, grasp, lift, pull back.


def _unit(deg: float) -> np.ndarray:
    return np.array([math.cos(math.radians(deg)), math.sin(math.radians(deg)), 0.0])


def _grip_pos(o) -> np.ndarray:
    return np.array(_st(o)["robot"]["gripper"]["pos"])


def _forward(o, arm, dist: float, speed: float = 0.3) -> None:
    vmax = 0.5 if arm.key == "google" else 0.45
    n = max(1, int(round(abs(dist) / (speed * vmax * 0.05))))
    o.act(arm.vec(twist=(math.copysign(speed, dist), 0, 0)), min(n, 50))
    o.act(arm.vec(), 6)


def _grasp_dz(o, name: str) -> float:
    """Height of the grasp above the object's centre (the centre works best with these grippers)."""
    return 0.0


def ik_pick(o, arm, name: str, face_deg: float, surface_h: float, lift: float = 0.06) -> dict:
    a = _unit(face_deg)
    c = _obj(o, name) + [0, 0, _grasp_dz(o, name)]
    stand = c - STAND[arm.key] * a
    drive(o, (stand[0], stand[1]), yaw=face_deg, tol=0.03)
    o.skill("ready")
    o.skill("open_gripper")
    g = _grip_pos(o)
    arm.reach([g[0], g[1], max(c[2], surface_h + 0.08)])
    _forward(o, arm, STAND[arm.key] - REACH_AHEAD[arm.key] - 0.15)
    c = _obj(o, name) + [0, 0, _grasp_dz(o, name)]
    arm.reach(c - 0.12 * a)
    arm.reach(c)
    res = o.skill("grasp")
    arm.reach(c + [0, 0, lift])
    arm.reach(c + [0, 0, lift] - 0.25 * a)  # pull the load in before driving
    arm.back(0.2)
    return res


def ik_place(
    o, arm, name: str, p, face_deg: float, surface_h: float, release_gap: float = 0.005, lift: float = 0.0
) -> None:
    """Put the held object down with its centre at p (x, y) resting on a surface of height surface_h
    (release_gap > 0.02 drops it from that height, e.g. into a bin or a drawer). The gripper height is set from the
    observed offset between the gripper and the object's centre, since an object can slip a little in the grasp.
    `lift` raises the open gripper that far before it backs away, so the fingers do not drag a light object."""
    a = _unit(face_deg)
    half_h = _half_h(o, name)

    def grip_z(centre_z):
        return centre_z + (_grip_pos(o)[2] - _obj(o, name)[2])

    stand = np.array([p[0], p[1], 0.0]) - STAND[arm.key] * a
    drive(o, (stand[0], stand[1]), yaw=face_deg, tol=0.03)
    g = _grip_pos(o)
    above = grip_z(surface_h + half_h + max(release_gap, 0) + 0.05)
    arm.reach([g[0], g[1], above])
    _forward(o, arm, STAND[arm.key] - REACH_AHEAD[arm.key] - 0.15)
    arm.reach([p[0], p[1], above])
    down = np.array([p[0], p[1], grip_z(surface_h + half_h + release_gap)])
    arm.reach(down)
    o.skill("open_gripper")
    hold(o, 6, arm.dim)
    if lift:
        down = down + np.array([0.0, 0.0, lift])
        arm.reach(down)
    arm.reach(down - 0.14 * a)
    arm.back(0.3)


def _half_h(o, name: str) -> float:
    sz = _st(o)["objects"][name]["size_m"]
    return (sz["height"] if "height" in sz else sz["box"][2]) / 2


def _table(o, name: str):
    t = _st(o)["apartment"]["furniture"][name]
    (cx, cy, h), (sx, sy) = t["top_center"], t["top_size"]
    return cx, cy, h, sx, sy


def ik_open_drawer(o, key: str, dist: float = 0.32) -> None:
    arm = Arm(o, key)
    h = np.array(_drawer(o)["handle"])
    drive(o, KITCHEN_DOOR["east"], KITCHEN_DOOR["west"])
    drive(o, (h[0], h[1] - REACH_AHEAD[key] - 0.1), yaw=90, tol=0.03)
    o.skill("ready")
    arm.reach(h + [0, -0.12, 0])
    o.skill("open_gripper")
    arm.reach(h + [0, -0.01, 0])
    o.skill("grasp")
    arm.back(dist, speed=0.3)
    o.skill("open_gripper")
    arm.reach(_grip_pos(o) + [0, -0.1, 0])


REACH_AHEAD = {"tiago": 0.70, "google": 0.66}  # gripper distance ahead of the base centre in the ready pose
STAND = {"tiago": 1.0, "google": 0.9}  # where to stop before reaching: this far behind the object


def google_open_drawer(o) -> None:
    ik_open_drawer(o, "google")


def tiago_drawer_stow(o) -> None:
    key = "tiago"
    arm = Arm(o, key)
    ik_open_drawer(o, key, dist=0.36)
    arm.back(0.2)
    ik_pick(o, arm, "spice_box", 90, COUNTER_H)
    dr = _drawer(o)
    c = np.array(dr["interior_center"])
    # drop it in from just above the drawer's front panel (top at z = 0.87)
    panel_top = dr["interior_floor_z"] - 0.01 + 0.20
    ik_place(o, arm, "spice_box", (c[0], c[1] + 0.03), 90, panel_top, release_gap=0.025)


COUNTER_H = 0.9


def tiago_fetch_juice(o) -> None:
    arm = Arm(o, "tiago")
    b = _obj(o, "juice_box")
    drive(o, (1.0, 0.6), KITCHEN_DOOR["east"], KITCHEN_DOOR["west"], (b[0] + 0.3, 1.1))
    ik_pick(o, arm, "juice_box", 90, COUNTER_H)
    drive(o, KITCHEN_DOOR["west"], KITCHEN_DOOR["east"], (0.75, 0.1))
    cx, cy, h, sx, sy = _table(o, "dining_table")
    ik_place(o, arm, "juice_box", (cx, cy - sy / 2 + 0.15), 90, h)


def tiago_study_to_kitchen(o) -> None:
    arm = Arm(o, "tiago")
    drive(o, (-1.0, -1.4), STUDY_DOOR["east"], STUDY_DOOR["west"])
    ik_pick(o, arm, "mug", -90, _table(o, "desk")[2])
    drive(o, (-1.0, -1.4), STUDY_DOOR["west"], STUDY_DOOR["east"], KITCHEN_DOOR["east"], KITCHEN_DOOR["west"])
    cx, cy, h, sx, sy = _table(o, "kitchen_table")
    # the kitchen table is against the west wall: approach its east edge facing west
    # lift before backing away: on Linux the open fingers otherwise drag the 0.25 kg mug off the table
    ik_place(o, arm, "mug", (cx + sx / 2 - 0.15, cy), 180, h, lift=0.03)


def _bin(o):
    b = _st(o)["apartment"]["furniture"]["bin"]
    return b["center"][0], b["center"][1], b["rim_height"]


def tiago_bin_cans(o) -> None:
    arm = Arm(o, "tiago")
    bx, by, rim = _bin(o)
    th = _table(o, "dining_table")[2]
    for name in ("red_can", "green_can"):
        ik_pick(o, arm, name, 90, th)
        # the bin is in the south-east: approach it from the west, facing east; drop the can in from above the rim
        drive(o, (bx - 1.0, 0.1))
        ik_place(o, arm, name, (bx, by), 0, rim, release_gap=0.05)


def google_fetch_can(o) -> None:
    arm = Arm(o, "google")
    drive(o, (0.9, 0.9), KITCHEN_DOOR["east"], KITCHEN_DOOR["west"])
    # the kitchen table is in the south-west corner of the kitchen: approach the can from the east, facing west
    ik_pick(o, arm, "can", 180, _table(o, "kitchen_table")[2])
    drive(o, KITCHEN_DOOR["west"], KITCHEN_DOOR["east"])
    cx, cy, h, sx, sy = _table(o, "dining_table")
    ik_place(o, arm, "can", (cx, cy - sy / 2 + 0.15), 90, h)


def google_sort(o) -> None:
    arm = Arm(o, "google")
    st = _st(o)["apartment"]["furniture"]
    sh = st["bookshelf"]
    up = sh["shelf_heights"]["upper"]
    sx0 = sh["x_range"][0]
    shy = sum(sh["y_range"]) / 2
    bx, by, rim = _bin(o)
    th = _table(o, "dining_table")[2]
    # bottle -> upper shelf (approach the bookshelf from the west, facing east)
    ik_pick(o, arm, "bottle", 90, th)
    ik_place(o, arm, "bottle", (sx0 + 0.16, shy), 0, up)
    # can -> bin
    ik_pick(o, arm, "can", 90, th)
    drive(o, (bx - 1.0, 0.0))
    ik_place(o, arm, "can", (bx, by), 0, rim, release_gap=0.05)


# ---- Stretch ------------------------------------------------------------------------------------------------------
# The Stretch arm points to the robot's right (-y in the robot frame). With heading yaw, the arm points along
# yaw - 90 degrees. To reach a point P from the side, stand at P - (0.42 + e) * arm_dir with the right heading.


def _stretch_stand(p, arm_dir_deg: float, ext: float = 0.25):
    a = math.radians(arm_dir_deg)
    d = 0.415 + ext
    return (p[0] - d * math.cos(a), p[1] - d * math.sin(a)), arm_dir_deg + 90


def stretch_open_drawer(o) -> None:
    arm = StretchArm(o)
    h = np.array(_drawer(o)["handle"])
    drive(o, KITCHEN_DOOR["east"], KITCHEN_DOOR["west"])
    (sx, sy), yaw = _stretch_stand(h, 90, ext=0.36)  # arm points north (+y): heading 180 (west)
    drive(o, (sx - 0.3, sy), tol=0.05)
    drive(o, (sx, sy), yaw=yaw, tol=0.02)
    o.skill("ready")
    arm.reach(h + [0, -0.1, 0])
    o.skill("open_gripper")
    arm.reach(h + [0, -0.012, 0])
    o.skill("grasp")
    arm.retract(0.02)
    hold(o, 5, 8)
    o.skill("open_gripper")
    o.act(arm.vec(lift=1), 8)


def stretch_fetch_cup(o) -> None:
    arm = StretchArm(o)
    c = _obj(o, "cup")
    drive(o, STUDY_DOOR["east"], STUDY_DOOR["west"])
    # the desk is against the south wall: arm points south (-y) -> heading 0 (east)
    (sx, sy), yaw = _stretch_stand(c, -90, ext=0.25)
    drive(o, (sx - 0.4, sy), tol=0.05)
    drive(o, (sx, sy), yaw=yaw, tol=0.02)
    o.skill("ready")
    o.skill("open_gripper")
    arm.reach(c + [0, 0.12, 0.0])
    arm.reach(c)
    o.skill("grasp")
    arm.lift_by(0.06)
    arm.retract(0.0)
    drive(o, STUDY_DOOR["west"], STUDY_DOOR["east"])
    t = _st(o)["apartment"]["furniture"]["dining_table"]
    cx, cy, h = t["top_center"]
    y0 = cy - t["top_size"][1] / 2
    p = np.array([cx, y0 + 0.15, h + 0.0475 + 0.012])
    (sx, sy), yaw = _stretch_stand(p, 90, ext=0.25)  # arm points north: heading 180
    drive(o, (sx + 0.4, sy), tol=0.05)
    drive(o, (sx, sy), yaw=yaw, tol=0.02)
    arm.reach(p + [0, -0.1, 0.02])
    arm.reach(p)
    o.skill("open_gripper")
    arm.retract(0.0)


def stretch_shelve_two(o) -> None:
    arm = StretchArm(o)
    st = _st(o)["apartment"]["furniture"]
    sh = st["bookshelf"]
    lo = sh["shelf_heights"]["lower"]
    sx0 = sh["x_range"][0]
    shy = sum(sh["y_range"]) / 2
    for i, name in enumerate(("can", "box")):
        c = _obj(o, name)
        half_h = 0.061 if name == "can" else 0.07
        (sx, sy), yaw = _stretch_stand(c, 90, ext=0.25)  # table to the north: arm north, heading 180
        drive(o, (sx + 0.4, sy), tol=0.05)
        drive(o, (sx, sy), yaw=yaw, tol=0.02)
        o.skill("ready")
        o.skill("open_gripper")
        arm.reach(c + [0, -0.12, 0])
        arm.reach(c)
        o.skill("grasp")
        arm.lift_by(0.05)
        arm.retract(0.0)
        # shelf: arm points east (+x) -> heading 90 (north); objects side by side on the lower shelf
        p = np.array([sx0 + 0.18, shy + (-0.18 if i == 0 else 0.18), lo + half_h + 0.012])
        (tx, ty), yaw = _stretch_stand(p, 0, ext=0.25)
        drive(o, (tx, ty - 0.4), tol=0.05)
        drive(o, (tx, ty), yaw=yaw, tol=0.02)
        arm.reach(p + [-0.1, 0, 0.02])
        arm.reach(p)
        o.skill("open_gripper")
        arm.retract(0.0)
        arm.lift_by(0.05)
        o.act(arm.vec(v=-1), 20)
