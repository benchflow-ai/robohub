"""Reference solutions for the RLE-Bench tasks. They drive the robot only through the robo socket, like an agent.

- Tabletop tasks (TowerMaxHeight, CantileverOverhang, BalanceCoins): closed-loop controllers (below). RLE-Bench's own
  reference solutions for these replay recorded control trajectories open-loop; on this build the replay diverges (its
  first grasp closes about 2 cm above the block), so they are not used.
- HiddenCOM: RLE-Bench's own reference solution (tasks/task03/_template/hidden_com_solution/oracle.py): grasp the handle,
  lift the box, find the box in the top camera's RGB image and read the ballast quadrant from how the box hangs relative
  to the gripper. It uses only public observations (no oracle token).
"""

from __future__ import annotations

import struct
import zlib
from pathlib import Path

import numpy as np

from robouse.agent_cli import _send


class Finished(Exception):
    pass


def _call(req: dict) -> dict:
    r = _send(req)
    if not r.get("ok"):
        raise Finished(r.get("error"))
    return r["result"]


# ---- HiddenCOM ------------------------------------------------------------------------------------------------------


def _read_png(path: str) -> np.ndarray:
    """An 8-bit RGB or RGBA PNG (as the episode server writes them) as an HxWx3 array; standard library + numpy only."""
    data = Path(path).read_bytes()
    if data[:8] != b"\x89PNG\r\n\x1a\n":
        raise ValueError(f"{path} is not a PNG")
    pos, idat, w = 8, b"", 0
    while pos < len(data):
        n, kind = struct.unpack(">I4s", data[pos : pos + 8])
        chunk = data[pos + 8 : pos + 8 + n]
        if kind == b"IHDR":
            w, h, depth, color = struct.unpack(">IIBB", chunk[:10])
            if depth != 8 or color not in (2, 6) or chunk[12] != 0:
                raise ValueError("only 8-bit, non-interlaced RGB/RGBA PNGs are supported")
            ch = 3 if color == 2 else 4
        elif kind == b"IDAT":
            idat += chunk
        elif kind == b"IEND":
            break
        pos += 12 + n
    raw = np.frombuffer(zlib.decompress(idat), dtype=np.uint8).reshape(h, 1 + w * ch)
    out = np.zeros((h, w * ch), dtype=np.int32)
    for y in range(h):
        f, line = raw[y, 0], raw[y, 1:].astype(np.int32)
        prev = out[y - 1] if y else np.zeros(w * ch, dtype=np.int32)
        if f == 0:
            cur = line
        elif f == 1:
            cur = line.copy()
            for x in range(ch, w * ch):
                cur[x] = (cur[x] + cur[x - ch]) & 255
        elif f == 2:
            cur = (line + prev) & 255
        elif f == 3:
            cur = line.copy()
            for x in range(w * ch):
                left = cur[x - ch] if x >= ch else 0
                cur[x] = (cur[x] + ((left + prev[x]) >> 1)) & 255
        elif f == 4:
            cur = line.copy()
            for x in range(w * ch):
                a = cur[x - ch] if x >= ch else 0
                b, c = prev[x], (prev[x - ch] if x >= ch else 0)
                p = a + b - c
                pa, pb, pc = abs(p - a), abs(p - b), abs(p - c)
                cur[x] = (cur[x] + (a if pa <= pb and pa <= pc else b if pb <= pc else c)) & 255
        else:
            raise ValueError(f"unknown PNG filter {f}")
        out[y] = cur
    return out.reshape(h, w, ch)[:, :, :3].astype(np.uint8)


def hidden_com() -> None:
    q = [2**-0.5, 2**-0.5, 0, 0]  # the gripper pointing down, fingers across the handle (RLE-Bench's oracle)
    try:
        for position, gripper, steps in (
            ([0, 0, 1.05], -1, 60),
            ([0, 0, 0.86], -1, 60),
            ([0, 0, 0.86], 1, 40),
            ([0, 0, 1.06], 1, 80),
        ):
            _call({"op": "skill", "name": "move", "args": [*position, *q, gripper, steps]})
        obs = _call({"op": "observe", "image": True, "camera": "top"})
    except Finished:
        return
    paths = obs["image_path"] if isinstance(obs["image_path"], list) else [obs["image_path"]]
    top = next((p for p in paths if p.endswith("_top.png")), paths[0])
    rgb = _read_png(top).astype(float)
    yellow = (
        (rgb[:, :, 0] > 150)
        & (rgb[:, :, 1] > 100)
        & (rgb[:, :, 0] - rgb[:, :, 2] > 60)
        & (rgb[:, :, 1] - rgb[:, :, 2] > 50)
    )
    rows, cols = np.where(yellow)
    if len(rows) < 200:
        _send({"op": "give_up", "text": "box is not visible"})
        return
    eef = obs["state"]["eef_pos"]
    # The known handle-to-lid separation sets the approximate projection plane.
    lid_z = eef[2] - 0.056
    scale = rgb.shape[0] / (2 * np.tan(np.deg2rad(43 / 2)) * (1.71 - lid_z))
    u, v = (cols.min() + cols.max()) / 2, (rows.min() + rows.max()) / 2
    cx, cy = (rgb.shape[1] - 1) / 2, (rgb.shape[0] - 1) / 2
    center = np.array([(u - cx) / scale + 0.03, -(v - cy) / scale])
    delta = center - np.array(eef[:2])
    if np.min(np.abs(delta)) < 0.001:
        _send({"op": "give_up", "text": "lift deflection is inconclusive"})
        return
    answer = ("A" if delta[0] > 0 else "B") if delta[1] < 0 else ("C" if delta[0] > 0 else "D")
    _call({"op": "skill", "name": "submit", "args": [answer]})
    _send({"op": "done", "text": f"submitted {answer}"})


# ---- tabletop (TowerMaxHeight, CantileverOverhang, BalanceCoins) ----------------------------------------------------
# Closed-loop reference solutions: RLE-Bench's recorded trajectories are open-loop and do not reproduce on this build
# (the replay's first grasp closes about 2 cm above the block), so these solutions read the object poses the episode
# server gives the reference solution (oracle token; never masses or the heavy cube's identity) and drive the robot
# with RLE-Bench's `move` skill and base velocity actions.

TABLE_Z = 0.9


def _quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return [
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ]


def down(yaw: float = 0.0) -> list[float]:
    """Gripper pointing straight down, turned by `yaw` about the world z axis (quaternion x, y, z, w)."""
    return _quat_mul([0.0, 0.0, float(np.sin(yaw / 2)), float(np.cos(yaw / 2))], [1.0, 0.0, 0.0, 0.0])


class Arm:
    def __init__(self):
        self.grip = -1.0

    def state(self) -> dict:
        return _call({"op": "observe"})["state"]

    def move(self, p, q, grip=None, steps=40, tol=0.004, tries=6) -> dict:
        if grip is not None:
            self.grip = grip
        st = None
        for _ in range(tries):
            st = _call({"op": "skill", "name": "move", "args": [*map(float, p), *map(float, q), self.grip, steps]})[
                "state"
            ]
            if np.linalg.norm(np.asarray(st["eef_pos"]) - np.asarray(p)) < tol:
                break
        return st

    def hold(self, grip: float, steps: int = 20) -> None:
        self.grip = grip
        st = self.state()
        _call({"op": "skill", "name": "move", "args": [*st["eef_pos"], *st["eef_quat"], grip, steps]})

    def base_to(self, x: float, y: float, tol: float = 0.01) -> None:
        """Drive the mobile base (base mode) until its position is within tol of (x, y); the heading stays. Reads the
        base pose from each action's reply (no camera images)."""
        st = _call({"op": "act", "action": [0, 0, 0, 0, 0, 0, self.grip, 0, 0, 0, 0, -1.0], "repeat": 1})["state"]
        for _ in range(400):
            b = np.asarray(st["base_pos"][:2])
            err = np.array([x, y]) - b
            if np.linalg.norm(err) < tol:
                break
            qx, qy, qz, qw = st["base_quat"]
            yaw = np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
            c, s = np.cos(yaw), np.sin(yaw)
            fwd, lat = c * err[0] + s * err[1], -s * err[0] + c * err[1]
            v = np.clip(np.array([fwd, lat]) * 25.0, -1, 1)
            a = [0, 0, 0, 0, 0, 0, self.grip, float(v[0]), float(v[1]), 0.0, 0.0, 1.0]
            st = _call({"op": "act", "action": a, "repeat": 2})["state"]
        _call(
            {"op": "act", "action": [0, 0, 0, 0, 0, 0, self.grip, 0, 0, 0, 0, -1.0], "repeat": 10}
        )  # settle, arm mode

    def pick(self, obj: dict, yaw: float = 0.0, grasp_dz: float = 0.0) -> None:
        c = (np.asarray(obj["box_min"]) + np.asarray(obj["box_max"])) / 2
        top = obj["box_max"][2]
        q = down(yaw)
        self.move([c[0], c[1], top + 0.12], q, grip=-1, steps=60)
        self.move([c[0], c[1], c[2] + grasp_dz], q, steps=40, tol=0.003)
        self.hold(1.0, 25)
        self.move([c[0], c[1], top + 0.15], q, steps=50)

    def place(self, xy, bottom_z: float, obj_h: float, grasp_off: float, yaw: float = 0.0, lift: float = 0.1) -> None:
        """Lower the held object (its centre grasp_off below its top... measured from its bottom) onto bottom_z."""
        q = down(yaw)
        z = bottom_z + grasp_off + 0.004
        self.move([xy[0], xy[1], z + 0.10], q, steps=60)
        self.move([xy[0], xy[1], z], q, steps=50, tol=0.003)
        self.hold(-1.0, 25)
        self.move([xy[0], xy[1], z + lift], q, steps=40)

    def retract(self) -> None:
        st = self.state()
        p = np.asarray(st["eef_pos"])
        self.move([p[0], p[1], p[2] + 0.1], st["eef_quat"], steps=40)
        self.move([-0.40, 0.0, 1.30], down(), steps=80)


def _debug(*a) -> None:
    """Progress notes on stderr (the reference solution's log)."""
    import sys

    print(*a, file=sys.stderr, flush=True)


def _objects() -> dict:
    return _call({"op": "observe"})["state"]["objects"]


def _dims(o: dict) -> np.ndarray:
    return np.asarray(o["box_max"]) - np.asarray(o["box_min"])


def tower() -> None:
    """Stack cube_a, cyl_b, cyl_a and cube_b (flat tops) at one spot in front of the robot: about 55 % of the optimum."""
    arm = Arm()
    spot = np.array([-0.12, -0.05])
    top = TABLE_Z
    for name in ("cube_a", "cyl_b", "cyl_a", "cube_b"):
        o = _objects()[name]
        h = float(_dims(o)[2])
        arm.pick(o)
        arm.place(spot, top, h, grasp_off=h / 2)
        o = _objects()[name]
        top = float(o["box_max"][2])
        _debug(name, "placed; top", round(top, 3), "centre", o["pos"], "eef", arm.state()["eef_pos"])
    arm.retract()


def cantilever() -> None:
    """Two blocks over the -y table edge: the lower one overhanging 3 cm, the upper one 7.5 cm."""
    arm = Arm()
    names = sorted(_objects(), key=lambda n: _objects()[n]["pos"][1])[:2]  # the two blocks nearest the -y edge
    arm.base_to(-0.62, -0.45)
    for i, name in enumerate(names):
        o = _objects()[name]
        h = float(_dims(o)[2])
        arm.pick(o, yaw=np.pi / 2)
        y = -0.8 + 0.06 - 0.030 if i == 0 else -0.8 + 0.06 - 0.075 - 0.0
        top = TABLE_Z if i == 0 else float(_objects()[names[0]]["box_max"][2])
        arm.place([-0.12, y], top, h, grasp_off=h / 2, yaw=np.pi / 2)
    arm.retract()


BASE_X = -0.62
PAN_SPOTS = ((-0.035, -0.03), (-0.035, 0.03), (0.035, 0.0))
MAT = (-0.02, -0.55)


def _coins() -> dict:
    return _call({"op": "observe"})["state"]["cube_positions"]


def _pans() -> dict:
    return _call({"op": "observe"})["state"]["pan_positions"]


def _transfer(arm: Arm, coin: str, xy, surface_z: float, pan: str | None = None, spot=(0.0, 0.0)) -> None:
    """Pick `coin` and put it down at xy (or on `pan`, at `spot` from its centre) with its bottom on surface_z."""
    half = 0.016
    c = _coins()[coin]
    arm.base_to(BASE_X, float(np.clip(c[1], -0.45, 0.45)))
    c = _coins()[coin]
    arm.pick({"box_min": [c[0] - half, c[1] - half, c[2] - half], "box_max": [c[0] + half, c[1] + half, c[2] + half]})
    if pan:
        p = _pans()[pan]
        arm.base_to(BASE_X, float(p[1]) - 0.15)
        p = _pans()[pan]
        xy, surface_z = (p[0] + spot[0], p[1] + spot[1]), float(p[2])
    else:
        arm.base_to(BASE_X, float(np.clip(xy[1], -0.45, 0.45)))
    arm.place(xy, surface_z, 2 * half, grasp_off=half, lift=0.12)


def _on_pan(coin: str, side: str) -> bool:
    c, p = np.asarray(_coins()[coin]), np.asarray(_pans()[side])
    return bool(np.linalg.norm(c[:2] - p[:2]) < 0.10 and 0.0 < c[2] - p[2] < 0.08)


def _weigh(arm: Arm, left: list[str], right: list[str]) -> str:
    """Load the pans (left first, so the counts are equal only when both are full), read the tilt, unload (left
    first again). Returns 'left' or 'right' for the heavier side, or 'equal'."""
    home = {c: _coins()[c][:2] for c in left + right}
    for side, coins in (("left", left), ("right", right)):
        for coin, spot in zip(coins, PAN_SPOTS, strict=False):
            for _attempt in range(3):  # place it again if it did not end up resting on the pan
                _transfer(arm, coin, None, 0.0, pan=side, spot=spot)
                if _on_pan(coin, side):
                    break
                _debug("retry", coin, "not on the", side, "pan")
    st = arm.state()
    arm.move([st["eef_pos"][0] - 0.25, st["eef_pos"][1], 1.3], down(), steps=60)
    for _ in range(5):
        _call({"op": "act", "action": [0, 0, 0, 0, 0, 0, arm.grip, 0, 0, 0, 0, -1.0], "repeat": 20})
    _debug("on pans", {s: [c for c in left + right if _on_pan(c, s)] for s in ("left", "right")})
    p = _pans()
    dz = float(p["left"][2]) - float(p["right"][2])
    _debug("weighing", left, right, "left-right z", round(dz, 4))
    verdict = "equal" if abs(dz) < 0.002 else ("left" if dz < 0 else "right")
    for coin in left + right:
        _transfer(arm, coin, home[coin], TABLE_Z)
    return verdict


def balance() -> None:
    """Find the heavy cube with two weighings (3 v 3, then 1 v 1) and put it alone on the answer mat."""
    arm = Arm()
    try:
        names = sorted(_coins())
        groups = [names[0:3], names[3:6], names[6:9]]
        v = _weigh(arm, groups[0], groups[1])
        g = groups[0] if v == "left" else groups[1] if v == "right" else groups[2]
        v = _weigh(arm, [g[0]], [g[1]])
        heavy = g[0] if v == "left" else g[1] if v == "right" else g[2]
        _debug("heavy", heavy)
        _transfer(arm, heavy, MAT, TABLE_Z + 0.004)
        arm.retract()
    except Finished:
        return
    _send({"op": "done", "text": f"{heavy} is on the answer mat"})


def run_tabletop(kind: str) -> None:
    try:
        {"tower": tower, "cantilever": cantilever}[kind]()
    except Finished:
        return
    _send({"op": "done", "text": "stack built"})


def run(env: str) -> None:
    if env in ("rle-tower-max-height", "rle-cantilever-overhang"):
        run_tabletop(env.split("-")[1])
    elif env == "rle-balance-coins":
        balance()
    elif env.startswith("rle-hidden-com"):
        hidden_com()
    else:
        raise SystemExit(f"no RLE-Bench reference solution for {env!r}")
