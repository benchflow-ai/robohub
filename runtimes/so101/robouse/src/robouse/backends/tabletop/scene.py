"""Tabletop scenes as MJCF: the table, the gripper, objects and fixtures, from a scenario dict."""

from __future__ import annotations

import math

import numpy as np

COLORS: dict[str, tuple[float, float, float]] = {
    "red": (0.85, 0.15, 0.15),
    "blue": (0.15, 0.32, 0.85),
    "green": (0.15, 0.62, 0.25),
    "yellow": (0.95, 0.80, 0.10),
    "orange": (0.97, 0.50, 0.10),
    "purple": (0.55, 0.25, 0.75),
    "violet": (0.62, 0.40, 0.90),
    "cyan": (0.10, 0.75, 0.80),
    "white": (0.93, 0.93, 0.93),
    "black": (0.10, 0.10, 0.10),
    "grey": (0.55, 0.55, 0.55),
    "darkgrey": (0.30, 0.30, 0.32),
    "brown": (0.50, 0.33, 0.20),
    "teal": (0.10, 0.55, 0.55),
    "gold": (0.85, 0.68, 0.20),
    "pink": (0.95, 0.55, 0.70),
    "silver": (0.78, 0.80, 0.84),
    "tan": (0.80, 0.68, 0.50),
}

TABLE_HX, TABLE_HY = 0.35, 0.30
FLOOR_Z = -0.2
HAND_LO = np.array([-0.34, -0.27, 0.012])
HAND_HI = np.array([0.34, 0.28, 0.35])
HAND_HOME = [0.0, 0.2, 0.25]
STEP_M = 0.01  # metres per action unit per step
SUBSTEPS = 10  # 10 x 2 ms = 20 ms per env step
FINGER_OPEN = 0.05  # per-finger travel; fully open gap = 0.10 m
SETTLE_SUBSTEPS = 150
WRIST_CAM_POS = (0.04, 0.0, 0.06)  # hard suite: eye-in-hand camera, relative to the tool point, looking straight down
WRIST_CAM_FOVY = 50

# collision classes: static world ct=4 ca=1; drawer ct=2 ca=1; objects + gripper ct=1 ca=7
STATIC = 'contype="4" conaffinity="1"'
DRAWER = 'contype="2" conaffinity="1"'
MOVING = 'contype="1" conaffinity="7"'
VISUAL = 'contype="0" conaffinity="0"'

# drawer geometry (cabinet frame = cabinet centre on the table)
DR_IN = 0.059  # drawer interior half size (x and y)
DR_FLOOR = 0.006  # drawer floor top
DR_WALL_TOP = 0.056
DR_HANDLE = (-0.117, 0.045)  # handle bar (y offset from cabinet centre when closed, z)
DR_TRAVEL = 0.12


def _f(*v) -> str:
    return " ".join(f"{float(x):.5g}" for x in v)


def _rgba(c: str, a: float = 1.0) -> str:
    return _f(*COLORS.get(c, COLORS["grey"]), a)


# --------------------------------------------------------------------------------------------------
# MJCF builder
# --------------------------------------------------------------------------------------------------


def _obj_xml(o: dict) -> str:
    n, kind, col = o["name"], o["kind"], o.get("color", "grey")
    x, y = o["pos"][:2]
    z = o.get("z", _obj_rest_half(o))
    yaw = o.get("yaw", 0.0)
    fr = (
        'friction="1.0 0.01 0.001" condim="4"'
        if "friction" not in o
        else f'friction="{_f(o["friction"])} 0.01 0.001" condim="4"'
    )
    head = f'<body name="obj_{n}" pos="{_f(x, y, z)}" euler="0 0 {yaw}"><freejoint name="fj_{n}"/>'
    if kind == "box":
        hx, hy, hz = o["size"]
        dens = o.get("density", 400)
        g = f'<geom name="g_{n}" type="box" size="{_f(hx, hy, hz)}" rgba="{_rgba(col)}" density="{dens}" {fr} {MOVING}/>'
        if o.get("label") == "heavy":  # dark band so the heavy blocks read as metal
            g += f'<geom type="box" size="{_f(hx + 0.0005, hy + 0.0005, hz * 0.25)}" rgba="0.15 0.15 0.15 1" {VISUAL}/>'
        return head + g + "</body>"
    if kind == "cyl":
        r, hh = o["size"]
        dens = o.get("density", 400)
        return (
            head
            + (
                f'<geom name="g_{n}" type="cylinder" size="{_f(r, hh)}" rgba="{_rgba(col, o.get("alpha", 1.0))}" '
                f'density="{dens}" {fr} {MOVING}/>'
            )
            + "</body>"
        )
    if kind == "knife":
        # origin = geometric centre; handle on local -x, blade on local +x
        return (
            head
            + (
                f'<geom name="g_{n}" type="box" pos="-0.04 0 0" size="0.035 0.011 0.012" rgba="{_rgba("brown")}" density="500" {fr} {MOVING}/>'
                f'<geom name="g_{n}_blade" type="box" pos="0.035 0 -0.004" size="0.04 0.0025 0.008" rgba="{_rgba("silver")}" density="300" {fr} {MOVING}/>'
            )
            + "</body>"
        )
    raise ValueError(f"unknown object kind {kind!r}")


def _obj_rest_half(o: dict) -> float:
    if o["kind"] == "box":
        return o["size"][2]
    if o["kind"] == "cyl":
        return o["size"][1]
    if o["kind"] == "knife":
        return 0.012
    return 0.02


def _walls_xml(name: str, hx: float, hy: float, h: float, col: str, t: float = 0.006, floor: bool = True) -> str:
    s = []
    if floor:
        s.append(
            f'<geom name="{name}_floor" type="box" pos="0 0 0.003" size="{_f(hx + t, hy + t, 0.003)}" rgba="{_rgba(col)}" {STATIC}/>'
        )
    for sx in (-1, 1):
        s.append(
            f'<geom type="box" pos="{_f(sx * (hx + t / 2), 0, h / 2)}" size="{_f(t / 2, hy + t, h / 2)}" rgba="{_rgba(col)}" {STATIC}/>'
        )
    for sy in (-1, 1):
        s.append(
            f'<geom type="box" pos="{_f(0, sy * (hy + t / 2), h / 2)}" size="{_f(hx, t / 2, h / 2)}" rgba="{_rgba(col)}" {STATIC}/>'
        )
    return "".join(s)


def _fixture_xml(fx: dict, grid: dict | None) -> str:
    k, n = fx["kind"], fx["name"]
    col = fx.get("color", "grey")
    x, y = _fixture_xy(fx, grid)
    z0 = fx.get("z", 0.0)
    if k == "pad":
        hx, hy = fx.get("size", [0.04, 0.04])
        dz = 0.0012 if fx.get("cell") else 0.0008
        return f'<geom name="pad_{n}" type="box" pos="{_f(x, y, z0 + dz)}" size="{_f(hx, hy, dz)}" rgba="{_rgba(col)}" {VISUAL}/>'
    if k == "tray":
        hx, hy = fx.get("size", [0.07, 0.06])
        return (
            f'<body name="fx_{n}" pos="{_f(x, y, z0)}">'
            + _walls_xml(n, hx, hy, fx.get("height", 0.03), col)
            + "</body>"
        )
    if k == "bowl":  # octagonal low-walled bowl
        R, h, t = fx.get("radius", 0.07), fx.get("height", 0.035), 0.006
        s = [
            f'<body name="fx_{n}" pos="{_f(x, y, z0)}">',
            f'<geom name="{n}_floor" type="cylinder" pos="0 0 0.003" size="{_f(R + t, 0.003)}" rgba="{_rgba(col)}" {STATIC}/>',
        ]
        half_len = R * math.tan(math.pi / 8) + t / 2
        for i in range(8):
            a = i * math.pi / 4
            s.append(
                f'<geom type="box" pos="{_f((R + t / 2) * math.cos(a), (R + t / 2) * math.sin(a), h / 2)}" '
                f'euler="0 0 {math.degrees(a):.3f}" size="{_f(t / 2, half_len, h / 2)}" rgba="{_rgba(col)}" {STATIC}/>'
            )
        return "".join(s) + "</body>"
    if k == "plate":
        R = fx.get("radius", 0.065)
        return (
            f'<geom name="{n}_plate" type="cylinder" pos="{_f(x, y, z0 + 0.004)}" size="{_f(R, 0.004)}" rgba="{_rgba(col)}" {STATIC}/>'
            f'<geom type="cylinder" pos="{_f(x, y, z0 + 0.0085)}" size="{_f(R * 0.7, 0.0005)}" rgba="0.85 0.85 0.85 1" {VISUAL}/>'
        )
    if k == "socket":
        s = fx.get("inner", 0.022)
        return (
            f'<body name="fx_{n}" pos="{_f(x, y, z0)}">'
            + _walls_xml(n, s, s, fx.get("height", 0.035), col, t=0.01, floor=False)
            + "</body>"
        )
    if k == "drawer":
        return _drawer_xml(n, x, y, col)
    if k == "figure":
        return _figure_xml(n, x, y, z0, fx.get("yaw", 0.0))
    if k == "lane":  # hard suite: a raised low-friction slab (shuffleboard lane); may extend past the table edge
        hx, hy = fx.get("size", [0.1, 0.4])
        h = fx.get("height", 0.004)
        mu = fx.get("friction", 0.03)
        return (
            f'<geom name="lane_{n}" type="box" pos="{_f(x, y, z0 + h / 2)}" size="{_f(hx, hy, h / 2)}" rgba="{_rgba(col)}" '
            f'friction="{_f(mu)} 0.005 0.0001" condim="3" {STATIC}/>'
        )
    raise ValueError(f"unknown fixture kind {k!r}")


def _drawer_xml(n: str, cx: float, cy: float, col: str) -> str:
    c = _rgba(col)
    cab = [f'<body name="fx_{n}" pos="{_f(cx, cy, 0)}">']
    for sx in (-1, 1):
        cab.append(
            f'<geom type="box" pos="{_f(sx * 0.075, 0.005, 0.045)}" size="0.005 0.075 0.045" rgba="{c}" {STATIC}/>'
        )
    cab.append(f'<geom type="box" pos="0 0.075 0.045" size="0.08 0.005 0.045" rgba="{c}" {STATIC}/>')
    cab.append(f'<geom name="{n}_top" type="box" pos="0 0.005 0.085" size="0.08 0.075 0.005" rgba="{c}" {STATIC}/>')
    cab.append("</body>")
    d = _rgba("tan")
    dr = [
        f'<body name="drawer_{n}" pos="{_f(cx, cy, 0)}">',
        f'<joint name="dj_{n}" type="slide" axis="0 -1 0" range="0 {DR_TRAVEL}" damping="4" frictionloss="0.3" armature="0.05"/>',
        f'<geom type="box" pos="0 0 0.003" size="0.065 0.065 0.003" rgba="{d}" density="300" {DRAWER}/>',
    ]
    for sx in (-1, 1):
        dr.append(
            f'<geom type="box" pos="{_f(sx * 0.062, 0, 0.031)}" size="0.003 0.065 0.025" rgba="{d}" density="300" {DRAWER}/>'
        )
    dr.append(f'<geom type="box" pos="0 0.062 0.031" size="0.065 0.003 0.025" rgba="{d}" density="300" {DRAWER}/>')
    dr.append(
        f'<geom name="{n}_front" type="box" pos="0 -0.068 0.04" size="0.072 0.004 0.036" rgba="{c}" density="300" {DRAWER}/>'
    )
    for sx in (-1, 1):
        dr.append(
            f'<geom type="box" pos="{_f(sx * 0.03, -0.092, DR_HANDLE[1])}" size="0.004 0.02 0.004" rgba="{_rgba("silver")}" density="300" {DRAWER}/>'
        )
    dr.append(
        f'<geom name="{n}_handle" type="box" pos="{_f(0, DR_HANDLE[0], DR_HANDLE[1])}" size="0.04 0.005 0.006" rgba="{_rgba("silver")}" density="300" {DRAWER}/>'
    )
    dr.append("</body>")
    return "".join(cab + dr)


def _figure_xml(n: str, x: float, y: float, z0: float, yaw: float) -> str:
    """A person-like figure (mannequin) lying on its back along local +x (head at +x)."""
    shirt, skin, pants = "0.30 0.45 0.62 1", "0.92 0.78 0.64 1", "0.25 0.25 0.32 1"
    g = [
        f'<body name="fx_{n}" pos="{_f(x, y, z0)}" euler="0 0 {yaw}">',
        f'<geom name="{n}_torso" type="capsule" fromto="0 0 0.034 0.1 0 0.034" size="0.034" rgba="{shirt}" {STATIC}/>',
        f'<geom name="{n}_neck" type="capsule" fromto="0.12 0 0.03 0.14 0 0.03" size="0.014" rgba="{skin}" {STATIC}/>',
        f'<geom name="{n}_head" type="sphere" pos="0.172 0 0.032" size="0.032" rgba="{skin}" {STATIC}/>',
    ]
    for s in (-1, 1):
        g.append(
            f'<geom type="capsule" fromto="{_f(-0.02, s * 0.018, 0.02, -0.17, s * 0.024, 0.02)}" size="0.02" rgba="{pants}" {STATIC}/>'
        )
        g.append(
            f'<geom type="capsule" fromto="{_f(0.1, s * 0.048, 0.016, -0.01, s * 0.062, 0.016)}" size="0.015" rgba="{shirt}" {STATIC}/>'
        )
    return "".join(g) + "</body>"


def _fixture_xy(fx: dict, grid: dict | None) -> tuple[float, float]:
    if "cell" in fx and grid:
        return _cell_xy(grid, *fx["cell"])
    return float(fx["pos"][0]), float(fx["pos"][1])


def _cell_xy(grid: dict, r: int, c: int) -> tuple[float, float]:
    x0, y0 = grid["origin"]
    cs = grid["cell"]
    return x0 + c * cs, y0 - r * cs


def _look(pos, target) -> str:
    p, t = np.asarray(pos, float), np.asarray(target, float)
    zc = p - t
    zc /= np.linalg.norm(zc)
    xc = np.cross([0, 0, 1.0], zc)
    xc /= np.linalg.norm(xc)
    yc = np.cross(zc, xc)
    return f'pos="{_f(*p)}" xyaxes="{_f(*xc, *yc)}"'


def build_xml(sc: dict) -> str:
    grid = sc.get("grid")
    wb: list[str] = []
    wb.append(f'<geom name="floor" type="plane" pos="0 0 {FLOOR_Z}" size="2 2 0.1" material="floor" {STATIC}/>')
    wb.append(
        f'<geom name="table" type="box" pos="0 0 {FLOOR_Z / 2}" size="{_f(TABLE_HX, TABLE_HY, -FLOOR_Z / 2)}" material="wood" {STATIC}/>'
    )
    if grid:
        for r in range(grid["rows"]):
            for c in range(grid["cols"]):
                x, y = _cell_xy(grid, r, c)
                h = grid["cell"] / 2 - 0.004
                wb.append(
                    f'<geom type="box" pos="{_f(x, y, 0.0005)}" size="{_f(h, h, 0.0005)}" rgba="0.90 0.87 0.80 1" {VISUAL}/>'
                )
    for fx in sc.get("fixtures", []):
        wb.append(_fixture_xml(fx, grid))
    for o in sc.get("objects", []):
        wb.append(_obj_xml(_resolved_obj(o, grid)))
    hx, hy, hz = HAND_HOME
    fcol = "0.25 0.25 0.28 1"
    fr = 'friction="1.5 0.02 0.002" condim="4" solref="0.01 1"'
    wb.append(f'<body name="mocap" mocap="true" pos="{_f(hx, hy, hz)}"/>')
    wb.append(
        f'<body name="hand" pos="{_f(hx, hy, hz)}" gravcomp="1"><freejoint name="hand_free"/>'
        f'<geom name="palm" type="box" pos="0 0 0.074" size="0.022 0.065 0.012" rgba="0.18 0.18 0.2 1" mass="0.25" {MOVING}/>'
        f'<geom name="wrist" type="cylinder" pos="0 0 0.16" size="0.02 0.075" rgba="0.55 0.56 0.6 1" mass="0.05" {VISUAL}/>'
        f'<site name="tcp" pos="0 0 0" size="0.004" rgba="1 0 0 0"/>'
        + (
            f'<camera name="wrist" pos="{_f(*WRIST_CAM_POS)}" fovy="{WRIST_CAM_FOVY}"/>'
            if sc.get("wrist_camera")
            else ""
        )
        + f'<body name="finger_l" gravcomp="1"><joint name="fl" type="slide" axis="0 1 0" range="0 {FINGER_OPEN}" damping="30" armature="0.01"/>'
        f'<geom name="finger_l" type="box" pos="0 0.006 0.026" size="0.012 0.006 0.036" rgba="{fcol}" mass="0.03" {fr} {MOVING}/></body>'
        f'<body name="finger_r" gravcomp="1"><joint name="fr" type="slide" axis="0 -1 0" range="0 {FINGER_OPEN}" damping="30" armature="0.01"/>'
        f'<geom name="finger_r" type="box" pos="0 -0.006 0.026" size="0.012 0.006 0.036" rgba="{fcol}" mass="0.03" {fr} {MOVING}/></body>'
        f'</body>'
    )
    cams = {
        "front": _look([0, -0.52, 0.78], [0, 0.03, 0.0]),
        "top": _look([0, -0.001, 1.05], [0, 0, 0]),
        "wide": _look([0, -1.25, 1.15], [0, 0.0, -0.1]),
        "side": _look([0.85, -0.35, 0.45], [0, 0.02, 0]),
    }
    wb += [f'<camera name="{k}" {v} fovy="45"/>' for k, v in cams.items()]
    return f"""<mujoco model="robouse-tabletop">
  <option timestep="0.002" cone="elliptic" impratio="10" integrator="implicitfast"><flag multiccd="enable"/></option>
  <compiler angle="degree"/>
  <visual><global offwidth="640" offheight="640"/><quality shadowsize="4096"/><map shadowclip="0.8"/><headlight ambient="0.35 0.35 0.35" diffuse="0.45 0.45 0.45"/></visual>
  <asset>
    <texture name="tfloor" type="2d" builtin="checker" rgb1="0.82 0.84 0.86" rgb2="0.74 0.76 0.78" width="256" height="256"/>
    <material name="floor" texture="tfloor" texrepeat="6 6"/>
    <material name="wood" rgba="0.72 0.58 0.42 1"/>
  </asset>
  <worldbody>
    <light pos="0.3 -0.4 1.4" dir="-0.2 0.3 -1" diffuse="0.55 0.55 0.55" castshadow="true"/>
    {chr(10).join(wb)}
  </worldbody>
  <contact><exclude body1="finger_l" body2="finger_r"/></contact>
  <equality><weld body1="mocap" body2="hand" solref="0.01 1"/></equality>
  <actuator>
    <position name="a_fl" joint="fl" kp="300" ctrlrange="0 {FINGER_OPEN}" forcerange="-15 15"/>
    <position name="a_fr" joint="fr" kp="300" ctrlrange="0 {FINGER_OPEN}" forcerange="-15 15"/>
  </actuator>
</mujoco>"""


def _resolved_obj(o: dict, grid: dict | None) -> dict:
    if "cell" in o and grid:
        o = dict(o)
        o["pos"] = list(_cell_xy(grid, *o["cell"]))
    return o


# --------------------------------------------------------------------------------------------------
# Backend
