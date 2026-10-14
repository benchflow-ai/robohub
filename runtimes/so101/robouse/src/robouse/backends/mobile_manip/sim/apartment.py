"""The apartment scene for the mobile-manip suite, built with MjSpec.

World frame in metres: +x east, +y north, +z up, floor at z = 0. The apartment is 8 m x 5 m (x -4..4, y -2.5..2.5)
with 2 m high walls, three rooms and two doorways:

  living room  x 0..4,  y -2.5..2.5   dining table, sofa, bookshelf (two shelves) against the east wall, waste bin
  kitchen      x -4..0, y 0..2.5      counter along the north wall with a cabinet drawer (slide joint), kitchen table
  study        x -4..0, y -2.5..0     desk against the south wall

A wall at x = 0 separates the living room from the kitchen and the study; each has a 1 m doorway in it (centred at
y = 1.4 and y = -1.4). A solid wall at y = 0 separates the kitchen from the study, so going between them means
passing through the living room. Everything is a MuJoCo collision geom except the decorative floor rug.
"""

from __future__ import annotations

import math

X0, X1, Y0, Y1 = -4.0, 4.0, -2.5, 2.5
WALL_T, WALL_H = 0.1, 2.0
DOOR_W = 1.0
DOORS = {"kitchen_door": (0.0, 1.4), "study_door": (0.0, -1.4)}
ROOMS = {"living_room": (0.0, 4.0, -2.5, 2.5), "kitchen": (-4.0, 0.0, 0.0, 2.5), "study": (-4.0, 0.0, -2.5, 0.0)}

# tables: centre (x, y), top size (x, y), top height
TABLES = {
    "dining_table": dict(c=(2.0, 1.3), size=(1.0, 0.7), h=0.75, room="living_room", rgba=(0.55, 0.38, 0.22, 1)),
    "kitchen_table": dict(c=(-3.35, 0.55), size=(0.7, 0.5), h=0.75, room="kitchen", rgba=(0.75, 0.72, 0.66, 1)),
    "desk": dict(c=(-2.4, -2.15), size=(1.2, 0.6), h=0.74, room="study", rgba=(0.42, 0.30, 0.20, 1)),
}
# kitchen counter along the north wall; the part x -2.4..-1.6 is a cabinet with a drawer under the top
COUNTER = dict(x=(-3.9, -1.2), y=(1.85, 2.45), h=0.9, top_t=0.03)
CABINET_X = (-2.4, -1.6)
DRAWER = dict(
    x=-2.0,
    front_y=1.85,
    z=0.67,
    travel=0.40,
    half_w=0.36,
    depth=0.42,
    wall_h=0.125,
    panel_half_w=0.39,
    panel_h=0.20,
    handle_off=0.07,
    handle_half=0.06,
    handle_r=0.0125,
)
# bookshelf against the east wall, open towards -x
SHELF = dict(x=(3.55, 3.95), y=(-0.45, 0.45), levels={"lower": 0.40, "upper": 0.80}, top=1.2, board_t=0.02)
BIN = dict(c=(3.3, -1.0), half=(0.2, 0.2), h=0.35, wall_t=0.015)
SOFA = dict(c=(2.2, -2.1), size=(1.6, 0.7), h=0.45)

# objects: size and mass are realistic (a 330 ml can, a 330 ml bottle, a mug-sized cup, a small cardboard box, a spice box)
OBJECT_KINDS = {
    "can": dict(shape="cylinder", r=0.033, h=0.122, mass=0.35),
    "bottle": dict(shape="cylinder", r=0.032, h=0.20, mass=0.36),
    "cup": dict(shape="cylinder", r=0.04, h=0.095, mass=0.25),
    "box": dict(shape="box", half=(0.05, 0.025, 0.07), mass=0.20),
    "spice_box": dict(shape="box", half=(0.03, 0.02, 0.05), mass=0.12),
}
COLORS = {
    "red": (0.85, 0.15, 0.12, 1),
    "green": (0.15, 0.65, 0.25, 1),
    "blue": (0.15, 0.35, 0.85, 1),
    "yellow": (0.95, 0.8, 0.1, 1),
    "white": (0.92, 0.92, 0.92, 1),
    "orange": (0.95, 0.5, 0.1, 1),
    "purple": (0.55, 0.25, 0.7, 1),
}


def object_height(kind: str) -> float:
    k = OBJECT_KINDS[kind]
    return k["h"] if k["shape"] == "cylinder" else 2 * k["half"][2]


def _box(parent, name, c, half, rgba, **kw):
    import mujoco

    return parent.add_geom(
        name=name, type=mujoco.mjtGeom.mjGEOM_BOX, pos=list(c), size=list(half), rgba=list(rgba), **kw
    )


def build_apartment(s, objects: list[dict]) -> None:
    """Add walls, furniture, the drawer and the task objects to the MjSpec `s` (compiler in radians)."""
    import mujoco

    s.visual.global_.offwidth, s.visual.global_.offheight = 1280, 960
    s.visual.headlight.ambient = [0.45, 0.45, 0.45]
    s.visual.headlight.diffuse = [0.45, 0.45, 0.45]
    s.visual.headlight.specular = [0, 0, 0]
    s.visual.map.znear = 0.01
    s.add_texture(
        name="floor",
        type=mujoco.mjtTexture.mjTEXTURE_2D,
        builtin=mujoco.mjtBuiltin.mjBUILTIN_CHECKER,
        rgb1=[0.62, 0.52, 0.40],
        rgb2=[0.58, 0.48, 0.37],
        width=256,
        height=256,
    )
    s.add_material(name="floor", textures=["", "floor"], texrepeat=[16, 10], texuniform=False)
    s.add_texture(
        name="sky",
        type=mujoco.mjtTexture.mjTEXTURE_SKYBOX,
        builtin=mujoco.mjtBuiltin.mjBUILTIN_GRADIENT,
        rgb1=[0.85, 0.86, 0.88],
        rgb2=[0.97, 0.97, 0.97],
        width=64,
        height=64,
    )
    w = s.worldbody
    w.add_light(
        name="sun", pos=[0, 0, 8], dir=[0, 0, -1], directional=True, diffuse=[0.45, 0.45, 0.45], castshadow=False
    )
    w.add_geom(
        name="floor",
        type=mujoco.mjtGeom.mjGEOM_PLANE,
        size=[X1 + 0.5, Y1 + 0.5, 0.1],
        material="floor",
        friction=[1, 0.005, 0.0001],
    )
    wall = (0.93, 0.92, 0.88, 1)
    H, T = WALL_H, WALL_T
    # outer walls
    _box(w, "wall_west", (X0 - T / 2, 0, H / 2), (T / 2, Y1 + T, H / 2), wall)
    _box(w, "wall_east", (X1 + T / 2, 0, H / 2), (T / 2, Y1 + T, H / 2), wall)
    _box(w, "wall_south", (0, Y0 - T / 2, H / 2), (X1, T / 2, H / 2), wall)
    _box(w, "wall_north", (0, Y1 + T / 2, H / 2), (X1, T / 2, H / 2), wall)
    # partition x = 0 with two doorways
    ys = sorted([Y0] + [c for _, (_, yc) in DOORS.items() for c in (yc - DOOR_W / 2, yc + DOOR_W / 2)] + [Y1])
    for i in range(0, len(ys), 2):
        a, b = ys[i], ys[i + 1]
        _box(w, f"wall_partition_{i // 2}", (0, (a + b) / 2, H / 2), (T / 2, (b - a) / 2, H / 2), wall)
    # kitchen / study wall y = 0
    _box(w, "wall_kitchen_study", ((X0 + 0) / 2 - T / 4, 0, H / 2), ((0 - X0) / 2 - T / 4, T / 2, H / 2), wall)
    # door frames (visual strips on the floor)
    for name, (x, y) in DOORS.items():
        _box(
            w,
            f"{name}_sill",
            (x, y, 0.001),
            (T / 2 + 0.02, DOOR_W / 2, 0.001),
            (0.35, 0.3, 0.25, 1),
            contype=0,
            conaffinity=0,
        )
    # tables with legs
    for name, t in TABLES.items():
        (cx, cy), (sx, sy), h = t["c"], t["size"], t["h"]
        _box(w, f"{name}_top", (cx, cy, h - 0.02), (sx / 2, sy / 2, 0.02), t["rgba"])
        for i, (dx, dy) in enumerate(((1, 1), (1, -1), (-1, 1), (-1, -1))):
            _box(
                w,
                f"{name}_leg{i}",
                (cx + dx * (sx / 2 - 0.04), cy + dy * (sy / 2 - 0.04), (h - 0.04) / 2),
                (0.025, 0.025, (h - 0.04) / 2),
                t["rgba"],
            )
    _build_counter(s)
    # sofa (seat + backrest against the south wall)
    (cx, cy), (sx, sy), h = SOFA["c"], SOFA["size"], SOFA["h"]
    _box(w, "sofa_seat", (cx, cy, h / 2), (sx / 2, sy / 2, h / 2), (0.35, 0.45, 0.6, 1))
    _box(w, "sofa_back", (cx, Y0 + 0.1, 0.45), (sx / 2, 0.1, 0.45), (0.32, 0.42, 0.56, 1))
    # bookshelf
    (x0, x1), (y0, y1) = SHELF["x"], SHELF["y"]
    bt = SHELF["board_t"]
    wood = (0.6, 0.45, 0.3, 1)
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    _box(w, "shelf_back", (x1 - 0.01, cy, SHELF["top"] / 2), (0.01, (y1 - y0) / 2, SHELF["top"] / 2), wood)
    for sgn, nm in ((-1, "shelf_side_s"), (1, "shelf_side_n")):
        _box(
            w,
            nm,
            (cx, cy + sgn * ((y1 - y0) / 2 - 0.01), SHELF["top"] / 2),
            ((x1 - x0) / 2, 0.01, SHELF["top"] / 2),
            wood,
        )
    _box(w, "shelf_base", (cx, cy, 0.025), ((x1 - x0) / 2, (y1 - y0) / 2 - 0.02, 0.025), wood)
    for lv, z in SHELF["levels"].items():
        _box(w, f"shelf_{lv}", (cx, cy, z - bt / 2), ((x1 - x0) / 2, (y1 - y0) / 2 - 0.02, bt / 2), wood)
    _box(w, "shelf_top", (cx, cy, SHELF["top"] - bt / 2), ((x1 - x0) / 2, (y1 - y0) / 2, bt / 2), wood)
    # waste bin (fixed to the floor)
    (bx, by), (hx, hy), bh, wt = BIN["c"], BIN["half"], BIN["h"], BIN["wall_t"]
    grey = (0.3, 0.32, 0.35, 1)
    _box(w, "bin_bottom", (bx, by, 0.015), (hx, hy, 0.015), grey)
    _box(w, "bin_wall_w", (bx - hx + wt / 2, by, bh / 2), (wt / 2, hy, bh / 2), grey)
    _box(w, "bin_wall_e", (bx + hx - wt / 2, by, bh / 2), (wt / 2, hy, bh / 2), grey)
    _box(w, "bin_wall_s", (bx, by - hy + wt / 2, bh / 2), (hx, wt / 2, bh / 2), grey)
    _box(w, "bin_wall_n", (bx, by + hy - wt / 2, bh / 2), (hx, wt / 2, bh / 2), grey)
    # a rug (visual only)
    _box(w, "rug", (2.0, -0.3, 0.004), (1.0, 0.7, 0.001), (0.7, 0.3, 0.25, 1), contype=0, conaffinity=0)
    # cameras
    from ...embodied import lookat_xyaxes

    def cam(name, pos, target, fovy):
        w.add_camera(name=name, pos=list(pos), xyaxes=[float(v) for v in lookat_xyaxes(pos, target).split()], fovy=fovy)

    w.add_camera(name="overview", pos=[0, 0, 10.5], xyaxes=[1, 0, 0, 0, 1, 0], fovy=45)
    cam("living_room_cam", (0.3, -2.3, 1.95), (2.3, 0.6, 0.4), 70)
    cam("kitchen_cam", (-0.3, 0.2, 1.95), (-2.4, 1.8, 0.5), 70)
    cam("study_cam", (-0.3, -0.2, 1.95), (-2.4, -1.9, 0.5), 70)
    # task objects
    for o in objects:
        _add_object(w, o)


def _build_counter(s) -> None:
    import mujoco

    w = s.worldbody
    (x0, x1), (y0, y1), h, tt = COUNTER["x"], COUNTER["y"], COUNTER["h"], COUNTER["top_t"]
    cab0, cab1 = CABINET_X
    body = (0.9, 0.9, 0.88, 1)
    top = (0.35, 0.35, 0.38, 1)
    yc, hy = (y0 + y1) / 2, (y1 - y0) / 2
    _box(w, "counter_top", ((x0 + x1) / 2, yc, h - tt / 2), ((x1 - x0) / 2, hy, tt / 2), top)
    _box(w, "counter_west", ((x0 + cab0) / 2, yc, (h - tt) / 2), ((cab0 - x0) / 2, hy, (h - tt) / 2), body)
    _box(w, "counter_east", ((cab1 + x1) / 2, yc, (h - tt) / 2), ((x1 - cab1) / 2, hy, (h - tt) / 2), body)
    # cabinet: lower block, two side panels, a back panel; the drawer slides in the cavity z 0.66..0.87
    dz = DRAWER["z"]
    _box(w, "cabinet_lower", ((cab0 + cab1) / 2, yc, (dz - 0.01) / 2), ((cab1 - cab0) / 2, hy, (dz - 0.01) / 2), body)
    cav_lo, cav_hi = dz - 0.01, h - tt
    for sgn, nm in ((-1, "cabinet_side_w"), (1, "cabinet_side_e")):
        _box(
            w,
            nm,
            ((cab0 + cab1) / 2 + sgn * ((cab1 - cab0) / 2 - 0.01), yc, (cav_lo + cav_hi) / 2),
            (0.01, hy, (cav_hi - cav_lo) / 2),
            body,
        )
    _box(
        w,
        "cabinet_back",
        ((cab0 + cab1) / 2, y1 - 0.01, (cav_lo + cav_hi) / 2),
        ((cab1 - cab0) / 2, 0.01, (cav_hi - cav_lo) / 2),
        body,
    )
    # the drawer: a slide joint along -y with a 0.40 m travel, a vertical bar handle on its front
    D = DRAWER
    dr = w.add_body(name="drawer", pos=[D["x"], D["front_y"], dz])
    dr.add_joint(
        name="drawer_slide",
        type=mujoco.mjtJoint.mjJNT_SLIDE,
        axis=[0, -1, 0],
        range=[0, D["travel"]],
        damping=6.0,
        frictionloss=2.0,
        armature=0.2,
    )
    face = (0.82, 0.8, 0.76, 1)
    inner = (0.7, 0.62, 0.5, 1)
    hw, dep, wh = D["half_w"], D["depth"], D["wall_h"]
    _box(dr, "drawer_front", (0, -0.01, D["panel_h"] / 2), (D["panel_half_w"], 0.01, D["panel_h"] / 2), face, mass=1.0)
    _box(dr, "drawer_bottom", (0, dep / 2, 0.005), (hw, dep / 2, 0.005), inner, mass=0.6)
    _box(dr, "drawer_wall_w", (-hw + 0.005, dep / 2, wh / 2), (0.005, dep / 2, wh / 2), inner, mass=0.25)
    _box(dr, "drawer_wall_e", (hw - 0.005, dep / 2, wh / 2), (0.005, dep / 2, wh / 2), inner, mass=0.25)
    _box(dr, "drawer_wall_n", (0, dep - 0.005, wh / 2), (hw, 0.005, wh / 2), inner, mass=0.25)
    zc = D["panel_h"] / 2
    steel = (0.75, 0.76, 0.78, 1)
    dr.add_geom(
        name="drawer_handle",
        type=mujoco.mjtGeom.mjGEOM_CYLINDER,
        pos=[0, -D["handle_off"], zc],
        size=[D["handle_r"], D["handle_half"], 0],
        rgba=list(steel),
        mass=0.1,
        friction=[1.2, 0.01, 0.001],
    )
    for sgn in (-1, 1):
        _box(
            dr,
            f"drawer_handle_post{'n' if sgn > 0 else 's'}",
            (0, -D["handle_off"] / 2 - 0.005, zc + sgn * (D["handle_half"] - 0.008)),
            (0.007, D["handle_off"] / 2 - 0.005, 0.007),
            steel,
            mass=0.02,
        )


def _add_object(w, o: dict):
    import mujoco

    k = OBJECT_KINDS[o["kind"]]
    x, y, z = o["pos"]
    yaw = math.radians(o.get("yaw", 0.0))
    b = w.add_body(
        name=f"obj_{o['name']}",
        pos=[x, y, z + object_height(o["kind"]) / 2 + 0.002],
        quat=[math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)],
    )
    b.add_freejoint(name=f"obj_{o['name']}_free")
    rgba = list(COLORS[o["color"]])
    common = dict(rgba=rgba, mass=k["mass"], friction=[1.0, 0.005, 0.0001], condim=4)
    if k["shape"] == "cylinder":
        b.add_geom(name=f"obj_{o['name']}", type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=[k["r"], k["h"] / 2, 0], **common)
    else:
        b.add_geom(name=f"obj_{o['name']}", type=mujoco.mjtGeom.mjGEOM_BOX, size=list(k["half"]), **common)
    return b


# ------------------------------------------------------------------------------------------------------------------
# public facts for `robo observe` and region tests for the judge
# ------------------------------------------------------------------------------------------------------------------


def table_rect(name: str) -> tuple[float, float, float, float, float]:
    t = TABLES[name]
    (cx, cy), (sx, sy) = t["c"], t["size"]
    return cx - sx / 2, cx + sx / 2, cy - sy / 2, cy + sy / 2, t["h"]


def surfaces() -> dict:
    """Horizontal surfaces objects can rest on: name -> (xmin, xmax, ymin, ymax, top_z, geom name)."""
    out = {n: (*table_rect(n), f"{n}_top") for n in TABLES}
    (x0, x1), (y0, y1) = COUNTER["x"], COUNTER["y"]
    out["counter"] = (x0, x1, y0, y1, COUNTER["h"], "counter_top")
    (sx0, sx1), (sy0, sy1) = SHELF["x"], SHELF["y"]
    for lv, z in SHELF["levels"].items():
        out[f"shelf_{lv}"] = (sx0, sx1 - 0.02, sy0 + 0.02, sy1 - 0.02, z, f"shelf_{lv}")
    return out


def layout(drawer_open: float, drawer_handle: list[float]) -> dict:
    """Public positions of the rooms, doorways and furniture (all numbers in metres, world frame)."""
    r2 = lambda v: [round(float(x), 3) for x in v]
    furniture = {}
    for n, t in TABLES.items():
        furniture[n] = {"room": t["room"], "top_center": r2([*t["c"], t["h"]]), "top_size": r2(t["size"])}
    (x0, x1), (y0, y1) = COUNTER["x"], COUNTER["y"]
    furniture["counter"] = {
        "room": "kitchen",
        "top_center": r2([(x0 + x1) / 2, (y0 + y1) / 2, COUNTER["h"]]),
        "top_size": r2([x1 - x0, y1 - y0]),
        "front_y": y0,
        "note": "along the north wall; its front face is at y = 1.85",
    }
    D = DRAWER
    furniture["drawer"] = {
        "room": "kitchen",
        "opening_m": round(float(drawer_open), 3),
        "max_opening_m": D["travel"],
        "handle": r2(drawer_handle),
        "handle_kind": "vertical bar, 0.12 m long, 0.025 m thick, its axis 0.07 m in front of the drawer face",
        "pull_direction": [0, -1, 0],
        "interior_size": r2([2 * D["half_w"] - 0.02, D["depth"] - 0.02, D["wall_h"]]),
        "interior_floor_z": round(D["z"] + 0.01, 3),
        "interior_center": r2([D["x"], D["front_y"] - drawer_open + D["depth"] / 2, D["z"] + 0.01]),
        "note": "in the counter cabinet under the counter top (x -2.4..-1.6); pull the handle towards -y to open",
    }
    (sx0, sx1), (sy0, sy1) = SHELF["x"], SHELF["y"]
    furniture["bookshelf"] = {
        "room": "living_room",
        "x_range": r2([sx0, sx1]),
        "y_range": r2([sy0, sy1]),
        "open_side": "-x (west)",
        "shelf_heights": {k: v for k, v in SHELF["levels"].items()},
        "height": SHELF["top"],
        "note": "against the east wall; shelves `lower` (top surface z = 0.40) and `upper` (z = 0.80), 0.38 m deep",
    }
    (bx, by), (hx, hy) = BIN["c"], BIN["half"]
    furniture["bin"] = {
        "room": "living_room",
        "center": r2([bx, by, 0]),
        "inner_size": r2([2 * hx - 0.03, 2 * hy - 0.03]),
        "rim_height": BIN["h"],
        "note": "open-top waste bin fixed to the floor",
    }
    (cx, cy), (sx, sy) = SOFA["c"], SOFA["size"]
    furniture["sofa"] = {"room": "living_room", "center": r2([cx, cy]), "size": r2([sx, sy]), "height": SOFA["h"]}
    return {
        "rooms": {k: {"x_range": [a, b], "y_range": [c, d]} for k, (a, b, c, d) in ROOMS.items()},
        "doorways": {k: {"center": [x, y], "width": DOOR_W, "wall": "x = 0"} for k, (x, y) in DOORS.items()},
        "walls": (
            "outer walls enclose x -4..4, y -2.5..2.5 (2 m high, 0.1 m thick); a wall at x = 0 has the two doorways; "
            "a solid wall at y = 0 for x < 0 separates the kitchen (north) from the study (south)"
        ),
        "furniture": furniture,
    }


def in_rect(p, x0, x1, y0, y1, margin: float = 0.0) -> bool:
    return x0 + margin <= p[0] <= x1 - margin and y0 + margin <= p[1] <= y1 - margin


def drawer_interior_contains(p, opening: float) -> bool:
    D = DRAWER
    y_front = D["front_y"] - opening
    return (
        abs(p[0] - D["x"]) <= D["half_w"] - 0.01
        and y_front <= p[1] <= y_front + D["depth"] - 0.01
        and D["z"] <= p[2] <= D["z"] + D["wall_h"] + 0.05
    )


def bin_contains(p) -> bool:
    (bx, by), (hx, hy) = BIN["c"], BIN["half"]
    return abs(p[0] - bx) <= hx - BIN["wall_t"] and abs(p[1] - by) <= hy - BIN["wall_t"] and p[2] <= BIN["h"]
