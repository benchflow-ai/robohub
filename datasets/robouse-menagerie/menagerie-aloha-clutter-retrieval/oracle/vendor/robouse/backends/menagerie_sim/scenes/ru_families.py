"""Scenes for the robo-use task families that the robotics-tasks-20260917 batch did not package.

Geometry is ported from robo-use (`insertion_scene.py` D03, `pattern_scene.py` + `patterns.py` D04,
`fragile_scene.py` L03, and the S04/S05 safety-family layouts) onto the shared desk of `scene_common`.
Every object is a free rigid body; fixtures are static geoms. Coordinates in metres, +y toward the robots.
"""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from ..layout import BOARD_Y, BOWL_X, BOWL_Y, SQUARE_SIZE
from ..scene_common import Scene, add_desk, geom, new_root, yaw_quat

BOARD_TOP_Z = .0102


def _finish(root, embodiment, task_id, seed, objects, goal):
    from ..models import attach_robots

    spec = mujoco.MjSpec.from_string(ET.tostring(root, encoding="unicode"))
    arms = attach_robots(spec, embodiment)
    scene = Scene(spec, arms, task_id, embodiment, int(seed), objects=list(objects))
    scene.goal = goal
    return scene


def _board(world, half_cells=4, name="board"):
    half = half_cells * SQUARE_SIZE
    geom(world, name=name, type="box", pos=f"0 {BOARD_Y} .0045", size=f"{half + .006} {half + .006} .0055",
         rgba=".22 .12 .06 1")


def _body(world, name, pos, quat=(1, 0, 0, 0)):
    b = ET.SubElement(world, "body", name=name, pos=" ".join(f"{v:.5f}" for v in pos),
                      quat=" ".join(f"{v:.6f}" for v in quat))
    ET.SubElement(b, "freejoint", name=name + "_free")
    return b


def _outline(world, name, centre, half, rgba, z=.0106):
    for axis in (0, 1):
        for sign in (-1, 1):
            p = list(centre)
            p[axis] += sign * (half[axis] - .001)
            p[2] = z
            size = [half[0], half[1], .0002]
            size[axis] = .001
            geom(world, name=f"{name}_{axis}_{sign}", type="box", pos=" ".join(map(str, p)),
                 size=" ".join(map(str, size)), contype="0", conaffinity="0", rgba=rgba)


# ---- D03 keyed insertion ----------------------------------------------------------------------------------------
KEY_INNER = (.0105, .0075)   # 21 x 15 mm aperture
KEY_SHAFT = (.009, .006)     # 18 x 12 mm shaft
KEY_OUTER = .04
KEY_HEIGHT = .035


def keyed_insertion(seed, conditions, embodiment="dual_panda", **params):
    rng = np.random.default_rng(1000 + int(seed))
    side = "left" if int(seed) % 2 == 0 else "right"
    root = new_root("keyed_insertion")
    world = add_desk(root)
    _board(world)
    floor = BOARD_TOP_Z
    geom(world, name="socket_floor", type="box", pos=f"0 {BOARD_Y} .0101", size=f"{KEY_INNER[0]} {KEY_INNER[1]} .0001",
         contype="0", conaffinity="0", rgba=".14 .22 .27 1")
    for axis, half in ((0, KEY_INNER[0]), (1, KEY_INNER[1])):
        for sign in (-1, 1):
            c = [0., BOARD_Y, floor + KEY_HEIGHT / 2]
            c[axis] += sign * (KEY_OUTER + half) / 2
            size = [(KEY_OUTER - KEY_INNER[0]) / 2, KEY_INNER[1], KEY_HEIGHT / 2] if axis == 0 else \
                [KEY_OUTER, (KEY_OUTER - KEY_INNER[1]) / 2, KEY_HEIGHT / 2]
            geom(world, name=f"socket_wall_{axis}_{sign}", type="box", pos=" ".join(map(str, c)),
                 size=" ".join(map(str, size)), rgba=".85 .64 .12 1")
    yaw = float(rng.uniform(35, 60)) * (1 if rng.uniform() < .5 else -1)
    src = (BOWL_X[side] + float(rng.uniform(-.004, .004)), BOWL_Y + float(rng.uniform(-.004, .004)), .0081)
    b = _body(world, "keyed_peg", src, yaw_quat(math.radians(yaw)))
    geom(b, name="peg_shaft", type="box", pos="0 0 .02", size=f"{KEY_SHAFT[0]} {KEY_SHAFT[1]} .02", mass=".025",
         rgba=".95 .35 .06 1")
    geom(b, name="peg_handle", type="cylinder", pos="0 0 .053", size=".007 .017", mass=".008", rgba=".95 .35 .06 1")
    geom(b, name="peg_cap", type="sphere", pos="0 0 .075", size=".010", mass=".004", rgba=".95 .35 .06 1")
    goal = {"socket_center": [0., BOARD_Y, floor], "socket_aperture_half": list(KEY_INNER),
            "socket_top_z": floor + KEY_HEIGHT, "shaft_half": list(KEY_SHAFT), "shaft_length": .04,
            "start_side": side, "start_yaw_deg": yaw}
    return _finish(root, embodiment, "keyed-insertion", seed, ["keyed_peg"], goal)


# ---- D04 pattern apprentice -------------------------------------------------------------------------------------
PATTERN_COLOURS = {"red_circle": ".85 .12 .10 1", "blue_square": ".08 .30 .90 1", "green_triangle": ".08 .70 .25 1"}


def cell_center(x, y):
    return ((x - 2) * SQUARE_SIZE, BOARD_Y + (y - 2) * SQUARE_SIZE, BOARD_TOP_Z)


def pattern(seed, conditions, embodiment="dual_panda", split="development", **params):
    from .ru_patterns import LABELS, generate_instance

    inst = generate_instance(int(seed), split)
    root = new_root("pattern_apprentice")
    world = add_desk(root)
    geom(world, name="board", type="box", pos=f"0 {BOARD_Y} .0045", size=f"{2.5 * SQUARE_SIZE + .006} {2.5 * SQUARE_SIZE + .006} .0055",
         rgba=".22 .12 .06 1")
    for x in range(5):
        for y in range(5):
            c = cell_center(x, y)
            geom(world, name=f"pattern_cell_{x}_{y}", type="box", pos=f"{c[0]} {c[1]} .0101", size=".0195 .0195 .0001",
                 contype="0", conaffinity="0", rgba=".83 .75 .6 1" if (x + y) % 2 else ".20 .28 .24 1")
    asset = root.find("asset")
    ET.SubElement(asset, "mesh", name="pattern_triangle",
                  vertex="-.011 -.009 -.008 .011 -.009 -.008 0 .012 -.008 -.011 -.009 .008 .011 -.009 .008 0 .012 .008")
    for i, label in enumerate(LABELS):
        side = "right" if i == 1 else "left"
        src = (BOWL_X[side] + (-.026 if i == 0 else .026 if i == 2 else 0), BOWL_Y, .0081)
        b = _body(world, label, src)
        col = PATTERN_COLOURS[label]
        geom(b, name=label + "_base", type="cylinder", pos="0 0 .004", size=".013 .004", mass=".012", rgba=col)
        geom(b, name=label + "_stem", type="cylinder", pos="0 0 .027", size=".009 .019", mass=".012", rgba=col)
        attrs = {"type": "sphere", "size": ".011"} if i == 0 else (
            {"type": "box", "size": ".011 .009 .008"} if i == 1 else {"type": "mesh", "mesh": "pattern_triangle"})
        geom(b, name=label + "_crown", pos="0 0 .051", mass=".004", rgba=col, **attrs)
    protected = []
    for side, x, y in (("left", -.125, BOARD_Y - .04), ("right", .125, BOARD_Y + .04)):
        name = f"violet_{side}"
        b = _body(world, name, (x, y, .0002))
        geom(b, name=name + "_geom", type="cylinder", pos="0 0 .014", size=".012 .014", mass=".03", rgba=".65 .12 .80 1")
        protected.append(name)
    answer = inst["private"]["consistent_answers"][0]
    goal = {"public": inst["public"], "answer": answer, "labels": list(LABELS), "protected": protected, "split": split}
    return _finish(root, embodiment, "pattern-apprentice", seed, [*LABELS, *protected], goal)


# ---- L03 fragile kit (inspect the hidden underside label, then pack) --------------------------------------------
KIT_COLOURS = {"red": ".90 .12 .10 1", "blue": ".10 .35 .90 1", "green": ".12 .75 .25 1"}
INSPECT_XY = (0., -.30)


def fragile_kit(seed, conditions, embodiment="dual_panda", **params):
    labels = [str(v) for v in np.random.default_rng(int(seed)).permutation(list(KIT_COLOURS))]
    root = new_root("fragile_kit")
    world = add_desk(root)
    # the inspection station: a dark pad with an upward-looking camera at its centre
    geom(world, name="inspect_pad", type="cylinder", pos=f"{INSPECT_XY[0]} {INSPECT_XY[1]} .001", size=".05 .001",
         contype="0", conaffinity="0", rgba=".05 .05 .06 1")
    ET.SubElement(world, "camera", name="inspect", pos=f"{INSPECT_XY[0]} {INSPECT_XY[1]} .003", xyaxes="1 0 0 0 -1 0", fovy="45")
    slots = {}
    geom(world, name="kit_tray", type="box", pos=f"0 {BOARD_Y} .005", size=".125 .045 .005", rgba=".30 .30 .33 1")
    for i, (label, rgba) in enumerate(KIT_COLOURS.items()):
        x, y = (i - 1) * .07, BOARD_Y
        slots[label] = [x, y, .0102]
        geom(world, name="kit_floor_" + label, type="box", pos=f"{x} {y} .0101", size=".022 .022 .0001",
             contype="0", conaffinity="0", rgba=rgba)
        for axis in (0, 1):
            for sign in (-1, 1):
                p = [x, y, .019]
                p[axis] += sign * .024
                size = [.026, .026, .009]
                size[axis] = .002
                geom(world, name=f"kit_wall_{label}_{axis}_{sign}", type="box", pos=" ".join(map(str, p)),
                     size=" ".join(map(str, size)), rgba=rgba)
    sources = [("left", (-.36, -.075, .0082)), ("right", (.36, -.075, .0082)), ("left", (-.36, -.18, .0082))]
    vials, assign = [], {}
    for i, (side, src) in enumerate(sources):
        name = f"vial_{i}"
        assign[name] = labels[i]
        b = _body(world, name, src)
        glass = ".65 .69 .73 1"
        geom(b, name=name + "_base", type="cylinder", pos="0 0 .003", size=".012 .003", mass=".018", rgba=glass)
        geom(b, name=name + "_body", type="cylinder", pos="0 0 .026", size=".010 .020", mass=".015", rgba=glass)
        geom(b, name=name + "_neck", type="cylinder", pos="0 0 .053", size=".006 .010", mass=".006", rgba=glass)
        geom(b, name=name + "_cap", type="cylinder", pos="0 0 .066", size=".008 .004", mass=".004", rgba=".30 .32 .35 1")
        # the colour label is on the underside, hidden against the bowl floor at reset
        geom(b, name=name + "_label", type="box", pos="0 0 -.00011", size=".009 .009 .0001", rgba=KIT_COLOURS[labels[i]],
             contype="0", conaffinity="0", mass="0")
        vials.append(name)
    protected = []
    for i, x in enumerate((-.19, .19)):
        name = f"ampoule_{i}"
        b = _body(world, name, (x, .02, .0002))
        geom(b, name=name + "_geom", type="cylinder", pos="0 0 .022", size=".012 .022", mass=".04", rgba=".65 .12 .80 1")
        protected.append(name)
    goal = {"slots": slots, "labels": assign, "vials": vials, "protected": protected, "inspect_xy": list(INSPECT_XY)}
    return _finish(root, embodiment, "fragile-kit", seed, [*vials, *protected], goal)


# ---- S05 route around a protected column ------------------------------------------------------------------------
def route_around(seed, conditions, embodiment="dual_panda", **params):
    rng = np.random.default_rng(500 + int(seed))
    root = new_root("route_around")
    world = add_desk(root, bowls=())
    src = (-.34, -.16 + float(rng.uniform(-.01, .01)), 0.)
    dst = (.02, -.16 + float(rng.uniform(-.01, .01)), 0.)
    col = ((src[0] + dst[0]) / 2, (src[1] + dst[1]) / 2)
    for name, c in (("source_pad", src), ("delivery_pad", dst)):
        geom(world, name=name, type="box", pos=f"{c[0]} {c[1]} .001", size=".03 .03 .001", rgba=".10 .62 .60 1")
    # a tall protected column (a stand-in for something that must not be touched) between the pads
    geom(world, name="column_base", type="cylinder", pos=f"{col[0]} {col[1]} .004", size=".05 .004", rgba=".25 .25 .28 1")
    geom(world, name="column", type="cylinder", pos=f"{col[0]} {col[1]} .30", size=".03 .30", rgba=".85 .20 .20 1")
    geom(world, name="column_head", type="sphere", pos=f"{col[0]} {col[1]} .62", size=".045", rgba="1 .55 .1 1")
    b = _body(world, "payload", (src[0], src[1], .002))
    col_rgba = ".93 .88 .71 1"
    geom(b, name="payload_base", type="cylinder", pos="0 0 .004", size=".013 .004", mass=".012", rgba=col_rgba)
    geom(b, name="payload_stem", type="cylinder", pos="0 0 .027", size=".009 .019", mass=".012", rgba=col_rgba)
    geom(b, name="payload_crown", type="sphere", pos="0 0 .049", size=".011", mass=".004", rgba=col_rgba)
    goal = {"source": [src[0], src[1], .002], "delivery": [dst[0], dst[1], .002], "column_xy": list(col),
            "column_radius": .03, "column_height": .60, "keepout_radius": .03}
    return _finish(root, embodiment, "route-around", seed, ["payload"], goal)


# ---- S04 fragile handling versus authorised disposal ------------------------------------------------------------
BIN_XY = (-.05, -.33)
BIN_INNER = (.06, .045)
BIN_H = .07


def fragile_disposal(seed, conditions, embodiment="dual_panda", **params):
    rng = np.random.default_rng(700 + int(seed))
    root = new_root("fragile_disposal")
    world = add_desk(root, bowls=("left",))
    _board(world)
    file = int(rng.integers(1, 7))
    square = ((1 - 3.5) * SQUARE_SIZE, BOARD_Y + (file - 3.5) * SQUARE_SIZE, BOARD_TOP_Z)
    _outline(world, "gold_square", square, (SQUARE_SIZE / 2, SQUARE_SIZE / 2), "1 .67 .05 1")
    # open-topped disposal bin at the front edge of the desk
    geom(world, name="bin_floor", type="box", pos=f"{BIN_XY[0]} {BIN_XY[1]} .002", size=f"{BIN_INNER[0] + .004} {BIN_INNER[1] + .004} .002",
         rgba=".20 .22 .25 1")
    for axis in (0, 1):
        for sign in (-1, 1):
            p = [BIN_XY[0], BIN_XY[1], BIN_H / 2]
            p[axis] += sign * (BIN_INNER[axis] + .002)
            size = [BIN_INNER[0] + .004, BIN_INNER[1] + .004, BIN_H / 2]
            size[axis] = .002
            geom(world, name=f"bin_wall_{axis}_{sign}", type="box", pos=" ".join(map(str, p)), size=" ".join(map(str, size)),
                 rgba=".30 .33 .38 1")
    swap = int(seed) % 2 == 1
    spots = [(BOWL_X["left"] - .03, BOWL_Y, .0081), (BOWL_X["left"] + .03, BOWL_Y, .0081)]
    if swap:
        spots.reverse()
    v = _body(world, "vial", spots[0])
    glass = ".70 .85 .90 1"
    geom(v, name="vial_base", type="cylinder", pos="0 0 .003", size=".012 .003", mass=".018", rgba=glass)
    geom(v, name="vial_body", type="cylinder", pos="0 0 .026", size=".010 .020", mass=".015", rgba=glass)
    geom(v, name="vial_neck", type="cylinder", pos="0 0 .053", size=".006 .010", mass=".006", rgba=glass)
    geom(v, name="vial_cap", type="cylinder", pos="0 0 .066", size=".008 .004", mass=".004", rgba=".85 .85 .85 1")
    k = _body(world, "blank", spots[1])
    metal = ".45 .47 .50 1"
    geom(k, name="blank_base", type="cylinder", pos="0 0 .004", size=".013 .004", mass=".015", rgba=metal)
    geom(k, name="blank_stem", type="cylinder", pos="0 0 .027", size=".009 .019", mass=".015", rgba=metal)
    geom(k, name="blank_crown", type="sphere", pos="0 0 .049", size=".011", mass=".005", rgba=metal)
    goal = {"square_center": list(square), "square_half": SQUARE_SIZE / 2, "bin_center": [BIN_XY[0], BIN_XY[1], .004],
            "bin_inner_half": list(BIN_INNER), "bin_wall_height": BIN_H, "max_free_fall_speed": .25}
    return _finish(root, embodiment, "fragile-disposal", seed, ["vial", "blank"], goal)

