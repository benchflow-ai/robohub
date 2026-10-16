"""The so101 suite's tasks: layout (props), rig, step budget and success predicate of each. The predicates read the
simulated state through the backend (backend.py) and run in the trusted episode server; `holds` names conditions
that must stay true for a time (`held_for`), `track` follows a task's stages (the ruler handover).

Twins of Robo Use's real SO-101 tasks (tasks/real): so101-reach is real-so101-reach (the same point; 1.5 cm instead
of 2 cm) and so101-block-lift is real-so101-block-lift (the same 2.5 cm cube at the same place). The two-arm tasks
use the rig of the stationerybench suite; so101-bimanual-ruler-handover is a simulated StationeryBench ruler handover.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field

import numpy as np

from . import scene as S

HOLD_S = 0.99  # "held for 1 s" (10 control steps at 10 Hz; the float tolerance of the simulated clock)
LIFT_CUBE = 0.05
LIFT_BOX = 0.04
BOX_TILT = 10.0
REACH_TOL = 0.015
STACK_XY = 0.01
DRAWER_OPEN = 0.04


@dataclass
class Task:
    id: str
    robot: str  # "so101" | "koch"
    rig: str  # "single" | "bimanual"
    props: list[dict]
    max_steps: int
    check: Callable  # backend -> (ok, why)
    holds: Callable = field(default=lambda b: {})  # backend -> {key: bool}, timed by the backend
    track: Callable = field(default=lambda b: None)  # backend -> None (stage tracking)
    progress: Callable | None = None  # backend -> stage as a fraction, reported as `progress`
    start: list[float] | None = None  # start pose of the arm joints (deg); default: the robot's home
    start_grip: float = 2.0
    code: list[str] | None = None  # the buttons to press, in order (the button panel; a name twice = press it twice)


def _free(b, name: str) -> bool:
    return not b.robot_touches(name)


# ---- predicates ------------------------------------------------------------------------------------------------------


def reach_check(b):
    p = b.tool("arm")[0]
    goal = np.asarray(next(x for x in b.task.props if x["kind"] == "marker")["pos"], float)
    d = float(np.linalg.norm(p - goal))
    return d <= REACH_TOL, f"tool point {d * 100:.1f} cm from the marked point (needs <= {REACH_TOL * 100:.1f} cm)"


def lift_holds(b):
    return {"lifted": b.obj_bottom("cube") >= LIFT_CUBE and not b.on_desk("cube")}


def lift_check(b):
    t, z = b.held_for("lifted"), b.obj_bottom("cube")
    return t >= HOLD_S, f"cube bottom {z * 100:.1f} cm above the desk, lifted >= 5 cm for {t:.1f} s (needs 1 s)"


def cube_in_cup_check(b):
    inside, free = b.in_cup("cube", "cup"), _free(b, "cube")
    return inside and free, f"cube in the cup: {inside}; cube free of the gripper: {free}"


def stack_check(b, top: str = "red_cube", bottom: str = "blue_cube"):
    pt, pb = b.obj_pos(top), b.obj_pos(bottom)
    dxy = np.abs((pt - pb)[:2])
    dz = float(pt[2] - pb[2])
    ok = (
        bool(np.all(dxy <= STACK_XY))
        and abs(dz - S.CUBE) < 0.004
        and b.touching(b.obj_geoms[top], b.obj_geoms[bottom])
        and not b.on_desk(top)
        and b.on_desk(bottom)
        and _free(b, top)
        and _free(b, bottom)
    )
    return ok, (
        f"{top} offset from {bottom} x {dxy[0] * 100:.1f} cm, y {dxy[1] * 100:.1f} cm (needs <= 1 cm each), "
        f"height difference {dz * 100:.1f} cm (a cube is 2.5 cm), both free of the gripper: "
        f"{_free(b, top) and _free(b, bottom)}"
    )


def sort_check(b):
    """Every cube with a tray of its colour is in that tray, free of the gripper; a cube of another colour (the green
    one) is in neither tray."""
    trays = [n for n, p in b.obj.items() if p["kind"] == "tray"]
    bad, stray = [], []
    for n, p in b.obj.items():
        if p["kind"] != "cube":
            continue
        tray = f"{p['color']}_tray"
        if tray in trays:
            if not (b.in_tray(n, tray) and _free(b, n)):
                bad.append(n)
        elif any(b.in_tray(n, t) for t in trays):
            stray.append(n)
    ok = not bad and not stray
    why = "every red and blue cube in the tray of its colour" if not bad else f"not in their tray (or held): {bad}"
    why += f"; in a tray but should not be: {stray}" if stray else "; the green cube in neither tray"
    return ok, why


def puck_check(b):
    zone = next(x for x in b.task.props if x["kind"] == "zone")
    d = float(np.linalg.norm(b.obj_pos("puck")[:2] - np.asarray(zone["pos"][:2])))
    ok = d <= S.ZONE_R and b.obj_tilt("puck") < 20 and b.on_desk("puck")
    return ok, f"puck centre {d * 100:.1f} cm from the zone's centre (needs <= {S.ZONE_R * 100:.0f} cm)"


def drawer_check(b):
    q = b.drawer_opening()
    return q >= DRAWER_OPEN, f"drawer open {q * 100:.1f} cm (needs >= {DRAWER_OPEN * 100:.0f} cm)"


def code_check(b):
    n = len(b.task.code)
    return b.unlocked, (
        f"all {n} presses entered in order" if b.unlocked else f"{len(b.entered)} of {n} presses entered in order"
    )


# ruler handover: stages 1 left arm holds it alone in the air, 2 both arms hold it, 3 right arm holds it alone
def handover_track(b):
    lh, rh = b.holding("left") == "ruler", b.holding("right") == "ruler"
    air = not b.on_desk("ruler")
    lt, rt = b.robot_touches("ruler", "left"), b.robot_touches("ruler", "right")
    if b.stage == 0 and lh and not rt and air:
        b.stage = 1
    elif b.stage == 1 and lh and rh and air:
        b.stage = 2
    elif b.stage == 2 and rh and not lt and air:
        b.stage = 3


def handover_check(b):
    tray = "tray"
    in_tray = b.in_tray("ruler", tray) or _ruler_on_tray(b, tray)
    rests = b.touching(b.obj_geoms["ruler"], b.obj_geoms[tray]) and not b.on_desk("ruler")
    opened = all(b.measured(p)[5] >= 30 for p in ("left", "right"))
    free = _free(b, "ruler")
    ok = b.stage >= 3 and in_tray and rests and opened and free
    return ok, (
        f"handover stage {b.stage}/3 (1 left arm lifts it, 2 both hold it, 3 right arm holds it alone); ruler's "
        f"centre over the tray: {in_tray}; resting on the tray, off the desk: {rests}; both grippers open: {opened}; "
        f"ruler free of both grippers: {free}"
    )


def _ruler_on_tray(b, tray: str) -> bool:
    """The ruler's centre is over the tray's floor (the 15 cm ruler is longer than the 12 cm tray, so it may rest on
    the tray's end walls)."""
    tp, R = b.obj_pos(tray), b.obj_xmat(tray)
    loc = R.T @ (b.obj_pos("ruler") - tp)
    return abs(loc[0]) < S.TRAY_L / 2 - S.TRAY_WALL and abs(loc[1]) < S.TRAY_W / 2 - S.TRAY_WALL and loc[2] < 0.04


def handover_progress(b):
    return round(b.stage / 3, 3)


def box_holds(b):
    both = b.robot_touches("box", "left") and b.robot_touches("box", "right")
    return {"lifted": both and b.obj_bottom("box") >= LIFT_BOX and b.obj_tilt("box") < BOX_TILT}


def box_check(b):
    t = b.held_for("lifted")
    return t >= HOLD_S, (
        f"box bottom {b.obj_bottom('box') * 100:.1f} cm above the desk (needs >= 4), tilt {b.obj_tilt('box'):.1f} deg "
        f"(needs < 10), held by both arms; all of it for {t:.1f} s (needs 1 s)"
    )


# ---- layouts ---------------------------------------------------------------------------------------------------------


def cube(name, color, x, y, yaw=0.0):
    return {"kind": "cube", "name": name, "color": color, "pos": [x, y], "yaw": yaw}


TASKS: dict[str, Task] = {}


def _add(t: Task) -> None:
    TASKS[t.id] = t


_add(
    Task(
        "so101-reach",
        "so101",
        "single",
        [{"kind": "marker", "name": "marker", "pos": [0.20, 0.05, 0.08]}],
        300,
        reach_check,
    )
)
_add(Task("so101-block-lift", "so101", "single", [cube("cube", "red", 0.22, -0.05)], 600, lift_check, holds=lift_holds))
_add(
    Task(
        "so101-cube-in-cup",
        "so101",
        "single",
        [cube("cube", "red", 0.20, -0.08, 12.0), {"kind": "cup", "name": "cup", "color": "white", "pos": [0.19, 0.09]}],
        800,
        cube_in_cup_check,
    )
)
_add(
    Task(
        "so101-stack-two-cubes",
        "so101",
        "single",
        [cube("red_cube", "red", 0.17, 0.08, -8.0), cube("blue_cube", "blue", 0.24, -0.06, 15.0)],
        800,
        stack_check,
    )
)
_add(
    Task(
        "so101-sort-by-color",
        "so101",
        "single",
        [
            cube("red_cube_1", "red", 0.16, -0.05, 5.0),
            cube("red_cube_2", "red", 0.25, 0.05, -12.0),
            cube("blue_cube_1", "blue", 0.16, 0.06, 20.0),
            cube("blue_cube_2", "blue", 0.25, -0.06, 0.0),
            cube("green_cube", "green", 0.205, 0.0, 30.0),  # belongs in neither tray
            {"kind": "tray", "name": "red_tray", "color": "red", "pos": [0.20, 0.155]},
            {"kind": "tray", "name": "blue_tray", "color": "blue", "pos": [0.20, -0.155]},
        ],
        2000,
        sort_check,
    )
)
_add(
    Task(
        "so101-push-puck-to-zone",
        "so101",
        "single",
        [{"kind": "puck", "name": "puck", "pos": [0.15, 0.07]}, {"kind": "zone", "name": "zone", "pos": [0.25, -0.03]}],
        1000,
        puck_check,
    )
)
_add(
    Task(
        "so101-open-drawer",
        "so101",
        "single",
        [{"kind": "drawer", "name": "drawer", "pos": [0.20, 0.0]}],
        600,
        drawer_check,
    )
)
_add(
    Task(
        "so101-press-buttons-in-order",
        "so101",
        "single",
        [
            {"kind": "button", "name": "red_button", "color": "red", "pos": [0.179, -0.065]},
            {"kind": "button", "name": "green_button", "color": "green", "pos": [0.15, -0.117]},
            {"kind": "button", "name": "blue_button", "color": "blue", "pos": [0.106, -0.158]},
        ],
        600,
        code_check,
        code=["green_button", "green_button", "red_button", "blue_button"],
    )
)
_add(
    Task(
        "so101-bimanual-ruler-handover",
        "so101",
        "bimanual",
        [
            {"kind": "ruler", "name": "ruler", "pos": [0.18, 0.10], "yaw": 90.0},
            {"kind": "tray", "name": "tray", "color": "grey", "pos": [0.20, -0.15], "yaw": 90.0},
        ],
        1600,
        handover_check,
        track=handover_track,
        progress=handover_progress,
    )
)
_add(
    Task(
        "so101-bimanual-lift-box",
        "so101",
        "bimanual",
        [{"kind": "box", "name": "box", "pos": [0.20, 0.0]}],
        1000,
        box_check,
        holds=box_holds,
    )
)
_add(
    Task(
        "koch-cube-in-cup",
        "koch",
        "single",
        [cube("cube", "red", 0.13, -0.07, 10.0), {"kind": "cup", "name": "cup", "color": "white", "pos": [0.13, 0.08]}],
        800,
        cube_in_cup_check,
    )
)
_add(
    Task(
        "koch-stack-two-cubes",
        "koch",
        "single",
        [cube("red_cube", "red", 0.11, 0.07, -10.0), cube("blue_cube", "blue", 0.15, -0.05, 12.0)],
        800,
        stack_check,
    )
)
