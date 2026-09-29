"""Two bowls to a board: eight pieces, two source bowls, one shared board.

Four light pieces start in the left bowl and four dark pieces in the right bowl.
Every piece has an assigned square. The two arms share one physics clock, so the
two halves of the job can run at the same time.
"""
import math
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from ..layout import BOARD_Y, BOWL_X, BOWL_Y, SQUARE_SIZE, board_xy
from ..scene_common import (Scene, add_desk, geom, jitter_pose, new_root,
                                    object_rng)

TASK_ID = "dual-panda-two-bowls-to-board"
EMBODIMENT = "dual_panda"
BUDGET_STEPS = 2000
SETTLE_STEPS = 40
DESK_PREFIXES = ("table", "desk_leg", "mount_plate", "board", "square")
FORCE_PAIRS = None

BOARD_TOP_Z = .0102
PLACEMENT_TOLERANCE_M = .012
PLACEMENT_SIGMA_M = .010
PLACEMENT_CUTOFF_M = .020
TILT_SIGMA_DEG = 8.
TILT_LIMIT_DEG = 10.
REST_TOLERANCE_M = .004
BOARD_HALF_M = 4 * SQUARE_SIZE + .006
KINDS = ("pawn", "pawn", "rook", "bishop")
# Height above the piece origin at which the shaft is a plain parallel cylinder.
SHAFT_GRASP_HEIGHT = .030
QUALITY_SPLIT = {"placement": .85, "inventory": .15}
SINGLE_ARM_FACTOR = .55


def build(seed, conditions, embodiment=EMBODIMENT, **params):
    rng = np.random.default_rng(int(seed) * 2654435761 % (2 ** 32))
    jitter = object_rng(seed, conditions)
    root = new_root("two_bowls_to_board")
    world = add_desk(root)
    _add_board(world)
    pieces = []
    files = {"left": list(rng.permutation(4)), "right": list(rng.permutation(4))}
    for side, rank in (("left", 1), ("right", 6)):
        for index, kind in enumerate(params.get("kinds", KINDS)):
            name = f"{'light' if side == 'left' else 'dark'}_{kind}_{index}"
            file = int(files[side][index]) + 2
            target = (*board_xy(rank, file), BOARD_TOP_Z)
            source = (BOWL_X[side] + (index // 2 - .5) * .052 + float(rng.uniform(-.001, .001)),
                      BOWL_Y + (index % 2 - .5) * .052 + float(rng.uniform(-.001, .001)), .0081)
            source, quaternion = jitter_pose(jitter, source,
                                             radius=conditions["pose_jitter_m"] * .35,
                                             yaw=conditions["pose_jitter_rad"])
            _add_piece(world, name, kind, side, source, quaternion, conditions)
            _mark_square(world, name, target)
            pieces.append({"body": name, "kind": kind, "source_side": side,
                           "target_m": [float(v) for v in target],
                           "square": f"{chr(97 + file)}{rank + 1}",
                           "grasp_height_m": SHAFT_GRASP_HEIGHT})
    spec = mujoco.MjSpec.from_string(ET.tostring(root, encoding="unicode"))
    from ..models import attach_robots
    arms = attach_robots(spec, embodiment, conditions.get("base_offsets"))
    scene = Scene(spec, arms, TASK_ID, embodiment, int(seed),
                  objects=[p["body"] for p in pieces], physics=dict(conditions))
    scene.goal = {"pieces": pieces, "board_centre_m": [0., BOARD_Y, BOARD_TOP_Z],
                  "board_half_size_m": BOARD_HALF_M, "square_size_m": SQUARE_SIZE,
                  "piece_rest_z_m": BOARD_TOP_Z,
                  "placement_tolerance_m": PLACEMENT_TOLERANCE_M,
                  "max_tilt_degrees": TILT_LIMIT_DEG,
                  "tool_down_quaternion_wxyz": [0., 1., 0., 0.]}
    return scene


def _add_board(world):
    half = 4 * SQUARE_SIZE
    geom(world, name="board", type="box", pos=f"0 {BOARD_Y} .0045",
         size=f"{half + .006} {half + .006} .0055", rgba=".22 .12 .06 1")
    for rank in range(8):
        for file in range(8):
            colour = ".83 .75 .6 1" if (rank + file) % 2 else ".20 .28 .24 1"
            x, y = board_xy(rank, file)
            geom(world, name=f"square_{chr(97 + file)}{rank + 1}", type="box",
                 pos=f"{x} {y} .0101", size=f"{SQUARE_SIZE / 2} {SQUARE_SIZE / 2} .0001",
                 contype="0", conaffinity="0", rgba=colour)


def _mark_square(world, name, target):
    for axis in (0, 1):
        for sign in (-1, 1):
            position = list(target)
            position[axis] += sign * (SQUARE_SIZE / 2 - .001)
            position[2] = .0106
            size = [.019, .019, .0002]
            size[axis] = .001
            geom(world, name=f"assigned_{name}_{axis}_{sign}", type="box",
                 pos=" ".join(map(str, position)), size=" ".join(map(str, size)),
                 contype="0", conaffinity="0", rgba="1 .67 .05 1")


def _add_piece(world, name, kind, side, source, quaternion, conditions):
    body = ET.SubElement(world, "body", name=name, pos=" ".join(map(str, source)),
                         quat=" ".join(map(str, quaternion)))
    ET.SubElement(body, "freejoint", name=name + "_free")
    colour = ".93 .88 .71 1" if side == "left" else ".12 .13 .17 1"
    friction = f"{round(1.0 * conditions['friction_scale'], 4)} .005 .0005"
    scale = conditions["mass_scale"]
    geom(body, name=name + "_base", type="cylinder", pos="0 0 .004", size=".013 .004",
         mass=round(.012 * scale, 6), friction=friction, rgba=colour)
    geom(body, name=name + "_stem", type="cylinder", pos="0 0 .027", size=".009 .019",
         mass=round(.012 * scale, 6), friction=friction, rgba=colour)
    if kind == "pawn":
        geom(body, name=name + "_crown", type="sphere", pos="0 0 .049", size=".011",
             mass=round(.004 * scale, 6), friction=friction, rgba=colour)
    elif kind == "rook":
        geom(body, name=name + "_crown", type="box", pos="0 0 .052", size=".011 .008 .009",
             mass=round(.004 * scale, 6), friction=friction, rgba=colour)
    else:
        geom(body, name=name + "_crown", type="ellipsoid", pos="0 0 .054",
             size=".010 .010 .015", mass=round(.004 * scale, 6), friction=friction, rgba=colour)


def observation_bodies(scene):
    return [p["body"] for p in scene.goal["pieces"]]


def goal(scene):
    return scene.goal


def _piece_state(sim, scene, monitor):
    result = {}
    for piece in scene.goal["pieces"]:
        name = piece["body"]
        body = sim.data.body(name)
        target = np.asarray(piece["target_m"])
        error = float(np.linalg.norm(body.xpos[:2] - target[:2]))
        tilt = float(np.degrees(np.arccos(np.clip(body.xmat.reshape(3, 3)[2, 2], -1., 1.))))
        joint = sim.model.body(name).jntadr[0]
        address = sim.model.jnt_dofadr[joint]
        velocity = sim.data.qvel[address:address + 6]
        on_board = (abs(float(body.xpos[0])) <= BOARD_HALF_M
                    and abs(float(body.xpos[1]) - BOARD_Y) <= BOARD_HALF_M
                    and abs(float(body.xpos[2]) - BOARD_TOP_Z) <= REST_TOLERANCE_M)
        result[name] = {"error_m": error, "tilt_deg": tilt, "z_m": float(body.xpos[2]),
                        "on_board": on_board,
                        "released": not monitor.current_touch.get(name),
                        "still": float(np.linalg.norm(velocity[:3])) <= .01
                                 and float(np.linalg.norm(velocity[3:])) <= .1}
    return result


def success(sim, scene, monitor):
    states = _piece_state(sim, scene, monitor)
    return all(state["error_m"] <= PLACEMENT_TOLERANCE_M
               and state["tilt_deg"] <= TILT_LIMIT_DEG
               and abs(state["z_m"] - BOARD_TOP_Z) <= REST_TOLERANCE_M
               and state["released"] and state["still"]
               for state in states.values())


def quality(sim, scene, monitor):
    states = _piece_state(sim, scene, monitor)
    placement, inventory, placed_by = [], [], set()
    for name, state in states.items():
        place = 0. if state["error_m"] > PLACEMENT_CUTOFF_M else float(
            np.exp(-(state["error_m"] / PLACEMENT_SIGMA_M) ** 2))
        upright = float(np.exp(-(state["tilt_deg"] / TILT_SIGMA_DEG) ** 2))
        rested = float(abs(state["z_m"] - BOARD_TOP_Z) <= REST_TOLERANCE_M)
        released = float(state["released"])
        score = place * upright * rested * released
        placement.append(score)
        inventory.append(float(state["on_board"] and state["released"]))
        if score > .5:
            # The arm that last had hold of the piece, not every arm that ever
            # brushed it, so a token touch by the idle arm proves nothing.
            placed_by |= set(monitor.last_touch.get(name, set()))
    placed = float(np.mean(placement)) if placement else 0.
    complete = float(np.mean(inventory)) if inventory else 0.
    both = 1. if len(placed_by) >= 2 else SINGLE_ARM_FACTOR
    combined = QUALITY_SPLIT["placement"] * placed + QUALITY_SPLIT["inventory"] * complete
    return {"placement": placed, "inventory": complete, "both_arms_factor": both,
            "arms_used": sorted(placed_by),
            "correct_squares": int(sum(1 for s in placement if s > .5)),
            "per_piece": {k: round(v, 4) for k, v in zip(states, placement)},
            "quality": combined * both}


def penalties(sim, scene, monitor):
    summary = monitor.summary()
    desk = .3 * min(1., summary["desk_contact_samples"] / 40.)
    limits = .2 * min(1., summary["joint_limit_fraction"] / .1)
    lowest = summary["min_object_z_m"]
    fallen = sum(1 for value in lowest.values() if value is not None and value < -.02)
    dropped = .4 * fallen / max(1, len(lowest))
    return {"desk_contact": round(desk, 6), "joint_limits": round(limits, 6),
            "dropped": round(dropped, 6)}
