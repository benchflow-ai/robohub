"""Precise landing: carry one block from its bowl onto the marked pad.

The scene is the shared desk with both Panda arms at the rear edge, a chessboard
style target surface in front of them, two bowls, and one block that starts in
one of the bowls. Which bowl holds the block alternates with the episode seed, so
the arm that has to do the work alternates too.
"""
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from ..layout import BOARD_Y, BOWL_X, BOWL_Y, SQUARE_SIZE, board_xy
from ..scene_common import Scene, add_desk, geom, jitter_pose, new_root, object_rng

TASK_ID = "dual-panda-precise-landing"
EMBODIMENT = "dual_panda"
BUDGET_STEPS = 800
SETTLE_STEPS = 40
DESK_PREFIXES = ("table", "desk_leg", "mount_plate", "board", "square")
FORCE_PAIRS = None

BLOCK = "target_block"
BLOCK_HALF = (.011, .011, .025)
GRASP_HEIGHT = .038
BOARD_TOP_Z = .0102
PLACEMENT_TOLERANCE_M = .012
PLACEMENT_SIGMA_M = .012
PLACEMENT_CUTOFF_M = .04
TILT_SIGMA_DEG = 8.
TILT_LIMIT_DEG = 10.
SETTLE_DRIFT_SIGMA_M = .003
REST_TOLERANCE_M = .004


def build(seed, conditions, embodiment=EMBODIMENT, **params):
    rng = np.random.default_rng(int(seed) * 2654435761 % (2 ** 32))
    side = "left" if int(seed) % 2 == 0 else "right"
    rank = 1 if side == "left" else 6
    file = int(rng.integers(1, 7))
    pad = (*board_xy(rank, file), BOARD_TOP_Z)
    root = new_root("precise_landing")
    world = add_desk(root)
    _add_board(world, pad)
    source = (BOWL_X[side] + float(rng.uniform(-.002, .002)),
              BOWL_Y + float(rng.uniform(-.002, .002)), .0081)
    jitter = object_rng(seed, conditions)
    source, quaternion = jitter_pose(jitter, source, radius=conditions["pose_jitter_m"],
                                     yaw=conditions["pose_jitter_rad"])
    body = ET.SubElement(world, "body", name=BLOCK, pos=" ".join(map(str, source)),
                         quat=" ".join(map(str, quaternion)))
    ET.SubElement(body, "freejoint", name=BLOCK + "_free")
    kind = params.get("kind", "block")
    if kind == "pawn":
        # robo-use D01 pawn (scene.py): base disc, stem and spherical crown, free rigid body.
        colour = ".93 .88 .71 1"
        geom(body, name=BLOCK + "_base", type="cylinder", pos="0 0 .004", size=".013 .004",
             mass=".012", rgba=colour)
        geom(body, name=BLOCK + "_stem", type="cylinder", pos="0 0 .027", size=".009 .019",
             mass=".012", rgba=colour)
        geom(body, name=BLOCK + "_crown", type="sphere", pos="0 0 .049", size=".011",
             mass=".004", rgba=colour)
    else:
        geom(body, name=BLOCK + "_geom", type="box", pos=f"0 0 {BLOCK_HALF[2]}",
             size=" ".join(map(str, BLOCK_HALF)), mass=round(.030 * conditions["mass_scale"], 6),
             friction=f"{round(1.0 * conditions['friction_scale'], 4)} .005 .0005",
             rgba=".93 .55 .12 1")
        geom(body, name=BLOCK + "_mark", type="box", pos=f"0 0 {2 * BLOCK_HALF[2] - .001}",
             size=f"{BLOCK_HALF[0] * .8} {BLOCK_HALF[1] * .8} .001", mass=".0005",
             rgba=".15 .15 .18 1")
    spec = mujoco.MjSpec.from_string(ET.tostring(root, encoding="unicode"))
    from ..models import attach_robots
    arms = attach_robots(spec, embodiment, conditions.get("base_offsets"))
    scene = Scene(spec, arms, TASK_ID, embodiment, int(seed),
                  objects=[BLOCK], physics=dict(conditions))
    scene.goal = {"block_body": BLOCK, "pad_center_m": [float(v) for v in pad],
                  "pad_half_size_m": SQUARE_SIZE / 2,
                  "block_half_extents_m": list(BLOCK_HALF),
                  "block_rest_z_m": BOARD_TOP_Z,
                  "placement_tolerance_m": PLACEMENT_TOLERANCE_M,
                  "max_tilt_degrees": TILT_LIMIT_DEG,
                  "settle_seconds": 2.0,
                  "tool_down_quaternion_wxyz": [0., 1., 0., 0.],
                  "start_side": side}
    return scene


def _add_board(world, pad):
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
    for axis in (0, 1):
        for sign in (-1, 1):
            position = list(pad)
            position[axis] += sign * (SQUARE_SIZE / 2 - .001)
            position[2] = .0106
            size = [.019, .019, .0002]
            size[axis] = .001
            geom(world, name=f"landing_pad_{axis}_{sign}", type="box",
                 pos=" ".join(map(str, position)), size=" ".join(map(str, size)),
                 contype="0", conaffinity="0", rgba="1 .67 .05 1")


def observation_bodies(scene):
    return [BLOCK]


def goal(scene):
    return scene.goal


def _state(sim, scene):
    body = sim.data.body(BLOCK)
    pad = np.asarray(scene.goal["pad_center_m"])
    error = float(np.linalg.norm(body.xpos[:2] - pad[:2]))
    up = body.xmat.reshape(3, 3)[2, 2]
    tilt = float(np.degrees(np.arccos(np.clip(up, -1., 1.))))
    joint = sim.model.body(BLOCK).jntadr[0]
    address = sim.model.jnt_dofadr[joint]
    velocity = sim.data.qvel[address:address + 6]
    return {"error_m": error, "tilt_deg": tilt, "z_m": float(body.xpos[2]),
            "linear_speed": float(np.linalg.norm(velocity[:3])),
            "angular_speed": float(np.linalg.norm(velocity[3:]))}


def success(sim, scene, monitor):
    state = _state(sim, scene)
    held = bool(monitor.current_touch.get(BLOCK))
    return (state["error_m"] <= PLACEMENT_TOLERANCE_M
            and state["tilt_deg"] <= TILT_LIMIT_DEG
            and abs(state["z_m"] - BOARD_TOP_Z) <= REST_TOLERANCE_M
            and state["linear_speed"] <= .01 and state["angular_speed"] <= .1
            and not held)


def quality(sim, scene, monitor):
    state = _state(sim, scene)
    place = 0. if state["error_m"] > PLACEMENT_CUTOFF_M else float(
        np.exp(-(state["error_m"] / PLACEMENT_SIGMA_M) ** 2))
    upright = float(np.exp(-(state["tilt_deg"] / TILT_SIGMA_DEG) ** 2))
    drift = 0.
    if monitor.settle_start and BLOCK in monitor.settle_start:
        start = monitor.settle_start[BLOCK][0]
        drift = float(np.linalg.norm(sim.data.body(BLOCK).xpos - start))
    stability = float(np.exp(-(drift / SETTLE_DRIFT_SIGMA_M) ** 2))
    rested = float(abs(state["z_m"] - BOARD_TOP_Z) <= REST_TOLERANCE_M)
    released = float(not monitor.current_touch.get(BLOCK))
    return {"placement": place, "upright": upright, "settle_stability": stability,
            "resting_on_board": rested, "released": released,
            "placement_error_m": state["error_m"], "tilt_deg": state["tilt_deg"],
            "settle_drift_m": drift,
            "quality": place * upright * stability * rested * released}


def penalties(sim, scene, monitor):
    summary = monitor.summary()
    desk = .3 * min(1., summary["desk_contact_samples"] / 20.)
    limits = .2 * min(1., summary["joint_limit_fraction"] / .1)
    lowest = summary["min_object_z_m"].get(BLOCK)
    dropped = .4 if (lowest is not None and lowest < -.02) else 0.
    return {"desk_contact": round(desk, 6), "joint_limits": round(limits, 6),
            "dropped": dropped}
