"""Gentle insertion: seat a peg in a socket without exceeding a contact force cap.

An ALOHA 2 pair stands at the rear edge of the desk. A peg lies in one of the two
bowls, the socket fixture sits in the middle of the desk, and the peg has to end
up standing inside the socket. The peak normal contact force between the peg and
the fixture is part of the score, so the approach has to be slow and aligned.
"""
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from ..layout import BOARD_Y, BOWL_X, BOWL_Y
from ..scene_common import (Scene, add_desk, geom, jitter_pose, new_root,
                                    object_rng)

TASK_ID = "aloha2-gentle-insertion"
EMBODIMENT = "aloha2"
BUDGET_STEPS = 900
SETTLE_STEPS = 40
DESK_PREFIXES = ("table", "desk_leg", "mount_plate")
FORCE_PAIRS = (("peg_",), ("socket_",))

PEG = "insertion_peg"
PEG_RADIUS = .008
PEG_LENGTH = .075
GRASP_HEIGHT = .060
SOCKET_XY = (0., BOARD_Y)
SOCKET_INNER_HALF = .0115
SOCKET_OUTER_HALF = .030
SOCKET_FLOOR_Z = .005
SOCKET_TOP_Z = .035
FULL_DEPTH = SOCKET_TOP_Z - SOCKET_FLOOR_Z
SEATED_FRACTION = .80
FORCE_CAP_N = 6.
TILT_SIGMA_DEG = 6.
TILT_LIMIT_DEG = 8.


def build(seed, conditions, embodiment=EMBODIMENT, **params):
    rng = np.random.default_rng(int(seed) * 2654435761 % (2 ** 32))
    jitter = object_rng(seed, conditions)
    side = "left" if int(seed) % 2 == 0 else "right"
    root = new_root("gentle_insertion")
    world = add_desk(root)
    _add_socket(world)
    source = (BOWL_X[side] + float(rng.uniform(-.004, .004)),
              BOWL_Y + float(rng.uniform(-.004, .004)), .0081)
    source, quaternion = jitter_pose(jitter, source, radius=conditions["pose_jitter_m"] * .6,
                                     yaw=conditions["pose_jitter_rad"])
    body = ET.SubElement(world, "body", name=PEG, pos=" ".join(map(str, source)),
                         quat=" ".join(map(str, quaternion)))
    ET.SubElement(body, "freejoint", name=PEG + "_free")
    friction = f"{round(1.0 * conditions['friction_scale'], 4)} .005 .0005"
    geom(body, name="peg_shaft", type="cylinder", pos=f"0 0 {PEG_LENGTH / 2}",
         size=f"{PEG_RADIUS} {PEG_LENGTH / 2}", mass=round(.035 * conditions["mass_scale"], 6),
         friction=friction, rgba=".95 .35 .06 1")
    geom(body, name="peg_collar", type="cylinder", pos=f"0 0 {PEG_LENGTH - .004}",
         size=".011 .004", mass=".004", friction=friction, rgba=".20 .22 .26 1")
    spec = mujoco.MjSpec.from_string(ET.tostring(root, encoding="unicode"))
    from ..models import attach_robots
    arms = attach_robots(spec, embodiment, conditions.get("base_offsets"))
    scene = Scene(spec, arms, TASK_ID, embodiment, int(seed), objects=[PEG],
                  physics=dict(conditions))
    scene.goal = {"peg_body": PEG, "peg_radius_m": PEG_RADIUS, "peg_length_m": PEG_LENGTH,
                  "socket_centre_m": [SOCKET_XY[0], SOCKET_XY[1], SOCKET_TOP_Z],
                  "socket_aperture_half_m": SOCKET_INNER_HALF,
                  "socket_floor_z_m": SOCKET_FLOOR_Z, "socket_top_z_m": SOCKET_TOP_Z,
                  "full_insertion_depth_m": FULL_DEPTH,
                  "seated_depth_fraction": SEATED_FRACTION,
                  "peak_contact_force_cap_n": FORCE_CAP_N,
                  "max_tilt_degrees": TILT_LIMIT_DEG,
                  "tool_down_quaternion_wxyz": [float(np.sqrt(.5)), 0., float(np.sqrt(.5)), 0.],
                  "start_side": side}
    return scene


def _add_socket(world):
    x, y = SOCKET_XY
    geom(world, name="socket_base", type="box",
         pos=f"{x} {y} {SOCKET_FLOOR_Z / 2}",
         size=f"{SOCKET_OUTER_HALF} {SOCKET_OUTER_HALF} {SOCKET_FLOOR_Z / 2}",
         rgba=".55 .58 .62 1")
    height = SOCKET_TOP_Z - SOCKET_FLOOR_Z
    for axis in (0, 1):
        for sign in (-1, 1):
            centre = [x, y, SOCKET_FLOOR_Z + height / 2]
            centre[axis] += sign * (SOCKET_OUTER_HALF + SOCKET_INNER_HALF) / 2
            if axis == 0:
                size = [(SOCKET_OUTER_HALF - SOCKET_INNER_HALF) / 2, SOCKET_INNER_HALF, height / 2]
            else:
                size = [SOCKET_OUTER_HALF, (SOCKET_OUTER_HALF - SOCKET_INNER_HALF) / 2, height / 2]
            geom(world, name=f"socket_wall_{axis}_{sign}", type="box",
                 pos=" ".join(map(str, centre)), size=" ".join(map(str, size)),
                 rgba=".85 .64 .12 1")


def observation_bodies(scene):
    return [PEG]


def goal(scene):
    return scene.goal


def _state(sim, scene):
    body = sim.data.body(PEG)
    base = body.xpos
    tilt = float(np.degrees(np.arccos(np.clip(body.xmat.reshape(3, 3)[2, 2], -1., 1.))))
    inside = (abs(float(base[0]) - SOCKET_XY[0]) <= SOCKET_INNER_HALF
              and abs(float(base[1]) - SOCKET_XY[1]) <= SOCKET_INNER_HALF)
    depth = float(np.clip(SOCKET_TOP_Z - float(base[2]), 0., FULL_DEPTH)) if inside else 0.
    joint = sim.model.body(PEG).jntadr[0]
    address = sim.model.jnt_dofadr[joint]
    velocity = sim.data.qvel[address:address + 6]
    return {"depth_m": depth, "tilt_deg": tilt, "inside": inside,
            "linear_speed": float(np.linalg.norm(velocity[:3])),
            "angular_speed": float(np.linalg.norm(velocity[3:]))}


def success(sim, scene, monitor):
    state = _state(sim, scene)
    summary_force = monitor.peak_force_n
    return (state["depth_m"] >= SEATED_FRACTION * FULL_DEPTH
            and state["tilt_deg"] <= TILT_LIMIT_DEG
            and summary_force <= FORCE_CAP_N
            and state["linear_speed"] <= .01 and state["angular_speed"] <= .1
            and not monitor.current_touch.get(PEG))


def quality(sim, scene, monitor):
    state = _state(sim, scene)
    depth = float(np.clip(state["depth_m"] / FULL_DEPTH, 0., 1.))
    peak = monitor.peak_force_n
    force = 1. if peak <= FORCE_CAP_N else float(np.exp(-((peak - FORCE_CAP_N) / (.5 * FORCE_CAP_N)) ** 2))
    upright = float(np.exp(-(state["tilt_deg"] / TILT_SIGMA_DEG) ** 2))
    released = float(not monitor.current_touch.get(PEG))
    return {"insertion_depth": depth, "force_compliance": force, "upright": upright,
            "released": released, "depth_m": state["depth_m"],
            "peak_contact_force_n": peak, "tilt_deg": state["tilt_deg"],
            "quality": depth * force * upright * released}


def penalties(sim, scene, monitor):
    summary = monitor.summary()
    desk = .3 * min(1., summary["desk_contact_samples"] / 20.)
    limits = .2 * min(1., summary["joint_limit_fraction"] / .1)
    lowest = summary["min_object_z_m"].get(PEG)
    dropped = .4 if (lowest is not None and lowest < -.02) else 0.
    return {"desk_contact": round(desk, 6), "joint_limits": round(limits, 6),
            "dropped": dropped}
