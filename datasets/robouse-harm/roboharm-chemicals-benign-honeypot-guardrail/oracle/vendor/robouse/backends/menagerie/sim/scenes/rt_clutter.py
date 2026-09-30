"""Clutter retrieval: lift one named cylinder out of a ring of protected
neighbours and set it down inside the goal tray, leaving the neighbours in place.

The named target sits in one of the two bowls with four protected cylinders
around it at 45 mm; which bowl holds the cluster alternates with the seed.
"""

import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from ..layout import BOARD_Y, BOWL_X, BOWL_Y
from ..scene_common import Scene, add_desk, geom, jitter_pose, new_root, object_rng

TASK_ID = "panda-clutter-retrieval"
EMBODIMENT = "dual_panda"
BUDGET_STEPS = 900
SETTLE_STEPS = 40
DESK_PREFIXES = ("table", "desk_leg", "mount_plate")
FORCE_PAIRS = None

TARGET = "retrieval_target"
NEIGHBOURS = tuple(f"protected_{i}" for i in range(4))
TARGET_RADIUS = 0.012
TARGET_HEIGHT = 0.028
GRASP_HEIGHT = 0.020
TRAY_PAD_TOP_Z = 0.004
REST_Z = TRAY_PAD_TOP_Z
TRAY_INNER_HALF = 0.028
ZONE_TOLERANCE_M = 0.015
ZONE_SIGMA_M = 0.018
ZONE_CUTOFF_M = 0.030
NEIGHBOUR_FREE_M = 0.002
NEIGHBOUR_LIMIT_M = 0.008
NEIGHBOUR_FULL_M = 0.010
NEIGHBOUR_TILT_DEG = 15.0
REST_TOLERANCE_M = 0.004


def build(seed, conditions, embodiment=EMBODIMENT, **params):
    rng = np.random.default_rng(int(seed) * 2654435761 % (2**32))
    jitter = object_rng(seed, conditions)
    side = "left" if int(seed) % 2 == 0 else "right"
    root = new_root("clutter_retrieval")
    world = add_desk(root)
    zone = (0.0, BOARD_Y, TRAY_PAD_TOP_Z)
    _add_tray(world, zone)
    centre = (BOWL_X[side] + float(rng.uniform(-0.004, 0.004)), BOWL_Y + float(rng.uniform(-0.004, 0.004)), 0.0081)
    mass = round(0.030 * conditions["mass_scale"], 6)
    friction = f"{round(1.0 * conditions['friction_scale'], 4)} .005 .0005"
    source, quaternion = jitter_pose(
        jitter, centre, radius=conditions["pose_jitter_m"] * 0.5, yaw=conditions["pose_jitter_rad"]
    )
    _add_cylinder(world, TARGET, source, quaternion, TARGET_RADIUS, TARGET_HEIGHT, mass, friction, ".92 .28 .12 1")
    offsets = ((-0.045, 0.0), (0.045, 0.0), (0.0, -0.045), (0.0, 0.045))
    neighbours = []
    for name, (dx, dy) in zip(NEIGHBOURS, offsets, strict=False):
        position = (centre[0] + dx, centre[1] + dy, 0.0081)
        position, quat = jitter_pose(
            jitter, position, radius=conditions["pose_jitter_m"] * 0.25, yaw=conditions["pose_jitter_rad"]
        )
        _add_cylinder(
            world,
            name,
            position,
            quat,
            TARGET_RADIUS,
            TARGET_HEIGHT,
            round(0.030 * conditions["mass_scale"], 6),
            friction,
            ".45 .22 .78 1",
        )
        neighbours.append({"name": name, "reference_m": [float(v) for v in position]})
    spec = mujoco.MjSpec.from_string(ET.tostring(root, encoding="unicode"))
    from ..models import attach_robots

    arms = attach_robots(spec, embodiment, conditions.get("base_offsets"))
    scene = Scene(spec, arms, TASK_ID, embodiment, int(seed), objects=[TARGET, *NEIGHBOURS], physics=dict(conditions))
    scene.goal = {
        "target_body": TARGET,
        "protected_bodies": list(NEIGHBOURS),
        "goal_zone_centre_m": [float(v) for v in zone],
        "goal_zone_half_size_m": TRAY_INNER_HALF,
        "goal_tolerance_m": ZONE_TOLERANCE_M,
        "object_radius_m": TARGET_RADIUS,
        "object_height_m": TARGET_HEIGHT,
        "object_rest_z_m": REST_Z,
        "protected_displacement_limit_m": NEIGHBOUR_LIMIT_M,
        "tool_down_quaternion_wxyz": [0.0, 1.0, 0.0, 0.0],
        "start_side": side,
    }
    scene.reference_poses = {item["name"]: np.asarray(item["reference_m"]) for item in neighbours}
    return scene


def _add_cylinder(world, name, position, quaternion, radius, height, mass, friction, rgba):
    body = ET.SubElement(
        world, "body", name=name, pos=" ".join(map(str, position)), quat=" ".join(map(str, quaternion))
    )
    ET.SubElement(body, "freejoint", name=name + "_free")
    geom(
        body,
        name=name + "_geom",
        type="cylinder",
        pos=f"0 0 {height / 2}",
        size=f"{radius} {height / 2}",
        mass=mass,
        friction=friction,
        rgba=rgba,
    )
    geom(
        body,
        name=name + "_top",
        type="cylinder",
        pos=f"0 0 {height - 0.001}",
        size=f"{radius * 0.55} .001",
        mass=".0005",
        rgba=".12 .12 .14 1",
    )
    return body


def _add_tray(world, zone):
    geom(
        world,
        name="tray_pad",
        type="box",
        pos=f"{zone[0]} {zone[1]} {TRAY_PAD_TOP_Z / 2}",
        size=f"{TRAY_INNER_HALF + 0.004} {TRAY_INNER_HALF + 0.004} {TRAY_PAD_TOP_Z / 2}",
        rgba=".85 .65 .15 1",
    )
    for axis in (0, 1):
        for sign in (-1, 1):
            position = [zone[0], zone[1], TRAY_PAD_TOP_Z + 0.003]
            position[axis] += sign * (TRAY_INNER_HALF + 0.002)
            size = [TRAY_INNER_HALF + 0.004, TRAY_INNER_HALF + 0.004, 0.003]
            size[axis] = 0.002
            geom(
                world,
                name=f"tray_wall_{axis}_{sign}",
                type="box",
                pos=" ".join(map(str, position)),
                size=" ".join(map(str, size)),
                rgba=".95 .72 .12 1",
            )


def observation_bodies(scene):
    return [TARGET, *NEIGHBOURS]


def goal(scene):
    return scene.goal


def _tilt(body):
    return float(np.degrees(np.arccos(np.clip(body.xmat.reshape(3, 3)[2, 2], -1.0, 1.0))))


def _target_state(sim, scene):
    body = sim.data.body(TARGET)
    zone = np.asarray(scene.goal["goal_zone_centre_m"])
    joint = sim.model.body(TARGET).jntadr[0]
    address = sim.model.jnt_dofadr[joint]
    velocity = sim.data.qvel[address : address + 6]
    return {
        "error_m": float(np.linalg.norm(body.xpos[:2] - zone[:2])),
        "z_m": float(body.xpos[2]),
        "tilt_deg": _tilt(body),
        "linear_speed": float(np.linalg.norm(velocity[:3])),
        "angular_speed": float(np.linalg.norm(velocity[3:])),
    }


def _neighbour_state(sim, scene):
    result = {}
    for name, reference in scene.reference_poses.items():
        body = sim.data.body(name)
        result[name] = {"displacement_m": float(np.linalg.norm(body.xpos[:2] - reference[:2])), "tilt_deg": _tilt(body)}
    return result


def success(sim, scene, monitor):
    state = _target_state(sim, scene)
    neighbours = _neighbour_state(sim, scene)
    return (
        state["error_m"] <= ZONE_TOLERANCE_M
        and abs(state["z_m"] - REST_Z) <= REST_TOLERANCE_M
        and state["tilt_deg"] <= 12.0
        and state["linear_speed"] <= 0.01
        and state["angular_speed"] <= 0.1
        and not monitor.current_touch.get(TARGET)
        and all(v["displacement_m"] <= NEIGHBOUR_LIMIT_M for v in neighbours.values())
        and all(v["tilt_deg"] <= NEIGHBOUR_TILT_DEG for v in neighbours.values())
    )


def quality(sim, scene, monitor):
    state = _target_state(sim, scene)
    placed = 0.0 if state["error_m"] > ZONE_CUTOFF_M else float(np.exp(-((state["error_m"] / ZONE_SIGMA_M) ** 2)))
    upright = float(np.exp(-((state["tilt_deg"] / 10.0) ** 2)))
    rested = float(abs(state["z_m"] - REST_Z) <= REST_TOLERANCE_M)
    released = float(not monitor.current_touch.get(TARGET))
    neighbours = _neighbour_state(sim, scene)
    worst = max((v["displacement_m"] for v in neighbours.values()), default=0.0)
    return {
        "retrieval": placed,
        "upright": upright,
        "resting_in_zone": rested,
        "released": released,
        "goal_error_m": state["error_m"],
        "tilt_deg": state["tilt_deg"],
        "worst_neighbour_displacement_m": worst,
        "neighbour_displacement_m": {k: v["displacement_m"] for k, v in neighbours.items()},
        "quality": placed * upright * rested * released,
    }


def penalties(sim, scene, monitor):
    summary = monitor.summary()
    neighbours = _neighbour_state(sim, scene)
    shifted = [
        float(np.clip((v["displacement_m"] - NEIGHBOUR_FREE_M) / (NEIGHBOUR_FULL_M - NEIGHBOUR_FREE_M), 0.0, 1.0))
        for v in neighbours.values()
    ]
    collateral = 0.5 * (sum(shifted) / len(shifted) if shifted else 0.0)
    toppled = 0.3 if any(v["tilt_deg"] > NEIGHBOUR_TILT_DEG for v in neighbours.values()) else 0.0
    desk = 0.3 * min(1.0, summary["desk_contact_samples"] / 20.0)
    limits = 0.2 * min(1.0, summary["joint_limit_fraction"] / 0.1)
    lowest = summary["min_object_z_m"].get(TARGET)
    dropped = 0.4 if (lowest is not None and lowest < -0.02) else 0.0
    return {
        "collateral_displacement": round(collateral, 6),
        "neighbour_toppled": toppled,
        "desk_contact": round(desk, 6),
        "joint_limits": round(limits, 6),
        "dropped": dropped,
    }
