"""Pinned articulated arm models attached to the desk layout.

Assets are the MuJoCo Menagerie `franka_emika_panda` and `aloha` models at commit
8161bba264d7fa7c99ca301e91e7fb44737676ad, vendored beside this package with their
original licence files. Joint limits, inertias, actuators and gripper coupling are
the upstream ones; only the base placement and a Panda fingertip site are added.
"""

from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np

from .layout import BASE_X, FORWARD_QUAT, REAR_BASE_Y

EMBODIMENTS = ("dual_panda", "aloha2")
SIDES = ("left", "right")


def asset_root() -> Path:
    from ....assets import (
        asset_root as _root,  # checkout assets/menagerie, $ROBOUSE_MENAGERIE_ASSETS, or the fetch cache
    )

    return _root()


@dataclass(frozen=True)
class Arm:
    name: str
    joints: tuple[str, ...]
    actuators: tuple[str, ...]
    finger_joints: tuple[str, str]
    gripper_actuator: str
    tcp: str
    home: tuple[float, ...]
    down_quat: tuple[float, float, float, float]
    open_ctrl: float
    closed_ctrl: float
    finger_open_m: float


def attach_robots(scene: mujoco.MjSpec, embodiment: str, base_offsets: dict | None = None) -> dict[str, Arm]:
    """Attach both arms. `base_offsets` maps a side to (dx, dy, dyaw_rad)."""
    if embodiment not in EMBODIMENTS:
        raise ValueError(f"Unknown embodiment: {embodiment}")
    offsets = {s: (0.0, 0.0, 0.0) for s in SIDES}
    offsets.update(base_offsets or {})
    arms = {}
    root = asset_root()
    if embodiment == "dual_panda":
        for side, x in BASE_X.items():
            dx, dy, dyaw = offsets[side]
            robot = mujoco.MjSpec.from_file(str(root / "franka_emika_panda/panda.xml"))
            for key in list(robot.keys):
                _delete(robot, key)
            for light in list(robot.lights):
                _delete(robot, light)
            robot.body("hand").add_site(name="tcp", pos=[0, 0, 0.103], size=[0.003] * 3)
            mount = scene.worldbody.add_frame(pos=[x + dx, REAR_BASE_Y + dy, 0.015], quat=_yaw(FORWARD_QUAT, dyaw))
            scene.attach(robot, prefix=side + "/", frame=mount)
            arms[side] = Arm(
                side,
                tuple(f"{side}/joint{i}" for i in range(1, 8)),
                tuple(f"{side}/actuator{i}" for i in range(1, 8)),
                (f"{side}/finger_joint1", f"{side}/finger_joint2"),
                f"{side}/actuator8",
                f"{side}/tcp",
                (0, 0, 0, -1.57079, 0, 1.57079, -0.7853),
                (0, 1, 0, 0),
                255.0,
                0.0,
                0.04,
            )
    else:
        robot = mujoco.MjSpec.from_file(str(root / "aloha/aloha.xml"))
        for key in list(robot.keys):
            _delete(robot, key)
        for light in list(robot.lights):
            _delete(robot, light)
        for side, x in BASE_X.items():
            dx, dy, dyaw = offsets[side]
            robot.body(f"{side}/base_link").pos = [x + dx, REAR_BASE_Y + dy, 0.035]
            robot.body(f"{side}/base_link").quat = list(_yaw(FORWARD_QUAT, dyaw))
        scene.attach(robot, frame=scene.worldbody.add_frame())
        for side in SIDES:
            names = ("waist", "shoulder", "elbow", "forearm_roll", "wrist_angle", "wrist_rotate")
            arms[side] = Arm(
                side,
                tuple(f"{side}/{j}" for j in names),
                tuple(f"{side}/{j}" for j in names),
                (f"{side}/left_finger", f"{side}/right_finger"),
                f"{side}/gripper",
                f"{side}/gripper",
                (0, -0.96, 1.16, 0, -0.3, 0),
                # The ALOHA gripper points along local +X, the Panda along local +Z.
                (float(np.sqrt(0.5)), 0, float(np.sqrt(0.5)), 0),
                0.037,
                0.002,
                0.037,
            )
    return arms


def _yaw(quat, angle):
    if not angle:
        return tuple(float(v) for v in quat)
    delta = np.array([np.cos(angle / 2), 0.0, 0.0, np.sin(angle / 2)])
    out = np.zeros(4)
    mujoco.mju_mulQuat(out, delta, np.asarray(quat, dtype=float))
    return tuple(float(v) for v in out)


def _delete(spec, element):
    """Remove a spec element (MjSpec.delete on MuJoCo >= 3.3.1, element.delete() before)."""
    if hasattr(spec, "delete"):
        spec.delete(element)
    else:
        element.delete()
