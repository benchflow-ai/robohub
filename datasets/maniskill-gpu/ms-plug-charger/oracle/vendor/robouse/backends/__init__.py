"""Backend registry. The `robouse:` block of a task's task.md names a backend (`backend: KIND`) and its parameters;
each backend is one subpackage of robouse.backends. Third-party simulators register a class taking the block under the
`robouse.backends` entry point group."""

from __future__ import annotations

import importlib

from .base import Backend

# kind -> (subpackage, class)
BACKENDS: dict[str, tuple[str, str]] = {
    "adroit": ("adroit", "AdroitBackend"),  # Gymnasium-Robotics Adroit hand
    "behavior": ("behavior", "BehaviorBackend"),  # BEHAVIOR-1K; remote GPU worker
    "capx": ("capx", "CapxBackend"),  # CaP-X code-as-policy tasks (privileged API); simulator worker
    "capx_libero": ("capx", "CapxLiberoBackend"),  # CaP-X's LIBERO-PRO tasks (privileged API); simulator worker
    "components": ("components", "ComposedBackend"),  # tasks composed from components (robouse tasks init)
    "remix": ("components", "ComposedBackend"),  # the name composed tasks had before 0.2
    "crazyflie": ("crazyflie", "CrazyflieBackend"),
    "cubepick": ("cubepick", "CubePickBackend"),  # inspect-robots' CubePick mock world
    "dexhand": ("dexhand", "DexHandBackend"),
    "dexjoco": ("dexjoco", "DexjocoBackend"),  # simulator worker
    "dmcontrol": ("dmcontrol", "DMControlBackend"),  # simulator worker
    "driving": ("driving", "DrivingBackend"),  # MetaDrive; simulator worker
    "drone": ("drone", "DroneBackend"),
    "gpu": ("gpu", "GpuBackend"),  # ManiSkill3, Genesis, MuJoCo Playground, Isaac Lab; remote GPU workers
    "gymrobotics": ("gymrobotics", "GymRoboticsBackend"),
    "humanoid": ("humanoid", "HumanoidBackend"),
    "humanoidbench": ("humanoidbench", "HumanoidBenchBackend"),  # simulator worker
    "kitchen": ("kitchen", "KitchenBackend"),  # Gymnasium-Robotics Franka Kitchen
    "libero": ("libero", "LiberoBackend"),
    "maniskill": ("maniskill", "ManiSkillBackend"),  # ManiSkill3 on the CPU; simulator worker
    "menagerie": ("menagerie", "MenagerieBackend"),
    "metaworld": ("metaworld", "MetaWorldBackend"),
    "mobile_manip": ("mobile_manip", "MobileManipBackend"),
    "myosuite": ("myosuite", "MyoSuiteBackend"),  # simulator worker
    "nav": ("nav", "NavBackend"),  # object-goal navigation in homes (geometry only)
    "quadruped": ("quadruped", "QuadrupedBackend"),
    "real": ("real", "RealArmBackend"),  # real arms: live hardware or the hardware-in-the-loop mock
    "rlebench": ("rlebench", "RLEBenchBackend"),  # RLE-Bench interactive-control tasks; simulator worker
    "robocasa": ("robocasa", "RobocasaBackend"),  # simulator worker
    "robodojo": ("robodojo", "RoboDojoBackend"),  # RoboDojo on Isaac Sim; remote GPU worker
    "roboharm": ("roboharm", "RoboHarmBackend"),
    "robosuite": ("robosuite", "RobosuiteBackend"),
    "tabletop": ("tabletop", "TabletopBackend"),
}


def backend_class(kind: str) -> type[Backend]:
    """The Backend class for a `backend:` kind (built in, or from the `robouse.backends` entry point group)."""
    if kind in BACKENDS:
        mod, cls = BACKENDS[kind]
        return getattr(importlib.import_module(f"{__name__}.{mod}"), cls)
    from importlib.metadata import entry_points

    for ep in entry_points(group="robouse.backends"):
        if ep.name == kind:
            return ep.load()
    raise KeyError(f"unknown backend {kind!r}; built in: {', '.join(sorted(BACKENDS))}")


def make_backend(spec: dict) -> Backend:
    return backend_class(spec["backend"])(spec)
