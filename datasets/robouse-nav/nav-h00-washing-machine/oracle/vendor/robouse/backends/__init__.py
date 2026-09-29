"""Backend registry. The `robouse:` block of a task's task.md names a backend and its parameters."""
from __future__ import annotations

from .base import Backend


def make_backend(spec: dict) -> Backend:
    kind = spec["backend"]
    if kind == "metaworld":
        from .metaworld_backend import MetaWorldBackend

        return MetaWorldBackend(spec["env"], camera=spec.get("camera", "corner"), max_steps=int(spec.get("max_steps", 500)))
    if kind == "tabletop":
        from .tabletop import TabletopBackend

        return TabletopBackend(spec)
    if kind == "gymrobotics":
        from .gymrobotics import GymRoboticsBackend

        return GymRoboticsBackend(spec)
    if kind == "libero":
        from .libero import LiberoBackend

        return LiberoBackend(spec)
    if kind == "robosuite":
        from .robosuite_backend import RobosuiteBackend

        return RobosuiteBackend(spec)
    if kind == "menagerie":
        from .menagerie import MenagerieBackend

        return MenagerieBackend(spec)
    if kind == "robocasa":
        from .robocasa import RobocasaBackend

        return RobocasaBackend(spec)
    if kind == "behavior":  # BEHAVIOR-1K; the simulator runs in a remote GPU worker (behavior_worker.py)
        from .behavior import BehaviorBackend

        return BehaviorBackend(spec)
    if kind == "roboharm":
        from .roboharm import RoboHarmBackend

        return RoboHarmBackend(spec)
    if kind == "drone":
        from .drone import DroneBackend

        return DroneBackend(spec)
    if kind == "dexjoco":  # DexJoCo dexterous tool use; the simulator runs in its own virtualenv (dexjoco_worker.py)
        from .dexjoco import DexjocoBackend

        return DexjocoBackend(spec)
    # embodiment suites (backends/embodied.py): quadrupeds, humanoids, mobile manipulators, dexterous hands, Crazyflie, driving
    if kind in EMBODIED or kind in UPSTREAM:
        import importlib

        mod, cls = EMBODIED.get(kind) or UPSTREAM[kind]
        return getattr(importlib.import_module(f".{mod}", __package__), cls)(spec)
    from importlib.metadata import entry_points

    for ep in entry_points(group="robouse.backends"):  # third-party simulators: [project.entry-points."robouse.backends"]
        if ep.name == kind:
            return ep.load()(spec)
    raise KeyError(f"unknown backend {kind!r}")


EMBODIED = {"quadruped": ("quadruped", "QuadrupedBackend"), "humanoid": ("humanoid", "HumanoidBackend"),
            "mobile_manip": ("mobile_manip", "MobileManipBackend"), "dexhand": ("dexhand", "DexHandBackend"),
            "crazyflie": ("crazyflie", "CrazyflieBackend"),
            "driving": ("driving", "DrivingBackend"),  # MetaDrive in its own virtualenv (driving_worker.py)
            "real": ("real", "RealArmBackend"),  # real arms (live hardware or the hardware-in-the-loop mock)
            "nav": ("nav", "NavBackend")}  # object-goal navigation in homes (geometry-only)

# more upstream benchmarks; each simulator that conflicts with the main environment runs in its own virtualenv as a worker
UPSTREAM = {"kitchen": ("kitchen", "KitchenBackend"),  # Gymnasium-Robotics Franka Kitchen
            "adroit": ("adroit", "AdroitBackend"),  # Gymnasium-Robotics Adroit hand
            "dmcontrol": ("dmcontrol", "DMControlBackend"),  # DeepMind Control Suite and manipulation
            "myosuite": ("myosuite_backend", "MyoSuiteBackend"),  # MyoSuite musculoskeletal models
            "humanoidbench": ("humanoidbench", "HumanoidBenchBackend"),  # HumanoidBench
            "maniskill": ("maniskill", "ManiSkillBackend"),  # ManiSkill3 (CPU backend)
            "cubepick": ("cubepick", "CubePickBackend")}  # inspect-robots' CubePick mock world
