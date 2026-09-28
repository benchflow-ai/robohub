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
    raise KeyError(f"unknown backend {kind!r}")
