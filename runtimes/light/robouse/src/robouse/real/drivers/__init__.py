"""Driver registry: `make_driver(profile, cfg)` by `cfg["driver"]` (lerobot | piper | mock) or a `robouse.drivers` entry point."""
from __future__ import annotations

from ..robots import Profile
from .base import ArmDriver, Reading  # noqa: F401


def make_driver(prof: Profile, cfg: dict) -> ArmDriver:
    kind = cfg.get("driver", "mock")
    if kind == "mock":
        from ..replay import Fixture, ReplayPiperSDK, ReplayRobot

        fx = Fixture(cfg.get("fixture") or f"{prof.name}-synthetic")
        if fx.robot != prof.name and not (prof.name == "so100" and fx.robot == "so101"):
            raise ValueError(f"fixture {fx.dir.name} was recorded on {fx.robot}, not {prof.name}")
        if prof.name == "piper":
            from .piper import PiperDriver

            return PiperDriver(prof, cfg, sdk_factory=lambda c: ReplayPiperSDK(fx, prof), mock=True)
        from .lerobot_arm import LeRobotArmDriver

        cams = list(cfg.get("cameras") or fx.cameras)
        drv = LeRobotArmDriver(prof, {**cfg, "cameras": {c: "mock" for c in cams}},
                               robot_factory=lambda c, p: ReplayRobot(fx, p, cameras=cams, noise_deg=float(cfg.get("mock_noise_deg", 0.0))),
                               mock=True)
        drv.fixture = fx
        return drv
    if kind == "lerobot":
        from .lerobot_arm import LeRobotArmDriver

        return LeRobotArmDriver(prof, cfg)
    if kind == "piper":
        from .piper import PiperDriver

        return PiperDriver(prof, cfg)
    from importlib.metadata import entry_points

    for ep in entry_points(group="robouse.drivers"):
        if ep.name == kind:
            return ep.load()(prof, cfg)
    raise KeyError(f"unknown driver {kind!r} (mock, lerobot, piper, or a robouse.drivers entry point)")
