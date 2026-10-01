"""Driver registry: `make_driver(profile, cfg)` by `cfg["driver"]` (lerobot | piper | rosbridge | mock) or a `robouse.drivers`
entry point. `mock` replays a recorded fixture, or for `robot: ros` runs a mock rosbridge server."""

from __future__ import annotations

from ..robots import Profile
from .base import ArmDriver, Reading  # noqa: F401


def make_driver(prof: Profile, cfg: dict) -> ArmDriver:
    kind = cfg.get("driver", "mock")
    if kind == "mock" and prof.name == "ros":  # a mock rosbridge server in this process; the real driver talks to it
        import math

        from ..rosbridge_mock import MockRosbridge
        from .rosbridge import RosbridgeDriver

        scale = math.pi / 180 if prof.units[0] == "deg" else 1.0
        cams = list(cfg.get("cameras") or {})
        topics = (
            {c: f"/{c}/image_raw/compressed" for c in cams}
            if not isinstance(cfg.get("cameras"), dict)
            else cfg["cameras"]
        )
        srv = MockRosbridge(prof.joints, [v * scale for v in prof.park], cameras=list(topics.values()))
        drv = RosbridgeDriver(prof, {**cfg, "url": srv.url, "cameras": topics}, mock=True)
        drv.mock_server = srv
        return drv
    if kind == "rosbridge":
        from .rosbridge import RosbridgeDriver

        return RosbridgeDriver(prof, cfg)
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
        drv = LeRobotArmDriver(
            prof,
            {**cfg, "cameras": {c: "mock" for c in cams}},
            robot_factory=lambda c, p: ReplayRobot(
                fx, p, cameras=cams, noise_deg=float(cfg.get("mock_noise_deg", 0.0))
            ),
            mock=True,
        )
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
