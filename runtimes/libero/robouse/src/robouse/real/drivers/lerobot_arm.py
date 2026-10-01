"""Any LeRobot follower arm as a Robo Use driver: MakerMods Metal (`metal_follower`, makermods-robotics/lerobot branch
arm/makermods-metal) and SO-100/SO-101 (`so_follower`). LeRobot's `Robot` contract is `connect(calibrate)`,
`get_observation() -> {"<motor>.pos": value, <camera>: image}`, `send_action({"<motor>.pos": value})`, `disconnect()`.

The robot object is built by a factory, so the hardware-in-the-loop mock (real/replay.py) plugs in at exactly the seam
the real SDK occupies and the rest of this driver runs unchanged. Live-hardware factories import LeRobot lazily; install
it on the rig machine (`pip install -e "lerobot[metal]"` from the makermods fork for the Metal arm, `pip install
"lerobot[feetech]"` for SO-101).

Status: the Metal factory mirrors the configuration used by inspect-robots-metal on the 2026-09-05 bench sessions; the
SO-101 factory mirrors ~/benchflow/robot/so101_sort_control.py. Neither live path has run through Robo Use yet (the
hardware was not connected while this was built); both are covered by the mock.
"""

from __future__ import annotations

from collections.abc import Callable
from pathlib import Path
from typing import Any

import numpy as np

from ..cameras import open_cameras
from ..robots import Profile
from .base import ArmDriver, Reading


def _metal_factory(cfg: dict, prof: Profile):  # pragma: no cover - real hardware
    from lerobot.robots.metal_follower import MetalFollower, MetalFollowerConfig

    per_step = {n: float(cfg.get("driver_max_step_deg", 12.0)) for n in prof.joints}
    rc = MetalFollowerConfig(
        id=cfg.get("robot_id", "metal"),
        port=cfg.get("port", "/dev/ttyACM0"),
        can_interface=cfg.get("can_interface", "slcan"),
        calibration_dir=Path(cfg["calibration_dir"]) if cfg.get("calibration_dir") else None,
        max_relative_target=per_step,
        startup_sync_speed_deg=1.0,
        disable_torque_on_disconnect=bool(cfg.get("disable_torque_on_close", False)),
        cameras={},
    )
    return MetalFollower(rc)


def _so101_factory(cfg: dict, prof: Profile):  # pragma: no cover - real hardware
    from lerobot.robots.so_follower import SOFollower, SOFollowerRobotConfig

    rc = SOFollowerRobotConfig(
        id=cfg.get("robot_id", "so101"),
        port=cfg["port"],
        calibration_dir=Path(cfg["calibration_dir"]) if cfg.get("calibration_dir") else None,
        max_relative_target=float(cfg.get("driver_max_step_deg", 12.0)),
        disable_torque_on_disconnect=bool(cfg.get("disable_torque_on_close", False)),
        cameras={},
        use_degrees=True,
    )
    return SOFollower(rc)


def _so101_bimanual_factory(cfg: dict, prof: Profile):  # pragma: no cover - real hardware
    from lerobot.robots.bi_so_follower import BiSOFollower, BiSOFollowerConfig
    from lerobot.robots.so_follower import SOFollowerConfig

    def arm(side):
        return SOFollowerConfig(
            port=cfg[f"{side}_port"],
            max_relative_target=float(cfg.get("driver_max_step_deg", 12.0)),
            disable_torque_on_disconnect=bool(cfg.get("disable_torque_on_close", False)),
            use_degrees=True,
        )

    rc = BiSOFollowerConfig(
        id=cfg.get("robot_id", "so101_bimanual"),
        left_arm_config=arm("left"),
        right_arm_config=arm("right"),
        calibration_dir=Path(cfg["calibration_dir"]) if cfg.get("calibration_dir") else None,
    )
    return BiSOFollower(rc)


FACTORIES: dict[str, Callable[[dict, Profile], Any]] = {
    "metal": _metal_factory,
    "so101": _so101_factory,
    "so100": _so101_factory,
    "so101-bimanual": _so101_bimanual_factory,
}


class LeRobotArmDriver(ArmDriver):
    def __init__(
        self, prof: Profile, cfg: dict, robot_factory: Callable[[dict, Profile], Any] | None = None, mock: bool = False
    ):
        self.profile, self.cfg, self.mock = prof, dict(cfg), mock
        self._factory = robot_factory or FACTORIES.get(prof.name)
        if self._factory is None:
            raise KeyError(f"no LeRobot factory for {prof.name!r}")
        self.robot = None
        self._cams = {}
        cams = cfg.get("cameras") or {}
        self.cam_specs = (
            cams if isinstance(cams, dict) else {}
        )  # {name: OpenCV index / path / MJPEG URL} from the rig file
        self.cameras = list(cams.keys() if isinstance(cams, dict) else cams)
        self.description = f"LeRobot {prof.display}" + (" (hardware-in-the-loop mock)" if mock else "")
        self._n = 0

    def connect(self) -> None:
        robot = self._factory(self.cfg, self.profile)
        if self.cfg.get("require_calibration") and not getattr(robot, "is_calibrated", True):
            raise RuntimeError(f"{self.profile.display} has no calibration file; run lerobot-calibrate first")
        # calibrate=False: LeRobot's default would start an interactive, arm-moving calibration mid-episode
        robot.connect(calibrate=False)
        self.robot = robot
        if not self.mock:
            self._cams = open_cameras(self.cam_specs, *self.cfg.get("camera_size", (640, 480)))
        # hold on enable: command the measured pose at once so motors with zero gains until their first frame hold
        self.send(self.read().q)

    def _raw(self) -> dict:
        return self.robot.get_observation()

    def read(self) -> Reading:
        raw = self._raw()
        try:
            q = np.asarray([float(raw[f"{n}.pos"]) for n in self.profile.joints], dtype=float)
        except KeyError as e:
            raise RuntimeError(f"observation is missing motor key {e}") from e
        if not np.all(np.isfinite(q)):
            raise RuntimeError(f"non-finite joint feedback {q.tolist()}")
        images = {}
        for name in self.cameras:
            if name in raw:  # cameras served by the robot object (LeRobot camera configs, or the mock)
                images[name] = np.ascontiguousarray(raw[name], dtype=np.uint8)
            elif name in self._cams:
                fr = self._cams[name].frame()
                if fr is not None:
                    images[name] = fr
        return Reading(q=q, extra=self._telemetry(raw), images=images)

    def _telemetry(self, raw: dict) -> dict:
        out: dict = {}
        states = getattr(getattr(self.robot, "bus", None), "_last_known_states", None)  # Damiao bus cache (Metal)
        if isinstance(states, dict):
            for fld, key in (("torque", "joint_torque_nm"), ("temp_mos", "temp_mos_c"), ("temp_rotor", "temp_rotor_c")):
                vals = [states.get(n, {}).get(fld) for n in self.profile.joints]
                if all(v is not None for v in vals):
                    out[key] = [float(v) for v in vals]
        if not out and isinstance(
            raw.get("_telemetry"), dict
        ):  # the hardware-in-the-loop mock replays recorded telemetry
            out.update(raw["_telemetry"])
        self._n += 1
        bus = getattr(self.robot, "bus", None)
        if (
            not out and bus is not None and hasattr(bus, "sync_read") and self._n % 10 == 1
        ):  # Feetech: temperature at 1 Hz
            try:
                t = bus.sync_read("Present_Temperature")
                out["temp_c"] = [float(t[n]) for n in self.profile.joints if n in t]
            except Exception:  # noqa: BLE001 - telemetry is best effort
                pass
        return out

    def send(self, q) -> None:
        self.robot.send_action(
            {f"{n}.pos": float(v) for n, v in zip(self.profile.joints, np.asarray(q, dtype=float), strict=False)}
        )

    def disconnect(self, release_torque: bool) -> None:
        for c in self._cams.values():
            c.stop()
        if self.robot is not None:
            try:
                cfg = getattr(self.robot, "config", None)
                if cfg is not None and hasattr(cfg, "disable_torque_on_disconnect"):
                    cfg.disable_torque_on_disconnect = bool(release_torque)
            except Exception:  # noqa: BLE001
                pass
            self.robot.disconnect()
            self.robot = None

    def hardware_id(self) -> str:
        return f"{self.profile.name}:{self.cfg.get('port', 'mock' if self.mock else '')}"
