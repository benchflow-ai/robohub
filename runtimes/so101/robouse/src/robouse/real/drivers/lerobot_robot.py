"""Any LeRobot robot as a Robo Use driver, behind a Robo Use safety profile: the `lerobot-robot` driver, registered
under the `robouse.drivers` entry point (pyproject.toml), so it is found the way a third-party driver is.

The robot is built by LeRobot's own registry: `RobotConfig.get_choice_class(type)` and `make_robot_from_config`, after
`register_third_party_plugins()` has imported every installed `lerobot_robot_*` package. So it works for LeRobot's
built-in robots (`so101_follower`, `koch_follower`, ...) and for any plugin robot: anything with `connect`,
`get_observation() -> {"<motor>.pos": value, <camera>: image}`, `send_action({"<motor>.pos": value})` and
`disconnect`. The driver only reads and sends; every command is decided by the episode server's guard
(real/safety.py) and contact watch (real/contact.py) for the profile the task and the rig name: joint limits, the
workspace veto by forward kinematics, speed and wind-up limits, the e-stop latch, the loop watchdog, stale telemetry,
temperature and contact stops all run exactly as for the built-in drivers.

Rig file entry (keyed by the task's `real.robot`):

    so101:
      driver: lerobot-robot
      lerobot:
        type: so101_follower          # a LeRobot robot type: built in, or from an installed lerobot_robot_* plugin
        import: []                    # optional: modules to import first, for a type defined outside such a package
        config:                       # that type's config fields (LeRobot's names)
          port: /dev/ttyACM0
          id: benchmark_left
          calibration_dir: ~/.cache/huggingface/lerobot/calibration/robots/so_follower
          cameras: {camera0: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}}
        motors: {}                    # profile joint -> the robot's motor name, where they differ
        scale: {}                     # profile value = scale * robot value + offset, per joint (default 1 and 0)
        offset: {}
      profile: so101                  # optional: the safety profile (a built-in name, or a full profile: robots.py)
      envelope: {}                    # optional: tighter workspace and speed limits

A live robot whose profile has no kinematic chain has no workspace veto; the driver refuses it unless the rig says
`workspace_check: joint_limits_only` (joint limits and speed limits only, as on the Piper and bimanual profiles).

Status: tested against LeRobot's mock robot, a LeRobot plugin robot wrapping the recorded SO-101 sessions (replays)
and the SO-101 sim twin as a LeRobot robot (integrations/lerobot_robot_robouse). Not run on hardware yet.
"""

from __future__ import annotations

import importlib
import time
from pathlib import Path
from typing import Any

import numpy as np

from ..robots import Profile
from .lerobot_arm import LeRobotArmDriver

TELEMETRY = {"temperature": "temp_c", "torque": "joint_torque_nm", "effort": "joint_torque_nm"}


def _lerobot():
    try:
        from lerobot.robots import RobotConfig, make_robot_from_config
        from lerobot.utils.import_utils import register_third_party_plugins
    except ImportError as e:  # pragma: no cover - depends on the rig machine
        raise RuntimeError(
            'the lerobot-robot driver needs LeRobot in this environment (pip install "lerobot[feetech]" for an '
            "SO-101, plus the lerobot_robot_* plugin of your robot)"
        ) from e
    return RobotConfig, make_robot_from_config, register_third_party_plugins


def _import_builtin_robots() -> None:
    """LeRobot registers a robot type when its module is imported (its scripts import them one by one): import every
    built-in robot package, skipping those whose optional dependencies are missing."""
    import pkgutil

    import lerobot.robots as pkg

    for m in pkgutil.iter_modules(pkg.__path__):
        if m.ispkg:
            try:
                importlib.import_module(f"lerobot.robots.{m.name}")
            except Exception:  # noqa: BLE001 - an optional dependency of another robot
                pass


def robot_config(spec: dict, max_step_deg: float | None = None):
    """The LeRobot RobotConfig for the rig's `lerobot:` block (type and config fields)."""
    RobotConfig, _, register = _lerobot()
    register()  # import every installed lerobot_robot_* / lerobot_camera_* package so their types are registered
    _import_builtin_robots()
    for mod in spec.get("import") or []:  # robots defined in a module that is not a lerobot_robot_* package
        importlib.import_module(str(mod))
    kind = spec.get("type")
    if not kind:
        raise ValueError("the lerobot-robot driver needs lerobot.type (a LeRobot robot type, e.g. so101_follower)")
    try:
        cls = RobotConfig.get_choice_class(kind)
    except KeyError:
        known = sorted(RobotConfig.get_known_choices())
        raise KeyError(f"LeRobot has no robot type {kind!r} (installed: {', '.join(known)})") from None
    fields = dict(spec.get("config") or {})
    if fields.get("calibration_dir"):
        fields["calibration_dir"] = Path(str(fields["calibration_dir"])).expanduser()
    if isinstance(fields.get("cameras"), dict):
        fields["cameras"] = {n: _camera_config(c) for n, c in fields["cameras"].items()}
    names = set(getattr(cls, "__dataclass_fields__", {}))
    unknown = sorted(set(fields) - names)
    if unknown:
        raise ValueError(f"{kind}: unknown config field(s) {unknown}; fields: {sorted(names)}")
    # a second line of defence inside the robot class (the guard is the first): LeRobot's per-step target cap
    if "max_relative_target" in names and fields.get("max_relative_target") is None and max_step_deg:
        fields["max_relative_target"] = float(max_step_deg)
    # torque stays on when the connection closes: the episode server decides how the arm is left (disconnect)
    if "disable_torque_on_disconnect" in names:
        fields.setdefault("disable_torque_on_disconnect", False)
    return cls(**fields)


def _camera_config(c: Any):
    if not isinstance(c, dict):
        return c
    from lerobot.cameras import CameraConfig

    c = dict(c)
    return CameraConfig.get_choice_class(c.pop("type", "opencv"))(**c)


class _MotorMap:
    """The LeRobot robot seen in the profile's joint names and units; everything else passes through (bus, config)."""

    def __init__(self, robot: Any, prof: Profile, spec: dict):
        self.robot = robot
        motors = dict(spec.get("motors") or {})
        self.motor = {j: str(motors.get(j, j)) for j in prof.joints}
        self.scale = {j: float((spec.get("scale") or {}).get(j, 1.0)) for j in prof.joints}
        self.offset = {j: float((spec.get("offset") or {}).get(j, 0.0)) for j in prof.joints}
        if any(s == 0 for s in self.scale.values()):
            raise ValueError("lerobot.scale values must be non-zero")
        feats = getattr(robot, "action_features", None)
        if isinstance(feats, dict) and feats:
            missing = [f"{m}.pos" for m in self.motor.values() if f"{m}.pos" not in feats]
            if missing:
                raise ValueError(
                    f"the robot has no action feature(s) {missing}; its actions: {sorted(feats)} "
                    "(map the profile's joints with lerobot.motors)"
                )

    def __getattr__(self, name: str) -> Any:
        return getattr(self.robot, name)

    def connect(self, calibrate: bool = False) -> None:
        self.robot.connect(calibrate=calibrate)

    def get_observation(self) -> dict:
        raw = self.robot.get_observation()
        out = {k: v for k, v in raw.items() if not isinstance(v, (int, float, np.floating))}  # cameras
        for j, m in self.motor.items():
            if f"{m}.pos" in raw:
                out[f"{j}.pos"] = self.scale[j] * float(raw[f"{m}.pos"]) + self.offset[j]
        tel: dict = {}
        for fld, key in TELEMETRY.items():
            vals = [raw.get(f"{m}.{fld}") for m in self.motor.values()]
            if all(isinstance(v, (int, float, np.floating)) for v in vals):
                tel[key] = [float(v) for v in vals]
        if tel:
            out["_telemetry"] = tel
        return out

    def send_action(self, action: dict) -> Any:
        return self.robot.send_action(
            {f"{m}.pos": (float(action[f"{j}.pos"]) - self.offset[j]) / self.scale[j] for j, m in self.motor.items()}
        )

    def disconnect(self) -> None:
        self.robot.disconnect()


class LeRobotRobotDriver(LeRobotArmDriver):
    """LeRobotArmDriver (reads, sends, stamps, telemetry) on a robot built by LeRobot's registry."""

    def __init__(self, prof: Profile, cfg: dict, mock: bool = False):
        spec = dict(cfg.get("lerobot") or {})
        if prof.chain() is None and cfg.get("workspace_check") != "joint_limits_only" and not mock:
            raise ValueError(
                f"profile {prof.name!r} has no kinematic chain, so the workspace veto cannot run: give the profile a "
                "urdf (robots.profile_from_config), or set workspace_check: joint_limits_only in the rig file to accept "
                "joint limits and speed limits only"
            )
        self.spec = spec
        self.kind = str(spec.get("type"))
        self.require_cal = bool(cfg.get("require_calibration", not mock))
        self.rc = robot_config(spec, cfg.get("driver_max_step_deg", 12.0))
        super().__init__(prof, {**cfg, "require_calibration": False}, robot_factory=self._make, mock=mock)
        cams = list(getattr(self.rc, "cameras", None) or {})
        self.cameras = list(dict.fromkeys(self.cameras + cams))
        self.description = f"LeRobot {self.kind} as {prof.display}" + (" (mock)" if mock else "")

    def _make(self, cfg: dict, prof: Profile):
        _, make, _ = _lerobot()
        robot = make(self.rc)
        fpath = getattr(robot, "calibration_fpath", None)
        # a robot on a motor bus needs LeRobot's calibration file (rig `require_calibration: false` for one that does not)
        if (
            self.require_cal
            and getattr(robot, "bus", None) is not None
            and isinstance(fpath, Path)
            and not fpath.is_file()
        ):
            raise RuntimeError(
                f"{self.kind} {getattr(self.rc, 'id', '')!r} has no calibration file ({fpath}); run lerobot-calibrate "
                "first (an uncalibrated connect would start LeRobot's interactive, arm-moving calibration)"
            )
        return _MotorMap(robot, prof, self.spec)

    def connect(self) -> None:
        t0 = time.monotonic()
        super().connect()
        calibrated = getattr(self.robot, "is_calibrated", True)
        if not self.mock and calibrated is False:
            self.robot.disconnect()
            self.robot = None
            raise RuntimeError(f"{self.kind}: the calibration file does not match the motors; run lerobot-calibrate")
        self.connect_s = round(time.monotonic() - t0, 3)

    def hardware_id(self) -> str:
        port = getattr(self.rc, "port", "") or getattr(self.rc, "id", "") or ""
        return f"{self.profile.name}:lerobot:{self.kind}:{port}"


def make_driver(prof: Profile, cfg: dict) -> LeRobotRobotDriver:
    """The `robouse.drivers` entry point."""
    return LeRobotRobotDriver(prof, cfg, mock=bool(cfg.get("mock", False)))
