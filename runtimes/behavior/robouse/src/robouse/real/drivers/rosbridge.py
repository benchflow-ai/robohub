"""Any ROS 1 or ROS 2 arm through rosbridge (rosbridge_suite's JSON protocol over a websocket; no ROS install on the
evaluation machine). UNTESTED on a real robot: checked against the mock rosbridge server in real/rosbridge_mock.py.

Rig keys (`real:` block or the rig file, robot `ros`):
  url               ws://robot:9090
  joints            [joint1, ..., joint6]         names as in /joint_states and the command
  low, high         joint limits, in `units`
  units             deg (default; converted to radians on the wire) or rad
  gripper           index of a gripper joint in `joints` (optional)
  state_topic       /joint_states (sensor_msgs/JointState)
  command_topic     /joint_trajectory_controller/joint_trajectory
  command_type      trajectory_msgs/JointTrajectory (one point per step) or std_msgs/Float64MultiArray
  cameras           {name: /camera/image_raw/compressed}  (sensor_msgs/CompressedImage, JPEG or PNG)
  control_hz        10
  stale_s           1.0   a joint state older than this is a fault (the robot stopped publishing)

Protocol used: {"op": "subscribe", "topic", "type"}, {"op": "advertise", "topic", "type"}, {"op": "publish", "topic",
"msg"}, {"op": "unsubscribe" / "unadvertise"}. Messages arrive as {"op": "publish", "topic", "msg"}.
"""

from __future__ import annotations

import base64
import io
import json
import math
import threading
import time

import numpy as np

from ..robots import EnvelopeDefaults, Profile
from .base import ArmDriver, Reading


def ros_profile(cfg: dict) -> Profile:
    """A profile built from the rig configuration (any arm)."""
    joints = list(cfg.get("joints") or [f"joint{i + 1}" for i in range(6)])
    n = len(joints)
    units = cfg.get("units", "deg")
    lim = 180.0 if units == "deg" else math.pi
    low = [float(x) for x in cfg.get("low", [-lim] * n)]
    high = [float(x) for x in cfg.get("high", [lim] * n)]
    g = cfg.get("gripper")
    speed = float(cfg.get("max_joint_speed", 20.0 if units == "deg" else 0.35))
    return Profile(
        name="ros",
        display=cfg.get("display", "ROS arm (rosbridge)"),
        joints=joints,
        low=low,
        high=high,
        units=[units] * n,
        gripper=None if g is None else int(g),
        gripper_open=float(cfg.get("gripper_open", high[g] if g is not None else 0)),
        gripper_closed=float(cfg.get("gripper_closed", low[g] if g is not None else 0)),
        control_hz=float(cfg.get("control_hz", 10.0)),
        home=[float(x) for x in cfg.get("home", [0.0] * n)],
        park=[float(x) for x in cfg.get("park", [0.0] * n)],
        envelope=EnvelopeDefaults(
            joint_speed_deg_s=speed,
            slow_joint_speed_deg_s=speed / 2,
            windup_deg=float(cfg.get("windup", 10.0 if units == "deg" else 0.2)),
            windup_gripper=float(cfg.get("windup", 10.0 if units == "deg" else 0.2)),
            gripper_speed=float(cfg.get("gripper_speed", speed)),
        ),
        notes=cfg.get("notes", "Joint names and limits come from the rig file; positions in " + units + "."),
        tested_on_hardware=False,
    )


class RosbridgeDriver(ArmDriver):
    def __init__(self, prof: Profile, cfg: dict, mock: bool = False):
        self.profile, self.cfg, self.mock = prof, dict(cfg), mock
        cams = cfg.get("cameras") or {}
        self.cam_topics = cams if isinstance(cams, dict) else {}
        self.cameras = list(self.cam_topics)
        self.to_rad = math.pi / 180 if prof.units[0] == "deg" else 1.0
        self.ws = None
        self._state: dict[str, float] = {}
        self._state_t = 0.0
        self._images: dict[str, np.ndarray] = {}
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._err = ""
        self.description = f"ROS arm via rosbridge at {cfg.get('url')}" + (
            " (mock rosbridge)" if mock else " (untested on hardware)"
        )

    def _send(self, msg: dict) -> None:
        self.ws.send(json.dumps(msg))

    def connect(self) -> None:
        from ...harnesses.vla.client import WebSocket

        self.ws = WebSocket(self.cfg["url"], timeout=float(self.cfg.get("timeout_s", 10)))
        self.ws.sock.settimeout(None)  # the reader blocks; staleness is detected from the joint-state timestamps
        self._send(
            {"op": "subscribe", "topic": self.cfg.get("state_topic", "/joint_states"), "type": "sensor_msgs/JointState"}
        )
        for topic in self.cam_topics.values():
            self._send({"op": "subscribe", "topic": topic, "type": "sensor_msgs/CompressedImage", "throttle_rate": 100})
        self._send({"op": "advertise", "topic": self._cmd_topic(), "type": self._cmd_type()})
        threading.Thread(target=self._reader, daemon=True).start()
        deadline = time.time() + float(self.cfg.get("connect_wait_s", 5.0))
        while time.time() < deadline and not self._have_all():
            time.sleep(0.02)
        if not self._have_all():
            raise RuntimeError(f"no /joint_states with joints {self.profile.joints} from {self.cfg['url']} {self._err}")
        self.send(self.read().q)  # hold where it is

    def _cmd_topic(self) -> str:
        return self.cfg.get("command_topic", "/joint_trajectory_controller/joint_trajectory")

    def _cmd_type(self) -> str:
        return self.cfg.get("command_type", "trajectory_msgs/JointTrajectory")

    def _have_all(self) -> bool:
        with self._lock:
            return all(j in self._state for j in self.profile.joints)

    def _reader(self) -> None:
        state_topic = self.cfg.get("state_topic", "/joint_states")
        cams = {t: n for n, t in self.cam_topics.items()}
        while not self._stop.is_set():
            try:
                raw = self.ws.recv()
            except Exception as e:  # noqa: BLE001
                self._err = f"{type(e).__name__}: {e}"
                return
            try:
                m = json.loads(raw if isinstance(raw, str) else raw.decode())
            except ValueError:
                continue
            if m.get("op") != "publish":
                continue
            msg = m.get("msg") or {}
            if m.get("topic") == state_topic:
                with self._lock:
                    for name, pos in zip(msg.get("name", []), msg.get("position", []), strict=False):
                        self._state[name] = float(pos)
                    self._state_t = time.time()
            elif m.get("topic") in cams:
                try:
                    from PIL import Image

                    img = np.asarray(Image.open(io.BytesIO(base64.b64decode(msg["data"]))).convert("RGB"))
                    with self._lock:
                        self._images[cams[m["topic"]]] = img
                except Exception:  # noqa: BLE001 - a bad frame is skipped
                    pass

    def read(self) -> Reading:
        with self._lock:
            if self._err and not self._stop.is_set():
                raise RuntimeError(f"rosbridge connection lost: {self._err}")
            age = time.time() - self._state_t
            if age > float(self.cfg.get("stale_s", 1.0)):
                raise RuntimeError(f"joint states are {age:.1f} s old (the robot stopped publishing)")
            q = np.asarray([self._state[j] / self.to_rad for j in self.profile.joints], dtype=float)
            imgs = dict(self._images)
        return Reading(q=q, extra={}, images=imgs)

    def send(self, q) -> None:
        pos = [float(v) * self.to_rad for v in np.asarray(q, dtype=float)]
        if self._cmd_type() == "std_msgs/Float64MultiArray":
            msg = {"data": pos}
        else:
            dt = 1.0 / float(self.cfg.get("control_hz", self.profile.control_hz))
            msg = {
                "joint_names": self.profile.joints,
                "points": [{"positions": pos, "time_from_start": {"sec": int(dt), "nanosec": int((dt % 1) * 1e9)}}],
            }
        self._send({"op": "publish", "topic": self._cmd_topic(), "msg": msg})

    def disconnect(self, release_torque: bool) -> None:
        self._stop.set()
        if self.ws is not None:
            try:
                self._send({"op": "unadvertise", "topic": self._cmd_topic()})
                self.ws.close()
            except Exception:  # noqa: BLE001
                pass
            self.ws = None
        srv = getattr(self, "mock_server", None)
        if srv is not None:
            srv.close()

    def hardware_id(self) -> str:
        return f"ros:{self.cfg.get('url')}"
