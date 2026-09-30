"""AgileX Piper driver over `piper_sdk` (CAN). UNTESTED: written from the SDK's documented interface; no Piper was
connected while it was built. Install on the rig: `pip install piper_sdk python-can`, bring up the CAN interface
(`bash can_activate.sh can0 1000000`).

SDK calls used (piper_sdk V2 interface): `C_PiperInterface_V2(can_name)`, `ConnectPort()`, `EnablePiper()`,
`GetArmJointMsgs()` (joint_state.joint_1..joint_6 in 0.001 deg), `GetArmGripperMsgs()` (gripper_state.grippers_angle in
0.001 mm), `MotionCtrl_2(ctrl_mode=0x01, move_mode=0x01, move_spd_rate_ctrl, 0x00)` (CAN command mode, joint moves),
`JointCtrl(j1..j6)` (0.001 deg), `GripperCtrl(pos_0.001mm, effort_0.001Nm, 0x01, 0)`, `DisablePiper()`.
The `sdk_factory` seam lets tests (and the mock) substitute the SDK object.
"""

from __future__ import annotations

import time
from collections.abc import Callable
from typing import Any

import numpy as np

from ..cameras import open_cameras
from ..robots import Profile
from .base import ArmDriver, Reading


def _sdk_factory(cfg: dict) -> Any:  # pragma: no cover - real hardware
    from piper_sdk import C_PiperInterface_V2

    return C_PiperInterface_V2(cfg.get("can", "can0"))


class PiperDriver(ArmDriver):
    def __init__(self, prof: Profile, cfg: dict, sdk_factory: Callable[[dict], Any] | None = None, mock: bool = False):
        self.profile, self.cfg, self.mock = prof, dict(cfg), mock
        self._factory = sdk_factory or _sdk_factory
        self.sdk = None
        cams = cfg.get("cameras") or {}
        self.cam_specs = cams if isinstance(cams, dict) else {}
        self.cameras = list(cams.keys() if isinstance(cams, dict) else cams)
        self._cams = {}
        self.description = "AgileX Piper via piper_sdk" + (" (mock)" if mock else " (untested on hardware)")

    def connect(self) -> None:
        self.sdk = self._factory(self.cfg)
        self.sdk.ConnectPort()
        deadline = time.time() + float(self.cfg.get("enable_timeout_s", 5.0))
        while not self.sdk.EnablePiper():
            if time.time() > deadline:
                raise RuntimeError("Piper did not enable within the timeout (check CAN and the e-stop)")
            time.sleep(0.05)
        if not self.mock:
            self._cams = open_cameras(self.cam_specs)
        self.send(self.read().q)  # hold where it is

    def read(self) -> Reading:
        js = self.sdk.GetArmJointMsgs().joint_state
        q = [getattr(js, f"joint_{i}") / 1000.0 for i in range(1, 7)]
        g = self.sdk.GetArmGripperMsgs().gripper_state.grippers_angle / 1000.0
        images = {n: c.frame() for n, c in self._cams.items() if c.frame() is not None}
        return Reading(q=np.asarray(q + [g], dtype=float), extra={}, images=images)

    def send(self, q) -> None:
        q = np.asarray(q, dtype=float)
        speed = int(self.cfg.get("speed_percent", 30))
        self.sdk.MotionCtrl_2(0x01, 0x01, speed, 0x00)
        self.sdk.JointCtrl(*[int(round(v * 1000)) for v in q[:6]])
        self.sdk.GripperCtrl(int(round(abs(q[6]) * 1000)), int(self.cfg.get("gripper_effort_mnm", 1000)), 0x01, 0)

    def disconnect(self, release_torque: bool) -> None:
        for c in self._cams.values():
            c.stop()
        if self.sdk is not None and release_torque:
            self.sdk.DisablePiper()
        self.sdk = None
