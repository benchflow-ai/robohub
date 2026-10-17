"""AgileX Piper driver over `piper_sdk` (CAN). UNTESTED: written from the SDK's documented interface; no Piper was
connected while it was built. Install on the rig: `pip install piper_sdk python-can`, bring up the CAN interface
(`bash can_activate.sh can0 1000000`).

SDK calls used (piper_sdk V2 interface): `C_PiperInterface_V2(can_name)`, `ConnectPort()`, `EnablePiper()`,
`GetArmJointMsgs()` (joint_state.joint_1..joint_6 in 0.001 deg), `GetArmGripperMsgs()` (gripper_state.grippers_angle in
0.001 mm), `MotionCtrl_2(ctrl_mode=0x01, move_mode=0x01, move_spd_rate_ctrl, 0x00)` (CAN command mode, joint moves),
`JointCtrl(j1..j6)` (0.001 deg), `GripperCtrl(pos_0.001mm, effort_0.001Nm, 0x01, 0)`, `DisablePiper()`.
The `sdk_factory` seam lets tests (and the mock) substitute the SDK object.

Freshness per joint pair (checked against piper_sdk 0.6.2): the joints arrive in three CAN frames, joints 1-2, 3-4 and
5-6 (ids 0x2A5-0x2A7). The SDK's receive thread decodes each one (`ParseCANFrame`) into a message whose `type_` is
`ArmMsgType.PiperMsgJointFeedBack_12`, `_34` or `_56` and hands it to the name-mangled `__UpdateArmJointState`, which
updates that pair's two joints and the one `ArmJoint.time_stamp`, or returns None when it drops the frame as abnormal
data. One time stamp for all three pairs means a pair that falls silent goes unnoticed, its joints frozen, while the
others keep the stamp fresh. So the driver wraps that update and records when each pair last arrived; a reading is as
old as its oldest pair, and a stop for lost telemetry names that pair. With an SDK that lacks the method, the joint
state's own time stamp dates the reading (the newest pair).
"""

from __future__ import annotations

import functools
import time
from collections.abc import Callable
from typing import Any

import numpy as np

from ..cameras import open_cameras
from ..robots import Profile
from .base import ArmDriver, Reading

JOINT_PAIRS = {  # piper_sdk ArmMsgType names of the joint feedback frames, and the joints each carries
    "PiperMsgJointFeedBack_12": "joints 1-2",
    "PiperMsgJointFeedBack_34": "joints 3-4",
    "PiperMsgJointFeedBack_56": "joints 5-6",
}
JOINT_UPDATE = "_C_PiperInterface_V2__UpdateArmJointState"  # C_PiperInterface_V2.__UpdateArmJointState, name-mangled


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
        self._pairs: dict[str, float | None] | None = None  # joint pair -> when it last arrived (time.monotonic())
        self._pairs_t0 = 0.0  # when the driver started listening for them

    def connect(self) -> None:
        self.sdk = self._factory(self.cfg)
        self._stamp_pairs()  # before the receive thread starts
        self.sdk.ConnectPort()
        deadline = time.time() + float(self.cfg.get("enable_timeout_s", 5.0))
        while not self.sdk.EnablePiper():
            if time.time() > deadline:
                raise RuntimeError("Piper did not enable within the timeout (check CAN and the e-stop)")
            time.sleep(0.05)
        if not self.mock:
            self._cams = open_cameras(self.cam_specs)
        while self._pairs is not None and (silent := [p for p, t in self._pairs.items() if t is None]):
            if time.time() > deadline:  # the SDK's joint state would still hold zeros for these joints
                raise RuntimeError(f"no joint feedback from {', '.join(silent)} after the Piper was enabled")
            time.sleep(0.01)
            self.read()  # (the mock SDK delivers its frames on a read)
        self.send(self.read().q)  # hold where it is: read after every pair has arrived

    def _stamp_pairs(self) -> None:
        """Record when each joint pair's feedback frame last arrived, by wrapping the SDK's joint-state update (see the
        module notes); without that method, `_pairs` stays None and the SDK's own time stamp dates readings."""
        update = getattr(self.sdk, JOINT_UPDATE, None)
        if not callable(update):
            self.description += " (joint freshness from the SDK's single time stamp)"
            return
        update = getattr(update, "__wrapped__", update)  # one SDK object per CAN port: never wrap it twice
        pairs: dict[str, float | None] = dict.fromkeys(JOINT_PAIRS.values())

        def stamped(msg: Any) -> Any:
            out = update(msg)
            pair = JOINT_PAIRS.get(getattr(getattr(msg, "type_", None), "name", ""))
            if pair is not None and out is not None:  # None: the SDK dropped the frame as abnormal data
                pairs[pair] = time.monotonic()
            return out

        functools.update_wrapper(stamped, update)
        setattr(self.sdk, JOINT_UPDATE, stamped)
        self._pairs, self._pairs_t0 = pairs, time.monotonic()

    def read(self) -> Reading:
        jm = self.sdk.GetArmJointMsgs()
        js = jm.joint_state
        q = [getattr(js, f"joint_{i}") / 1000.0 for i in range(1, 7)]
        g = self.sdk.GetArmGripperMsgs().gripper_state.grippers_angle / 1000.0
        images = {n: c.frame() for n, c in self._cams.items() if c.frame() is not None}
        stamp: float | None
        if self._pairs is not None:  # the oldest joint pair dates the reading (one not yet heard: since listening)
            pairs, t0 = self._pairs, self._pairs_t0
            oldest = min(pairs, key=lambda p: pairs[p] or t0)
            stamp, stamp_of = pairs[oldest] or t0, oldest
        else:  # the SDK's one time stamp: when the newest joint frame arrived (wall clock)
            ts = getattr(jm, "time_stamp", None)
            stamp = None
            if isinstance(ts, (int, float)) and ts > 0:
                stamp = time.monotonic() - max(0.0, time.time() - float(ts))
            stamp_of = None
        return Reading(q=np.asarray(q + [g], dtype=float), extra={}, images=images, stamp=stamp, stamp_of=stamp_of)

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
