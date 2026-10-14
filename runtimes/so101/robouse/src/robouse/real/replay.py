"""Hardware-in-the-loop mock: replays recorded real-robot sessions behind the vendor SDK seam.

A fixture (`real/fixtures/<name>/fixture.json` + `keyframes/*.jpg`, built by `tools/assets/make_hil_fixture.py` from a session
recorded on the bench by ~/benchflow/robot/recorder.py) holds:
  - the recorded control log: per step the command sent, the measured joints, torques and temperatures;
  - a per-joint tracking model fitted to that log (measured_next = measured + gain * (command - measured) + bias), so
    the mock follows commands the way the real arm did (lag, gravity sag);
  - keyframes: measured poses with the camera frames recorded at that moment.

`ReplayRobot` has LeRobot's `Robot` shape (connect / get_observation / send_action / disconnect, plus the Damiao bus
cache the Metal driver reads, which serves positions and telemetry from its last replies), so the real driver code
(drivers/lerobot_arm.py), the safety guard and the episode server run unchanged against it. Camera frames are the
recorded frames whose pose is closest to the mock's current pose. Two modes (`mock_playback` in the task's `real:` block):

  tracking  (default) the joints follow commands through the fitted tracking model; temperatures replay the recorded
            sequence. No joint torque is reported: the recorded torques belong to the recorded motion, not to the
            mock's, and nothing in the mock models contact with objects. Manipulation outcomes on the mock are decided by
            the (scripted) operator, never inferred.
  playback  each control step returns the next recorded step as measured on the bench (joints, torques, temperatures),
            whatever is commanded: the recorded session replayed through the safety layer, e.g. its contacts.

`mock_fault` injects a link failure from a control step on (tests of the watchdog and the stale-telemetry stop):
{"at_step": K, "kind": "stale"} (the bus stops getting replies and keeps serving its cache, as the Damiao bus does when
the CAN link dies), "dead" (reads and writes raise, as when the USB adapter is unplugged) or "slow" (each read takes
"seconds" longer, for "steps" steps); on the Piper mock, "silent_pair" (one joint pair's feedback frames stop).


`conformance()` is the strict replay check behind `robouse hil-check`: it feeds the recorded command sequence through the
Robo Use safety guard, with the measured joints taken from the recording, and reports vetoes, clamps and how well the
fitted tracking model reproduces the recorded motion.
"""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any

import numpy as np

from .. import config
from .robots import Profile

HIL_DIR = Path(__file__).resolve().with_name("fixtures")


def fixture_dir(name: str) -> Path:
    p = Path(name)
    if p.is_dir():
        return p
    for root in [config.env("ROBOUSE_HIL_DIR"), str(HIL_DIR)]:
        if root and (Path(root) / name).is_dir():
            return Path(root) / name
    raise FileNotFoundError(
        f"hardware-in-the-loop fixture {name!r} not found (looked in ROBOUSE_HIL_DIR and {HIL_DIR})"
    )


class Fixture:
    def __init__(self, name: str):
        self.dir = fixture_dir(name)
        d = json.loads((self.dir / "fixture.json").read_text())
        self.meta = d
        self.robot = d["robot"]
        self.steps = d.get("steps", [])
        self.gain = np.asarray(d["model"]["gain"], dtype=float)
        self.bias = np.asarray(d["model"]["bias"], dtype=float)
        self.start = np.asarray(d["start"], dtype=float)
        self.keyframes = d.get("keyframes", [])
        self.cameras = list(d.get("cameras", []))
        self._kq = (
            np.asarray([k["q"] for k in self.keyframes], dtype=float)
            if self.keyframes
            else np.zeros((0, len(self.start)))
        )
        self._img_cache: dict[str, np.ndarray] = {}

    def image(self, cam: str, q) -> np.ndarray | None:
        if not len(self._kq):
            return None
        w = np.ones(self._kq.shape[1])
        w[-1] = 0.2  # the gripper matters less for which view is closest
        i = int(np.argmin((((self._kq - np.asarray(q)) * w) ** 2).sum(axis=1)))
        f = self.keyframes[i]["files"].get(cam)
        if not f:
            return None
        if f not in self._img_cache:
            from PIL import Image

            self._img_cache[f] = np.asarray(Image.open(self.dir / "keyframes" / f).convert("RGB"))
        return self._img_cache[f]

    def telemetry(self, k: int, torque: bool = True, wrap: bool = True) -> dict:
        if not self.steps:
            return {}
        s = self.steps[k % len(self.steps) if wrap else min(k, len(self.steps) - 1)]
        keys = (
            ("joint_torque_nm", "temp_mos_c", "temp_rotor_c", "temp_c")
            if torque
            else ("temp_mos_c", "temp_rotor_c", "temp_c")
        )
        return {key: s[key] for key in keys if key in s}

    def measured(self, k: int) -> np.ndarray:
        """The joints measured at recorded step k (the last one after the end of the recording)."""
        return np.asarray(self.steps[min(k, len(self.steps) - 1)]["pos"], dtype=float)


class _Bus:
    """The Damiao bus as the Metal driver sees it: a per-motor cache of the last reply, updated by `_process_response`."""

    def __init__(self):
        self._last_known_states: dict = {}

    def _process_response(self, motor: str, msg: dict) -> None:
        self._last_known_states[motor] = dict(msg)


class _Cfg:
    disable_torque_on_disconnect = False


class ReplayRobot:
    """LeRobot-Robot-shaped mock driven by a fixture. Also records every command it receives (`sent`)."""

    is_calibrated = True
    calibration_fpath = "(hardware-in-the-loop mock)"

    def __init__(
        self,
        fx: Fixture,
        prof: Profile,
        cameras: list[str] | None = None,
        noise_deg: float = 0.0,
        seed: int = 0,
        playback: bool = False,
        fault: dict | None = None,
    ):
        self.fx, self.prof = fx, prof
        self.playback = bool(playback and fx.steps)
        self.q = fx.measured(0) if self.playback else fx.start.copy()
        self.cmd = self.q.copy()
        self.cameras = cameras if cameras is not None else fx.cameras
        self.k = 0
        self.connected = False
        self.sent: list[list[float]] = []
        self.bus = _Bus() if fx.meta.get("bus") == "damiao" else None
        self.config = _Cfg()
        self.rng = np.random.default_rng(seed)
        self.noise = float(noise_deg)
        self.fault = dict(fault or {})

    def _faulty(self, kind: str) -> bool:
        return self.fault.get("kind") == kind and self.k >= int(self.fault.get("at_step", 0))

    def connect(self, calibrate: bool = True) -> None:
        self.connected = True
        self._update_bus()

    def _telemetry(self) -> dict:
        # tracking mode: temperatures only (the recorded torques belong to the recorded motion)
        return self.fx.telemetry(self.k, torque=self.playback, wrap=not self.playback)

    def _update_bus(self) -> None:
        if self.bus is None or self._faulty("stale"):  # stale: no replies, the cache keeps the last ones
            return
        tel = self._telemetry()
        n = len(self.prof.joints)
        for i, name in enumerate(self.prof.joints):
            msg = {
                "position": float(self.q[i]),
                "temp_mos": (tel.get("temp_mos_c") or [30.0] * n)[i],
                "temp_rotor": (tel.get("temp_rotor_c") or [30.0] * n)[i],
                "velocity": 0.0,
            }
            if "joint_torque_nm" in tel:
                msg["torque"] = tel["joint_torque_nm"][i]
            self.bus._process_response(name, msg)

    def get_observation(self) -> dict:
        if not self.connected:
            raise RuntimeError("not connected")
        if self._faulty("dead"):
            raise ConnectionError("hardware-in-the-loop mock fault: the link is gone")
        if self._faulty("slow") and self.k < int(self.fault.get("at_step", 0)) + int(self.fault.get("steps", 1)):
            time.sleep(float(self.fault.get("seconds", 1.0)))
        self._update_bus()
        if self.bus is not None:  # positions come from the bus cache, as on the Damiao bus
            st = self.bus._last_known_states
            q = [float(st[n]["position"]) for n in self.prof.joints]
        else:
            q = [float(v) for v in self.q]
        out: dict = {f"{n}.pos": v for n, v in zip(self.prof.joints, q, strict=False)}
        for cam in self.cameras:  # the cameras see the arm where it is, whatever the bus serves
            img = self.fx.image(cam, self.q)
            if img is not None:
                out[cam] = img
        if self.bus is None:
            out["_telemetry"] = self._telemetry()
        return out

    def send_action(self, action: dict) -> dict:
        if not self.connected:
            raise RuntimeError("not connected")
        if self._faulty("dead"):
            raise ConnectionError("hardware-in-the-loop mock fault: the link is gone")
        cmd = np.asarray([float(action[f"{n}.pos"]) for n in self.prof.joints], dtype=float)
        self.sent.append(cmd.round(3).tolist())
        self.cmd = cmd
        self.k += 1
        if self.playback:  # the joints as measured at this step of the recording
            self.q = self.fx.measured(self.k)
            return action
        # the fitted tracking model: one control period of motion toward the command
        self.q = self.q + self.fx.gain * (cmd - self.q) + self.fx.bias
        if self.noise:
            self.q = self.q + self.rng.normal(0.0, self.noise, size=self.q.shape)
        return action

    def disconnect(self) -> None:
        self.connected = False


class _Obj:
    def __init__(self, **kw: Any):
        self.__dict__.update(kw)


class ReplayPiperSDK:
    """piper_sdk-shaped mock for the Piper driver (synthetic fixture: no Piper recordings exist yet). As in piper_sdk
    0.6.2's C_PiperInterface_V2, the joints arrive in three feedback frames (joints 1-2, 3-4, 5-6), each passed to the
    name-mangled `__UpdateArmJointState`, which updates that pair's joints and the one `ArmJoint.time_stamp`; here the
    frames arrive whenever the driver reads. `mock_fault` {"kind": "silent_pair", "pair": "34", "at_step": K}: from
    control step K on, that pair's frames stop and its joints freeze, while the other pairs keep the time stamp fresh."""

    def __init__(self, fx: Fixture, prof: Profile, fault: dict | None = None):
        self.robot = ReplayRobot(fx, prof, cameras=[])
        self.robot.connect()
        self.fault = dict(fault or {})
        joint_state = _Obj(**{f"joint_{i}": 0 for i in range(1, 7)})
        self._joint_msgs: Any = _Obj(time_stamp=0.0, Hz=0.0, joint_state=joint_state)  # the SDK's ArmJoint

    def ConnectPort(self):
        return True

    def EnablePiper(self):
        return True

    def DisablePiper(self):
        return True

    def _C_PiperInterface_V2__UpdateArmJointState(self, msg):  # the SDK's own update, for one joint feedback frame
        a = int(msg.type_.name[-2])  # PiperMsgJointFeedBack_34 -> joints 3 and 4
        for j in (a, a + 1):
            setattr(self._joint_msgs.joint_state, f"joint_{j}", getattr(msg.arm_joint_feedback, f"joint_{j}"))
        self._joint_msgs.time_stamp = msg.time_stamp
        return self._joint_msgs

    def _receive(self) -> None:
        """The SDK's receive thread: one frame per joint pair, as decoded by ParseCANFrame."""
        f = self.fault
        on = f.get("kind") == "silent_pair" and self.robot.k >= int(f.get("at_step", 0))
        silent = str(f.get("pair")) if on else ""
        joints = {f"joint_{i + 1}": int(round(v * 1000)) for i, v in enumerate(self.robot.q[:6])}
        for pair in ("12", "34", "56"):
            if pair != silent:
                msg = _Obj(
                    type_=_Obj(name=f"PiperMsgJointFeedBack_{pair}"),
                    time_stamp=time.time(),  # python-can's frame time (wall clock on socketcan)
                    arm_joint_feedback=_Obj(**joints),
                )
                self._C_PiperInterface_V2__UpdateArmJointState(msg)

    def GetArmJointMsgs(self):
        self._receive()
        return self._joint_msgs

    def GetArmGripperMsgs(self):
        state = _Obj(grippers_angle=int(round(self.robot.q[6] * 1000)))
        return _Obj(time_stamp=time.time(), gripper_state=state)

    def MotionCtrl_2(self, *a):
        return True

    def JointCtrl(self, *j):
        self._pending = [v / 1000.0 for v in j]

    def GripperCtrl(self, pos, effort, code, zero):
        q = list(getattr(self, "_pending", self.robot.q[:6])) + [pos / 1000.0]
        self.robot.send_action({f"{n}.pos": v for n, v in zip(self.robot.prof.joints, q, strict=False)})


# ------------------------------------------------------------------------------------------------------------------
# fitting and conformance
# ------------------------------------------------------------------------------------------------------------------


def fit_tracking(cmd: np.ndarray, pos: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Per joint least squares: pos[k] - pos[k-1] = gain * (cmd[k] - pos[k-1]) + bias; gain clipped to (0.05, 1]."""
    n = cmd.shape[1]
    gain, bias = np.ones(n), np.zeros(n)
    for j in range(n):
        e = cmd[1:, j] - pos[:-1, j]
        d = pos[1:, j] - pos[:-1, j]
        if np.std(e) < 1e-6:
            continue
        A = np.stack([e, np.ones_like(e)], axis=1)
        (g, b), *_ = np.linalg.lstsq(A, d, rcond=None)
        gain[j], bias[j] = float(np.clip(g, 0.05, 1.0)), float(np.clip(b, -3.0, 3.0))
    return gain, bias


def conformance(fx: Fixture, prof: Profile, overrides: dict | None = None) -> dict:
    """Feed the recorded commands through the Guard with the recorded measurements; simulate the tracking model."""
    from .safety import Guard, Veto

    events: list[dict] = []
    g = Guard(
        prof, overrides, log=lambda _event, **kw: events.append({**kw, "event": _event}), estop_check=lambda: None
    )
    cmd = np.asarray([s["cmd"] for s in fx.steps], dtype=float)
    pos = np.asarray([s["pos"] for s in fx.steps], dtype=float)
    vetoes, deltas, per_joint = 0, [], np.zeros(cmd.shape[1], dtype=int)
    g.last_pos, g.last_cmd = pos[0], cmd[0]
    for k in range(1, len(cmd)):
        try:
            out = g.step_command(cmd[k])
            deltas.append(float(np.max(np.abs(out - cmd[k]))))
            per_joint += (np.abs(out - cmd[k]) > 0.05).astype(int)
        except Veto:
            vetoes += 1
        g.last_cmd, g.last_pos = cmd[k], pos[k]
    # open-loop simulation of the tracking model from the recorded commands
    sim = [pos[0]]
    for k in range(1, len(cmd)):
        sim.append(sim[-1] + fx.gain * (cmd[k] - sim[-1]) + fx.bias)
    sim = np.asarray(sim)
    err = np.abs(sim - pos)[:, prof.arm_idx]
    one_step = np.abs((pos[:-1] + fx.gain * (cmd[1:] - pos[:-1]) + fx.bias) - pos[1:])
    return {
        "fixture": fx.dir.name,
        "robot": prof.name,
        "steps": int(len(cmd)),
        "vetoed_steps": vetoes,
        "clamp_events": sum(1 for e in events if e["event"] == "clamp"),
        "guard_changed_steps": int(sum(1 for d in deltas if d > 0.05)),
        "max_guard_change_deg": round(max(deltas) if deltas else 0.0, 3),
        "guard_changed_steps_by_joint": {n: int(c) for n, c in zip(prof.joints, per_joint, strict=False) if c},
        "tracking_one_step_rms_deg": round(float(np.sqrt(np.mean(one_step[:, prof.arm_idx] ** 2))), 3),
        "tracking_open_loop_rms_deg": round(float(np.sqrt(np.mean(err**2))), 3),
        "tracking_open_loop_max_deg": round(float(err.max()), 3),
    }
