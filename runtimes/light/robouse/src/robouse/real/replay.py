"""Hardware-in-the-loop mock: replays recorded real-robot sessions behind the vendor SDK seam.

A fixture (`assets/hil/<name>/fixture.json` + `keyframes/*.jpg`, built by `scripts/make_hil_fixture.py` from a session
recorded on the bench by ~/benchflow/robot/recorder.py) holds:
  - the recorded control log: per step the command sent, the measured joints, torques and temperatures;
  - a per-joint tracking model fitted to that log (measured_next = measured + gain * (command - measured) + bias), so
    the mock follows commands the way the real arm did (lag, gravity sag);
  - keyframes: measured poses with the camera frames recorded at that moment.

`ReplayRobot` has LeRobot's `Robot` shape (connect / get_observation / send_action / disconnect, plus the Damiao bus
cache the Metal driver reads), so the real driver code (drivers/lerobot_arm.py), the safety guard and the episode server
run unchanged against it. Camera frames are the recorded frames whose pose is closest to the mock's current pose;
telemetry replays the recorded sequence. Nothing in the mock models contact with objects: manipulation outcomes on the
mock are decided by the (scripted) operator, never inferred.

`conformance()` is the strict replay check behind `robouse hil-check`: it feeds the recorded command sequence through the
Robo Use safety guard, with the measured joints taken from the recording, and reports vetoes, clamps and how well the
fitted tracking model reproduces the recorded motion.
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from .robots import Profile

HIL_DIR = Path(__file__).resolve().parents[3] / "assets" / "hil"


def fixture_dir(name: str) -> Path:
    import os

    p = Path(name)
    if p.is_dir():
        return p
    for root in [os.environ.get("ROBOUSE_HIL_DIR", ""), str(HIL_DIR)]:
        if root and (Path(root) / name).is_dir():
            return Path(root) / name
    raise FileNotFoundError(f"hardware-in-the-loop fixture {name!r} not found (looked in ROBOUSE_HIL_DIR and {HIL_DIR})")


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
        self._kq = np.asarray([k["q"] for k in self.keyframes], dtype=float) if self.keyframes else np.zeros((0, len(self.start)))
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

    def telemetry(self, k: int) -> dict:
        if not self.steps:
            return {}
        s = self.steps[k % len(self.steps)]
        return {key: s[key] for key in ("joint_torque_nm", "temp_mos_c", "temp_rotor_c", "temp_c") if key in s}


class _Bus:
    def __init__(self):
        self._last_known_states: dict = {}


class _Cfg:
    disable_torque_on_disconnect = False


class ReplayRobot:
    """LeRobot-Robot-shaped mock driven by a fixture. Also records every command it receives (`sent`)."""

    is_calibrated = True
    calibration_fpath = "(hardware-in-the-loop mock)"

    def __init__(self, fx: Fixture, prof: Profile, cameras: list[str] | None = None, noise_deg: float = 0.0, seed: int = 0):
        self.fx, self.prof = fx, prof
        self.q = fx.start.copy()
        self.cmd = self.q.copy()
        self.cameras = cameras if cameras is not None else fx.cameras
        self.k = 0
        self.connected = False
        self.sent: list[list[float]] = []
        self.bus = _Bus() if fx.meta.get("bus") == "damiao" else None
        self.config = _Cfg()
        self.rng = np.random.default_rng(seed)
        self.noise = float(noise_deg)

    def connect(self, calibrate: bool = True) -> None:
        self.connected = True

    def _update_bus(self) -> None:
        if self.bus is None:
            return
        tel = self.fx.telemetry(self.k)
        st = {}
        for i, n in enumerate(self.prof.joints):
            st[n] = {"torque": (tel.get("joint_torque_nm") or [0.0] * 7)[i], "temp_mos": (tel.get("temp_mos_c") or [30.0] * 7)[i],
                     "temp_rotor": (tel.get("temp_rotor_c") or [30.0] * 7)[i], "velocity": 0.0}
        self.bus._last_known_states = st

    def get_observation(self) -> dict:
        if not self.connected:
            raise RuntimeError("not connected")
        self._update_bus()
        out = {f"{n}.pos": float(v) for n, v in zip(self.prof.joints, self.q)}
        for cam in self.cameras:
            img = self.fx.image(cam, self.q)
            if img is not None:
                out[cam] = img
        if self.bus is None:
            out["_telemetry"] = self.fx.telemetry(self.k)
        return out

    def send_action(self, action: dict) -> dict:
        if not self.connected:
            raise RuntimeError("not connected")
        cmd = np.asarray([float(action[f"{n}.pos"]) for n in self.prof.joints], dtype=float)
        self.sent.append(cmd.round(3).tolist())
        self.cmd = cmd
        # the fitted tracking model: one control period of motion toward the command
        self.q = self.q + self.fx.gain * (cmd - self.q) + self.fx.bias
        if self.noise:
            self.q = self.q + self.rng.normal(0.0, self.noise, size=self.q.shape)
        self.k += 1
        return action

    def disconnect(self) -> None:
        self.connected = False


class ReplayPiperSDK:
    """piper_sdk-shaped mock for the Piper driver (synthetic fixture: no Piper recordings exist yet)."""

    def __init__(self, fx: Fixture, prof: Profile):
        self.robot = ReplayRobot(fx, prof, cameras=[])
        self.robot.connect()

    def ConnectPort(self):  # noqa: N802 - SDK naming
        return True

    def EnablePiper(self):  # noqa: N802
        return True

    def DisablePiper(self):  # noqa: N802
        return True

    def GetArmJointMsgs(self):  # noqa: N802
        q = self.robot.q

        class JS:  # noqa: D106
            pass
        js = JS()
        for i in range(6):
            setattr(js, f"joint_{i + 1}", int(round(q[i] * 1000)))
        m = JS()
        m.joint_state = js
        return m

    def GetArmGripperMsgs(self):  # noqa: N802
        class G:  # noqa: D106
            pass
        g, m = G(), G()
        g.grippers_angle = int(round(self.robot.q[6] * 1000))
        m.gripper_state = g
        return m

    def MotionCtrl_2(self, *a):  # noqa: N802
        return True

    def JointCtrl(self, *j):  # noqa: N802
        self._pending = [v / 1000.0 for v in j]

    def GripperCtrl(self, pos, effort, code, zero):  # noqa: N802
        q = list(getattr(self, "_pending", self.robot.q[:6])) + [pos / 1000.0]
        self.robot.send_action({f"{n}.pos": v for n, v in zip(self.robot.prof.joints, q)})


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
    g = Guard(prof, overrides, log=lambda kind, **kw: events.append({"kind": kind, **kw}), estop_check=lambda: None)
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
    return {"fixture": fx.dir.name, "robot": prof.name, "steps": int(len(cmd)), "vetoed_steps": vetoes,
            "clamp_events": sum(1 for e in events if e["kind"] == "clamp"),
            "guard_changed_steps": int(sum(1 for d in deltas if d > 0.05)),
            "max_guard_change_deg": round(max(deltas) if deltas else 0.0, 3),
            "guard_changed_steps_by_joint": {n: int(c) for n, c in zip(prof.joints, per_joint) if c},
            "tracking_one_step_rms_deg": round(float(np.sqrt(np.mean(one_step[:, prof.arm_idx] ** 2))), 3),
            "tracking_open_loop_rms_deg": round(float(np.sqrt(np.mean(err ** 2))), 3),
            "tracking_open_loop_max_deg": round(float(err.max()), 3)}
