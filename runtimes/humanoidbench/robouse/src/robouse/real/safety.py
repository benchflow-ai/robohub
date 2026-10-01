"""Safety layer for real arms, enforced inside the trusted episode server (the agent cannot bypass it).

Every command goes through `Guard.command()`, in this order (a port of ~/benchflow/robot/safety.py and the
inspect-robots-metal adapter, which ran the 2026-09-05 Metal arm sessions without damage):

1. e-stop latch: if the e-stop file exists (`robouse estop`), nothing is sent; the episode ends with `safety_stop`.
2. finite values, right length; clamp to the joint limits (bounds clamp).
3. workspace envelope by forward kinematics: the straight joint-space path from the last command to the target is
   sampled and every monitored point (elbow, wrist, flange, tool point) must stay above the table, inside the box, and
   high enough when behind the base. A point already outside at the start (e.g. an arm resting on the table) may only
   move back toward the envelope. A violating move is vetoed before anything is sent (`Veto`); the agent sees why.
4. per-step delta limit (velocity ceiling; slower in the slow zone near the table) from the last command.
5. wind-up guard: the command never leads the measured position by more than `windup_deg`, so a blocked joint pushes
   with a bounded torque.
After each step: thermal guard (warn, then stop and hold above `temp_stop_c`) and a loop watchdog (a step slower than
`watchdog_s` is logged). Every veto, clamp, warning and stop is appended to the intervention log (events.jsonl).

`HardwareLock` makes sure only one controller uses a robot at a time (flock on ~/.config/robouse-operator/<robot>.lock).
"""

from __future__ import annotations

import fcntl
import json
import os
import time
from pathlib import Path

import numpy as np

from .. import config
from .robots import EnvelopeDefaults, Profile

OPERATOR_DIR = config.operator_dir()


class Veto(RuntimeError):
    """The requested motion would leave the safety envelope; nothing was sent to the robot."""


class SafetyStop(RuntimeError):
    """E-stop, over-temperature or a hardware fault: the episode must end and the robot hold (halt-for-human)."""


def estop_path(robot: str = "") -> Path:
    return config.path("ROBOUSE_ESTOP_FILE", OPERATOR_DIR / "ESTOP")


def estop_engaged(robot: str = "") -> str | None:
    p = estop_path(robot)
    if p.exists():
        try:
            return p.read_text().strip() or "e-stop engaged"
        except OSError:
            return "e-stop engaged"
    return None


def set_estop(reason: str = "operator e-stop") -> Path:
    p = estop_path()
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(f"{reason} ({time.strftime('%Y-%m-%d %H:%M:%S')})\n")
    return p


def clear_estop() -> bool:
    p = estop_path()
    if p.exists():
        p.unlink()
        return True
    return False


class HardwareLock:
    """One controller per robot: an exclusive, non-blocking flock held for the life of the episode."""

    def __init__(self, robot: str, port: str = ""):
        OPERATOR_DIR.mkdir(parents=True, exist_ok=True)
        tag = "".join(c if c.isalnum() else "_" for c in f"{robot}-{port}".strip("-"))
        self.path = OPERATOR_DIR / f"{tag}.lock"
        self._fh = open(self.path, "a+")  # noqa: SIM115 - held (and locked) for the whole session
        try:
            fcntl.flock(self._fh, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError as exc:
            self._fh.close()
            raise SafetyStop(f"another process controls {robot} ({self.path}); only one controller may use it") from exc
        self._fh.seek(0)
        self._fh.truncate()
        self._fh.write(f"{os.getpid()} {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
        self._fh.flush()

    def release(self) -> None:
        try:
            fcntl.flock(self._fh, fcntl.LOCK_UN)
        finally:
            self._fh.close()


class Envelope:
    """Workspace checks on the monitored points of a profile's kinematic chain (joint limits only without a chain)."""

    def __init__(self, prof: Profile, overrides: dict | None = None):
        self.prof = prof
        env = EnvelopeDefaults(**{**prof.envelope.__dict__, **(overrides or {})})
        self.e = env
        self.chain = prof.chain()
        self.low = np.asarray(prof.low, dtype=float)
        self.high = np.asarray(prof.high, dtype=float)

    def floor_of(self, name: str) -> float:
        return self.e.table_z + (self.e.tool_clearance if name == "tool" else self.e.body_min_z)

    def violation(self, q) -> dict[str, float]:
        """Per monitored point, how far (m) it is outside the envelope (0 = inside); empty without a chain."""
        if self.chain is None:
            return {}
        out = {}
        for name, p in self.chain.monitored(q).items():
            v = max(
                0.0, self.floor_of(name) - p[2], p[2] - self.e.z_max, abs(p[1]) - self.e.y_abs_max, p[0] - self.e.x_max
            )
            if p[0] < self.e.x_min:
                v = max(v, self.e.table_z + self.e.behind_min_z - p[2])
            out[name] = float(v)
        return out

    def describe(self, q) -> list[str]:
        if self.chain is None:
            return []
        pts = self.chain.monitored(q)
        return [
            f"{n} at {np.round(pts[n], 3).tolist()} is {v * 1000:.0f} mm outside the workspace (table z={self.e.table_z:.3f}, "
            f"box x<={self.e.x_max}, |y|<={self.e.y_abs_max}, z<={self.e.z_max})"
            for n, v in self.violation(q).items()
            if v > 1e-4
        ]

    def check_path(self, q_from, q_to, samples: int = 20) -> None:
        """Raise Veto if the straight joint-space path leaves the envelope (points already outside may only recover)."""
        q_from, q_to = np.asarray(q_from, dtype=float), np.asarray(q_to, dtype=float)
        if self.chain is None:
            return
        start = self.violation(q_from)
        for s in np.linspace(0.0, 1.0, samples)[1:]:
            q = (1 - s) * q_from + s * q_to
            v = self.violation(q)
            bad = [n for n, x in v.items() if x > 1e-4 and x > start.get(n, 0.0) + 1e-4]
            if bad:
                raise Veto(
                    f"path leaves the workspace at {s:.0%} of the move: "
                    + "; ".join(d for d in self.describe(q) if d.split(" ")[0] in bad)
                )

    def in_slow_zone(self, q) -> bool:
        if self.chain is None:
            return False
        pts = self.chain.monitored(q)
        return min(p[2] - self.floor_of(n) for n, p in pts.items()) < self.e.slow_zone


class Guard:
    """Applies the safety layer to each command. `log(kind, **fields)` records interventions."""

    def __init__(self, prof: Profile, overrides: dict | None = None, log=None, estop_check=estop_engaged):
        self.prof = prof
        self.env = Envelope(prof, overrides)
        self.e = self.env.e
        self.hz = float(prof.control_hz)
        self.log = log or (lambda _kind, **kw: None)
        self.estop_check = estop_check
        n = len(prof.joints)
        speeds = np.full(n, self.e.joint_speed_deg_s / self.hz)
        slow = np.full(n, self.e.slow_joint_speed_deg_s / self.hz)
        wind = np.full(n, self.e.windup_deg)
        for g in prof.gidx:
            speeds[g] = slow[g] = self.e.gripper_speed / self.hz
            wind[g] = self.e.windup_gripper
        self.max_step, self.max_step_slow, self.windup = speeds, slow, wind
        self.last_cmd: np.ndarray | None = None
        self.last_pos: np.ndarray | None = None
        self._warned_at = -1e9
        self.n_clamped = 0

    def check_estop(self) -> None:
        why = self.estop_check()
        if why:
            self.log("estop", reason=why)
            raise SafetyStop(f"e-stop: {why}")

    def validate_target(self, target) -> np.ndarray:
        t = np.asarray(target, dtype=float).reshape(-1)
        if t.shape != (len(self.prof.joints),):
            raise Veto(f"expected {len(self.prof.joints)} joint values {self.prof.joints}, got {t.shape[0]}")
        if not np.all(np.isfinite(t)):
            raise Veto("joint values must be finite numbers")
        c = np.clip(t, self.env.low, self.env.high)
        if np.any(np.abs(c - t) > 1e-9):
            self.n_clamped += 1
            self.log("clamp", kind="joint_limits", requested=np.round(t, 2).tolist(), clamped=np.round(c, 2).tolist())
        return c

    def check_move(self, target) -> np.ndarray:
        """Validate a whole move (target pose) before starting it; returns the clamped target."""
        self.check_estop()
        tgt = self.validate_target(target)
        start = self.last_cmd if self.last_cmd is not None else self.last_pos
        if start is not None:
            try:
                self.env.check_path(start, tgt)
            except Veto as v:
                self.log("veto", reason=str(v), target=np.round(tgt, 2).tolist())
                raise
        return tgt

    def step_command(self, target) -> np.ndarray:
        """The command actually sent this control step toward `target` (already path-checked)."""
        self.check_estop()
        tgt = self.validate_target(target)
        if self.last_cmd is None:
            return tgt if self.last_pos is None else self.last_pos.copy()
        ms = self.max_step_slow if self.env.in_slow_zone(self.last_cmd) else self.max_step
        cmd = np.clip(tgt, self.last_cmd - ms, self.last_cmd + ms)
        if self.last_pos is not None:
            cmd = np.clip(cmd, self.last_pos - self.windup, self.last_pos + self.windup)
        try:  # the one-step move must also stay in the envelope (catches drift between the check and now)
            self.env.check_path(self.last_cmd, cmd, samples=3)
        except Veto as v:
            self.log("veto", reason=str(v), target=np.round(cmd, 2).tolist())
            raise
        return cmd

    def after_step(self, pos, extra: dict, dt: float) -> None:
        self.last_pos = np.asarray(pos, dtype=float)
        temps = [float(x) for k in ("temp_mos_c", "temp_rotor_c", "temp_c") for x in (extra.get(k) or [])]
        if temps and max(temps) > self.e.temp_stop_c:
            self.log("stop", kind="overtemperature", temps=temps, limit_c=self.e.temp_stop_c)
            raise SafetyStop(f"motor temperature {max(temps):.0f} C above {self.e.temp_stop_c:.0f} C")
        if temps and max(temps) > self.e.temp_warn_c and time.time() - self._warned_at > 5:
            self._warned_at = time.time()
            self.log("warn", kind="temperature", temps=temps)
        if dt > self.e.watchdog_s:
            self.log("warn", kind="loop_stall", dt=round(dt, 3))


class EventLog:
    """Session files next to the episode record: log.jsonl (every control step) and events.jsonl (interventions)."""

    def __init__(self, run_dir: Path | None):
        self.t0 = time.time()
        self.events: list[dict] = []
        self._log = self._ev = None
        if run_dir is not None:
            run_dir.mkdir(parents=True, exist_ok=True)
            self._log = open(run_dir / "log.jsonl", "w")  # noqa: SIM115 - open for the whole session
            self._ev = open(run_dir / "events.jsonl", "w")  # noqa: SIM115

    def event(self, _kind: str, **kw) -> None:
        rec = {"t": round(time.time() - self.t0, 3), "event": _kind, **kw}
        self.events.append(rec)
        if self._ev:
            self._ev.write(json.dumps(rec, default=float) + "\n")
            self._ev.flush()

    def step(self, **kw) -> None:
        if self._log:
            self._log.write(json.dumps({"t": round(time.time() - self.t0, 3), **kw}, default=float) + "\n")
            self._log.flush()

    def close(self) -> None:
        for f in (self._log, self._ev):
            if f:
                f.close()
