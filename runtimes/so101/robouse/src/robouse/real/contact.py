"""Contact watch for real arms, ported from the 2026-09-05 Metal harness (~/benchflow/robot/safety.py,
SafeArm.move_joints), which watched descents near the table on the MakerMods Metal arm.

Fed once per control step (the command just sent, the joints and telemetry read after it), it follows the motion from the
command stream (a move, the arm settling after it, a hold) and reports a contact when:

  moving   during a move, after `transient_steps` steps (a position controller's torque jumps when a move starts), a
           joint's torque leaves the torque at that step by more than its threshold, with at least `jump` of the
           threshold within this one step: a jump against the moving trend.
  settled  after a watched descent, once the arm has settled, the torque has shifted from the baseline taken before the
           move on `settle_joints` joints at once, or on one joint by more than `strong` x its threshold (on one joint
           alone a settled shift is usually the command lead changing with posture; a real contact unloads two or three).
  holding  while the command holds still, a joint's torque jumps away from where the hold started.
  stall    (arms with a kinematic chain) once a watched descent has settled, the tool point is more than `stall_mm`
           above where the commanded descent puts it: the measured height stopped following the command. The Sep 5
           touchdown found the true table floor this way; on that arm torque alone missed it.

A step is watched when the command or the target puts the tool point within `watch_height` of the table (the backend
decides; the Sep 5 harness watched its descents below 3 cm) and the move is not rising. Elsewhere the torque
thresholds are `unwatched_factor` times larger: moves away from the table, and above it, change the gravity load far
more than a light touch does. Gripper motion is not a move, and the gripper's own torque is not watched (grasping is
contact by design), but the arm joints stay watched while the gripper moves, as when a policy streams joint targets
that move both at once: on the Sep 5 recordings, the 24 gripper motions made with the arm still moved the arm joints'
torque by at most 0.74 of a threshold (shoulder_lift) against the torque before the gripper started. What a contact
does (stop the move and hold, or stop the episode) is up to the backend (backends/real/backend.py).

Measured offline on the 2026-09-05 recordings (10,747 control steps in 22 sessions with torque), the watch fires on
both verified contacts (the 12:17 table touchdown as a stall, the 13:16 elbow stop as a jump outside a watched move),
on the HIL fixture's two descent contacts, 11 more times within 1.5 s of a contact the Sep 5 harness flagged itself
(true or not; its READMEs call many of those artifacts), and nowhere else. Settings that differ from the Sep 5
harness: the single-joint settled limit (`strong` 2.5, not 2.0: a normal descent reached 2.02), the watched zone
(moves ending within `watch_height` of the table and not rising), and the larger thresholds elsewhere
(`unwatched_factor`: 4 was the smallest quiet on the recordings; 5 still stops the elbow one step after it hit). The
recorded traces and their replays through the episode server are in tests/fixtures/hil and tests/test_real.py.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

import numpy as np

from .robots import ContactSettings, Profile


@dataclass
class Contact:
    kind: str  # moving | settled | holding | stall | lag (parks)
    watched: bool  # in a watched move (near the table)
    change: dict[str, float]  # torque change per joint, {"tool_shortfall_mm": mm} for a stall, {joint: deg} for a lag
    step: int  # the watch's control step count when it was seen
    unit: str = "deg"  # of a lag

    def describe(self) -> str:
        if self.kind == "stall":
            return (
                f"the tool point stopped {self.change['tool_shortfall_mm']:.1f} mm above where the commanded descent "
                "puts it (it is resting on something)"
            )
        if self.kind == "lag":
            ((j, v),) = self.change.items()
            return f"{j} lags its command by {v:.2f} {self.unit} (something holds the arm back)"
        what = {
            "moving": "a torque jump during the move",
            "settled": "a torque shift after the move",
            "holding": "a torque jump while holding still",
        }[self.kind]
        return what + " (" + ", ".join(f"{j} {v:+.2f}" for j, v in self.change.items()) + " against the reference)"

    def record(self) -> dict:
        return {"kind": self.kind, "watched": self.watched, "change": self.change, "step": self.step}


class ContactWatch:
    MOVE_EPS = 0.01  # deg: smaller command changes count as holding still
    SETTLE_EPS = 0.05  # deg per step: the arm has settled when no arm joint moves more than this ...
    SETTLE_STEPS = 2  # ... for this many steps in a row,
    SETTLE_MAX = 30  # or this many steps after the command stopped
    DESCENT_RESET_M = (0.001, 0.02)  # a descent ends when the command rises 1 mm or moves 2 cm sideways

    def __init__(self, prof: Profile, settings: ContactSettings, tool: Callable | None = None):
        self.s = settings
        self.names = list(prof.joints)
        self.arm = list(prof.arm_idx)
        # arm joints with a torque threshold (a gripper threshold is ignored: grasping is contact by design)
        self.ti = [i for i in self.arm if prof.joints[i] in settings.dtorque]
        self.lim = np.asarray([settings.dtorque[prof.joints[i]] for i in self.ti], dtype=float)
        self.tool = tool  # q -> tool point (m, base frame); None: no stall check
        self.n = 0
        self.reset()

    @property
    def torque_watch(self) -> bool:
        return bool(self.ti)

    def reset(self) -> None:
        """Forget the motion and its references (after a contact, a stop or a reconnect): fresh baselines from now."""
        self.prev_cmd: np.ndarray | None = None
        self.prev_q: np.ndarray | None = None
        self.prev_tq: np.ndarray | None = None
        self.phase = "holding"  # moving | settling | holding
        self.move_k = 0  # steps into the current move
        self.moving_base: np.ndarray | None = None
        self.baseline: np.ndarray | None = None  # settled torque before the current move
        self.move_watched = False
        self.settle_k = 0
        self.still = 0
        self.hold_ref: np.ndarray | None = None  # torque where the current hold started
        self.descent: tuple[np.ndarray, np.ndarray] | None = None  # (commanded, measured) tool point at its start
        self.low_z = 0.0  # lowest commanded tool height of the descent
        self.rest: tuple[np.ndarray, np.ndarray] | None = None  # (commanded, measured) tool point last at rest
        self.prev_zc: float | None = None  # commanded tool height of the previous step
        self.move_z0: float | None = None  # commanded tool height before the current move
        self.recent: list[np.ndarray] = []  # torque of the last steps holding still

    def update(self, cmd, q, extra: dict, watched: bool, factor: float | None = None) -> Contact | None:
        """One control step: the command just sent, the joints and telemetry read after it. `watched`: the step is in
        a watched move (near the table). `factor`: the torque thresholds are this many times their base values
        everywhere (parking uses its own); by default 1 in a watched move and `unwatched_factor` elsewhere."""
        self.n += 1
        cmd, q = np.asarray(cmd, dtype=float), np.asarray(q, dtype=float)
        raw = extra.get(self.s.torque_key)
        tq = None
        if self.ti and isinstance(raw, (list, tuple)) and len(raw) == len(self.names):
            tq = np.asarray(raw, dtype=float)[self.ti]
        prev = self.prev_cmd
        moving = prev is not None and float(np.max(np.abs(cmd[self.arm] - prev[self.arm]))) > self.MOVE_EPS
        zc = float(np.asarray(self.tool(cmd), dtype=float)[2]) if self.tool is not None else None
        if moving and self.phase != "moving":
            self.move_z0 = self.prev_zc  # the commanded tool height this move starts from
        in_move = moving or self.phase != "holding"
        dz = zc - self.move_z0 if in_move and zc is not None and self.move_z0 is not None else 0.0
        watched = watched and not dz > 0.001  # a rising move leaves the table: the larger thresholds
        settled = self._phase(moving, watched, q)
        hit = self._stall(cmd, q, watched, settled)
        if hit is None and tq is not None:
            hit = self._torque(tq, watched, settled, descended=dz < -0.001, factor=factor)
        self.prev_cmd, self.prev_q, self.prev_zc = cmd, q, zc
        if tq is not None:
            self.prev_tq = tq
            # the last few readings while holding still: the next move's baseline
            self.recent = (self.recent + [tq])[-3:] if self.phase == "holding" else []
        if hit is not None:
            # fresh references once the arm has settled at the hold: the contact force releasing is not a new contact
            self.reset()
            self.phase = "settling"
        return hit

    # ---- motion phase from the command stream ---------------------------------------------------------------------
    def _phase(self, moving: bool, watched: bool, q: np.ndarray) -> bool:
        """Advance moving / settling / holding; True on the step the arm has settled after a move."""
        if moving:
            if self.phase != "moving":  # a move starts: the baseline is the torque just before it (Sep 5: 2 hold steps)
                self.baseline = np.mean(self.recent, axis=0) if self.recent else self.prev_tq
                self.move_k, self.moving_base, self.move_watched = 0, None, False
            self.phase = "moving"
            self.move_k += 1
            self.move_watched = self.move_watched or watched
            self.hold_ref = None
            return False
        if self.phase == "moving":  # the move's commands have ended: the arm converges
            self.phase, self.settle_k, self.still = "settling", 0, 0
        if self.phase != "settling":
            return False
        self.settle_k += 1
        if self.prev_q is not None and float(np.max(np.abs(q[self.arm] - self.prev_q[self.arm]))) < self.SETTLE_EPS:
            self.still += 1
        else:
            self.still = 0
        if self.still >= self.SETTLE_STEPS or self.settle_k >= self.SETTLE_MAX:
            self.phase = "holding"
            return True
        return False

    # ---- torque ---------------------------------------------------------------------------------------------------
    def _torque(
        self, tq: np.ndarray, watched: bool, settled: bool, descended: bool, factor: float | None
    ) -> Contact | None:
        lim = self.lim * (factor if factor is not None else 1.0 if watched else self.s.unwatched_factor)
        settle_lim = self.lim * (factor if factor is not None else 1.0)  # the settled rule: watched descents only
        if self.phase == "moving":
            if (
                self.move_k == self.s.transient_steps
                or self.moving_base is None
                and self.move_k > self.s.transient_steps
            ):
                self.moving_base = tq
            elif self.move_k > self.s.transient_steps and self.prev_tq is not None:
                return self._jump("moving", tq, self.moving_base, lim, watched)
            return None
        if settled:
            ref, self.hold_ref = self.baseline, tq
            if self.move_watched and descended and ref is not None:  # a contact from below ends a descent
                d = tq - ref
                over = np.abs(d) > settle_lim
                if over.sum() >= self.s.settle_joints or bool(np.any(np.abs(d) > self.s.strong * settle_lim)):
                    return Contact("settled", True, self._joints(d, over), self.n)
            return None
        if self.phase == "holding":
            if self.hold_ref is None:
                self.hold_ref = tq
                return None
            if self.prev_tq is not None:
                return self._jump("holding", tq, self.hold_ref, lim, watched)
        return None

    def _jump(self, kind: str, tq: np.ndarray, ref, lim: np.ndarray, watched: bool) -> Contact | None:
        assert self.prev_tq is not None
        d, dd = tq - ref, tq - self.prev_tq
        over = (np.abs(d) > lim) & (np.abs(dd) > self.s.jump * lim)
        return Contact(kind, watched, self._joints(d, over), self.n) if over.any() else None

    def _joints(self, d: np.ndarray, over: np.ndarray) -> dict[str, float]:
        return {self.names[self.ti[i]]: round(float(d[i]), 2) for i in np.flatnonzero(over)}

    # ---- stall (the touchdown criterion) --------------------------------------------------------------------------
    def _stall(self, cmd: np.ndarray, q: np.ndarray, watched: bool, settled: bool) -> Contact | None:
        """Compare settled poses only (a moving arm lags its command): from the settled pose a watched descent started
        at, the measured tool point must come down as far as the command did. What it falls short is how much the gravity
        sag has shrunk, i.e. how much of the arm's weight something under the tool now carries."""
        if self.tool is None:
            return None
        pc, pm = np.asarray(self.tool(cmd), dtype=float), np.asarray(self.tool(q), dtype=float)
        at_rest = self.phase == "holding"  # _phase has run: this includes the step the arm settled on
        before = self.rest  # the last pose at rest before this step
        if at_rest:
            self.rest = (pc, pm)
        if not watched:
            self.descent = None
            return None
        d = self.descent

        def side(pose: tuple[np.ndarray, np.ndarray]) -> float:  # how far the command is from the pose, sideways
            return float(np.linalg.norm(pc[:2] - pose[0][:2]))

        if d is None or pc[2] > self.low_z + self.DESCENT_RESET_M[0] or side(d) > self.DESCENT_RESET_M[1]:
            # a new descent: from the settled pose the move started at, if it is right above; else from the next rest
            near = before is not None and side(before) <= self.DESCENT_RESET_M[1] and before[0][2] >= pc[2] - 1e-4
            self.descent = before if near else ((pc, pm) if at_rest else None)
            self.low_z = float(pc[2])
            if not (near and settled):
                return None
        self.low_z = min(self.low_z, float(pc[2]))
        if not settled or self.descent is None:
            return None
        c0, m0 = self.descent
        descent_mm = (c0[2] - pc[2]) * 1000
        shortfall_mm = ((pm[2] - m0[2]) - (pc[2] - c0[2])) * 1000
        if descent_mm > self.s.stall_mm and shortfall_mm > self.s.stall_mm:
            return Contact("stall", True, {"tool_shortfall_mm": round(shortfall_mm, 1)}, self.n)
        return None


class ParkWatch:
    """The watch on a park, the move to the rest pose, where the contact watch's torque rules alone would stop most
    parks (`unwatched_factor` exists because gravity loads change far more on moves away from the table than a touch
    does). Three detectors, any of which stops the park:

      lag     an arm joint's measured position lags its command by more than the lag limit. A blocked joint's lag climbs
              to the wind-up limit. A calibrated limit (`park_lag_deg`) comes from the robot's recorded motion at the
              park speed (robots.py), which is why parks are held to `park_speed_deg_s`; without one, the limit is
              `COARSE_LAG` of the wind-up limit, which a fully blocked joint reaches only once its command leads it by
              nearly the whole wind-up limit, so it pushes for several steps first
      stall   near the table, the tool point stops short of a commanded descent (the contact watch's stall check)
      torque  the contact watch's torque rules at `park_torque_factor` times the contact thresholds: the backstop

    Only a robot with a calibrated lag limit (`calibrated`) parks on its own; any other parks only on the operator's
    yes, with the coarse limit (backends/real/backend.py).
    """

    COARSE_LAG = 0.9  # without a calibration, the lag limit is this part of the wind-up limit

    def __init__(
        self,
        prof: Profile,
        settings: ContactSettings,
        tool: Callable | None = None,
        table_z: float = 0.0,
        windup: float = 0.0,
    ):
        self.s, self.arm, self.names, self.units = settings, list(prof.arm_idx), list(prof.joints), list(prof.units)
        self.tool, self.table_z = tool, table_z
        self.calibrated = settings.park_lag_deg > 0
        self.lag_limit = settings.park_lag_deg if self.calibrated else self.COARSE_LAG * windup  # 0: no lag watch
        self.watch = ContactWatch(prof, settings, tool=tool)

    def reset(self) -> None:
        self.watch.reset()

    def update(self, cmd, q, extra: dict) -> Contact | None:
        cmd, q = np.asarray(cmd, dtype=float), np.asarray(q, dtype=float)
        if self.lag_limit > 0:
            lag = np.abs(cmd[self.arm] - q[self.arm])
            j = int(np.argmax(lag))
            if lag[j] > self.lag_limit:
                self.watch.n += 1
                i = self.arm[j]
                return Contact("lag", False, {self.names[i]: round(float(lag[j]), 2)}, self.watch.n, self.units[i])
        near = False
        if self.tool is not None:
            near = float(np.asarray(self.tool(cmd), dtype=float)[2]) - self.table_z < self.s.watch_height
        return self.watch.update(cmd, q, extra, near, factor=self.s.park_torque_factor)
