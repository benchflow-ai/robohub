"""Real robot arms behind the same episode server and `robo` interface as the simulators.

The task's `real:` block picks the robot profile (metal, so101, piper), the driver (`lerobot` / `piper` for live
hardware, `mock` for the hardware-in-the-loop mock that replays a recorded session) and the safety and operator
settings. Everything the agent sends goes through the safety guard (real/safety.py) inside this trusted process:
e-stop latch, joint-limit clamp, workspace envelope by forward kinematics (vetoed before anything moves), per-step
velocity limit (slower near the table), wind-up guard, stale-telemetry stop, thermal stop and loop watchdog; the
contact watch (real/contact.py) stops a move that touches something near the table and the episode on any other
contact (and, during the settle steps after the episode ends, any contact). Nothing moves on its own after a stop or a
refused gate: the arm joints' command goes to their measured position within the per-step delta limit, the gripper
keeps its command, and the arm holds; it is watched (temperatures, telemetry) until the operator has answered at close,
and parks only on their yes (`_leave_arm`). After a normal end it parks slowly at the profile's rest pose, watched by
tracking lag, the stall check and torque (contact.ParkWatch), and torque is released there; a park that meets anything
holds and is recorded as a stop after the end. Between requests and during operator questions the arm is checked at
least once a second (`idle_check`). The operator channel (real/operator.py) confirms a physical e-stop before a live
robot is armed, gates arming and scene reset and, for tasks without a measurable goal, gives the verdict.

Action (`robo act`): absolute joint targets in the profile's units, one per joint including the gripper, applied at the
control rate through the guard; `--repeat N` keeps commanding the same target for N control steps. Skills:
`move_joints` (interpolated move to a joint pose, checked end to end first), `move_to` (tool point to a position in the
base frame by inverse kinematics), `grip`, `home`, `park`.

Success (`real.verdict`): `pose` (the measured tool point within `goal.tol` of `goal.tool`, and the gripper open or
closed if `goal.gripper` says so; judged from the measured joints, which exist on real hardware) or `operator` (the
operator's yes/no after `robo done`). Safety stops score 0.
"""

from __future__ import annotations

import time
from pathlib import Path

import numpy as np

from ... import config
from ...real.contact import ContactWatch, ParkWatch
from ...real.drivers import make_driver
from ...real.operator import Operator
from ...real.robots import ContactSettings
from ...real.robots import profile as get_profile
from ...real.safety import EventLog, Guard, HardwareLock, SafetyStop, Veto, estop_engaged
from ..embodied import ActionGroup, Budget, EmbodiedBackend, Embodiment, Sensor, Skill, SkillArg


def real_config(spec: dict) -> dict:
    """The task's `real:` block; the operator's rig file (ROBOUSE_RIG) switches the same task onto live hardware."""
    cfg = dict(spec.get("real") or {})
    rig = config.env("ROBOUSE_RIG")
    if rig:
        import yaml

        entry = (yaml.safe_load(Path(rig).read_text()) or {}).get(cfg.get("robot", "metal"))
        if entry:
            cfg = {
                **{k: v for k, v in cfg.items() if k != "fixture"},
                **entry,
                "envelope": {**(cfg.get("envelope") or {}), **(entry.get("envelope") or {})},
                "contact": {**(cfg.get("contact") or {}), **(entry.get("contact") or {})},
            }
    return cfg


def real_mode(spec: dict) -> str | None:
    """ "real" (a live driver), "hil-mock" (the hardware-in-the-loop mock) or None (not a real-robot task)."""
    if spec.get("backend") != "real":
        return None
    return "hil-mock" if real_config(spec).get("driver", "mock") == "mock" else "real"


class RealArmBackend(EmbodiedBackend):
    name = "real"

    def __init__(self, spec: dict):
        super().__init__(spec)
        self.cfg = real_config(spec)
        if self.cfg.get("robot") == "ros":  # any ROS arm: the profile comes from the rig / task configuration
            from ...real.drivers.rosbridge import ros_profile

            self.prof = ros_profile(self.cfg)
        else:
            self.prof = get_profile(self.cfg.get("robot", "metal"))
        self.mock = self.cfg.get("driver", "mock") == "mock"
        self.mode = "hil-mock" if self.mock else "real"
        # live robots (and the real-time mock rosbridge server) are paced to the control rate; replay mocks run as fast as possible
        self.pace = bool(self.cfg.get("pace", not self.mock or self.cfg.get("robot") == "ros"))
        self.hz = float(self.cfg.get("control_hz", self.prof.control_hz))
        self.prof.control_hz = self.hz
        self.run_dir = None  # set by the episode server (session files next to the episode record)
        self.log = EventLog(None)
        self.guard = Guard(self.prof, self.cfg.get("envelope"), log=self._event, estop_check=estop_engaged)
        chain = self.guard.env.chain
        settings = ContactSettings(**{**self.prof.contact.__dict__, **(self.cfg.get("contact") or {})})
        self.watch = ContactWatch(self.prof, settings, tool=chain.tool if chain is not None else None)
        # parks: tracking lag, the stall check near the table, torque at the park factor (contact.ParkWatch); without a
        # calibrated lag limit, a coarse one just under the wind-up limit, and no park without the operator's yes
        self.park_watch = ParkWatch(
            self.prof,
            settings,
            tool=chain.tool if chain is not None else None,
            table_z=self.guard.e.table_z,
            windup=self.guard.e.windup_deg,
        )
        self.n_contacts = 0
        self.move_goal: np.ndarray | None = None  # the running skill's final joint target
        self._motion_event: dict | None = None  # what the safety layer did to the current request's motion
        self.stamp: float | None = None  # when the last joint reading was measured (driver's clock, time.monotonic)
        self.stamp_of: str | None = None  # which source that oldest value came from (the Piper's joint pairs)
        self.connected = False
        self.stop_kind: str | None = None
        self.estop_confirmed: dict | None = None
        self.secured: str | None = None  # how the arm was left: parked and released, or holding (and why)
        self.disconnected = False
        self.close_cause = "end of episode"
        self._park_delta: np.ndarray | None = None
        self._park_moved = False  # the last parking attempt moved the arm
        self._park_stop_kind = "park"  # what stopped it
        self.stop_after_end: dict | None = None  # a stop after a normal end (`_stop_after_end`: holds a run-many batch)
        self._hold_alarms: set[str] = set()  # warnings given while the arm holds after a stop (`_watch_hold`)
        self.ending = False  # the settle steps after the episode ended (`begin_settle`): any contact is a stop
        self._last_check = 0.0  # time.monotonic() of the last control step or idle check (`idle_check`)
        self.operator = Operator(
            self.cfg.get("operator", "script" if self.mock else "tty"),
            mock=self.mock,
            timeout_s=float(self.cfg.get("operator_timeout_s", 600)),
            log=self._event,
        )
        self.driver = make_driver(self.prof, self.cfg)
        self.cameras_list = list(self.driver.cameras)
        self.camera = "all" if self.cameras_list else ""
        self.halted: str | None = None
        self.target: np.ndarray | None = None
        self.q = np.asarray(self.prof.park or [0.0] * len(self.prof.joints), dtype=float)
        self.images: dict = {}
        self.extra: dict = {}
        self.lock = None
        self.attended = bool(self.cfg.get("attended", True))
        if not self.attended and not self.mock:
            if not config.flag("ROBOUSE_REAL_UNATTENDED"):
                raise SafetyStop("unattended live runs need ROBOUSE_REAL_UNATTENDED=1 in the operator's environment")
        self.goal = self.cfg.get("goal") or {}
        self.verdict_mode = self.cfg.get(
            "verdict", "pose" if (self.goal.get("tool") or self.goal.get("joints")) else "operator"
        )
        if self.verdict_mode == "vlm":  # checked before any motion: the verdict is asked only at the episode's end
            if not self.cfg.get("vlm_model"):
                raise ValueError(
                    "real.verdict: vlm needs real.vlm_model, a vision-language model at ROBOUSE_GRADER_URL (no default)"
                )
            from ...analysis.grader import endpoint

            endpoint()  # RuntimeError naming ROBOUSE_GRADER_URL / ROBOUSE_GRADER_API_KEY when either is missing
        # stages: ordered checkpoints on the measured state ({"tool": [x,y,z], "tol": m, "gripper": open|closed} or
        # {"joints": [...], "tol": deg}); they give the task-progress metric and the mock operator's verdict proxy
        self.stages = list(self.cfg.get("stages") or [])
        self.stage_i = 0
        self.last_judge = None
        self.n_steps = 0
        self.decl = self._declare()

    # ---- declaration ---------------------------------------------------------------------------------------------
    def _declare(self) -> Embodiment:
        p = self.prof
        groups = []
        for pre in p.arms:
            arm = [i for i in p.arm_idx if p.joints[i].startswith(pre)]
            groups.append(
                ActionGroup(
                    f"{pre}arm.joint_pos",
                    "joint_pos",
                    [p.joints[i] for i in arm],
                    [p.low[i] for i in arm],
                    [p.high[i] for i in arm],
                    units=p.units[arm[0]],
                    frame="joint",
                    doc="absolute joint targets; the safety guard limits speed and vetoes moves that leave the workspace",
                )
            )
            for g in p.gidx:
                if p.joints[g].startswith(pre):
                    groups.append(
                        ActionGroup(
                            f"{pre}gripper",
                            "gripper",
                            [p.joints[g]],
                            [p.low[g]],
                            [p.high[g]],
                            units=p.units[g],
                            doc=f"absolute; {p.gripper_closed:g} = closed, {p.gripper_open:g} = open",
                        )
                    )
        sensors = [
            Sensor("joint_pos", "proprio", ["joint_pos"], units=p.units[0], doc="measured joint positions by name"),
            Sensor("telemetry", "proprio", ["telemetry"], doc="max motor temperature and safety counters"),
            Sensor("operator", "events", ["operator_notes"], doc="remarks typed by the operator during the run"),
        ]
        if self.guard.env.chain is not None:
            sensors.insert(
                1,
                Sensor(
                    "tool",
                    "proprio",
                    ["tool_pos", "tool_axis"],
                    units="m",
                    frame="base",
                    doc="tool point (between the fingertips) by forward kinematics of the measured joints",
                ),
            )
        sensors += [
            Sensor(f"camera:{c}", "camera", mount="wrist" if "grip" in c or "wrist" in c else "world")
            for c in self.cameras_list
        ]
        grip_args = [SkillArg("value", "float", p.units[p.gidx[0]] if p.gidx else "")]
        if len(p.arms) > 1:
            grip_args.append(
                SkillArg("arm", "enum", default="both", choices=[a.rstrip("_") for a in p.arms] + ["both"])
            )
        skills = [
            Skill(
                "move_joints",
                [SkillArg(p.joints[i], "float", p.units[i]) for i in p.arm_idx]
                + [SkillArg("speed", "float", "fraction", 1.0, doc="fraction of the speed limit")],
                "interpolated move of the arm joints to the given pose (the whole path is checked first)",
                max_steps=400,
            ),
            Skill("grip", grip_args, "set the gripper(s) and wait until they stop moving", max_steps=60),
            Skill("home", [], "move to the reach-ready home pose", max_steps=400),
        ]
        if p.park:  # a rig without a rest pose has no park skill
            skills.append(Skill("park", [], "move to the rest pose", max_steps=400))
        if self.guard.env.chain is not None:
            skills.insert(
                1,
                Skill(
                    "move_to",
                    [
                        SkillArg("x", "float", "m"),
                        SkillArg("y", "float", "m"),
                        SkillArg("z", "float", "m"),
                        SkillArg("speed", "float", "fraction", 1.0),
                    ],
                    "move the tool point to (x, y, z) in the base frame (inverse kinematics; the path is checked first)",
                    max_steps=400,
                ),
            )
        return Embodiment(
            robot=p.display,
            family="arm",
            assets=[],
            sensors=sensors,
            action_groups=groups,
            skills=skills,
            budget=Budget(max_steps=int(self.spec.get("max_steps", 600)), control_dt=1.0 / self.hz),
            cameras=self.cameras_list,
        )

    def embodiment(self) -> dict:
        spec = super().embodiment()
        e = self.guard.e
        spec["kind"] = "arm"
        spec["mode"] = self.mode
        spec["safety"] = {
            "joint_limits": {"low": self.prof.low, "high": self.prof.high},
            "max_joint_speed": e.joint_speed_deg_s,
            "slow_joint_speed_deg_s": e.slow_joint_speed_deg_s,
            "slow_zone_m": e.slow_zone,
            "workspace": {"table_z": e.table_z, "x": [e.x_min, e.x_max], "y_abs_max": e.y_abs_max, "z_max": e.z_max}
            if self.guard.env.chain is not None
            else None,
            "windup_deg": e.windup_deg,
            "temp_stop_c": e.temp_stop_c,
            "watchdog": {
                "stall_s": self.guard.stall_s,
                "stop_s": self.guard.hard_s,
                "stalls_in_a_row": e.watchdog_max_stalls,
            },
            "idle_check_s": self.IDLE_CHECK_S,  # between requests: e-stop, fresh reading, temperature
            "telemetry_timeout_s": e.telemetry_timeout_s,
            "contact_watch": self._contact_spec(),
            "rest_pose": list(self.prof.park) or None,  # where the arm parks at a normal end (after a stop: on a yes)
            "estop": "latched file (robouse estop)",
            "physical_estop": "not asked (hardware-in-the-loop mock)"
            if self.mock and not self.cfg.get("estop_check")
            else "confirmed by the operator before the robot is armed",
            "attended": self.attended,
            "operator_channel": self.operator.channel,
        }
        return spec

    def _contact_spec(self) -> dict | None:
        s, chain = self.watch.s, self.guard.env.chain is not None
        if not self.watch.torque_watch and not chain:
            return None  # no torque thresholds and no kinematics: nothing to watch with
        return {
            "torque_change": dict(s.dtorque) or None,
            "stall_mm": s.stall_mm if chain else None,
            "watched_below_m": s.watch_height if chain else None,
            "elsewhere_factor": s.unwatched_factor,
            "contacts_allowed": s.max_contacts - 1,
        }

    def info_extra(self) -> dict:
        return {
            **super().info_extra(),
            "mode": self.mode,
            "robot_notes": self.prof.notes,
            "hardware": self.driver.description
            + ("" if self.prof.tested_on_hardware or self.mock else " (UNTESTED on hardware)"),
        }

    # ---- lifecycle -----------------------------------------------------------------------------------------------
    def _event(self, _kind: str, **kw) -> None:
        self.log.event(_kind, **kw)
        if _kind in ("estop", "stop", "contact"):  # safety events (vetoes are prevented motions, not events)
            detail = kw.get("reason") or kw.get("what") or kw.get("kind") or _kind
            self.event(f"{_kind}:{len(self.log.events)}", "safety", str(detail))

    def attach_run_dir(self, run_dir) -> None:
        """Called by the episode server: session log files go next to the episode record."""
        from pathlib import Path

        self.log = EventLog(Path(run_dir) / "session")

    def reset(self, seed: int) -> None:
        self.events, self._event_keys = [], set()
        try:
            if not self.mock:  # one controller per physical robot (the mock has no shared hardware)
                self.lock = HardwareLock(self.prof.name, str(self.cfg.get("port", "")))
            why = estop_engaged()
            if why:
                raise SafetyStop(f"e-stop is engaged ({why}); clear it with `robouse estop --clear` when safe", "estop")
            self._confirm_physical_estop()
            if self.attended:
                self.operator.gate(
                    "arm",
                    f"{self.prof.display} ({self.mode}): clear the workspace and keep a hand near the power "
                    "switch. Enable torque? The arm will hold its current pose.",
                )
            self.driver.connect()
            self.connected = True
            self._read()
            self.guard.check_telemetry(self.stamp, source=self.stamp_of)
            self.guard.check_thermal(self.extra)  # a motor still hot from the last run does not start
            self.guard.last_cmd = self.q.copy()
            self.target = self.q.copy()
            self.watch.reset()
            self._last_check = time.monotonic()
            self._event("connect", pose=self.q.round(2).tolist(), mode=self.mode, driver=self.driver.description)
            if self.attended:
                scene = self.cfg.get("scene_setup") or "Set up the scene as the task describes."
                self.operator.gate("reset", f"Scene for this episode: {scene} Ready to start?", tick=self._tick)
        except SafetyStop as e:
            self._halt(str(e), e.kind)

    def _confirm_physical_estop(self) -> None:
        """Before a live robot is armed, the operator confirms a physical e-stop (a power cut-off) is within reach:
        motors keep their last command when software stops talking to them (2026-09-05: with the controller gone, the
        Metal arm held its last pose against its elbow stop until the motor reached 76 C). No setting skips this for a
        live driver; on the mock it is asked only when the task sets `estop_check: true`."""
        if self.mock and not self.cfg.get("estop_check"):
            self._event("physical_estop", confirmed=None, note="not asked: hardware-in-the-loop mock")
            return
        ok, ans, note = self.operator.confirm(
            "physical_estop",
            f"Physical e-stop: is a power cut-off (e-stop button or switch) for the {self.prof.display} present and "
            "within your reach now? Software cannot stop motors that keep their last command when the link dies.",
        )
        self.estop_confirmed = {
            "answer": ans,
            "note": note,
            "channel": self.operator.channel,
            "at": time.strftime("%Y-%m-%d %H:%M:%S"),
        }
        if not ok:
            raise SafetyStop(f"no physical e-stop confirmed ({note or ans}); the robot was not armed", "physical_estop")

    def _halt(self, reason: str, kind: str = "safety_stop") -> None:
        """Latch the stop and hold the arm where it is (`_hold_here`): nothing moves on its own after a stop."""
        if self.halted:
            return
        self.halted, self.stop_kind = reason, kind
        self._event("stop", kind="safety_stop", cause=kind, reason=reason)
        self._hold_here(f"safety stop ({kind})")

    # ---- the arm after a stop and at the end: hold, park only at a normal end or when the operator approves ----------
    PARK_SETTLE_STEPS = 30  # control steps allowed for the arm to come to rest at the rest pose
    REFUSALS = ("operator_gate", "physical_estop", "operator_channel")  # stops the operator chose: nothing is asked

    def _hold_here(self, cause: str) -> None:
        """Hold where the arm is: the arm joints' command goes to a fresh measured pose within the per-step delta limit
        (`_retarget`; without a fresh reading the last command stays), the gripper keeps its command, and nothing else
        moves: no parking, no release. Before torque is on, nothing is commanded."""
        if self.connected:
            self._hold(cause, self._retarget(cause) or "after a stop the arm does not move on its own")

    def _leave_arm(self) -> None:
        """How the arm is left when the episode server closes:
        - after a normal end (`_end_of_episode`), the arm joints' command first goes to where they are. An arm whose
          gripper still holds something, or that gives no fresh reading, holds there, and that is recorded as a stop
          after the end (which holds a run-many batch). Otherwise a robot whose parks have a calibrated watch
          (contact.ParkWatch.calibrated: a lag limit measured on the robot, so far the Metal) parks slowly at its rest
          pose and is released there (`_end_park`); a park that is stopped holds where it stopped, is recorded as a stop
          after the end and is never retried on its own. Any other robot holds, and an attended operator is asked
          whether to park it, told that the park is not calibrated and to watch it (`_ask_to_park`);
        - after a safety stop, or a stop after the end, the arm holds, and an attended operator is asked to inspect it
          and may approve parking (`_ask_to_park`); the thermal guard and the telemetry checks run until they answer;
        - after a refused gate or an interruption, the arm holds and nothing is asked."""
        if self.close_cause == "interruption":
            if not self.halted:
                self._hold_here("interruption")
            return
        ended = not self.halted
        if ended:
            self._end_of_episode()
        stopped = (self.halted and self.stop_kind not in self.REFUSALS) or self.stop_after_end
        uncalibrated = ended and bool(self.prof.park) and not self.park_watch.calibrated  # parks on a yes only
        if (stopped or uncalibrated) and self.attended and self.connected:
            self._ask_to_park()

    def _end_of_episode(self) -> None:
        """A normal end: hold where the arm is, and check it can be left for the next trial; then park on its own only
        with a calibrated park watch (`_leave_arm`)."""
        cause = "end of episode"
        if (why := self._retarget(cause)) is not None:  # no fresh reading, or the hold could not be sent
            self._stop_after_end(self._link_kind(why), f"the arm could not be parked after the end: {why}")
        elif (why := self._grasp()) is not None:
            self._stop_after_end("object_in_gripper", f"the arm could not be parked after the end: {why}")
        elif not self.prof.park:
            why = "the robot profile has no rest pose"
        elif not self.park_watch.calibrated:
            why = "its parks have no calibrated watch, so it does not park on its own" + (
                "" if self.attended else " (no operator attends to approve a park)"
            )
        else:
            self._end_park(cause)
            return
        self._hold(cause, why)

    def _end_park(self, cause: str) -> None:
        """Park after a normal end (a calibrated park watch), and release torque at rest. A park stopped part-way holds
        where it stopped and is a stop after the end."""
        why = self._park(cause)
        if why is None:
            self._release(cause)
            return
        if self._park_moved:
            self._stop_after_end(self._park_stop_kind, f"parking after the end stopped: {why}")
            why = f"{why}; {self._retarget(cause) or 'it holds where it stopped'}"
        self._hold(cause, why)

    def _stop_after_end(self, kind: str, reason: str) -> None:
        """A stop after a normal end: logged, and the first one is recorded in the result (`session.stop_after_end`,
        rewritten and signed again when the server closes), which holds a run-many batch (batch.stop_of)."""
        self._event("stop", kind="safety_stop", cause=kind, reason=reason)
        if self.stop_after_end is None:
            self.stop_after_end = {"kind": kind, "reason": reason}

    @staticmethod
    def _link_kind(why: str) -> str:
        """The stop kind for a hold that `_retarget` could not make: a stale reading, or a send that failed."""
        return "telemetry_lost" if why.startswith("no fresh") else "hardware_fault"

    def _ask_to_park(self) -> None:
        """With an attended operator, after a stop, after a stop after the end, or at a normal end on a robot without a
        calibrated park watch: the arm holds while they inspect it; parking happens only on their yes, once (a park
        they approve that is stopped holds where it stopped). Without a calibrated park watch the question says the
        park is not calibrated and asks them to watch it. Until they answer, `_tick` keeps the checks running (after a
        stop: the thermal guard and the telemetry checks, `_watch_hold`)."""
        if self.halted or self.stop_after_end:
            stop = self.stop_after_end or {"kind": self.stop_kind, "reason": self.halted}
            when = "safety stop" if self.halted else "stop after the end of the episode"
            what = f"{self.prof.display}: {when} ({stop['kind']}: {stop['reason']})."
            cause = "after the stop"
        else:
            what, cause = f"{self.prof.display}: the episode has ended.", "end of episode"
        what += " The arm is holding at its present position with torque on."
        if blocker := self._park_blocker():
            ok, _, _ = self.operator.confirm(
                "hold_ack",
                f"{what} It cannot be parked: {blocker}. Inspect it, then answer yes; until you answer, the motor "
                "temperatures and the link are watched.",
                tick=self._tick,
            )
            self._hold(cause, f"{blocker} ({'acknowledged by the operator' if ok else 'not acknowledged'})")
            return
        latch = " Clear the e-stop latch first (`robouse estop --clear`)." if estop_engaged() else ""
        ok, ans, _ = self.operator.confirm(
            "park", f"{what} Inspect the robot and its workspace.{latch} {self._park_question()}", tick=self._tick
        )
        if not ok:
            self._hold(cause, "the operator did not approve parking" if ans == "no" else "no answer: it was not parked")
            return
        cause = "parking approved by the operator"
        why = self._park_blocker()  # anything that changed while the question was open
        if why is None:
            why = self._park(cause)
            if why is None:
                self._release(cause)
                return
            if self._park_moved:
                reason = f"approved parking stopped: {why}"
                if self.halted or self.stop_after_end:
                    self._event("stop", kind="safety_stop", cause=self._park_stop_kind, reason=reason)
                else:  # after a normal end, this is the stop after the end
                    self._stop_after_end(self._park_stop_kind, reason)
                why = f"{why}; {self._retarget(cause) or 'it holds where it stopped'}"
        self._hold(cause, why)

    def _park_question(self) -> str:
        """The park question: how the park is watched, and how the arm is left at the rest pose."""
        rest = (
            "is released"
            if getattr(self.driver, "releases_torque", True)
            else "holds there (the driver cannot release torque)"
        )
        w = self.park_watch
        if w.calibrated:
            return (
                "Park the arm at its rest pose now? yes: it moves slowly there, stopping if anything holds it back, and "
                f"{rest}; no: it keeps holding."
            )
        unit = self.prof.units[self.prof.arm_idx[0]]
        checks = [
            f"a joint lagging its command by more than {w.lag_limit:g} {unit} (just under its "
            f"{self.guard.e.windup_deg:g} {unit} wind-up limit)"
        ]
        if w.tool is not None:
            checks.append("near the table, the fingertips stopping short of a descent")
        if w.watch.torque_watch:
            checks.append(f"joint torque changing by more than {w.s.park_torque_factor:g}x the contact thresholds")
        one = len(checks) == 1
        return (
            "This robot's park is not calibrated. "
            + ("Only a coarse check watches it: " if one else "Only coarse checks watch it: ")
            + "; ".join(checks)
            + f". It can push against something for a while before {'that stops' if one else 'they stop'} it. Park the "
            "arm at its rest pose now? If yes, watch it all the way, with a hand on the physical e-stop. yes: it moves "
            f"slowly there and {rest}; no: it keeps holding."
        )

    def _park_blocker(self) -> str | None:
        """Why the arm cannot be parked now (the e-stop latch aside, which the operator may clear), or None."""
        if not self.prof.park:
            return "the robot profile has no rest pose"
        if (why := self._grasp()) is not None:
            return why
        if self._present() is None:
            return "no fresh reading of the present position (the link may be down)"
        return None

    def _release(self, cause: str) -> None:
        """At rest at the rest pose: release torque (a driver that cannot, the ROS bridge, keeps holding there)."""
        if not getattr(self.driver, "releases_torque", True):
            self.secured = "parked at the rest pose, holding there (the driver cannot release torque)"
            self._event("parked", cause=cause, pose=self.q.round(2).tolist(), torque="on: the driver cannot release it")
            self.operator.notify(
                f"{self.prof.display} ({cause}): parked at its rest pose and holding there with torque on (the driver "
                "cannot release torque)."
            )
            return
        self._event("torque_released", cause=cause, pose=self.q.round(2).tolist())
        self.secured = "parked at the rest pose, torque released"
        self.connected, self.disconnected = False, True
        try:
            self.driver.disconnect(release_torque=True)
        except Exception as e:  # noqa: BLE001 - the motors then keep holding the rest pose
            self._event("release_failed", error=f"{type(e).__name__}: {e}")
        self.operator.notify(f"{self.prof.display} ({cause}): parked at its rest pose and released (torque off).")

    def _tick(self) -> None:
        """While an operator question is open: the checks between requests (`idle_check`), at most once a second."""
        self.idle_check()

    def _watch_hold(self) -> None:
        """The arm holds after a stop, until the operator has answered: read it, and run the telemetry check and the
        thermal guard. They warn loudly (an operator notice and the session log) but change nothing: the arm keeps
        holding, as only the operator may move it now."""
        try:
            self._read()
        except Exception as e:  # noqa: BLE001 - a dead link: the motors keep their last command
            self._hold_alarm("link", f"its link does not answer ({type(e).__name__}: {e})")
            return
        lists = {k: v for k, v in self.extra.items() if isinstance(v, list)}
        self.log.step(label="hold", pos=self.q.round(3).tolist(), **lists)
        limit = self.guard.e.telemetry_timeout_s
        if self.stamp is not None and (age := time.monotonic() - self.stamp) > limit:
            of = f" from {self.stamp_of}" if self.stamp_of else ""
            self._hold_alarm("link", f"no fresh joint reading{of} for {age:.1f} s (limit {limit:g} s)")
        else:
            self._hold_alarms.discard("link")
        temps = [float(x) for k in ("temp_mos_c", "temp_rotor_c", "temp_c") for x in (self.extra.get(k) or [])]
        stop_c = self.guard.e.temp_stop_c
        if temps and max(temps) > stop_c:
            self._hold_alarm("temperature", f"a motor is at {max(temps):.0f} C, above the {stop_c:.0f} C stop limit")
        elif temps and max(temps) < stop_c - 2:
            self._hold_alarms.discard("temperature")

    def _hold_alarm(self, alarm: str, what: str) -> None:
        if alarm in self._hold_alarms:  # once until the condition clears
            return
        self._hold_alarms.add(alarm)
        self._event("hold_alarm", alarm=alarm, what=what)
        self.operator.notify(
            f"WARNING: {self.prof.display} is holding after a stop and {what}. It keeps holding with torque on; the "
            "physical e-stop cuts the power."
        )

    def _retarget(self, cause: str) -> str | None:
        """Bring the arm joints' command to where a fresh reading puts them, through the per-step delta limit (the slow
        limit in the slow zone), one paced control step at a time, with a fresh reading at every step; the gripper keeps
        its command. A stale reading is never sent: without a fresh one the command stays where it is. None once the
        command is there, else why it stays."""
        g, arm = self.guard, self.prof.arm_idx
        if g.last_cmd is None:
            return "the arm was never commanded"
        cmd = np.asarray(g.last_cmd, dtype=float).copy()
        stale = "no fresh reading of the present position (the link may be down), so the last command is held"
        present = self._present()
        if present is None:
            self._event("retarget", cause=cause, pose=cmd.round(2).tolist(), steps=0, done=False, reason=stale)
            return stale
        goal = cmd.copy()
        goal[arm] = present[arm]
        g.last_pos = present
        n = 0
        while True:
            t0 = time.perf_counter()
            step = g.max_step_slow if g.env.in_slow_zone(cmd) else g.max_step
            cmd = np.clip(goal, cmd - step, cmd + step)
            if not self._command(cmd):
                return "the hold command could not be sent (the link may be down)"
            n += 1
            self.log.step(label="retarget", cmd=cmd.round(3).tolist(), pos=self.q.round(3).tolist())
            if np.allclose(cmd, goal, atol=1e-9):
                break
            if self.pace:
                time.sleep(max(0.0, 1.0 / self.hz - (time.perf_counter() - t0)))
            if self._present() is None:  # the link went silent part-way: the command stays where it got to
                self._event("retarget", cause=cause, pose=cmd.round(2).tolist(), steps=n, done=False, reason=stale)
                return stale
        self._event("retarget", cause=cause, pose=cmd.round(2).tolist(), steps=n)
        return None

    GRASP_LEAD = 1.0  # gripper units: jaws this far open of their command are held open by something

    def _grasp(self) -> str | None:
        """Why the arm must not be parked and released: a gripper whose jaws stay open of their command (an object in
        the grasp, which parking would carry and releasing torque would drop). None when every gripper is free."""
        p, cmd = self.prof, self.guard.last_cmd
        if cmd is None:
            return None
        closes = 1.0 if p.gripper_closed <= p.gripper_open else -1.0  # the direction that closes the jaws
        held = [i for i in p.gidx if closes * (self.q[i] - cmd[i]) > self.GRASP_LEAD]
        if not held:
            return None
        jaws = ", ".join(f"{p.joints[i]} at {self.q[i]:.1f}, commanded {cmd[i]:.1f}" for i in held)
        return (
            f"the gripper holds something ({jaws}): it keeps its grip; take the object out before the arm is released"
        )

    def _present(self) -> np.ndarray | None:
        """The measured joint positions from a fresh reading, or None when there is none (dead or silent link)."""
        try:
            r = self.driver.read()
        except Exception:  # noqa: BLE001 - a dead link: the caller holds the last command
            return None
        if r.stamp is not None and time.monotonic() - r.stamp > self.guard.e.telemetry_timeout_s:
            return None
        self.q, self.extra, self.stamp, self.stamp_of = r.q, r.extra, r.stamp, r.stamp_of
        if r.images:
            self.images = r.images
        return np.asarray(r.q, dtype=float).copy()

    def _command(self, pose: np.ndarray) -> bool:
        """Send a hold command (bypassing the guard's limits: the caller keeps it within the per-step delta limit)."""
        try:
            self.driver.send(pose)
        except Exception as e:  # noqa: BLE001 - a dead link cannot take the hold either
            self._event("hold_failed", error=f"{type(e).__name__}: {e}")
            return False
        self.guard.last_cmd = pose.copy()
        self.target = pose.copy()
        return True

    def _hold(self, cause: str, why: str) -> None:
        """The arm keeps the command it has (where it is after a retarget, else the last command), sent once more
        unchanged; on a dead link that fails too, and is logged."""
        pose = self.guard.last_cmd
        if pose is not None:
            self._command(np.asarray(pose, dtype=float))
        self.secured = f"holding at the present position: {why}"
        self._event("holding", cause=cause, pose=None if pose is None else np.round(pose, 2).tolist(), reason=why)
        self.operator.notify(
            f"{self.prof.display} ({cause}): holding at its present position with torque on ({why}). The physical "
            "e-stop cuts the power."
        )

    def _park(self, cause: str) -> str | None:
        """Move slowly (the slow joint speed, at most the park speed the lag watch was calibrated at) to the rest pose
        with every check on and the park watch (`_park_step`); None once the arm is at rest there, else why not
        (`_park_moved`: it had started moving; `_park_stop_kind`: what stopped it). Called at a normal end with a
        calibrated park watch, or once per operator approval: never retried."""
        p = self.prof
        self._park_moved, self._park_stop_kind = False, "park"
        if not p.park:
            return "the robot profile has no rest pose"
        if latch := estop_engaged():
            return f"the e-stop is latched ({latch}); clear it with `robouse estop --clear` to park"
        goal = self._full([p.park[i] for i in p.arm_idx])  # the gripper keeps its command
        try:
            goal = self.guard.validate_target(goal)
            self.guard.env.check_path(self.guard.last_cmd, goal)
        except Veto as v:
            return f"the way to the rest pose leaves the workspace ({v})"
        # at most the park speed the lag watch was calibrated at (the slow joint speed, if that is slower)
        s, step = self.park_watch.s, self.guard.max_step_slow.copy()
        if s.park_speed_deg_s > 0:
            step[p.arm_idx] = np.minimum(step[p.arm_idx], s.park_speed_deg_s / self.hz)
        speed = round(float(np.max(step[p.arm_idx])) * self.hz, 2)
        lag = round(self.park_watch.lag_limit, 3) or None
        self._event(
            "park",
            cause=cause,
            goal=goal.round(2).tolist(),
            speed_deg_s=speed,
            lag_limit_deg=lag,
            calibrated=self.park_watch.calibrated,
        )
        self._park_moved = True
        start = np.asarray(self.guard.last_cmd, dtype=float).copy()
        n = int(max(1, np.ceil(np.max(np.abs(goal - start) / step))))
        self.park_watch.reset()
        self._park_delta = None
        try:
            for k in range(1, n + 1):
                self._park_step(start + (goal - start) * k / n)
            for _ in range(self.PARK_SETTLE_STEPS):
                if self._at_rest(goal):
                    return None
                self._park_step(goal)
            return None if self._at_rest(goal) else "the arm did not reach the rest pose"
        except KeyboardInterrupt:
            self._park_stop_kind = "interruption"
            return "parking was interrupted"
        except SafetyStop as e:
            self._park_stop_kind = e.kind
            return f"parking stopped: {e}"
        except Veto as e:
            self._park_stop_kind = "veto"
            return f"parking stopped: {e}"
        except Exception as e:  # noqa: BLE001 - a driver fault while parking
            self._park_stop_kind = "hardware_fault"
            return f"parking stopped by a hardware fault: {type(e).__name__}: {e}"

    def _park_step(self, target: np.ndarray) -> None:
        """One parking control step through the guard (e-stop latch, limits, speed, wind-up, workspace), then the
        stale-telemetry check, the loop watchdog and the park watch (tracking lag, the stall check near the table, torque
        at the park factor: contact.ParkWatch). Temperature is not checked: resting with torque off is how a hot motor
        cools."""
        t0 = time.perf_counter()
        cmd = self.guard.step_command(target)
        self.driver.send(cmd)
        self.guard.last_cmd = cmd
        if self.pace:
            time.sleep(max(0.0, 1.0 / self.hz - (time.perf_counter() - t0)))
        before = self.q.copy()
        self._read()
        dt = time.perf_counter() - t0
        arm = self.prof.arm_idx
        self._park_delta = np.abs(self.q[arm] - before[arm])
        self.log.step(
            label="park",
            target=np.asarray(target).round(3).tolist(),
            cmd=cmd.round(3).tolist(),
            pos=self.q.round(3).tolist(),
            dt=round(dt, 4),
            **{k: v for k, v in self.extra.items() if isinstance(v, list)},
        )
        self.guard.check_telemetry(self.stamp, source=self.stamp_of)
        self.guard.check_loop(dt)
        self.guard.last_pos = np.asarray(self.q, dtype=float)
        c = self.park_watch.update(cmd, self.q, self.extra)
        if c is not None:
            what = c.describe()
            self._event("contact", **c.record(), what=what, pose=self.q.round(2).tolist(), action="parking stopped")
            raise SafetyStop(f"contact while parking: {what}", "contact")

    def _at_rest(self, goal: np.ndarray) -> bool:
        """Every arm joint within 2 deg (or 0.035 rad) of the rest pose, and still."""
        arm = self.prof.arm_idx
        tol = np.asarray([0.035 if self.prof.units[i] == "rad" else 2.0 for i in arm])
        still = self._park_delta is not None and bool(np.all(self._park_delta <= tol / 20))
        return still and bool(np.all(np.abs(self.q[arm] - goal[arm]) <= tol))

    def on_interrupt(self) -> None:
        """The episode server's hook when the run is interrupted (Ctrl-C or SIGTERM): close() then holds the arm."""
        self.close_cause = "interruption"

    def _read(self) -> None:
        r = self.driver.read()
        self.q, self.extra, self.stamp, self.stamp_of = r.q, r.extra, r.stamp, r.stamp_of
        if r.images:
            self.images = r.images

    # ---- stepping --------------------------------------------------------------------------------------------------
    def step(self, action):
        if self.halted:
            raise SafetyStop(self.halted, self.stop_kind or "safety_stop")
        a = np.asarray(action, dtype=float)
        if not np.all(np.isfinite(a)):
            raise ValueError("action values must be finite numbers")
        self.control_step(a)
        from ..base import StepInfo

        return StepInfo(success=False)

    def control_step(self, a: np.ndarray) -> None:
        target = np.asarray(a, dtype=float)
        try:
            if self.target is None or np.max(np.abs(target - self.target)) > 1e-6:
                target = self.guard.check_move(target)  # whole-move check; raises Veto without moving
                self.target = target
            t0 = time.perf_counter()
            cmd = self.guard.step_command(self.target)
            self.driver.send(cmd)
            self.guard.last_cmd = cmd
            if self.pace:
                time.sleep(max(0.0, 1.0 / self.hz - (time.perf_counter() - t0)))
            self._read()
            dt = time.perf_counter() - t0
            self._last_check = time.monotonic()
            self.n_steps += 1
            self.log.step(
                i=self.n_steps,
                target=self.target.round(3).tolist(),
                cmd=cmd.round(3).tolist(),
                pos=self.q.round(3).tolist(),
                tool=self._tool().round(4).tolist() if self.guard.env.chain is not None else None,
                dt=round(dt, 4),
                **{k: v for k, v in self.extra.items() if isinstance(v, list)},
            )
            self.guard.check_telemetry(self.stamp, source=self.stamp_of)  # before anything uses this reading
            self.guard.after_step(self.q, self.extra, dt)  # thermal guard and loop watchdog
            self._advance_stages()
            self._watch_contact(cmd)
        except SafetyStop as e:
            self._halt(str(e), e.kind)
            raise
        except Veto:
            raise
        except Exception as e:  # driver fault: halt for a human
            self._halt(f"hardware fault: {type(e).__name__}: {e}", "hardware_fault")
            raise SafetyStop(str(self.halted), "hardware_fault") from e

    def _watched(self, cmd) -> bool:
        """A watched step: the command, the target or the skill's goal puts the tool point within the contact watch's
        height of the table (robots with a kinematic chain; others have no table to watch)."""
        ch = self.guard.env.chain
        if ch is None:
            return False
        poses = [cmd] + [p for p in (self.target, self.move_goal) if p is not None]
        low = min(float(ch.tool(p)[2]) for p in poses)
        return low - self.guard.e.table_z < self.watch.s.watch_height

    def _watch_contact(self, cmd) -> None:
        """A contact in a watched move stops the move and holds; the agent is told in the step result. A contact
        anywhere else, or one contact too many, stops the episode."""
        c = self.watch.update(cmd, self.q, self.extra, self._watched(cmd))
        if c is None:
            return
        s = self.watch.s
        rec = {**c.record(), "what": c.describe(), "pose": self.q.round(2).tolist()}
        if self.guard.env.chain is not None:
            rec["tool_pos"] = self._tool().round(4).tolist()
        # where the arm is held: the arm joints' command goes to where they are, through the per-step delta limit, and
        # the gripper keeps its command, so a grasp holds
        hold = np.asarray(self.guard.last_cmd, dtype=float).copy()
        hold[self.prof.arm_idx] = self.q[self.prof.arm_idx]
        if self.ending:  # the settle steps after the episode ended: there is no move to stop and no agent to tell
            act = "safety stop: a contact during the settle steps after the end"
            self._event("contact", **rec, action=act, hold=hold.round(2).tolist())
            raise SafetyStop(f"contact during the settle steps after the episode ended: {c.describe()}", "contact")
        if not c.watched:
            self._event("contact", **rec, action="safety stop: a contact outside a watched move")
            raise SafetyStop(f"contact outside a watched move: {c.describe()}", "contact")
        self.n_contacts += 1
        if self.n_contacts >= s.max_contacts:
            self._event("contact", **rec, count=self.n_contacts, action=f"safety stop: contact {self.n_contacts}")
            raise SafetyStop(
                f"repeated contacts ({self.n_contacts} in this episode, limit {s.max_contacts}): {c.describe()}",
                "contact",
            )
        self._event(
            "contact",
            **rec,
            count=self.n_contacts,
            action="move stopped; the arm holds at the measured pose",
            hold=hold.round(2).tolist(),
        )
        if why := self._retarget("contact"):  # a stale or a dead link
            raise SafetyStop(f"the arm could not be held after a contact: {why}", self._link_kind(why))
        self._motion_event = {
            "contact": {
                "what": c.describe(),
                "kind": c.kind,
                "change": c.change,
                "count": self.n_contacts,
                "limit": s.max_contacts,
                "done": "the move was stopped and the arm holds where it is; move away or send a new target "
                + f"(contact {s.max_contacts} in this episode, or a contact away from the table, ends it as safety_stop)",
            }
        }

    def pop_motion_event(self) -> dict | None:
        """The episode server's hook: what the safety layer did to the last step's motion (a contact), once."""
        ev, self._motion_event = self._motion_event, None
        return ev

    def hold_action(self) -> list[float]:
        """What "hold still" is now: the last command (after a contact, the hold), read again for every settle step."""
        return [float(x) for x in (self.guard.last_cmd if self.guard.last_cmd is not None else self.q)]

    def begin_settle(self) -> None:
        """The episode server's hook before the settle steps that follow the end of an episode: the agent can no longer
        react, so any contact from now on is a safety stop (and holds the batch) instead of stopping a move."""
        self.ending = True

    IDLE_CHECK_S = 1.0  # between requests, the arm is checked at least this often

    def idle_check(self) -> None:
        """The episode server's hook between requests (the agent thinking, the episode lingering) and while an operator
        question is open (`_tick`): at most once a second, the checks a control step makes that need no motion, namely
        the e-stop latch, a fresh joint reading and the thermal guard, while the motors keep the last command (sent
        again, unchanged). A failed check latches a stop and holds the arm, as during a control step. After a stop the
        arm is only watched (`_watch_hold`) until the server closes."""
        if not self.connected or self.guard.last_cmd is None:
            return
        if time.monotonic() - self._last_check < self.IDLE_CHECK_S:
            return
        self._last_check = time.monotonic()
        if self.halted or self.stop_after_end:
            self._watch_hold()
            return
        try:
            self.guard.check_estop()
            self.driver.send(np.asarray(self.guard.last_cmd, dtype=float))
            self._read()
            self.log.step(
                label="idle",
                cmd=np.round(self.guard.last_cmd, 3).tolist(),
                pos=self.q.round(3).tolist(),
                **{k: v for k, v in self.extra.items() if isinstance(v, list)},
            )
            self.guard.check_telemetry(self.stamp, source=self.stamp_of)
            self.guard.check_thermal(self.extra)
            self.guard.last_pos = np.asarray(self.q, dtype=float)
        except SafetyStop as e:
            self._halt(str(e), e.kind)
        except Exception as e:  # noqa: BLE001 - a driver fault while idle
            self._halt(f"hardware fault while idle: {type(e).__name__}: {e}", "hardware_fault")

    # ---- observation ------------------------------------------------------------------------------------------------
    def metrics_point(self):
        return self._tool() if self.guard.env.chain is not None else None

    def _tool(self, q=None) -> np.ndarray:
        return self.guard.env.chain.tool(self.q if q is None else q)

    def observe(self) -> dict:
        p = self.prof
        out = {
            "joint_pos": {n: round(float(v), 2) for n, v in zip(p.joints, self.q, strict=False)},
            "target": [round(float(v), 2) for v in (self.target if self.target is not None else self.q)],
            "telemetry": {
                "max_temp_c": round(
                    max(
                        [float(x) for k in ("temp_mos_c", "temp_rotor_c", "temp_c") for x in (self.extra.get(k) or [])]
                        or [0.0]
                    ),
                    1,
                ),
                "vetoes": sum(1 for e in self.log.events if e["event"] == "veto"),
                "estop": bool(estop_engaged()),
                "halted": self.halted,
            },
            "mode": self.mode,
        }
        if len(p.gidx) == 1:
            out["gripper"] = round(float(self.q[p.gidx[0]]), 2)
        elif p.gidx:
            out["gripper"] = {p.joints[g]: round(float(self.q[g]), 2) for g in p.gidx}
        if self.guard.env.chain is not None:
            out["tool_pos"] = [round(float(v), 4) for v in self._tool()]
            out["tool_axis"] = [round(float(v), 3) for v in self.guard.env.chain.tool_axis(self.q)]
        notes = self.operator.new_feedback()
        if notes:
            out["operator_notes"] = notes
        return out

    def render(self, width: int = 480, height: int = 480) -> np.ndarray:
        from PIL import Image

        cams = self.cameras_list if self.camera in ("all", "") else [self.camera]
        imgs = [self.images.get(c) for c in cams]
        imgs = [np.asarray(i, dtype=np.uint8) for i in imgs if i is not None]
        if not imgs:
            return np.full((176, 320, 3), 40, dtype=np.uint8)
        h = min(i.shape[0] for i in imgs)
        imgs = [np.asarray(Image.fromarray(i).resize((int(i.shape[1] * h / i.shape[0]), h))) for i in imgs]
        img = np.concatenate(imgs, axis=1)
        H, W = (img.shape[0] + 15) // 16 * 16, (img.shape[1] + 15) // 16 * 16  # video codecs want multiples of 16
        pad = np.zeros((H, W, 3), dtype=np.uint8)
        pad[: img.shape[0], : img.shape[1]] = img
        return pad

    def mj_model_data(self):
        return None

    # ---- skills --------------------------------------------------------------------------------------------------------
    def _move(self, q_target, speed: float = 1.0):
        """Generator: interpolate from the last command to q_target at `speed` x the speed limit, then wait to arrive."""
        q_target = self.guard.check_move(np.asarray(q_target, dtype=float))
        start = np.asarray(self.guard.last_cmd, dtype=float)
        per_step = self.guard.max_step * max(0.05, min(1.0, float(speed)))
        n = int(max(1, np.ceil(np.max(np.abs(q_target - start) / per_step))))
        self.move_goal = q_target  # the contact watch watches a move by where it ends
        try:
            for k in range(1, n + 1):
                yield list(start + (q_target - start) * k / n)
            for _ in range(30):  # settle: keep commanding the target until the measured pose arrives
                err = np.abs(self.q - q_target)
                err[self.prof.gidx] = 0.0
                if float(err.max()) < 1.0:
                    break
                yield list(q_target)
        finally:
            self.move_goal = None
        err = self.q - q_target
        return {
            "arrived": bool(np.max(np.abs(err[self.prof.arm_idx])) < 2.0),
            "error": [round(float(x), 2) for x in err],
        }

    def _full(self, arm_values) -> np.ndarray:
        q = np.asarray(self.guard.last_cmd if self.guard.last_cmd is not None else self.q, dtype=float).copy()
        idx = self.prof.arm_idx
        q[idx] = np.asarray(arm_values, dtype=float)[: len(idx)]
        return q

    def skill_move_joints(self, speed: float = 1.0, **joints):
        arm = [joints[self.prof.joints[i]] for i in self.prof.arm_idx]
        res = yield from self._move(self._full(arm), speed)
        return res

    def skill_move_to(self, x: float, y: float, z: float, speed: float = 1.0):
        """IK to the point, move, then up to two corrections that feed the measured error forward (gravity sag: 2 mm near
        the base, 12-18 mm at full reach on the Metal arm)."""
        ch = self.guard.env.chain
        goal = np.array([x, y, z], dtype=float)
        aim = goal.copy()
        n = ch.n_arm
        res: dict = {}
        for attempt in range(3):
            q0 = np.asarray(self.guard.last_cmd, dtype=float)
            q, err = ch.solve(aim, q0, self.prof.low, self.prof.high, seeds=self.prof.ik_seeds)
            if err > 0.005:
                return {
                    "reached": False,
                    "error": f"no joint solution reaches ({x}, {y}, {z}) (closest {err * 1000:.0f} mm)",
                }
            res = yield from self._move(np.concatenate([q[:n], q0[n:]]), speed if attempt == 0 else min(speed, 0.5))
            miss = goal - self._tool()
            if np.linalg.norm(miss) < 0.004 or np.linalg.norm(miss) > 0.05:
                break
            aim = aim + miss
        tool = self._tool()
        return {
            **res,
            "reached": bool(np.linalg.norm(tool - goal) < 0.01),
            "tool_pos": [round(float(v), 4) for v in tool],
            "distance_m": round(float(np.linalg.norm(tool - goal)), 4),
        }

    def skill_grip(self, value: float, arm: str = "both"):
        q = np.asarray(self.guard.last_cmd, dtype=float).copy()
        gs = [g for g in self.prof.gidx if arm == "both" or self.prof.joints[g].startswith(arm + "_")]
        q[gs] = float(value)
        res = yield from self._move(q, 1.0)
        now = {self.prof.joints[g]: round(float(self.q[g]), 2) for g in gs}
        short = any(abs(self.q[g] - value) > 3 for g in gs)
        return {
            "gripper": now if len(now) > 1 else next(iter(now.values())),
            **({"note": "stopped short (holding an object?)"} if short else {}),
            "arrived": res.get("arrived"),
        }

    def skill_home(self):
        res = yield from self._move(self._full([self.prof.home[i] for i in self.prof.arm_idx]), 1.0)
        return res

    def skill_park(self):
        res = yield from self._move(self._full([self.prof.park[i] for i in self.prof.arm_idx]), 1.0)
        return res

    def start_skill(self, name: str, raw_args: list[str]):
        if self.halted:
            raise ValueError(f"robot stopped: {self.halted}")
        sk = self.decl.skill(name)
        if sk is None:
            raise ValueError(f"unknown skill {name!r}; skills: {[s.name for s in self.decl.skills]}")
        kwargs = sk.parse(raw_args)
        try:
            if name == "move_joints":
                gen = self.skill_move_joints(**kwargs)
            else:
                gen = getattr(self, f"skill_{name}")(**kwargs)
            first = next(gen)
        except StopIteration as e:  # finished without moving (e.g. no IK solution)

            def done(v=e.value):
                return v
                yield  # unreachable: makes this function a generator

            return done()
        except Veto as v:
            raise ValueError(f"vetoed by the safety envelope: {v}") from None

        def chain():
            yield first
            return (yield from gen)

        from ..embodied import _capped

        return _capped(chain(), sk.max_steps)

    # ---- scoring ----------------------------------------------------------------------------------------------------------
    def _meets(self, g: dict) -> bool:
        if g.get("tool") is not None:
            ok = bool(np.linalg.norm(self._tool() - np.asarray(g["tool"], dtype=float)) <= float(g.get("tol", 0.02)))
        else:
            n = self.prof.arm_idx
            ok = bool(
                np.max(np.abs(self.q[n] - np.asarray(g["joints"], dtype=float)[: len(n)])) <= float(g.get("tol", 3.0))
            )
        if ok and g.get("gripper") in ("open", "closed") and self.prof.gidx:
            gv = float(self.q[self.prof.gidx[0]])
            mid = (self.prof.gripper_open + self.prof.gripper_closed) / 2
            ok = gv >= mid if g["gripper"] == "open" else gv < mid
        return ok

    def _advance_stages(self) -> None:
        while self.stage_i < len(self.stages) and self._meets(self.stages[self.stage_i]):
            self._event(
                "stage", index=self.stage_i, name=self.stages[self.stage_i].get("name", f"stage {self.stage_i + 1}")
            )
            self.stage_i += 1

    def progress(self) -> float | None:
        """Fraction of the task's stages reached, in order, or the operator's rubric stage over the top stage."""
        if self.cfg.get("rubric") and getattr(self, "rubric_stage", None) is not None:
            return round(self.rubric_stage / (len(self.cfg["rubric"]) - 1), 3)
        return round(self.stage_i / len(self.stages), 3) if self.stages else None

    def success(self) -> bool:
        if self.halted or self.verdict_mode != "pose":
            return False
        return self._meets(self.goal)

    def judge(self, outcome: str, text: str = "") -> bool:
        if self.halted:
            self.last_judge = {"verdict": "safety_stop", "reason": self.halted}
            return False
        if self.verdict_mode == "pose":
            ok = outcome == "done" and self.success()
            self.last_judge = {"verdict": "pose", "goal": self.goal, "joint_pos": [round(float(v), 2) for v in self.q]}
            if self.goal.get("tool") is not None:
                self.last_judge.update(
                    tool_pos=[round(float(v), 4) for v in self._tool()],
                    distance_m=round(float(np.linalg.norm(self._tool() - np.asarray(self.goal["tool"]))), 4),
                )
            return ok
        if outcome != "done":
            self.last_judge = {"verdict": "not_claimed", "outcome": outcome}
            return False
        if (
            self.verdict_mode == "vlm"
        ):  # a vision-language model judges the final camera frames (rigs without an operator)
            from ...analysis.grader import vlm_verdict

            frames = [self.images[c] for c in self.cameras_list if c in self.images]
            if not frames:
                self.last_judge = {"verdict": "vlm", "error": "no camera frames"}
                return False
            v = vlm_verdict(
                frames,
                self.cfg.get("goal_text") or str(self.spec.get("id")),
                self.cfg.get("rubric"),
                self.cfg["vlm_model"],
            )
            self.last_judge = {"verdict": "vlm", **v}
            if self.cfg.get("rubric") and isinstance(v.get("stage"), int):
                self.rubric_stage = int(v["stage"])
            return v["verdict"] == "yes"
        rubric = self.cfg.get("rubric") or []
        if rubric:  # staged rubric (StationeryBench style): the operator reports the highest stage reached
            top = len(rubric) - 1
            if self.mock and self.operator.channel == "script" and "verdict" not in self.operator._script:
                ans, note = (
                    "0",
                    "mock operator: the hardware-in-the-loop mock models no object contact, so no stage is credited",
                )
                self._event("operator_verdict", verdict=ans, note=note, channel="script")
            else:
                prompt = "Highest stage reached: " + "; ".join(f"{i} = {d}" for i, d in enumerate(rubric))
                stages = tuple(str(i) for i in range(top + 1)) + ("skip",)
                ans, note = self.operator.verdict(prompt, choices=stages, tick=self._tick)  # the arm watched meanwhile
            self.rubric_stage = int(ans) if ans.isdigit() else None
            self.last_judge = {
                "verdict": "operator_rubric",
                "stage": self.rubric_stage,
                "of": top,
                "note": note,
                "channel": self.operator.channel,
            }
            return self.rubric_stage == top
        if self.mock and self.operator.channel == "script" and "verdict" not in self.operator._script:
            # the scripted stand-in operator of the hardware-in-the-loop mock: yes iff every stage was reached in order
            ans = "yes" if self.stages and self.stage_i == len(self.stages) else "no"
            note = f"mock operator: {self.stage_i}/{len(self.stages)} stages reached"
            self._event("operator_verdict", verdict=ans, note=note, channel="script")
        else:
            ans, note = self.operator.verdict(
                f"Task: {self.cfg.get('goal_text') or self.spec.get('id')}. Did the robot succeed?", tick=self._tick
            )
        self.last_judge = {"verdict": "operator", "answer": ans, "note": note, "channel": self.operator.channel}
        return ans == "yes"

    def close(self) -> None:
        """End of the episode, or the server interrupted: leave the arm (`_leave_arm`), then let the driver go. Torque is
        released only at the rest pose; a holding arm keeps its present position after this process ends."""
        try:
            if self.driver is not None:
                if self.connected:
                    self._leave_arm()
                self._event("close", pose=self.q.round(2).tolist(), release_torque=self.disconnected)
                # holding, or never connected (the driver still cleans up, e.g. a mock server)
                if not self.disconnected:
                    self.connected, self.disconnected = False, True
                    self.driver.disconnect(release_torque=False)
        finally:
            if self.lock:
                self.lock.release()
            self.log.close()

    def session_summary(self) -> dict:
        ev = self.log.events
        return {
            "mode": self.mode,
            "robot": self.prof.name,
            "driver": self.driver.description,
            "control_steps": self.n_steps,
            "progress": self.progress(),
            "vetoes": sum(1 for e in ev if e["event"] == "veto"),
            "clamps": sum(1 for e in ev if e["event"] == "clamp"),
            "warnings": sum(1 for e in ev if e["event"] == "warn"),
            "contacts": sum(1 for e in ev if e["event"] == "contact"),
            "safety_stop": self.halted,
            "stop_kind": self.stop_kind,
            "secured": self.secured,
            "stop_after_end": self.stop_after_end,
            "attended": self.attended,
            "operator_channel": self.operator.channel,
            "physical_estop": self.estop_confirmed,
            "operator": [e for e in ev if e["event"].startswith("operator_")],
        }


def oracle_main(env: str) -> None:
    """Reference solutions for real tasks: move the tool point to the goal with the move_to skill (pose tasks), or run the
    task's scripted skill sequence (`real.oracle` in task.md: a list of [skill, args...])."""
    import json

    from ..embodied import Oracle

    o = Oracle()
    spec_oracle = json.loads(__import__("os").environ.get("ROBOUSE_REAL_ORACLE", "null") or "null")

    def solve(o: Oracle) -> None:
        for s in spec_oracle or []:
            o.skill(s[0], *s[1:])

    o.run(solve)
