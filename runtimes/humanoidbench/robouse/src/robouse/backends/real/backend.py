"""Real robot arms behind the same episode server and `robo` interface as the simulators.

The task's `real:` block picks the robot profile (metal, so101, piper), the driver (`lerobot` / `piper` for live
hardware, `mock` for the hardware-in-the-loop mock that replays a recorded session) and the safety and operator
settings. Everything the agent sends goes through the safety guard (real/safety.py) inside this trusted process:
e-stop latch, joint-limit clamp, workspace envelope by forward kinematics (vetoed before anything moves), per-step
velocity limit (slower near the table), wind-up guard, thermal stop and loop watchdog. The operator channel
(real/operator.py) gates arming and scene reset and, for tasks without a measurable goal, gives the verdict.

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
from ...real.drivers import make_driver
from ...real.operator import Operator
from ...real.robots import profile as get_profile
from ...real.safety import EventLog, Guard, HardwareLock, SafetyStop, Veto, estop_engaged
from ..embodied import ActionGroup, Budget, EmbodiedBackend, Embodiment, Sensor, Skill, SkillArg


class RealArmBackend(EmbodiedBackend):
    name = "real"

    def __init__(self, spec: dict):
        super().__init__(spec)
        self.cfg = dict(spec.get("real") or {})
        rig = config.env("ROBOUSE_RIG")  # the operator's rig file switches the same task onto live hardware
        if rig:
            import yaml

            entry = (yaml.safe_load(Path(rig).read_text()) or {}).get(self.cfg.get("robot", "metal"))
            if entry:
                self.cfg = {
                    **{k: v for k, v in self.cfg.items() if k != "fixture"},
                    **entry,
                    "envelope": {**(self.cfg.get("envelope") or {}), **(entry.get("envelope") or {})},
                }
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
        self.target = None
        self.q = np.asarray(self.prof.park, dtype=float)
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
            Skill("park", [], "move to the rest pose", max_steps=400),
        ]
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
            "estop": "latched file (robouse estop)",
            "attended": self.attended,
            "operator_channel": self.operator.channel,
        }
        return spec

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
        if _kind in ("estop", "stop"):  # safety events (vetoes are prevented motions, not events)
            self.event(f"{_kind}:{len(self.log.events)}", "safety", str(kw.get("reason") or kw.get("kind") or _kind))

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
                raise SafetyStop(f"e-stop is engaged ({why}); clear it with `robouse estop --clear` when safe")
            if self.attended:
                self.operator.gate(
                    "arm",
                    f"{self.prof.display} ({self.mode}): clear the workspace and keep a hand near the power "
                    "switch. Enable torque? The arm will hold its current pose.",
                )
            self.driver.connect()
            self._read()
            self.guard.last_cmd = self.q.copy()
            self.target = self.q.copy()
            self._event("connect", pose=self.q.round(2).tolist(), mode=self.mode, driver=self.driver.description)
            if self.attended:
                scene = self.cfg.get("scene_setup") or "Set up the scene as the task describes."
                self.operator.gate("reset", f"Scene for this episode: {scene} Ready to start?")
        except SafetyStop as e:
            self._halt(str(e))

    def _halt(self, reason: str) -> None:
        if self.halted:
            return
        self.halted = reason
        self._event("stop", kind="safety_stop", reason=reason)
        try:  # hold where it is
            if (
                self.driver
                and getattr(self.driver, "robot", None) is not None
                or getattr(self.driver, "sdk", None) is not None
            ):
                self.driver.send(self.q)
        except Exception:  # noqa: BLE001
            pass

    def _read(self) -> None:
        r = self.driver.read()
        self.q, self.extra = r.q, r.extra
        if r.images:
            self.images = r.images

    # ---- stepping --------------------------------------------------------------------------------------------------
    def step(self, action):
        if self.halted:
            raise SafetyStop(self.halted)
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
            self._advance_stages()
            self.guard.after_step(self.q, self.extra, dt if self.pace else 0.0)
        except SafetyStop as e:
            self._halt(str(e))
            raise
        except Veto:
            raise
        except Exception as e:  # driver fault: halt for a human
            self._halt(f"hardware fault: {type(e).__name__}: {e}")
            raise SafetyStop(self.halted) from e

    def hold_action(self) -> list[float]:
        return [float(x) for x in (self.guard.last_cmd if self.guard.last_cmd is not None else self.q)]

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
        for k in range(1, n + 1):
            yield list(start + (q_target - start) * k / n)
        for _ in range(30):  # settle: keep commanding the target until the measured pose arrives
            err = np.abs(self.q - q_target)
            err[self.prof.gidx] = 0.0
            if float(err.max()) < 1.0:
                break
            yield list(q_target)
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
                self.cfg.get("vlm_model", "moonshotai/Kimi-K3"),
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
                ans, note = self.operator.verdict(prompt, choices=tuple(str(i) for i in range(top + 1)) + ("skip",))
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
                f"Task: {self.cfg.get('goal_text') or self.spec.get('id')}. Did the robot succeed?"
            )
        self.last_judge = {"verdict": "operator", "answer": ans, "note": note, "channel": self.operator.channel}
        return ans == "yes"

    def close(self) -> None:
        try:
            if self.driver is not None:
                release = bool(self.cfg.get("release_on_close", False))
                if release and self.attended and not self.halted:
                    try:
                        self.operator.gate("release", "Support the arm: release torque? (the arm goes limp)")
                    except SafetyStop:
                        release = False
                self._event("close", pose=self.q.round(2).tolist(), release_torque=release)
                self.driver.disconnect(release_torque=release)
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
            "safety_stop": self.halted,
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
