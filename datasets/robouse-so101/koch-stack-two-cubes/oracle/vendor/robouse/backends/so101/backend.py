"""so101 backend: the SO-101 follower arm (one, or two as StationeryBench's rig) and the Koch v1.1 low-cost arm in
MuJoCo 3.3.0, on a small hobby desk scaled to them (sim/scene.py). The interface mirrors the real backend's `so101`
and `so101-bimanual` profiles (backends/real/backend.py, real/robots.py), so prompts and reference solutions transfer
to the real arm:

- skills: `move_to X Y Z [SPEED]` (the tool point to a position in the base frame, metres; inverse kinematics),
  `grip VALUE` (0 = closed, 100 = fully open; the real reference solutions open with 60 and close with 2), `home`,
  `park`, `move_joints J1..J5 [SPEED]` (degrees). On the two-arm rig, `move_to X Y Z ARM [SPEED]` and
  `grip VALUE [ARM]` take an arm (left or right; grip also both), and `move_joints` takes both arms' ten joints, as
  the real so101-bimanual profile does.
- joints in degrees with the zero of LeRobot's calibration (the middle of each joint's range; so101_new_calib, which
  the Menagerie SO-101 model is derived from); limits, home and rest poses and the 30 deg/s speed limit of the real
  profile; 10 Hz control.
- low level (`robo act`): joint-position deltas, five arm joints (deg) plus the gripper (0-100 units) per arm; zero
  holds.

Inverse kinematics (ik.py): damped least squares on the MJCF for the tool point's position, with the gripper's
approach axis held straight down (a fixed top-down pitch). The arms have five joints, so the wrist roll is not solved:
the gripper's yaw follows the base angle and the roll (`jaw_axis` in `robo observe` says where the jaws close).

Success predicates (tasks.py) are judged from the simulated state after `robo done` and 10 still steps. Reference
solutions (oracles.py) drive the arm only through `robo` (the episode socket), reading `robo observe`.
"""

from __future__ import annotations

import math

import numpy as np

# MuJoCo is imported inside the methods that need it: the reference solution (oracles.py, through oracle_main) imports
# this package in the agent container, which has numpy but no MuJoCo
from ..embodied import ActionGroup, Budget, EmbodiedBackend, Embodiment, Sensor, Skill, SkillArg, r3
from . import scene as S
from .ik import ArmIK
from .tasks import TASKS, Task

CONTROL_DT = 0.1  # 10 Hz, the real so101 profile's control rate
N_SUB = round(CONTROL_DT / S.PHYS_DT)
JOINT_SPEED = 30.0  # deg/s (the real so101 profile's joint speed limit)
GRIP_SPEED = 60.0  # gripper units/s (the real profile's)
MAX_DQ = JOINT_SPEED * CONTROL_DT  # deg per step
MAX_DG = GRIP_SPEED * CONTROL_DT  # gripper units per step
GRIP_LOW, GRIP_HIGH = 2.0, 98.0  # the real profile's gripper limits
TOOL_CLEARANCE = 0.004  # move_to refuses targets closer to the desk than this (the real envelope's tool clearance)
IK_TOL = 0.005  # m: a move_to whose best joint solution misses by more is refused, as on the real arm
SETTLE_DEG = 1.0
BUTTON_ON_DEPTH, BUTTON_OFF_DEPTH = 0.0045, 0.0015


class So101Backend(EmbodiedBackend):
    name = "so101"

    def __init__(self, spec: dict):
        super().__init__(spec)
        env = spec.get("env") or spec.get("id")
        if env not in TASKS:
            raise KeyError(f"unknown so101 task {env!r}; tasks: {', '.join(sorted(TASKS))}")
        self.task_id = env
        self.task: Task = TASKS[env]
        self.robot = S.ROBOTS[self.task.robot]
        self.rig = self.task.rig
        self.arms = list(S.arm_bases(self.rig))  # ["arm"] or ["left", "right"]
        self.bimanual = self.rig == "bimanual"
        self.camera = spec.get("camera", "front")
        self.decl = self._declare()

    # ---- declaration -------------------------------------------------------------------------------------------
    def _jname(self, pre: str, j: str) -> str:
        return f"{pre}_{j}" if self.bimanual else j

    def _cameras(self) -> list[str]:
        cams = ["front", "top"]
        if self.robot["wrist_cam"]:
            cams += [f"{pre}_wrist" for pre in self.arms] if self.bimanual else ["wrist"]
        return cams

    def _declare(self) -> Embodiment:
        R = self.robot
        groups = []
        for pre in self.arms:
            p = f"{pre}_" if self.bimanual else ""
            groups.append(
                ActionGroup(
                    f"{p}arm.joint_delta",
                    "joint_delta",
                    [self._jname(pre, j) for j in R["joints"]],
                    [-MAX_DQ] * 5,
                    [MAX_DQ] * 5,
                    "deg per step",
                    f"change of each joint's target this step (at most {MAX_DQ:g} deg, the {JOINT_SPEED:g} deg/s speed "
                    "limit at 10 Hz); zero holds; targets stay inside the joint limits",
                    frame="joint",
                )
            )
            groups.append(
                ActionGroup(
                    f"{p}gripper",
                    "gripper",
                    [self._jname(pre, "gripper")],
                    [-MAX_DG],
                    [MAX_DG],
                    "0-100 units per step",
                    f"change of the gripper target (0 = closed, 100 = fully open; kept within {GRIP_LOW:g}-{GRIP_HIGH:g}); "
                    "zero holds",
                )
            )
        sensors = [
            Sensor(
                "joint_pos",
                "proprio",
                ["joint_pos", "target"],
                "measured joints by name and the current targets",
                units="deg",
            ),
            Sensor(
                "gripper",
                "proprio",
                ["gripper", "gripper_opening_m", "holding"],
                "measured gripper (0-100), the distance between the jaw tips, the object both jaws touch",
            ),
            Sensor(
                "tool",
                "proprio",
                ["tool_pos", "tool_axis", "jaw_axis"],
                "tool point between the fingertips, approach direction and jaw closing direction",
                units="m",
                frame="world",
            ),
            Sensor(
                "objects",
                "world",
                ["objects"],
                "free objects: position, yaw, tilt, size",
                units="m, deg",
                frame="world",
            ),
            Sensor(
                "fixtures",
                "world",
                ["fixtures", "desk"],
                "drawer opening, button state, zones, trays' places",
                units="m",
                frame="world",
            ),
            *[Sensor(f"camera:{c}", "camera", [], mount="wrist" if "wrist" in c else "world") for c in self._cameras()],
        ]
        arm_arg = [SkillArg("arm", "enum", choices=["left", "right"], doc="which arm")] if self.bimanual else []
        grip_args = [SkillArg("value", "float", "0-100", doc="0 = closed, 100 = fully open")]
        if self.bimanual:
            grip_args.append(SkillArg("arm", "enum", default="both", choices=["left", "right", "both"]))
        joint_args = [SkillArg(self._jname(pre, j), "float", "deg") for pre in self.arms for j in R["joints"]]
        speed = SkillArg("speed", "float", "fraction", 1.0, doc="fraction of the speed limit")
        side = [SkillArg("arm", "enum", default="both", choices=["left", "right", "both"])] if self.bimanual else []
        skills = [
            Skill(
                "move_joints",
                joint_args + [speed],
                "interpolated move of the arm joints to the given pose",
                max_steps=400,
            ),
            Skill(
                "move_to",
                [SkillArg("x", "float", "m"), SkillArg("y", "float", "m"), SkillArg("z", "float", "m")]
                + arm_arg
                + [speed],
                "move the tool point to (x, y, z) in the base frame (inverse kinematics for position, gripper pointing "
                "straight down where the arm can; joint-space path), then correct the measured error up to twice",
                max_steps=400,
            ),
            Skill("grip", grip_args, "set the gripper(s) and wait until they stop moving", max_steps=60),
            Skill("home", list(side), "move to the reach-ready home pose", max_steps=400),
            Skill("park", list(side), "move to the rest pose", max_steps=400),
        ]
        return Embodiment(
            robot=("two " + R["name"] + "s" if self.bimanual else R["name"]) + " (simulated)",
            family="bimanual_arm" if self.bimanual else "arm",
            assets=[R["folder"]],
            sensors=sensors,
            action_groups=groups,
            skills=skills,
            budget=Budget(self.task.max_steps, CONTROL_DT),
            cameras=self._cameras(),
        )

    def info_extra(self) -> dict:
        R = self.robot
        return {
            **super().info_extra(),
            "robot_notes": (
                f"{R['name']} (MuJoCo Menagerie `{R['folder']}`, {R['licence']}), simulated. Joints in degrees "
                "(0 at the middle of each joint's range for the SO-101, LeRobot's calibration), gripper 0 = closed, "
                "100 = fully open. Base frame: +x forward, +y left, +z up, metres; the desk top is z = 0."
                + (
                    f" Two arms: bases at y = +{S.BIMANUAL_Y:g} (left) and y = -{S.BIMANUAL_Y:g} (right); positions "
                    "are in the rig frame, midway between the bases."
                    if self.bimanual
                    else ""
                )
            ),
            "joint_limits": {
                "low": R["low"] + [GRIP_LOW],
                "high": R["high"] + [GRIP_HIGH],
                "max_joint_speed_deg_s": JOINT_SPEED,
            },
            "home": R["home"],
            "rest_pose": R["park"],
        }

    # ---- episode -----------------------------------------------------------------------------------------------
    def reset(self, seed: int) -> None:
        import mujoco

        self._reset_renderers()
        R = self.robot
        self.model = m = S.build(self.task.robot, self.rig, self.task.props)
        self.data = d = mujoco.MjData(m)
        self.base = {pre: np.array(b, float) for pre, b in S.arm_bases(self.rig).items()}
        self.jq, self.jd, self.act = {}, {}, {}
        for pre in self.arms:
            names = [f"{pre}/{j}" for j in R["joints"]] + [f"{pre}/{R['gripper']}"]
            self.jq[pre] = np.array([m.jnt_qposadr[m.joint(n).id] for n in names])
            self.jd[pre] = np.array([m.jnt_dofadr[m.joint(n).id] for n in names])
            self.act[pre] = np.array([m.actuator(n).id for n in names])
        self.ik = {pre: ArmIK(m, pre, R) for pre in self.arms}
        start = self.task.start or R["home"]
        self.cmd = {}
        for pre in self.arms:
            c = np.array(list(start) + [float(self.task.start_grip)], float)
            self.cmd[pre] = c
            d.qpos[self.jq[pre]] = self._to_raw(c)
            d.ctrl[self.act[pre]] = self._to_raw(c)
        mujoco.mj_forward(m, d)
        self._tables()
        self.events, self._event_keys = [], set()
        self.button_on, self.button_pressed = False, False
        self.since: dict[str, float | None] = {}
        self.stage = 0
        self.task_state: dict = {}
        for _ in range(5):  # let the props settle on the desk (the arm holds its start pose)
            self.control_step(np.zeros(self.action_spec.dim))
        self.since, self.stage, self.task_state = {}, 0, {}

    def _tables(self) -> None:
        m = self.model
        R = self.robot
        self.arm_geoms, self.fixed_geoms, self.moving_geoms = {}, {}, {}
        body_pre = [m.body(b).name.split("/", 1)[0] if "/" in m.body(b).name else "" for b in range(m.nbody)]
        for pre in self.arms:
            self.arm_geoms[pre] = {g for g in range(m.ngeom) if body_pre[m.geom_bodyid[g]] == pre}
            fb = m.body(f"{pre}/{R['tool_body']}").id
            mb = m.body(f"{pre}/{R['moving_body']}").id
            self.fixed_geoms[pre] = {g for g in range(m.ngeom) if m.geom_bodyid[g] == fb}
            self.moving_geoms[pre] = {g for g in range(m.ngeom) if m.geom_bodyid[g] == mb}
        self.robot_geom = set().union(*self.arm_geoms.values())
        self.obj = {
            p["name"]: p for p in self.task.props if p["kind"] in ("cube", "cup", "tray", "ruler", "puck", "box")
        }
        self.obj_body = {n: m.body(n).id for n in self.obj}
        self.obj_geoms = {
            n: {g for g in range(m.ngeom) if m.geom_bodyid[g] == b and m.geom_contype[g]}
            for n, b in self.obj_body.items()
        }
        self.label = {}
        for n, gs in self.obj_geoms.items():
            for g in gs:
                self.label[g] = n
        self.desk = m.geom("desk").id
        self.label[self.desk] = "desk"
        self.drawer = next((p["name"] for p in self.task.props if p["kind"] == "drawer"), None)
        self.button = next((p["name"] for p in self.task.props if p["kind"] == "button"), None)
        if self.button:
            self.button_q = m.jnt_qposadr[m.joint(self.button).id]
            self.button_led = m.geom(f"{self.button}_led").id
        if self.drawer:
            self.drawer_q = m.jnt_qposadr[m.joint(self.drawer).id]

    # ---- units ---------------------------------------------------------------------------------------------------
    def _to_raw(self, c: np.ndarray) -> np.ndarray:
        """Agent units (5 joints in deg, gripper 0-100) -> joint angles (rad)."""
        lo, hi = self.robot["grip_q"]
        return np.concatenate([np.radians(c[:5]), [lo + (hi - lo) * c[5] / 100.0]])

    def _from_raw(self, q: np.ndarray) -> np.ndarray:
        lo, hi = self.robot["grip_q"]
        return np.concatenate([np.degrees(q[:5]), [(q[5] - lo) / (hi - lo) * 100.0]])

    def measured(self, pre: str) -> np.ndarray:
        return self._from_raw(self.data.qpos[self.jq[pre]])

    # ---- control -------------------------------------------------------------------------------------------------
    def control_step(self, a: np.ndarray) -> None:
        import mujoco

        a = np.asarray(a, float)
        R = self.robot
        lo = np.array(R["low"] + [GRIP_LOW])
        hi = np.array(R["high"] + [GRIP_HIGH])
        old, new = {}, {}
        for i, pre in enumerate(self.arms):
            delta = np.clip(a[6 * i : 6 * i + 6], [-MAX_DQ] * 5 + [-MAX_DG], [MAX_DQ] * 5 + [MAX_DG])
            old[pre] = self._to_raw(self.cmd[pre])
            self.cmd[pre] = np.clip(self.cmd[pre] + delta, lo, hi)
            new[pre] = self._to_raw(self.cmd[pre])
        m, d = self.model, self.data
        for k in range(N_SUB):
            f = (k + 1) / N_SUB  # the target ramps over the step, as a servo's interpolating controller does
            for pre in self.arms:
                d.ctrl[self.act[pre]] = old[pre] + f * (new[pre] - old[pre])
            mujoco.mj_step(m, d)
            if self.button and k % 5 == 4:
                self._button()
        self._monitor()

    def _button(self) -> None:
        depth = -float(self.data.qpos[self.button_q])
        if not self.button_pressed and depth > BUTTON_ON_DEPTH:
            self.button_pressed = True
            self.button_on = not self.button_on  # push on, push off
            self.model.geom_rgba[self.button_led] = [0.2, 0.95, 0.3, 1] if self.button_on else [0.25, 0.08, 0.08, 1]
        elif self.button_pressed and depth < BUTTON_OFF_DEPTH:
            self.button_pressed = False

    def _monitor(self) -> None:
        t = float(self.data.time)
        for key, ok in self.task.holds(self).items():  # conditions that must hold for a while (lifted, ...)
            if ok:
                if self.since.get(key) is None:
                    self.since[key] = t
            else:
                self.since[key] = None
        self.task.track(self)

    def held_for(self, key: str) -> float:
        s = self.since.get(key)
        return 0.0 if s is None else float(self.data.time) - s

    # ---- state helpers ---------------------------------------------------------------------------------------------
    def contacts(self):
        d = self.data
        for c in d.contact[: d.ncon]:
            yield int(c.geom1), int(c.geom2)

    def touching(self, a: set[int], b: set[int]) -> bool:
        return any((g1 in a and g2 in b) or (g2 in a and g1 in b) for g1, g2 in self.contacts())

    def touched_by(self, geoms: set[int]) -> set[str]:
        out = set()
        for g1, g2 in self.contacts():
            for g, o in ((g1, g2), (g2, g1)):
                if g in geoms and o in self.label:
                    out.add(self.label[o])
        return out

    def holding(self, pre: str) -> str | None:
        """The object both jaws of an arm touch."""
        both = self.touched_by(self.fixed_geoms[pre]) & self.touched_by(self.moving_geoms[pre])
        both.discard("desk")
        return sorted(both)[0] if both else None

    def robot_touches(self, name: str, pre: str | None = None) -> bool:
        geoms = self.arm_geoms[pre] if pre else self.robot_geom
        return self.touching(geoms, self.obj_geoms[name])

    def on_desk(self, name: str) -> bool:
        return self.touching({self.desk}, self.obj_geoms[name])

    def obj_pos(self, name: str) -> np.ndarray:
        return self.data.xpos[self.obj_body[name]].copy()

    def obj_xmat(self, name: str) -> np.ndarray:
        return self.data.xmat[self.obj_body[name]].reshape(3, 3).copy()

    def obj_yaw(self, name: str) -> float:
        R = self.obj_xmat(name)
        return math.degrees(math.atan2(R[1, 0], R[0, 0]))

    def obj_tilt(self, name: str) -> float:
        return S.tilt_deg(self.data.xmat[self.obj_body[name]])

    def obj_bottom(self, name: str) -> float:
        """Lowest point of a box-shaped object (cube, ruler, box) above the desk."""
        p = self.obj[name]
        half = {
            "cube": [S.CUBE / 2] * 3,
            "ruler": [S.RULER_L / 2, S.RULER_W / 2, S.RULER_T / 2],
            "box": [S.BOX_W / 2, S.BOX_L / 2, S.BOX_H / 2],
        }[p["kind"]]
        R = self.obj_xmat(name)
        return float(self.obj_pos(name)[2] - np.abs(R[2, :]) @ np.asarray(half))

    def in_tray(self, name: str, tray: str) -> bool:
        """The object's centre over the tray's floor (inside the walls) and below the top of the walls."""
        tp, R = self.obj_pos(tray), self.obj_xmat(tray)
        loc = R.T @ (self.obj_pos(name) - tp)
        return (
            abs(loc[0]) < S.TRAY_L / 2 - S.TRAY_WALL
            and abs(loc[1]) < S.TRAY_W / 2 - S.TRAY_WALL
            and loc[2] < S.TRAY_H + S.CUBE / 2
        )

    def in_cup(self, name: str, cup: str) -> bool:
        c, o = self.obj_pos(cup), self.obj_pos(name)
        r_in = S.CUP_R - S.CUP_WALL
        return float(np.linalg.norm((o - c)[:2])) < r_in - 0.004 and o[2] - c[2] < S.CUP_H and self.obj_tilt(cup) < 20

    def tool(self, pre: str) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Tool point, approach axis, jaw axis (world)."""
        ik = self.ik[pre]
        R = self.data.xmat[ik.body].reshape(3, 3)
        return self.data.site_xpos[ik.site].copy(), R @ ik.approach_local, R @ ik.jaw_local

    def tips(self, pre: str) -> tuple[np.ndarray, np.ndarray]:
        """Inner faces of the fixed and the moving jaw at their tips (world)."""
        m, d = self.model, self.data
        a = d.geom_xpos[m.geom(f"{pre}/{self.robot['fixed_tip']}").id]
        b = d.geom_xpos[m.geom(f"{pre}/{self.robot['moving_tip']}").id]
        jaw = self.tool(pre)[2]
        r = self.robot["tip_r"]
        return a + r * jaw, b - r * jaw

    def tip_gap(self, pre: str) -> float:
        f, mv = self.tips(pre)
        return float(np.linalg.norm(mv - f))

    def drawer_opening(self) -> float:
        return float(self.data.qpos[self.drawer_q]) if self.drawer else 0.0

    # ---- observation ---------------------------------------------------------------------------------------------
    def observe(self) -> dict:
        R = self.robot
        st: dict = {"robot": self.decl.robot}
        jp, grip, tool, axis, jaw, tgt, opening, hold, tips = {}, {}, {}, {}, {}, {}, {}, {}, {}
        for pre in self.arms:
            mv = self.measured(pre)
            for j, v in zip(R["joints"], mv[:5], strict=True):
                jp[self._jname(pre, j)] = round(float(v), 2)
            grip[self._jname(pre, "gripper")] = round(float(mv[5]), 2)
            p, a, jw = self.tool(pre)
            tool[pre], axis[pre], jaw[pre] = r3(p, 4), r3(a), r3(jw)
            tgt[pre] = [round(float(v), 2) for v in self.cmd[pre]]
            ft, mt = self.tips(pre)
            opening[pre] = round(float(np.linalg.norm(mt - ft)), 4)
            tips[pre] = {"fixed": r3(ft, 4), "moving": r3(mt, 4)}
            hold[pre] = self.holding(pre)
        one = (lambda dct: dct["arm"]) if not self.bimanual else (lambda dct: dct)
        st["joint_pos"] = jp
        st["gripper"] = grip["gripper"] if not self.bimanual else grip
        st["gripper_opening_m"] = one(opening)
        st["tool_pos"], st["tool_axis"], st["jaw_axis"] = one(tool), one(axis), one(jaw)
        st["jaw_tips"] = one(tips)
        st["target"] = one(tgt)
        st["holding"] = one(hold)
        if self.bimanual:
            st["arm_bases"] = {pre: r3(b) for pre, b in self.base.items()}
        objs = {}
        for n, p in self.obj.items():
            k = p["kind"]
            o: dict = {"kind": k, "pos": r3(self.obj_pos(n), 4)}
            if k in ("cube", "ruler", "box", "tray"):
                o["yaw_deg"] = round(self.obj_yaw(n), 1)
            if k in ("cube", "ruler", "box", "cup"):
                o["tilt_deg"] = round(self.obj_tilt(n), 1)
            if "color" in p:
                o["color"] = p["color"]
            o.update(
                {
                    "cube": {"size_m": S.CUBE},
                    "cup": {"diameter_m": 2 * S.CUP_R, "height_m": S.CUP_H, "pos_is": "centre of its base"},
                    "tray": {"size_m": [S.TRAY_L, S.TRAY_W, S.TRAY_H], "pos_is": "centre of its floor's underside"},
                    "ruler": {"size_m": [S.RULER_L, S.RULER_W, S.RULER_T]},
                    "puck": {"diameter_m": 2 * S.PUCK_R, "height_m": S.PUCK_H},
                    "box": {"size_m": [S.BOX_W, S.BOX_L, S.BOX_H]},
                }[k]
            )
            objs[n] = o
        st["objects"] = objs
        fx: dict = {}
        for p in self.task.props:
            k, n = p["kind"], p["name"]
            if k == "drawer":
                kz = S.DRAWER_H / 2 + 0.002
                q = self.drawer_opening()
                fx[n] = {
                    "opening_m": round(q, 4),
                    "travel_m": S.DRAWER_TRAVEL,
                    "front_x": round(p["pos"][0] - q, 4),
                    "knob_pos": r3([p["pos"][0] - q - S.KNOB_L * 0.6, p["pos"][1], kz], 4),
                    "knob": f"a {2 * S.KNOB_R * 1000:.0f} mm stem {S.KNOB_L * 100:.1f} cm long with a "
                    f"{2 * S.KNOB_HEAD_R * 1000:.0f} mm head; knob_pos is a point on the stem",
                    "opens_toward": [-1.0, 0.0, 0.0],
                    "width_m": S.DRAWER_W,
                }
            elif k == "button":
                depth = max(0.0, -float(self.data.qpos[self.button_q]))
                fx[n] = {
                    "pos": r3([p["pos"][0], p["pos"][1], S.BUTTON_BASE[2] + S.BUTTON_CAP_H - depth], 4),
                    "on": bool(self.button_on),
                    "pressed_in_m": round(depth, 4),
                    "cap_diameter_m": 2 * S.BUTTON_R,
                    "travel_m": S.BUTTON_TRAVEL,
                    "how": "push on, push off: each full press toggles it",
                }
            elif k == "zone":
                fx[n] = {"pos": r3([p["pos"][0], p["pos"][1], 0.0]), "diameter_m": 2 * S.ZONE_R}
            elif k == "marker":
                fx[n] = {"pos": r3(p["pos"], 4)}
        if fx:
            st["fixtures"] = fx
        x0, x1, y0, y1 = S.desk_extent(self.rig)
        st["desk"] = {"top_z": 0.0, "x": [round(x0, 3), round(x1, 3)], "y": [round(y0, 3), round(y1, 3)]}
        pr = self.task.progress(self)
        if pr is not None:
            st["progress"] = pr
        st["time_s"] = round(float(self.data.time), 2)
        return st

    # ---- rendering ---------------------------------------------------------------------------------------------
    def _cam_name(self, cam: str | None) -> str:
        cam = cam or "front"
        if cam == "wrist":
            return "arm/wrist_cam"
        if cam.endswith("_wrist"):
            return f"{cam[: -len('_wrist')]}/wrist_cam"
        return cam

    def render(self, width: int = 480, height: int = 480) -> np.ndarray:
        import mujoco

        key = (width, height)
        r = self._renderers.get(key)
        if r is None:
            r = self._renderers[key] = mujoco.Renderer(self.model, height=height, width=width)
        r.update_scene(self.data, camera=self._cam_name(self.camera))
        return r.render().copy()

    def render_video(self) -> np.ndarray:
        old = self.camera
        try:
            self.camera = "side"
            return self.render(480, 480)
        finally:
            self.camera = old

    def record_camera(self):
        return "side"

    def camera_info(self, cameras: list[str]) -> list[dict]:
        from ...core.cameras import mujoco_camera

        out = []
        for c in cameras:
            info = mujoco_camera(self.model, self.data, self._cam_name(c), 480, 480, calibrated="wrist" not in c)
            if info is not None:
                out.append({**info, "name": c})
        return out

    # ---- scoring -----------------------------------------------------------------------------------------------
    def success(self) -> bool:
        ok, _why = self.task.check(self)
        return bool(ok)

    def judge(self, outcome: str, text: str = "") -> bool:
        ok, why = self.task.check(self)
        self.last_judge = {"success": bool(ok), "why": why}
        return outcome == "done" and bool(ok)

    # ---- skills ------------------------------------------------------------------------------------------------
    def _arms_of(self, arm: str | None) -> list[str]:
        if not self.bimanual:
            return ["arm"]
        return ["left", "right"] if arm in (None, "both") else [arm]

    def _move(self, goals: dict[str, np.ndarray], speed: float = 1.0, grip_only: bool = False):
        """Generator: move each arm's six targets (deg, gripper units) linearly to `goals`, all arriving together at
        `speed` x the speed limit, then hold until the measured joints arrive (at most 30 steps)."""
        speed = max(0.05, min(1.0, float(speed)))
        R = self.robot
        lo = np.array(R["low"] + [GRIP_LOW])
        hi = np.array(R["high"] + [GRIP_HIGH])
        goals = {pre: np.clip(np.asarray(g, float), lo, hi) for pre, g in goals.items()}
        start = {pre: self.cmd[pre].copy() for pre in goals}
        lim = np.array([MAX_DQ] * 5 + [MAX_DG]) * speed
        n = max(1, int(math.ceil(max(float(np.max(np.abs(goals[p] - start[p]) / lim)) for p in goals))))
        for k in range(1, n + 1):
            a = np.zeros(self.action_spec.dim)
            for i, pre in enumerate(self.arms):
                if pre in goals:
                    a[6 * i : 6 * i + 6] = start[pre] + (goals[pre] - start[pre]) * k / n - self.cmd[pre]
            yield list(a)
        if grip_only:
            return {}
        for _ in range(30):  # settle: hold until the measured joints are there
            err = max(float(np.max(np.abs(self.measured(p)[:5] - goals[p][:5]))) for p in goals)
            if err < SETTLE_DEG:
                break
            yield [0.0] * self.action_spec.dim
        err = {p: self.measured(p)[:5] - goals[p][:5] for p in goals}
        return {"arrived": all(float(np.max(np.abs(e))) < 2.0 for e in err.values())}

    def skill_move_joints(self, speed: float = 1.0, **joints):
        goals = {}
        for pre in self.arms:
            g = self.cmd[pre].copy()
            g[:5] = [joints[self._jname(pre, j)] for j in self.robot["joints"]]
            goals[pre] = g
        res = yield from self._move(goals, speed)
        return res

    def skill_move_to(self, x: float, y: float, z: float, speed: float = 1.0, arm: str | None = None):
        pre = arm if self.bimanual else "arm"
        goal = np.array([x, y, z], float)
        if z < TOOL_CLEARANCE:
            return {"reached": False, "error": f"refused: z = {z:g} m is below the desk clearance ({TOOL_CLEARANCE} m)"}
        ik = self.ik[pre]
        aim = goal.copy()
        res: dict = {}
        pitch = 0.0
        for attempt in range(3):
            q_now = self._to_raw(self.cmd[pre])[:5]
            q, err, pitch = ik.solve(self.data.qpos, q_now, aim)
            if err > IK_TOL:
                if attempt == 0:
                    return {
                        "reached": False,
                        "error": f"no joint solution reaches ({x:g}, {y:g}, {z:g}) (closest {err * 1000:.0f} mm)",
                    }
                break
            g = self.cmd[pre].copy()
            g[:4] = np.degrees(q[:4])
            res = yield from self._move({pre: g}, speed if attempt == 0 else min(speed, 0.5))
            miss = goal - self.tool(pre)[0]
            if np.linalg.norm(miss) < 0.003 or np.linalg.norm(miss) > 0.05:
                break
            aim = aim + miss
        p = self.tool(pre)[0]
        dist = float(np.linalg.norm(p - goal))
        out = {
            **res,
            "reached": dist < 0.01,
            "tool_pos": r3(p, 4),
            "distance_m": round(dist, 4),
            "pitch_from_vertical_deg": round(pitch, 1),
        }
        if self.bimanual:
            out["arm"] = pre
        return out

    def skill_grip(self, value: float, arm: str = "both"):
        pres = self._arms_of(arm)
        goals = {}
        for pre in pres:
            g = self.cmd[pre].copy()
            g[5] = float(np.clip(value, GRIP_LOW, GRIP_HIGH))
            goals[pre] = g
        yield from self._move(goals, 1.0, grip_only=True)
        last = {pre: self.measured(pre)[5] for pre in pres}
        still = 0
        for _ in range(40):  # wait until the jaws stop moving
            yield [0.0] * self.action_spec.dim
            now = {pre: self.measured(pre)[5] for pre in pres}
            still = still + 1 if all(abs(now[p] - last[p]) < 0.3 for p in pres) else 0
            last = now
            if still >= 3:
                break
        now = {self._jname(p, "gripper"): round(float(last[p]), 2) for p in pres}
        short = any(abs(last[p] - goals[p][5]) > 3 for p in pres)
        out = {
            "gripper": now if len(now) > 1 else next(iter(now.values())),
            "holding": {p: self.holding(p) for p in pres} if self.bimanual else self.holding("arm"),
        }
        if short:
            out["note"] = "stopped short (holding an object?)"
        return out

    def _pose(self, which: str, arm: str = "both"):
        goals = {}
        for pre in self._arms_of(arm):
            g = self.cmd[pre].copy()
            g[:5] = self.robot[which]
            goals[pre] = g
        res = yield from self._move(goals, 1.0)
        return res

    def skill_home(self, arm: str = "both"):
        res = yield from self._pose("home", arm)
        return res

    def skill_park(self, arm: str = "both"):
        res = yield from self._pose("park", arm)
        return res


def oracle_main(env: str) -> None:
    from ..embodied import Oracle
    from .oracles import ORACLES

    Oracle().run(ORACLES[env])
