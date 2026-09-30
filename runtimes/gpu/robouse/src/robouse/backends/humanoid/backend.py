"""Humanoid backend: upper-body table-top manipulation with four MuJoCo Menagerie humanoids on a support stand.

Robots (sim/robots.py): Unitree G1 with Dex3-1 three-finger hands (BSD-3-Clause), Unitree H1 (BSD-3-Clause;
no hands, each forearm ends in a round fist), Apptronik Apollo (Apache-2.0; rigid hand plates, the fingers are visual
only) and Booster T1 (Apache-2.0; 4-DoF arms ending in rigid cylindrical hands).

Mounting: the free joint of each robot's root body is removed, so the pelvis (T1: trunk) is fixed to a support stand and
the robot does not balance or walk. Legs, waist and neck hold a standing pose with their own actuators (G1 and Apollo
lean forward at the waist, the T1 trunk is mounted leaning forward). Walking is not offered: a physical humanoid gait
needs a learned whole-body policy (e.g. MuJoCo Playground locomotion policies), which needs GPU training.

Controller (sim/control.py) under `robo act`, one control step = 40 ms = 20 physics steps of 2 ms:
  left.ee_delta / right.ee_delta [DX, DY, DZ] in [-1, 1] x 2 cm move each hand's goal point (zero holds it); damped
  least-squares IK (a fixed hand orientation on the 7-DoF arms, a posture term in the null space, a bounded integral
  term against load sag) turns the goals into joint setpoints; the joint servos (position actuators with gravity
  feed-forward; on the H1 a torque PD with gravity compensation) track them within the joints' torque limits.
  left.gripper / right.gripper (G1): +1 closes the Dex3 finger synergy, -1 opens it, 0 keeps it (0.2 per step).
  head.delta (Apollo, T1): neck yaw / pitch rate, x 3 deg per step.
Skills: reach, reach_both, home, wait; grasp / release (G1); look_at (Apollo, T1).
Tasks, scenes and success rules: sim/tasks.py; reference solutions: sim/oracles.py.
"""

from __future__ import annotations

import math

import mujoco
import numpy as np

from ..embodied import ActionGroup, Budget, EmbodiedBackend, Embodiment, Oracle, Sensor, Skill, SkillArg, r3
from .sim import scene as S
from .sim.control import GOAL_LEAD, MAX_DQ, REACH_SLACK, ArmIK, Servo
from .sim.robots import ROBOTS, SIDE_KEY
from .sim.tasks import TASKS, tilt_deg

CONTROL_DT = 0.04
N_SUB = round(CONTROL_DT / S.PHYS_DT)
EE_STEP = 0.02  # m per control step at |action| = 1
GRIP_RATE = 0.2  # finger synergy change per step at |grip| = 1
HEAD_RATE = math.radians(3)
I_GAIN, I_MAX = 0.25, 0.03  # integral action on the hand position error (per step, m)
SIDES = ("left", "right")


# fixed hand orientations held by the 7-DoF arms: rotation of the end-effector body in the world frame, given by where
# the body's y and z axes point (x = y cross z)
def _frame(y, z) -> np.ndarray:
    y, z = np.array(y, float), np.array(z, float)
    return np.column_stack([np.cross(y, z), y, z])


HAND_POSES = {
    # G1: fingers forward and level, palm toward the midline, thumb below-medial
    ("g1", "default"): {"left": np.eye(3), "right": np.eye(3)},
    # Apollo (the hand plate extends along -z of the wrist body; the palm is its -y face on the left hand, +y on the
    # right): fingers forward, palms facing each other
    ("apollo", "palms_in"): {"left": _frame((0, 1, 0), (-1, 0, 0)), "right": _frame((0, 1, 0), (-1, 0, 0))},
}
# Dex3-1 finger synergy (right hand; the left hand mirrors the signs of all but thumb_0):
# thumb_0, thumb_1, thumb_2, middle_0, middle_1, index_0, index_1
FINGER_OPEN = np.array([0.0, 0.3, 0.0, 0.0, 0.0, 0.0, 0.0])
FINGER_CLOSED = np.array([0.0, -0.9, -1.0, 0.7, 0.9, 0.7, 0.9])
FINGER_MIRROR = np.array([1, -1, -1, -1, -1, -1, -1])


class HumanoidBackend(EmbodiedBackend):
    name = "humanoid"

    def __init__(self, spec: dict):
        super().__init__(spec)
        env = spec.get("env") or spec.get("id")
        if env not in TASKS:
            raise KeyError(f"unknown humanoid task {env!r}")
        self.task_id, self.task = env, TASKS[env]
        self.rk = self.task.robot
        self.cfg = ROBOTS[self.rk]
        self.skey = SIDE_KEY[self.rk]
        self.has_hands = self.cfg["hand"] is not None
        self.has_head = self.cfg["head"] is not None
        self.camera = spec.get("camera", "front")
        self.decl = self._declare()

    # ---- declaration -------------------------------------------------------------------------------------------
    def _declare(self) -> Embodiment:
        c = self.cfg
        groups = [
            ActionGroup(
                "left.ee_delta",
                "ee_delta_pos",
                ["LDX", "LDY", "LDZ"],
                [-1] * 3,
                [1] * 3,
                "x 2 cm per step (world x, y, z)",
                "moves the left hand's goal point; zero holds it",
                frame="world",
            ),
            ActionGroup(
                "right.ee_delta",
                "ee_delta_pos",
                ["RDX", "RDY", "RDZ"],
                [-1] * 3,
                [1] * 3,
                "x 2 cm per step (world x, y, z)",
                "moves the right hand's goal point; zero holds it",
                frame="world",
            ),
        ]
        if self.has_hands:
            groups += [
                ActionGroup(
                    "left.gripper",
                    "gripper",
                    ["LGRIP"],
                    [-1],
                    [1],
                    "synergy rate, x 0.2 per step",
                    "+1 closes the Dex3 fingers, -1 opens them, 0 keeps the current finger command",
                ),
                ActionGroup(
                    "right.gripper",
                    "gripper",
                    ["RGRIP"],
                    [-1],
                    [1],
                    "synergy rate, x 0.2 per step",
                    "+1 closes the Dex3 fingers, -1 opens them, 0 keeps the current finger command",
                ),
            ]
        if self.has_head:
            groups.append(
                ActionGroup(
                    "head.delta",
                    "joint_delta",
                    ["HYAW", "HPITCH"],
                    [-1] * 2,
                    [1] * 2,
                    "x 3 deg per step",
                    "neck yaw (+ = left) and pitch (+ = down) rate; zero holds the head",
                )
            )
        sensors = [
            Sensor(
                "robot",
                "proprio",
                ["robot"],
                "hand positions and goals, what each hand touches, grip (G1), head angles",
                units="m, deg",
                frame="world",
            ),
            Sensor(
                "objects",
                "world",
                ["objects"],
                "free objects: position, tilt, whether the robot touches them",
                units="m, deg",
                frame="world",
            ),
            Sensor(
                "scene",
                "world",
                ["scene"],
                "table, fixtures and target zones (public positions)",
                units="m",
                frame="world",
            ),
            Sensor("events", "events", ["progress", "safety_events"], "task progress and safety events"),
            *[
                Sensor(
                    f"camera:{cam}",
                    "camera",
                    [],
                    "",
                    mount=("head" if self.has_head else "body") if cam == "head" else "world",
                )
                for cam in self._cameras()
            ],
        ]
        side = SkillArg("side", "enum", choices=["left", "right"], doc="which arm")
        xyz = [SkillArg("x", unit="m"), SkillArg("y", unit="m"), SkillArg("z", unit="m")]
        tol = SkillArg("tol", "float", "m", 0.015, doc="arrival tolerance")
        skills = [
            Skill(
                "reach",
                [side] + xyz + [tol],
                "move one hand's control point in a straight line to (X, Y, Z) (world frame); "
                "stops when there, when blocked or at the edge of its reach, and reports which",
                150,
            ),
            Skill(
                "reach_both",
                [SkillArg(f"{c_}{s}", unit="m") for s in ("l", "r") for c_ in "xyz"] + [tol],
                "move both hands at once, in straight lines, to (XL, YL, ZL) and (XR, YR, ZR); for bimanual holds",
                150,
            ),
            Skill("home", [], "both hands back to their start points", 150),
            Skill("wait", [SkillArg("steps", "int", default=10)], "hold everything still for STEPS control steps", 100),
        ]
        if self.has_hands:
            skills += [
                Skill("grasp", [side], "close that hand's fingers until they stop; reports what the hand touches", 30),
                Skill("release", [side], "open that hand's fingers", 12),
            ]
        if self.has_head:
            skills.append(
                Skill(
                    "look_at",
                    xyz,
                    "turn the neck so the head camera points at (X, Y, Z); reports how far off it ended "
                    "(the neck has joint limits)",
                    60,
                )
            )
        return Embodiment(
            robot=f"{c['name']} (fixed to a support stand; no balance, no walking)",
            family="humanoid",
            assets=[c["folder"]],
            sensors=sensors,
            action_groups=groups,
            skills=skills,
            budget=Budget(self.task.max_steps, CONTROL_DT),
            cameras=self._cameras(),
        )

    def _cameras(self) -> list[str]:
        return ["front", "top", "side", "head"]

    # ---- episode -----------------------------------------------------------------------------------------------
    def reset(self, seed: int) -> None:
        self._reset_renderers()
        t, c = self.task, self.cfg
        s = S.new_scene(self.rk)
        t.build(s)
        head_body = {"g1": "torso_link", "h1": "torso_link", "apollo": "neck_pitch_link", "t1": "H2"}[self.rk]
        head_off = {"g1": (0.1, 0, 0.45), "h1": (0.13, 0, 0.62), "apollo": (0.26, 0, 0.1), "t1": (0.1, 0, 0.1)}[self.rk]
        S.add_cameras(s, self.rk, t.look, t.front_cam, head_body, head_off)
        self.model = m = s.compile()
        self.data = d = mujoco.MjData(m)
        self.stand = S.stand_qpos(self.rk)
        for n, v in self.stand.items():
            d.qpos[m.joint(n).qposadr[0]] = v
        mujoco.mj_forward(m, d)
        self.scratch = mujoco.MjData(m)
        self.ik, self.arm_idx = {}, {}
        self.servo = Servo(m, d, c["motor"])
        poses = t.hand_pose if isinstance(t.hand_pose, dict) else {sd: t.hand_pose for sd in SIDES}
        for side in SIDES:
            joints = c["arms"][self.skey[side]]
            qadr = m.jnt_qposadr[[m.joint(j).id for j in joints]]
            ik = ArmIK(m, self.scratch, joints, f"ee_{self.skey[side]}", c["orient"], d.qpos[qadr].copy())
            if c["orient"] is True:
                ik.R_goal = HAND_POSES[(self.rk, poses[side])][side]
            # start pose: the arm configuration that puts the hand at the task's home point (set once, before the
            # episode); multi-start IK, preferring solutions away from the joint limits
            q = self._home_config(ik, np.array(t.home[side], float))
            d.qpos[qadr] = q
            ik.q_nom = q.copy()
            self.ik[side] = ik
            self.arm_idx[side] = self.servo.idx(joints)
        mujoco.mj_forward(m, d)
        self.servo.q_des = d.qpos[self.servo.qadr].copy()
        self.grip = {sd: 0.0 for sd in SIDES}
        if self.has_hands:
            self.finger_idx = {sd: self.servo.idx(c["hand"][self.skey[sd]]["joints"]) for sd in SIDES}
            self._set_fingers()
        if self.has_head:
            self.head_idx = self.servo.idx(c["head"])
            self.head_q = self.servo.q_des[self.head_idx].copy()
        self.goal = {sd: self.ee(sd) for sd in SIDES}
        self.ibias = {sd: np.zeros(3) for sd in SIDES}
        self._geom_tables()
        self.events, self._event_keys = [], set()
        self.task_state: dict = {}
        # let the objects settle on their supports (the robot holds its start pose)
        for _ in range(10):
            self.control_step(np.array(self.hold_action(), float))
        self.events, self._event_keys = [], set()
        self.task_state = {}

    def _home_config(self, ik: ArmIK, goal: np.ndarray) -> np.ndarray:
        rng = np.random.default_rng(0)
        mid, span = (ik.lo + ik.hi) / 2, ik.hi - ik.lo
        seeds = [ik.q_nom.copy(), np.clip(np.zeros_like(mid), ik.lo, ik.hi)] + [
            mid + (rng.random(len(mid)) - 0.5) * 0.7 * span for _ in range(24)
        ]
        best, best_cost = seeds[0], 1e9
        for q in seeds:
            for _ in range(15):
                q, res = ik.solve(q, goal, self.data.qpos, iters=4, max_dq=0.5)
            cost = res + 0.01 * float(np.sum(((q - mid) / span) ** 2))
            if cost < best_cost:
                best, best_cost = q, cost
        return best

    def _geom_tables(self) -> None:
        m = self.model
        self.gname = [m.geom(g).name for g in range(m.ngeom)]
        self.bname = [m.body(b).name for b in range(m.nbody)]
        self.robot_geom = np.array([m.geom_contype[g] == S.ROBOT_CT for g in range(m.ngeom)])
        self.hand_geoms = {}
        for sd in SIDES:  # what counts as "the hand" in contact reports (the forearm too on the fingerless arms)
            root = m.body(self.cfg["touch_root"][self.skey[sd]]).id
            bodies = {b for b in range(m.nbody) if self._descends(b, root)}
            self.hand_geoms[sd] = {g for g in range(m.ngeom) if m.geom_bodyid[g] in bodies and self.robot_geom[g]}
        self.obj_body = {n: m.body(n).id for n in self.task.objects}
        self.obj_geoms = {n: {g for g in range(m.ngeom) if m.geom_bodyid[g] == b} for n, b in self.obj_body.items()}
        # contact label: a free object's name for all of its geoms, else the geom name
        self.label = list(self.gname)
        for n, gs in self.obj_geoms.items():
            for g in gs:
                self.label[g] = n
        self.floor = m.geom("floor").id

    def _descends(self, b: int, root: int) -> bool:
        while b > 0:
            if b == root:
                return True
            b = int(self.model.body_parentid[b])
        return False

    # ---- state helpers -----------------------------------------------------------------------------------------
    def ee(self, side: str) -> np.ndarray:
        return self.data.site_xpos[self.model.site(f"ee_{self.skey[side]}").id].copy()

    def obj_pos(self, name: str) -> np.ndarray:
        return self.data.xpos[self.obj_body[name]].copy()

    def obj_tilt(self, name: str) -> float:
        return tilt_deg(self.data.xmat[self.obj_body[name]])

    def contacts(self):
        d = self.data
        for c in d.contact[: d.ncon]:
            yield int(c.geom1), int(c.geom2)

    def touching(self, geoms: set[int]) -> set[str]:
        """Names of the non-robot geoms in contact with any of `geoms`."""
        out = set()
        for a, b in self.contacts():
            if a in geoms and not self.robot_geom[b]:
                out.add(self.label[b])
            elif b in geoms and not self.robot_geom[a]:
                out.add(self.label[a])
        return out

    def robot_touching(self, obj: str) -> bool:
        gs = self.obj_geoms[obj]
        return any((a in gs and self.robot_geom[b]) or (b in gs and self.robot_geom[a]) for a, b in self.contacts())

    def geom_touching(self, g1: int, g2: int) -> bool:
        return any({a, b} == {g1, g2} for a, b in self.contacts())

    def hand_holding(self, side: str) -> str | None:
        """An object squeezed between the thumb and a finger of a G1 hand (for reports only)."""
        if not self.has_hands:
            return None
        m = self.model
        thumb, finger = set(), set()
        for g in self.hand_geoms[side]:
            bn = self.bname[m.geom_bodyid[g]]
            (thumb if "thumb" in bn else finger if ("index" in bn or "middle" in bn) else set()).add(g)
        th, fi = self.touching(thumb), self.touching(finger)
        both = [n for n in th & fi if n in self.obj_body]
        return both[0] if both else None

    # ---- control -----------------------------------------------------------------------------------------------
    def _set_fingers(self) -> None:
        for sd in SIDES:
            q = FINGER_OPEN + self.grip[sd] * (FINGER_CLOSED - FINGER_OPEN)
            if sd == "left":
                q = q * FINGER_MIRROR
            self.servo.q_des[self.finger_idx[sd]] = q

    def control_step(self, a: np.ndarray) -> None:
        a = np.asarray(a, float)
        sv = self.servo
        for i, sd in enumerate(SIDES):
            delta = a[3 * i : 3 * i + 3] * EE_STEP
            x = self.ee(sd)
            g = self.goal[sd] + delta
            off = g - x
            n = float(np.linalg.norm(off))
            if n > GOAL_LEAD:  # anti wind-up: the goal never runs far ahead of the measured hand
                g = x + off * GOAL_LEAD / n
            ik = self.ik[sd]
            q0 = sv.q_des[self.arm_idx[sd]].copy()
            # slow integral term: removes the steady sag of a loaded arm (bounded, so a blocked hand does not wind up)
            # (only while the goal is still: during a motion the hand lags its goal by design)
            err = self.goal[sd] - x
            if float(np.linalg.norm(err)) < 0.03:
                self.ibias[sd] = np.clip(
                    self.ibias[sd] + (I_GAIN if not np.any(delta) else 0.3 * I_GAIN) * err, -I_MAX, I_MAX
                )
            g = g + self.ibias[sd]
            q, res = ik.solve(q0, g, self.data.qpos)
            if res > REACH_SLACK:
                # the warm-started solve is stuck (joint limit or local minimum): solve afresh from the posture
                # configuration and, if that reaches the goal, move toward it (still rate limited)
                qf, resf = ik.solve(ik.q_nom, g, self.data.qpos, iters=120, max_dq=10.0)
                qb, resb = (qf, resf) if resf < res else (q, res)
                if resf < res:
                    q = q0 + np.clip(qf - q0, -MAX_DQ, MAX_DQ)
                if resb > REACH_SLACK:  # out of reach: keep the goal at the edge of what the arm can do
                    xb, _ = ik.fk(qb, self.data.qpos)
                    g = xb + (g - xb) * REACH_SLACK / resb
            sv.q_des[self.arm_idx[sd]] = q
            self.goal[sd] = g - self.ibias[sd]
        k = 6
        if self.has_hands:
            for i, sd in enumerate(SIDES):
                self.grip[sd] = float(np.clip(self.grip[sd] + GRIP_RATE * a[k + i], 0, 1))
            self._set_fingers()
            k += 2
        if self.has_head:
            lo = self.model.jnt_range[[self.model.joint(j).id for j in self.cfg["head"]], 0]
            hi = self.model.jnt_range[[self.model.joint(j).id for j in self.cfg["head"]], 1]
            self.head_q = np.clip(self.head_q + HEAD_RATE * a[k : k + 2] * np.array([1, self._pitch_sign()]), lo, hi)
            sv.q_des[self.head_idx] = self.head_q
        m, d = self.model, self.data
        for i in range(N_SUB):
            if i == 0 or sv.motor:
                sv.apply()
            mujoco.mj_step(m, d)
            if i % 4 == 3:
                self._monitor()

    def _pitch_sign(self) -> float:
        """+1 if raising the neck pitch joint tilts the head camera down, else -1 (so HPITCH + always looks down)."""
        if not hasattr(self, "_psign"):
            q = self.head_q.copy()
            f0 = self._head_view(q)[1]
            q[1] += 0.05
            self._psign = 1.0 if self._head_view(q)[1][2] < f0[2] else -1.0
        return self._psign

    # ---- monitor -----------------------------------------------------------------------------------------------
    def _monitor(self) -> None:
        for a, b in self.contacts():
            for g, o in ((a, b), (b, a)):
                name = self.label[g]
                if name in self.obj_body and o == self.floor:
                    self.event(f"floor:{name}", "dropped", f"the {name.replace('_', ' ')} fell on the floor")
                if name in self.task.fragile and self.robot_geom[o]:
                    self.event(f"hit:{name}", "damage", f"the robot hit the {name.replace('_', ' ')}")
        for name in self.task.fragile:
            if name in self.obj_body and self.obj_tilt(name) > 35:
                self.event(f"tip:{name}", "damage", f"the {name.replace('_', ' ')} was knocked over")
        self.task.monitor(self) if hasattr(self.task, "monitor") else None

    # ---- observation -------------------------------------------------------------------------------------------
    def observe(self) -> dict:
        hands = {}
        for sd in SIDES:
            h = {
                "pos": r3(self.ee(sd)),
                "goal": r3(self.goal[sd]),
                "touching": sorted(self.touching(self.hand_geoms[sd])),
            }
            if self.has_hands:
                h["grip"] = round(self.grip[sd], 2)
                h["holding"] = self.hand_holding(sd)
            hands[sd] = h
        robot = {"model": self.cfg["name"], "mount": self.cfg["mount"], "hands": hands}
        if self.has_head:
            robot["head"] = {
                "yaw_deg": round(math.degrees(self.head_q[0]), 1),
                "pitch_deg": round(math.degrees(self.head_q[1]), 1),
            }
        objs = {}
        for n in self.task.objects:
            objs[n] = {
                "pos": r3(self.obj_pos(n)),
                "tilt_deg": round(self.obj_tilt(n), 1),
                "touched_by_robot": self.robot_touching(n),
            }
            objs[n].update(self.task.object_info(n) if hasattr(self.task, "object_info") else {})
        st = {
            "robot": robot,
            "objects": objs,
            "scene": self.task.landmarks(self),
            "time_s": round(float(self.data.time), 2),
        }
        pr = self.task.progress(self)
        if pr is not None:
            st["progress"] = pr
        st["safety_events"] = [e["detail"] for e in self.events]
        return st

    # ---- scoring -----------------------------------------------------------------------------------------------
    def success(self) -> bool:
        if self.events:
            return False
        ok, _why = self.task.check(self)
        return bool(ok)

    # ---- skills ------------------------------------------------------------------------------------------------
    def _act(self, deltas: dict | None = None, grip: dict | None = None, head=None) -> list[float]:
        v = list(self.hold_action())
        for i, sd in enumerate(SIDES):
            if deltas and sd in deltas:
                v[3 * i : 3 * i + 3] = [float(x) for x in np.clip(deltas[sd] / EE_STEP, -1, 1)]
        k = 6
        if self.has_hands:
            for i, sd in enumerate(SIDES):
                if grip and sd in grip:
                    v[k + i] = float(grip[sd])
            k += 2
        if self.has_head and head is not None:
            v[k : k + 2] = [float(x) for x in np.clip(head, -1, 1)]
        return v

    def _move(self, targets: dict, tol: float, speed: float = EE_STEP):
        """Move the goal of each side in `targets` along a straight line; stop on arrival, stall or reach limit."""
        tol = max(tol, 0.005)
        targets = {sd: np.asarray(p, float) for sd, p in targets.items()}
        hist: list[float] = []
        steps = 0
        status = "reached"
        while True:
            err = {sd: float(np.linalg.norm(self.ee(sd) - p)) for sd, p in targets.items()}
            if all(e <= tol for e in err.values()):
                break
            deltas = {}
            for sd, p in targets.items():
                off = p - self.goal[sd]
                n = float(np.linalg.norm(off))
                deltas[sd] = off if n <= speed else off * speed / n
            hist.append(sum(err.values()))
            steps += 1
            if steps > 25 and hist[-20] - hist[-1] < 0.004:
                gerr = max(float(np.linalg.norm(self.goal[sd] - p)) for sd, p in targets.items())
                status = "out_of_reach" if gerr > 0.02 else "blocked"
                break
            yield self._act(deltas)
        out = {"status": status, "reached": status == "reached"}
        for sd, p in targets.items():
            out[sd] = {
                "hand": r3(self.ee(sd)),
                "distance_m": round(float(np.linalg.norm(self.ee(sd) - p)), 3),
                "touching": sorted(self.touching(self.hand_geoms[sd])),
            }
        return out

    def skill_reach(self, side: str, x: float, y: float, z: float, tol: float):
        res = yield from self._move({side: (x, y, z)}, tol)
        return res

    def skill_reach_both(self, xl, yl, zl, xr, yr, zr, tol: float):
        res = yield from self._move({"left": (xl, yl, zl), "right": (xr, yr, zr)}, tol)
        return res

    def skill_home(self):
        res = yield from self._move({sd: self.task.home[sd] for sd in SIDES}, 0.02)
        return res

    def skill_wait(self, steps: int):
        for _ in range(max(1, min(int(steps), 100))):
            yield self.hold_action()
        return {}

    def skill_grasp(self, side: str):
        still = 0
        for i in range(30):
            q0 = self.data.qpos[self.servo.qadr[self.finger_idx[side]]].copy()
            yield self._act(grip={side: 1.0})
            dq = float(np.abs(self.data.qpos[self.servo.qadr[self.finger_idx[side]]] - q0).max())
            still = still + 1 if dq < 0.01 else 0
            if i >= 5 and still >= 3:
                break
        return {
            "grip": round(self.grip[side], 2),
            "touching": sorted(self.touching(self.hand_geoms[side])),
            "holding": self.hand_holding(side),
        }

    def skill_release(self, side: str):
        for _ in range(8):
            yield self._act(grip={side: -1.0})
        return {"grip": round(self.grip[side], 2), "touching": sorted(self.touching(self.hand_geoms[side]))}

    def _head_view(self, q_head: np.ndarray):
        """Head camera position and forward axis if the neck joints were at q_head (scratch kinematics)."""
        s = self.scratch
        s.qpos[:] = self.data.qpos
        s.qpos[self.servo.qadr[self.head_idx]] = q_head
        mujoco.mj_kinematics(self.model, s)
        c = self.model.camera("head").id
        mujoco.mj_camlight(self.model, s)
        return s.cam_xpos[c].copy(), -s.cam_xmat[c].reshape(3, 3)[:, 2].copy()

    def skill_look_at(self, x: float, y: float, z: float):
        target = np.array([x, y, z], float)

        def err(q):
            p, f = self._head_view(q)
            w = target - p
            w /= np.linalg.norm(w)
            return w - f

        lo = self.model.jnt_range[[self.model.joint(j).id for j in self.cfg["head"]], 0]
        hi = self.model.jnt_range[[self.model.joint(j).id for j in self.cfg["head"]], 1]
        qa = lambda: self.data.qpos[self.servo.qadr[self.head_idx]].copy()  # measured neck angles
        for _ in range(60):
            q = qa()
            e = err(q)
            if float(np.linalg.norm(e)) < math.radians(1.5):
                break
            J = np.zeros((3, 2))
            for j in range(2):  # numerical Jacobian of the view direction w.r.t. the two neck joints
                dq = np.zeros(2)
                dq[j] = 1e-3
                J[:, j] = (err(q) - err(q + dq)) / 1e-3
            want = np.clip(q + np.linalg.lstsq(J, e, rcond=None)[0], lo, hi)
            a = (want - self.head_q) / HEAD_RATE * np.array([1, self._pitch_sign()])
            if float(np.abs(want - q).max()) < math.radians(0.3):
                break  # at a joint limit, or there
            yield self._act(head=a)
        p, f = self._head_view(qa())
        w = target - p
        off = math.degrees(math.acos(float(np.clip(f @ (w / np.linalg.norm(w)), -1, 1))))
        return {
            "yaw_deg": round(math.degrees(self.head_q[0]), 1),
            "pitch_deg": round(math.degrees(self.head_q[1]), 1),
            "off_target_deg": round(off, 1),
        }


# ------------------------------------------------------------------------------------------------------------------
# reference solutions (socket only)
# ------------------------------------------------------------------------------------------------------------------


def oracle_main(env: str) -> None:
    from .sim.oracles import ORACLES

    Oracle().run(ORACLES[env])
