"""Embodiment drivers for remixed tasks: the robot model, its placement in a scene, its controllers ("firmware"
under `robo act`), its declared action groups and skills, and the robot part of `robo observe`.

Every driver works in any scene that offers its placement (`mount`): surface-mounted arms (`arm`, `bimanual`) get a
base pose on a work surface, floor robots (`floor`) a start pose, drones (`airspace`) a start pad.

Skills are the same small vocabulary on every embodiment that has the capability, so reference solutions (and
agents) can be written once:

  manipulation (reach, grasp)   move_to X Y Z [ARM], grasp [ARM], release [ARM], home [ARM]
  wheeled or legged base        go_to X Y [YAW_DEG], turn YAW_DEG, look_at X Y Z
  flight                        takeoff Z, fly_to X Y Z, land, turn YAW_DEG, look_at X Y Z

A skill is a generator that yields one action per 50 ms control step; the episode server runs each as a normal
step (budget, video, success checks). A skill never moves the robot in a way `robo act` could not.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from ..embodied import ActionGroup, Skill, SkillArg, lookat_xyaxes, r3, wrap, yaw_of

PHYS_DT = .002
CONTROL_DT = .05
N_SUB = round(CONTROL_DT / PHYS_DT)


def _yawq(yaw: float) -> list[float]:
    return [math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)]


def _mulq(a, b) -> np.ndarray:
    import mujoco

    out = np.zeros(4)
    mujoco.mju_mulQuat(out, np.asarray(a, float), np.asarray(b, float))
    return out


class Driver:
    key = ""
    name = ""
    kind = ""            # BenchFlow embodiment kind
    family = ""          # robouse embodiment family
    mount = ""           # "arm" | "bimanual" | "floor" | "airspace"
    caps: tuple = ()
    grasp_mode = None    # "top" | "front" | None
    max_grip = 0.0       # widest object the fingers can close on with a margin (m)
    assets: list = []
    arms: list = []
    prefix = "robot/"

    def __init__(self, layout, params: dict | None = None):
        self.layout = layout
        self.params = dict(params or {})
        self.m = self.d = None
        self.robot_geoms: set = set()
        self.finger_sides: dict = {}   # arm -> (geoms of one finger, geoms of the other)
        self.body_geoms: set = set()   # geoms that must not hit furniture hard (base, legs, torso)

    # ---- declaration -------------------------------------------------------------------------------------------
    def groups(self) -> list[ActionGroup]:
        raise NotImplementedError

    def skills(self) -> list[Skill]:
        raise NotImplementedError

    def cameras(self) -> list[str]:
        return []

    def inspect_camera(self) -> str | None:
        return None

    def vec(self, **kw) -> list[float]:
        """A full action vector: groups named in kw (dots -> underscores) get those values, the rest hold."""
        out: list[float] = []
        for g in self.groups():
            v = kw.get(g.name.replace(".", "_"))
            if v is None:
                out += list(g.hold) if g.hold is not None else [0.0] * len(g.names)
            else:
                v = list(np.atleast_1d(np.asarray(v, float)))
                out += [float(np.clip(x, lo, hi)) for x, lo, hi in zip(v, g.low, g.high)]
        return out

    def hold(self) -> list[float]:
        return self.vec()

    # ---- building ----------------------------------------------------------------------------------------------
    def options(self, s) -> None:
        import mujoco

        o = s.option
        o.timestep = PHYS_DT
        o.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
        o.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
        o.impratio = 10
        o.noslip_iterations = 3
        o.enableflags |= mujoco.mjtEnableBit.mjENBL_MULTICCD

    def attach(self, s) -> None:
        raise NotImplementedError

    def bind(self, m, d) -> None:
        import mujoco

        self.m, self.d = m, d
        pre = self.prefix
        self.robot_bodies = {b for b in range(m.nbody) if (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b) or "").startswith(pre)}
        self.robot_geoms = {g for g in range(m.ngeom) if m.geom_bodyid[g] in self.robot_bodies}

    def bodies_named(self, names) -> set[int]:
        return {self.m.body(self.prefix + n).id for n in names}

    def geoms_of(self, bodies: set[int]) -> set[int]:
        return {g for g in range(self.m.ngeom) if self.m.geom_bodyid[g] in bodies}

    def init_state(self) -> None:
        pass

    def post_forward(self) -> None:
        pass

    # ---- running -----------------------------------------------------------------------------------------------
    def control(self, a: np.ndarray) -> None:
        raise NotImplementedError

    def substep(self, dt: float) -> None:
        raise NotImplementedError

    def state(self) -> dict:
        raise NotImplementedError

    def public(self) -> dict:
        """What the robot is and can do (part of `robo observe`)."""
        return {"embodiment": self.key, "name": self.name, "capabilities": list(self.caps),
                **({"grasp": GRASP_DOC[self.grasp_mode]} if self.grasp_mode else {})}

    # arm and base helpers used by skills (overridden where they apply)
    def ee_pos(self, arm: str) -> np.ndarray:
        raise NotImplementedError

    def base_pose(self) -> tuple[float, float, float]:
        raise NotImplementedError


GRASP_DOC = {
    "top": "top-down: the gripper points straight down and its fingers close along the world y axis",
    "front": "front: the gripper points along the robot's heading and stays level; its fingers close horizontally",
}

ARM_ARG = SkillArg("arm", "enum", default="auto", choices=["auto", "left", "right"],
                   doc="which arm (bimanual rigs; auto: for move_to the arm whose base is nearer the target, otherwise the arm "
                       "that moved last)")


# ======================================================================================================================
# surface-mounted arms: Franka Panda (single) and ALOHA 2 (a pair of ViperX 300 s)
# ======================================================================================================================

@dataclass
class ArmCfg:
    name: str
    joints: tuple
    actuators: tuple
    finger_bodies: tuple      # (one side, other side)
    gripper_act: str
    tcp: str
    home: tuple
    down_quat: tuple
    open_ctrl: float
    closed_ctrl: float
    reach: float              # max horizontal reach of the tcp from the base axis (m)
    reach_min: float
    z_max: float              # max tcp height above the mount surface


class ArmRig(Driver):
    """One or two position-controlled arms with Cartesian gripper targets.

    arm.ee_delta [DX, DY, DZ]: the gripper target moves by the value x 2 cm per 50 ms step (world axes); damped
    least-squares IK turns the target into joint targets, slewed at 1.2 rad/s with gravity compensation on the arm
    joints. The target stays within 5 cm of the measured gripper point and inside the arm's reach.
    gripper [G]: 0 keeps the fingers, > 0 closes them, -f opens them to fraction f (-1 fully open).
    The gripper always points straight down (top-down grasps)."""

    STEP = .02
    LEAD = .05
    grasp_mode = "top"

    def __init__(self, layout, params=None):
        super().__init__(layout, params)
        self.cfg: dict[str, ArmCfg] = {}

    def arm_names(self) -> list[str]:
        return list(self.arms)

    def groups(self) -> list[ActionGroup]:
        out = []
        for a in self.arms:
            p = "arm" if a == "arm" else a
            out.append(ActionGroup(f"{p}.ee_delta", "ee_delta_pos", ["DX", "DY", "DZ"] if a == "arm" else [f"{a[0].upper()}_DX", f"{a[0].upper()}_DY", f"{a[0].upper()}_DZ"],
                                   [-1] * 3, [1] * 3, "x 2 cm per step",
                                   "moves the gripper target (world axes); IK follows it with the gripper pointing down", frame="world"))
            out.append(ActionGroup("gripper" if a == "arm" else f"{a}.gripper", "gripper", ["G"] if a == "arm" else [f"{a[0].upper()}_G"],
                                   [-1], [1], "command", "0 keeps the fingers, > 0 closes, -f opens to fraction f (-1 fully open)"))
        return out

    def skills(self) -> list[Skill]:
        arm = [ARM_ARG] if len(self.arms) > 1 else []
        xyz = [SkillArg("x", unit="m"), SkillArg("y", unit="m"), SkillArg("z", unit="m")]
        return [
            Skill("move_to", xyz + arm, "move the grasp point (between the fingertips) to (X, Y, Z) along a straight line, "
                  "gripper pointing down; reports whether it got there (it stops if blocked or out of reach)", 150),
            Skill("grasp", arm, "close the fingers until they stop; reports what is held", 30),
            Skill("release", arm, "open the fingers fully", 15),
            Skill("home", arm, "move the gripper back to its start point above the work area", 150),
        ]

    def cameras(self) -> list[str]:
        return [f"{self.prefix}{a}_wrist" for a in self.arms]

    # ---- building ----------------------------------------------------------------------------------------------
    def mount_info(self):
        am = self.layout.mounts[self.mount]
        return am, self.layout.surfaces[am.surface]

    def post_forward(self) -> None:
        import mujoco

        from ..menagerie_sim.ik import ArmIK

        m, d = self.m, self.d
        self.ik, self.q_cmd, self.target, self.open_cmd = {}, {}, {}, {}
        self.qadr, self.dadr, self.aid, self.gid = {}, {}, {}, {}
        am, sf = self.mount_info()
        self.top = sf.top
        self.base_xy = {}
        for a, c in self.cfg.items():
            jid = [m.joint(j).id for j in c.joints]
            self.qadr[a] = m.jnt_qposadr[jid]
            self.dadr[a] = m.jnt_dofadr[jid]
            self.aid[a] = np.array([m.actuator(n).id for n in c.actuators])
            self.gid[a] = m.actuator(c.gripper_act).id
            self.ik[a] = ArmIK(m, c)
            bx, by, _ = am.bases[a]
            self.base_xy[a] = np.array([bx, by])
            d.qpos[self.qadr[a]] = c.home
        mujoco.mj_forward(m, d)
        self.home_pt = {}
        for a, c in self.cfg.items():  # the start pose: gripper above the middle of the work area (part of the reset)
            self.home_pt[a] = self._home_point(a)
            q, _, _ = self.ik[a].solve(d, d.qpos[self.qadr[a]], self.home_pt[a], np.asarray(c.down_quat), iterations=400)
            d.qpos[self.qadr[a]] = q
            d.ctrl[self.aid[a]] = q
            d.ctrl[self.gid[a]] = c.open_ctrl
            self.q_cmd[a] = q.copy()
            self.open_cmd[a] = 1.0
        mujoco.mj_forward(m, d)
        for a in self.cfg:
            self.target[a] = self.ee_pos(a)
        self.fingers = {a: [self.m.body(b).id for b in self.cfg[a].finger_bodies] for a in self.cfg}
        self.finger_sides = {a: tuple(self.geoms_of({b}) for b in self.fingers[a]) for a in self.cfg}
        arm_dofs = [int(v) for a in self.cfg for v in self.dadr[a]]
        self._comp = np.array(arm_dofs)

    def _home_point(self, a: str) -> np.ndarray:
        am, sf = self.mount_info()
        x0, x1, y0, y1 = am.work
        bx, by = self.base_xy[a]
        cx = float(np.clip(bx, x0, x1)) if len(self.arms) > 1 else (x0 + x1) / 2
        cy = (y0 + y1) / 2
        v = np.array([cx - bx, cy - by])
        n = float(np.linalg.norm(v))
        r = min(max(n, self.cfg[a].reach_min + .05), self.cfg[a].reach - .1)
        p = np.array([bx, by]) + v / max(n, 1e-6) * r
        return np.array([p[0], p[1], sf.top + .25])

    # ---- control -----------------------------------------------------------------------------------------------
    def split(self, a: np.ndarray) -> dict:
        out = {}
        for i, arm in enumerate(self.arms):
            out[arm] = (a[4 * i:4 * i + 3], float(a[4 * i + 3]))
        return out

    def clamp(self, arm: str, t: np.ndarray) -> np.ndarray:
        c = self.cfg[arm]
        b = self.base_xy[arm]
        v = t[:2] - b
        n = float(np.linalg.norm(v))
        if n > c.reach:
            t[:2] = b + v * c.reach / n
        elif n < c.reach_min:
            t[:2] = b + v * c.reach_min / max(n, 1e-6)
        t[2] = float(np.clip(t[2], self.top + .013, self.top + c.z_max))
        return t

    def control(self, a: np.ndarray) -> None:
        d = self.d
        for arm, (dv, g) in self.split(a).items():
            dv = np.clip(np.asarray(dv, float), -1, 1) * self.STEP
            hand = self.ee_pos(arm)
            t = self.clamp(arm, self.target[arm] + dv)
            off = t - hand
            n = float(np.linalg.norm(off))
            if n > self.LEAD:
                t = hand + off * (self.LEAD / n)
            self.target[arm] = t
            if g > 1e-3:
                self.open_cmd[arm] = 0.0
            elif g < -1e-3:
                self.open_cmd[arm] = float(min(1.0, -g))
            c = self.cfg[arm]
            q, _, _ = self.ik[arm].solve(d, self.q_cmd[arm], t, np.asarray(c.down_quat))
            self.q_cmd[arm] = q
        self._ctrl_goal = {arm: self.q_cmd[arm] for arm in self.arms}

    def substep(self, dt: float) -> None:
        d = self.d
        lim = 1.2 * dt
        for arm, c in self.cfg.items():
            ids = self.aid[arm]
            d.ctrl[ids] += np.clip(self.q_cmd[arm] - d.ctrl[ids], -lim, lim)
            d.ctrl[self.gid[arm]] = c.closed_ctrl + self.open_cmd[arm] * (c.open_ctrl - c.closed_ctrl)
        d.qfrc_applied[:] = 0
        d.qfrc_applied[self._comp] = d.qfrc_bias[self._comp]

    # ---- state -------------------------------------------------------------------------------------------------
    def ee_pos(self, arm: str) -> np.ndarray:
        return self.d.site(self.cfg[arm].tcp).xpos.copy()

    def opening(self, arm: str) -> float:
        q = [abs(float(self.d.qpos[self.m.jnt_qposadr[self.m.body_jntadr[b]]])) for b in self.fingers[arm]]
        full = abs(self.finger_open_m(arm))
        return float(np.clip(np.mean(q) / full, 0, 1)) if full else 0.0

    def finger_open_m(self, arm: str) -> float:
        return .04

    def state(self) -> dict:
        out = {}
        for a in self.arms:
            key = "arm" if a == "arm" else f"{a}_arm"
            bx, by = self.base_xy[a]
            out[key] = {"gripper_pos": r3(self.ee_pos(a)), "gripper_target": r3(self.target[a]),
                        "opening": round(self.opening(a), 2), "base": [round(float(bx), 3), round(float(by), 3), round(self.top, 3)],
                        "reach_m": self.cfg[a].reach}
        return out

    def public(self) -> dict:
        out = super().public()
        c = next(iter(self.cfg.values()))
        out["workspace"] = (f"each arm's grasp point reaches {c.reach_min:g} to {c.reach:g} m horizontally from its base and up to "
                            f"{c.z_max:g} m above the surface it is mounted on")
        return out

    # ---- skills ------------------------------------------------------------------------------------------------
    def pick_arm(self, arm: str, target) -> str:
        if arm != "auto" and arm in self.arms:
            return arm
        if len(self.arms) == 1:
            return self.arms[0]
        t = np.asarray(target, float)[:2]
        return min(self.arms, key=lambda a: float(np.linalg.norm(t - self.base_xy[a])))

    def _arm_vec(self, arm: str, dv=(0, 0, 0), g: float = 0.0) -> list[float]:
        kw = {}
        for a in self.arms:
            p = "arm" if a == "arm" else a
            kw[f"{p}_ee_delta"] = list(dv) if a == arm else [0, 0, 0]
            kw["gripper" if a == "arm" else f"{a}_gripper"] = [g if a == arm else 0.0]
        return self.vec(**kw)

    def move_to(self, x: float, y: float, z: float, arm: str = "auto"):
        target = np.array([x, y, z], float)
        arm = self.pick_arm(arm, target)
        self.last_arm = arm
        goal = self.clamp(arm, target.copy())
        clamped = float(np.linalg.norm(goal - target)) > .002
        best, stuck = 9.0, 0
        for _ in range(120):
            e = goal - self.target[arm]
            n = float(np.linalg.norm(e))
            if n < 1e-4:
                break
            if n < best - 5e-4:
                best, stuck = n, 0
            else:
                stuck += 1
                if stuck > 10:
                    break
            yield self._arm_vec(arm, e * min(1.0, .015 / n) / self.STEP)
        best, stall = 9.0, 0
        for _ in range(40):
            err = float(np.linalg.norm(goal - self.ee_pos(arm)))
            if err < .004:
                break
            if err < best - 2e-4:
                best, stall = err, 0
            else:
                stall += 1
                if stall > 8:
                    break
            yield self._arm_vec(arm)
        err = float(np.linalg.norm(target - self.ee_pos(arm)))
        out = {"arm": arm, "reached": err < .012, "error_m": round(err, 3), "gripper_pos": r3(self.ee_pos(arm))}
        if clamped:
            out["note"] = "the point is outside this arm's reach; it went to the nearest reachable point"
        elif err >= .012:
            out["note"] = "the gripper did not get there (blocked?)"
        return out

    def _arm_or_last(self, arm: str) -> str:
        if arm in self.arms:
            self.last_arm = arm
            return arm
        return getattr(self, "last_arm", None) or self.arms[0]

    def grasp(self, arm: str = "auto"):
        arm = self._arm_or_last(arm)
        last, still = self.opening(arm), 0
        for i in range(30):
            yield self._arm_vec(arm, g=1.0)
            o = self.opening(arm)
            still = still + 1 if abs(o - last) < .003 else 0
            last = o
            if still >= 4 and i > 6:
                break
        return {"arm": arm, "opening": round(self.opening(arm), 2)}

    def release(self, arm: str = "auto"):
        arm = self._arm_or_last(arm)
        for _ in range(12):
            yield self._arm_vec(arm, g=-1.0)
        return {"arm": arm, "opening": round(self.opening(arm), 2)}

    def home(self, arm: str = "auto"):
        results = {}
        for a in ([arm] if arm in self.arms else self.arms):
            p = self.home_pt[a]
            cur = self.ee_pos(a)
            if cur[2] < p[2] - .02:  # lift first, then go over
                res = yield from self.move_to(cur[0], cur[1], p[2], a)
            res = yield from self.move_to(*p, a)
            results[a] = res.get("reached")
        return {"home": results}


class Panda(ArmRig):
    max_grip = .07
    key = "franka-panda"
    name = "Franka Emika Panda"
    kind = "arm"
    family = "arm"
    mount = "arm"
    caps = ("reach", "grasp", "top-grasp", "precise", "wrist-camera")
    assets = ["franka_emika_panda"]
    arms = ["arm"]

    def attach(self, s) -> None:
        import mujoco

        from ...assets import asset_root

        am, sf = self.mount_info()
        x, y, yaw = am.bases["arm"]
        r = mujoco.MjSpec.from_file(str(asset_root() / "franka_emika_panda/panda.xml"))
        for k in list(r.keys):
            _delete(r, k)
        for lt in list(r.lights):
            _delete(r, lt)
        r.body("hand").add_site(name="tcp", pos=[0, 0, .103], size=[.003] * 3)
        r.body("hand").add_camera(name="arm_wrist", pos=[.06, 0, .02], xyaxes=[0, -1, 0, -1, 0, 0], fovy=70)
        frame = s.worldbody.add_frame(pos=[x, y, sf.top + .015], quat=_yawq(math.radians(yaw)))
        s.attach(r, prefix=self.prefix, frame=frame)
        p = self.prefix
        self.cfg["arm"] = ArmCfg("arm", tuple(f"{p}joint{i}" for i in range(1, 8)), tuple(f"{p}actuator{i}" for i in range(1, 8)),
                                 (f"{p}left_finger", f"{p}right_finger"), f"{p}actuator8", f"{p}tcp",
                                 (0, 0, 0, -1.57079, 0, 1.57079, -.7853), (0, 1, 0, 0), 255., 0., reach=.78, reach_min=.25, z_max=.6)

    def finger_open_m(self, arm: str) -> float:
        return .04


class Aloha(ArmRig):
    max_grip = .05
    key = "aloha-2"
    name = "ALOHA 2 (two ViperX 300 s arms)"
    kind = "bimanual"
    family = "bimanual"
    mount = "bimanual"
    caps = ("reach", "grasp", "top-grasp", "bimanual", "precise", "wrist-camera")
    assets = ["aloha"]
    arms = ["left", "right"]

    def attach(self, s) -> None:
        import mujoco

        from ...assets import asset_root

        am, sf = self.mount_info()
        r = mujoco.MjSpec.from_file(str(asset_root() / "aloha/aloha.xml"))
        for k in list(r.keys):
            _delete(r, k)
        for lt in list(r.lights):
            _delete(r, lt)
        for cam in list(r.cameras):
            _delete(r, cam)
        for side in ("left", "right"):
            bx, by, yaw = am.bases[side]
            b = r.body(f"{side}/base_link")
            b.pos = [bx, by, sf.top + .02]
            b.quat = _yawq(math.radians(yaw))
            r.body(f"{side}/gripper_base").add_camera(name=f"{side}_wrist", pos=[-.03, 0, -.03], xyaxes=[0, 1, 0, 0, 0, 1], fovy=70)
        s.attach(r, prefix=self.prefix, frame=s.worldbody.add_frame())
        names = ("waist", "shoulder", "elbow", "forearm_roll", "wrist_angle", "wrist_rotate")
        p = self.prefix
        for side in ("left", "right"):
            self.cfg[side] = ArmCfg(side, tuple(f"{p}{side}/{j}" for j in names), tuple(f"{p}{side}/{j}" for j in names),
                                    (f"{p}{side}/left_finger_link", f"{p}{side}/right_finger_link"), f"{p}{side}/gripper",
                                    f"{p}{side}/gripper", (0, -.96, 1.16, 0, -.3, 0), (math.sqrt(.5), 0, math.sqrt(.5), 0),
                                    .037, .002, reach=.56, reach_min=.15, z_max=.45)

    def finger_open_m(self, arm: str) -> float:
        return .037


def _delete(spec, el) -> None:
    if hasattr(spec, "delete"):
        spec.delete(el)
    else:
        el.delete()


# ======================================================================================================================
# floor robots
# ======================================================================================================================

class FloorRobot(Driver):
    mount = "floor"

    def start_pose(self) -> tuple[float, float, float]:
        x, y, yaw = self.layout.mounts["floor"]["start"]
        return x, y, math.radians(yaw)


class MobileManip(FloorRobot):
    """PAL TIAGo or the Google Robot (mobile_manip_sim.robots): base.twist, arm.ee_delta, gripper."""

    kind = "mobile_manipulator"
    family = "mobile_manipulator"
    grasp_mode = "front"
    arms = ["arm"]
    robot_key = "tiago"

    def __init__(self, layout, params=None):
        super().__init__(layout, params)
        from ..mobile_manip_sim.robots import ROBOTS

        self.r = ROBOTS[self.robot_key]()
        self.r.prefix = self.prefix

    def groups(self) -> list[ActionGroup]:
        r = self.r
        if self.robot_key == "google":
            base = ActionGroup("base.twist", "base_twist", ["VX", "VY", "WZ"], [-1] * 3, [1] * 3, "x 0.5 m/s, x 0.5 m/s, x 60 deg/s",
                               "forward, leftward and yaw-rate command of the holonomic base (robot frame)", frame="body")
        else:
            base = ActionGroup("base.twist", "base_twist", ["V", "WZ"], [-1] * 2, [1] * 2, f"x {r.v_max:g} m/s, x 60 deg/s",
                               "forward speed and yaw rate of the differential drive", frame="body")
        return [base,
                ActionGroup("arm.ee_delta", "ee_delta_pos", ["DX", "DY", "DZ"], [-1] * 3, [1] * 3, "x 2 cm per step",
                            "moves the gripper target (world axes; IK; the gripper stays level and points along the heading)",
                            frame="world"),
                ActionGroup("gripper", "gripper", ["G"], [-1], [1], "rate", "> 0 closes, < 0 opens (1 = 10 % of the stroke per step), "
                            "0 holds; closing stops squeezing at a bounded force")]

    def skills(self) -> list[Skill]:
        xyz = [SkillArg("x", unit="m"), SkillArg("y", unit="m"), SkillArg("z", unit="m")]
        return [
            Skill("go_to", [SkillArg("x", unit="m"), SkillArg("y", unit="m"),
                            SkillArg("yaw_deg", "str", unit="deg", default="none", doc="final heading; none keeps the travel heading"),
                            SkillArg("speed", unit="m/s", default=self.r.v_max, doc="top speed")],
                  "drive to (X, Y) in a straight line, then turn to YAW_DEG; it does not plan around obstacles (give it waypoints) "
                  "and stops if the base is blocked", 900),
            Skill("turn", [SkillArg("yaw_deg", unit="deg")], "rotate in place to heading YAW_DEG (0 = +x, 90 = +y)", 200),
            Skill("look_at", xyz, "point the head camera at (X, Y, Z) (pan and tilt; the base does not move)", 40),
            Skill("move_to", xyz, "move the grasp point (between the fingertips) to (X, Y, Z) in a straight line with the IK arm "
                  "(the first call unfolds the arm); reports whether it got there", 400),
            Skill("grasp", [], "close the gripper until the fingers stop; reports what is held", 40),
            Skill("release", [], "open the gripper fully", 20),
            Skill("home", [], "fold the arm in for driving", 250),
        ]

    def cameras(self) -> list[str]:
        return ["chase", f"{self.prefix}head"]

    def inspect_camera(self) -> str:
        return f"{self.prefix}head"

    def attach(self, s) -> None:
        x, y, yaw = self.start_pose()
        self.r.attach(s, x, y, yaw)

    def bind(self, m, d) -> None:
        super().bind(m, d)
        r = self.r
        r.bind(m, d)
        self.finger_sides = {"arm": tuple(r.side_geoms)}
        self.body_geoms = set(r.base_geoms)

    def init_state(self) -> None:
        self.r.init_state()

    def post_forward(self) -> None:
        self.r.make_arm()
        self.posture = None
        self.stowed = True

    def control(self, a: np.ndarray) -> None:
        r = self.r
        if self.robot_key == "google":
            v, vy, wz = a[0], a[1], a[2]
            arm, g = a[3:6], a[6]
        else:
            v, wz, vy = a[0], a[1], 0.0
            arm, g = a[2:5], a[5]
        r.set_twist(float(v) * r.v_max, float(wz) * r.wz_max, float(vy) * r.v_max)
        if np.any(arm):
            self.stowed = False
            if self.posture:
                self.posture = None
                r.arm.release_joints()
            r.arm.nudge(np.asarray(arm, float) * .02)
        elif self.posture and r.arm.joints_done():
            self.posture = None
            r.arm.release_joints()
        r.grip(float(g))
        r.control_update()

    def substep(self, dt: float) -> None:
        self.r.substep(dt)

    def ee_pos(self, arm: str = "arm") -> np.ndarray:
        return self.r.ee_pos()

    def base_pose(self):
        return self.r.base_pose()

    def state(self) -> dict:
        r = self.r
        x, y, yaw = r.base_pose()
        pan, tilt = r.head_angles() if hasattr(r, "head_angles") else (0.0, 0.0)
        return {"base": {"x": round(x, 3), "y": round(y, 3), "yaw_deg": round(math.degrees(yaw), 1)},
                "arm": {"gripper_pos": r3(r.ee_pos()), "gripper_target": r3(r.arm.target_world()),
                        "opening": round(r.gripper_opening(), 2), "posture": "stowed" if self.stowed else "unfolded"},
                "head": {"pan_deg": round(math.degrees(pan), 1), "tilt_deg": round(math.degrees(tilt), 1)}}

    def public(self) -> dict:
        out = super().public()
        lo, hi = self.r.arm.box_lo, self.r.arm.box_hi
        out["workspace"] = (f"the grasp point reaches {lo[0]:g} to {hi[0]:g} m ahead of the base centre, {lo[1]:g} to {hi[1]:g} m to its left, "
                            f"and heights {lo[2]:g} to {hi[2]:g} m above the floor; the base footprint radius is {self.footprint():g} m")
        return out

    def footprint(self) -> float:
        return .33 if self.robot_key == "tiago" else .3

    # ---- skills ------------------------------------------------------------------------------------------------
    def _v(self, twist=(0.0, 0.0, 0.0), arm=(0.0, 0.0, 0.0), g=0.0) -> list[float]:
        v, wz, vy = twist
        base = [v, vy, wz] if self.robot_key == "google" else [v, wz]
        return self.vec(base_twist=base, arm_ee_delta=list(arm), gripper=[g])

    def turn(self, yaw_deg: float):
        r = self.r
        target = math.radians(yaw_deg)
        ok = 0
        for _ in range(190):
            e = wrap(target - r.base_pose()[2])
            if abs(e) < math.radians(1.5):
                ok += 1
                if ok >= 3:
                    break
            else:
                ok = 0
            wz = float(np.clip(2.0 * e / r.wz_max, -1, 1))
            if abs(wz) < .08:
                wz = .08 * np.sign(e)
            yield self._v(twist=(0, wz, 0))
        for _ in range(4):
            yield self._v()
        err = math.degrees(wrap(target - r.base_pose()[2]))
        return {"reached": abs(err) < 3, "heading_error_deg": round(err, 1), **self.state()["base"]}

    def go_to(self, x: float, y: float, yaw_deg="none", speed: float | None = None, tol: float = .04):
        r = self.r
        vmax = float(np.clip(speed if speed is not None else r.v_max, .05, r.v_max))
        if str(yaw_deg).strip().lower() in ("none", ""):
            yaw_goal = None
        else:
            try:
                yaw_goal = math.radians(float(yaw_deg))
            except ValueError:
                raise ValueError("yaw_deg must be a number of degrees or none") from None
        goal = np.array([x, y])
        blocked = False
        hist = []
        if self.robot_key == "google":
            for i in range(700):
                px, py, yaw = r.base_pose()
                e = goal - np.array([px, py])
                dist = float(np.linalg.norm(e))
                eyaw = 0.0 if yaw_goal is None else wrap(yaw_goal - yaw)
                if dist < tol and abs(eyaw) < math.radians(2):
                    break
                sp = min(vmax, 1.2 * dist + .01, math.sqrt(1.2 * r.a_max * dist))
                vw = e / max(dist, 1e-6) * sp
                c, s = math.cos(yaw), math.sin(yaw)
                vb = np.array([c * vw[0] + s * vw[1], -s * vw[0] + c * vw[1]]) / r.v_max
                hist.append(np.array([px, py]))
                if i > 30 and dist > tol and np.linalg.norm(hist[-1] - hist[-21]) < .01:
                    blocked = True
                    break
                yield self._v(twist=(vb[0], float(np.clip(2.0 * eyaw / r.wz_max, -1, 1)), vb[1]))
        else:
            px, py, _ = r.base_pose()
            if np.linalg.norm(goal - [px, py]) > tol:
                yield from self.turn(math.degrees(math.atan2(y - py, x - px)))
            for i in range(880):
                px, py, yaw = r.base_pose()
                e = goal - np.array([px, py])
                dist = float(np.linalg.norm(e))
                fwd = e @ np.array([math.cos(yaw), math.sin(yaw)])
                if dist < tol or (fwd < 0 and dist < 3 * tol):
                    break
                he = wrap(math.atan2(e[1], e[0]) - yaw)
                if abs(he) > math.radians(60):
                    yield from self.turn(math.degrees(math.atan2(e[1], e[0])))
                    continue
                sp = min(vmax, 1.2 * dist + .02, math.sqrt(1.2 * r.a_max * dist)) * max(0.0, math.cos(he))
                hist.append(np.array([px, py]))
                if i > 30 and np.linalg.norm(hist[-1] - hist[-21]) < .01:
                    blocked = True
                    break
                yield self._v(twist=(sp / r.v_max, float(np.clip(2.5 * he / r.wz_max, -1, 1)), 0))
        for _ in range(6):
            yield self._v()
        if yaw_goal is not None and self.robot_key != "google" and not blocked:
            yield from self.turn(math.degrees(yaw_goal))
        px, py, _ = r.base_pose()
        dist = float(np.linalg.norm(goal - [px, py]))
        out = {"reached": dist < max(tol * 1.5, .06) and not blocked, "distance_m": round(dist, 3), **self.state()["base"]}
        if blocked:
            out["blocked"] = "the base stopped moving (something is in the way)"
        return out

    def look_at(self, x: float, y: float, z: float):
        r = self.r
        if not hasattr(r, "set_head"):
            return {"error": "this robot's head camera is fixed"}
        cam = self.m.camera(self.inspect_camera()).id
        for _ in range(30):
            p = self.d.cam_xpos[cam]
            bx, by, byaw = r.base_pose()
            dx, dy, dz = x - p[0], y - p[1], z - p[2]
            pan = wrap(math.atan2(dy, dx) - byaw)
            tilt = -math.atan2(-dz, math.hypot(dx, dy))
            r.set_head(pan, tilt)
            yield self._v()
        pan, tilt = r.head_angles()
        return {"pan_deg": round(math.degrees(pan), 1), "tilt_deg": round(math.degrees(tilt), 1)}

    def _ready(self):
        r = self.r
        if not self.stowed and self.posture is None:
            return
        self.posture = "ready"
        r.arm.move_joints(r.READY)
        r.grip_target = r.FINGER_OPEN
        for _ in range(200):
            if r.arm.joints_done():
                break
            yield self._v()
        self.posture = None
        r.arm.release_joints()
        self.stowed = False

    def home(self):
        r = self.r
        self.posture = "stow"
        r.arm.move_joints(r.STOW)
        for _ in range(220):
            if r.arm.joints_done():
                break
            yield self._v()
        self.stowed = True
        return {"done": r.arm.joints_done()}

    def move_to(self, x: float, y: float, z: float):
        r = self.r
        target = np.array([x, y, z], float)
        yield from self._ready()
        best_t, stuck = 9.0, 0
        for _ in range(200):
            e_t = target - r.arm.target_world()
            n = float(np.linalg.norm(e_t))
            if n < 1e-4:
                break
            if n < best_t - 1e-3:
                best_t, stuck = n, 0
            else:
                stuck += 1
                if stuck > 15:
                    break
            yield self._v(arm=(e_t * min(1.0, .01 / n)) / .02)
        best, stall = 9.0, 0
        for _ in range(60):
            err = float(np.linalg.norm(target - r.ee_pos()))
            if err < .005:
                break
            if err < best - .0003:
                best, stall = err, 0
            else:
                stall += 1
                if stall > 12:
                    break
            yield self._v()
        err = float(np.linalg.norm(target - r.ee_pos()))
        out = {"reached": err < .015, "error_m": round(err, 3), "gripper_pos": r3(r.ee_pos())}
        if err >= .015:
            out["note"] = "the gripper did not get there (out of the arm's reach from here, or blocked); move the base closer"
        return out

    def grasp(self):
        r = self.r
        last, still = r.gripper_opening(), 0
        for _ in range(36):
            yield self._v(g=1)
            o = r.gripper_opening()
            still = still + 1 if abs(o - last) < .004 else 0
            last = o
            if still >= 4:
                break
        return {"opening": round(r.gripper_opening(), 2)}

    def release(self):
        for _ in range(14):
            yield self._v(g=-1)
        return {"opening": round(self.r.gripper_opening(), 2)}


class Tiago(MobileManip):
    max_grip = .08
    key = "pal-tiago"
    name = "PAL TIAGo"
    caps = ("reach", "grasp", "front-grasp", "locomote", "wheeled", "head-camera")
    assets = ["pal_tiago"]
    robot_key = "tiago"


class GoogleRobot(MobileManip):
    max_grip = .068
    key = "google-robot"
    name = "Google Robot (on a holonomic base)"
    caps = ("reach", "grasp", "front-grasp", "locomote", "wheeled", "holonomic", "head-camera")
    assets = ["google_robot"]
    robot_key = "google"


# ---------------------------------------------------------------------------------------------------------------------
# quadruped: Unitree Go2 with the model-based trot controller of the quadruped suite
# ---------------------------------------------------------------------------------------------------------------------

class Go2(FloorRobot):
    key = "unitree-go2"
    name = "Unitree Go2"
    kind = "quadruped"
    family = "quadruped"
    caps = ("locomote", "legged", "head-camera", "push")
    assets = ["unitree_go2"]
    HEAD = (.34, 0, .03)

    def options(self, s) -> None:
        import mujoco

        s.option.timestep = PHYS_DT
        s.option.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
        s.option.impratio = 100

    def groups(self) -> list[ActionGroup]:
        from ..quadruped_sim.gait import CFGS

        vx, vy, wz = CFGS["go2"].v_max
        wz_deg = round(math.degrees(wz))
        return [ActionGroup("base.twist", "base_twist", ["VX", "VY", "WZ"], [-vx, -vy, -wz_deg], [vx, vy, wz_deg], "m/s, m/s, deg/s",
                            "body-frame velocity command for the trot controller (x forward, y left, WZ counter-clockwise); "
                            "all zero = stop stepping and stand", frame="body")]

    def skills(self) -> list[Skill]:
        return [
            Skill("go_to", [SkillArg("x", unit="m"), SkillArg("y", unit="m"), SkillArg("yaw_deg", "str", unit="deg", default="none"),
                            SkillArg("speed", unit="m/s", default=.5, doc="top walking speed")],
                  "walk to (X, Y): turn towards it, trot along the straight line, stop (then turn to YAW_DEG); it does not plan "
                  "around obstacles (give it waypoints) and reports if blocked", 700),
            Skill("turn", [SkillArg("yaw_deg", unit="deg")], "turn in place to heading YAW_DEG (0 = +x east, 90 = +y north)", 200),
            Skill("look_at", [SkillArg("x", unit="m"), SkillArg("y", unit="m"), SkillArg("z", unit="m")],
                  "turn to face (X, Y) so the head camera looks at it, and hold still for a second", 240),
        ]

    def cameras(self) -> list[str]:
        return ["chase", f"{self.prefix}head"]

    def inspect_camera(self) -> str:
        return f"{self.prefix}head"

    def attach(self, s) -> None:
        import mujoco

        from ...assets import robot_dir
        from ..quadruped_sim.gait import CFGS

        cfg = CFGS["go2"]
        self.xml = str(robot_dir(cfg.folder) / cfg.xml)
        r = mujoco.MjSpec.from_file(self.xml)
        for k in list(r.keys):
            _delete(r, k)
        for lt in list(r.lights):
            _delete(r, lt)
        base = r.body(cfg.base)
        base.add_camera(name="head", pos=list(self.HEAD), xyaxes=[0, -1, 0, 0, 0, 1], fovy=70)
        pos, tgt = [-1.3, -.6, 1.0], [.4, 0, .1]
        base.add_camera(name="chase", pos=pos, xyaxes=[float(v) for v in lookat_xyaxes(pos, tgt).split()], fovy=60,
                        mode=mujoco.mjtCamLight.mjCAMLIGHT_TRACKCOM)
        s.attach(r, prefix=self.prefix, frame=s.worldbody.add_frame())
        for c in s.cameras:
            if c.name == self.prefix + "chase":
                c.name = "chase"

    def bind(self, m, d) -> None:
        super().bind(m, d)
        from ..quadruped_sim.gait import CFGS, Gait

        self.gait = Gait(CFGS["go2"], m, d, self.prefix, self.xml)
        self.body_geoms = {g for g in self.robot_geoms if m.geom_bodyid[g] == self.gait.base}

    def init_state(self) -> None:
        from ..quadruped_sim.gait import CFGS

        g = self.gait
        x, y, yaw = self.start_pose()
        self.d.qpos[g.q0:g.q0 + 7] = [x, y, g.h_nom + .01, math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]
        self.d.qpos[g.qadr.ravel()] = np.array(CFGS["go2"].home)

    def post_forward(self) -> None:
        import mujoco

        g = self.gait
        g.h_target = g.height = g.h_nom
        for _ in range(150):  # settle on the feet (part of the reset)
            g.step(PHYS_DT)
            mujoco.mj_step(self.m, self.d)

    def control(self, a: np.ndarray) -> None:
        vx, vy, wz = [float(v) for v in a[:3]]
        if self.fallen():
            vx = vy = wz = 0.0
        self.gait.set_command(vx, vy, math.radians(wz))

    def substep(self, dt: float) -> None:
        self.gait.step(dt)

    def fallen(self) -> bool:
        R = self.gait.R()
        return bool(R[2, 2] < math.cos(math.radians(50)))

    def base_pose(self):
        p = self.d.xpos[self.gait.base]
        return float(p[0]), float(p[1]), self.gait.yaw()

    def state(self) -> dict:
        x, y, yaw = self.base_pose()
        v = self.gait.vel_heading()
        return {"base": {"x": round(x, 3), "y": round(y, 3), "z": round(float(self.d.xpos[self.gait.base][2]), 3),
                         "yaw_deg": round(math.degrees(yaw), 1)},
                "velocity": {"forward_m_s": round(float(v[0]), 3), "left_m_s": round(float(v[1]), 3), "yaw_rate_deg_s": round(math.degrees(v[2]), 1)},
                "gait": "stepping" if self.gait.active else "standing", "fallen": self.fallen()}

    def public(self) -> dict:
        out = super().public()
        out["workspace"] = "a 0.70 x 0.31 m body that walks on the floor; the head camera looks forward from the front of the body"
        return out

    def footprint(self) -> float:
        return .38

    # ---- skills ------------------------------------------------------------------------------------------------
    def _v(self, vx=0.0, vy=0.0, wz_deg=0.0) -> list[float]:
        return self.vec(base_twist=[vx, vy, wz_deg])

    def _stop(self, n: int = 40):
        for _ in range(n):
            if not self.gait.active:
                break
            yield self._v()

    def turn(self, yaw_deg: float):
        tgt = math.radians(yaw_deg)
        for _ in range(160):
            e = wrap(tgt - self.gait.yaw())
            if abs(e) < math.radians(3) or self.fallen():
                break
            yield self._v(wz_deg=math.degrees(float(np.clip(2.5 * e, -2, 2))))
        yield from self._stop()
        return {"yaw_deg": round(math.degrees(self.gait.yaw()), 1)}

    def go_to(self, x: float, y: float, yaw_deg="none", speed: float = .5, tol: float = .12):
        vxm = float(np.clip(speed, .05, .6))
        hist = []
        why = ""
        while True:
            px, py, yaw = self.base_pose()
            dx, dy = x - px, y - py
            dist = math.hypot(dx, dy)
            if dist <= tol:
                break
            if self.fallen():
                why = "the robot fell"
                break
            hist.append((px, py))
            if len(hist) > 60 and math.dist(hist[-1], hist[-61]) < .08:
                why = "blocked: moved less than 8 cm in the last 3 s"
                break
            c, s = math.cos(yaw), math.sin(yaw)
            bx, by = c * dx + s * dy, -s * dx + c * dy
            e = math.atan2(by, bx)
            if dist < .5:
                sp = min(vxm, .9 * dist + .05)
                yield self._v(sp * bx / dist, sp * by / dist, 0)
            elif abs(e) > math.radians(35):
                yield self._v(0, 0, math.degrees(2.0 * e))
            else:
                sp = min(vxm, .8 * dist + .1)
                yield self._v(sp * math.cos(e), 0, math.degrees(2.0 * e))
        yield from self._stop()
        if str(yaw_deg).strip().lower() not in ("none", "") and not why:
            try:
                yv = float(yaw_deg)
            except ValueError:
                raise ValueError("yaw_deg must be a number of degrees or none") from None
            yield from self.turn(yv)
        px, py, yaw = self.base_pose()
        dist = math.hypot(x - px, y - py)
        out = {"reached": dist <= max(tol * 1.5, .15) and not self.fallen(), "distance_m": round(dist, 3),
               "pos": [round(px, 3), round(py, 3)], "yaw_deg": round(math.degrees(yaw), 1)}
        if why:
            out["stopped"] = why
        return out

    def look_at(self, x: float, y: float, z: float):
        px, py, _ = self.base_pose()
        res = yield from self.turn(math.degrees(math.atan2(y - py, x - px)))
        for _ in range(20):
            yield self._v()
        return res


# ---------------------------------------------------------------------------------------------------------------------
# drone: Bitcraze Crazyflie 2 with the cascaded controller of the crazyflie suite
# ---------------------------------------------------------------------------------------------------------------------

class Crazyflie(Driver):
    key = "crazyflie-2"
    name = "Bitcraze Crazyflie 2"
    kind = "drone"
    family = "aerial"
    mount = "airspace"
    caps = ("fly", "fpv-camera")
    assets = ["bitcraze_crazyflie_2"]

    V_XY, V_Z, YAW_RATE = 1.0, 1.0, math.radians(90)

    def options(self, s) -> None:
        import mujoco

        s.option.timestep = PHYS_DT
        s.option.integrator = mujoco.mjtIntegrator.mjINT_RK4
        s.option.density, s.option.viscosity = 1.225, 1.8e-5

    def groups(self) -> list[ActionGroup]:
        return [ActionGroup("velocity_setpoint", "velocity_setpoint", ["VX", "VY", "VZ", "YAW_RATE"], [-1] * 4, [1] * 4,
                            "x 1 m/s, x 1 m/s, x 1 m/s, x 90 deg/s",
                            "world-frame velocity setpoint and yaw rate; all zero = hold position (or stay landed)", frame="world")]

    def skills(self) -> list[Skill]:
        return [
            Skill("takeoff", [SkillArg("z", unit="m", default=1.0)], "climb straight up to height Z and hover", 200),
            Skill("fly_to", [SkillArg("x", unit="m"), SkillArg("y", unit="m"), SkillArg("z", unit="m")],
                  "fly in a straight line to (X, Y, Z) and hover; not obstacle-aware (give it waypoints)", 600),
            Skill("land", [], "descend where it is until it touches down; the motors stop on contact", 300),
            Skill("turn", [SkillArg("yaw_deg", unit="deg")], "turn in place to heading YAW_DEG", 200),
            Skill("look_at", [SkillArg("x", unit="m"), SkillArg("y", unit="m"), SkillArg("z", unit="m")],
                  "turn to face (X, Y) so the forward camera looks at it, then hover for a second", 240),
        ]

    def cameras(self) -> list[str]:
        return ["chase", f"{self.prefix}fpv"]

    def inspect_camera(self) -> str:
        return f"{self.prefix}fpv"

    def attach(self, s) -> None:
        import mujoco

        from ...assets import robot_dir
        from ..crazyflie import M_RP, M_Y

        cf = mujoco.MjSpec.from_file(str(robot_dir("bitcraze_crazyflie_2") / "cf2.xml"))
        for k in list(cf.keys):
            _delete(cf, k)
        body = cf.body("cf2")
        body.add_camera(name="chase", pos=[-1.1, 0, .55], xyaxes=[0, -1, 0, .45, 0, .9], mode=mujoco.mjtCamLight.mjCAMLIGHT_TRACKCOM)
        body.add_camera(name="fpv", pos=[.03, 0, .01], xyaxes=[0, -1, 0, .1, 0, 1], fovy=80)
        for act in cf.actuators:
            if act.name in ("x_moment", "y_moment"):
                act.ctrlrange = [-M_RP, M_RP]
            elif act.name == "z_moment":
                act.ctrlrange = [-M_Y, M_Y]
        s.attach(cf, prefix=self.prefix, frame=s.worldbody.add_frame())
        for c in s.cameras:
            if c.name == self.prefix + "chase":
                c.name = "chase"
        self.s_ref = s

    def bind(self, m, d) -> None:
        super().bind(m, d)
        self.bid = m.body(self.prefix + "cf2").id
        self.qa = m.jnt_qposadr[m.body_jntadr[self.bid]]
        self.dof = m.jnt_dofadr[m.body_jntadr[self.bid]]
        self.acts = [m.actuator(f"{self.prefix}{a}").id for a in ("body_thrust", "x_moment", "y_moment", "z_moment")]
        self.body_geoms = set(self.robot_geoms)

    def init_state(self) -> None:
        x, y = self.layout.mounts["airspace"]["pad"]
        self.d.qpos[self.qa:self.qa + 7] = [x, y, .03, 1, 0, 0, 0]

    def post_forward(self) -> None:
        import mujoco

        for _ in range(100):
            mujoco.mj_step(self.m, self.d)
        m = self.m
        self.mass = float(m.body_subtreemass[self.bid])
        self.J = m.body_inertia[self.bid].copy()
        self.v_cmd = np.zeros(3)
        self.yaw_rate = 0.0
        self.yaw_des = self.yaw()
        self.hold_p = self.pos()
        self.holding = True
        self.landed = True
        self.landing_geoms: set = set()

    def pos(self) -> np.ndarray:
        return self.d.xpos[self.bid].copy()

    def R(self) -> np.ndarray:
        return self.d.xmat[self.bid].reshape(3, 3).copy()

    def yaw(self) -> float:
        return yaw_of(self.R())

    def vel(self) -> np.ndarray:
        return self.d.qvel[self.dof:self.dof + 3].copy()

    def omega(self) -> np.ndarray:
        return self.d.qvel[self.dof + 3:self.dof + 6].copy()

    def base_pose(self):
        p = self.pos()
        return float(p[0]), float(p[1]), self.yaw()

    def touching_surface(self) -> bool:
        d = self.d
        for c in d.contact[:d.ncon]:
            a, b = int(c.geom1), int(c.geom2)
            if (a in self.robot_geoms) != (b in self.robot_geoms):
                return True
        return False

    def control(self, a: np.ndarray) -> None:
        cmd = np.asarray(a[:3], float)
        self.yaw_rate = float(a[3]) * self.YAW_RATE
        zero = not np.any(cmd) and a[3] == 0
        if zero and not self.holding:
            self.hold_p = self.pos() + .25 * self.vel()
        self.holding = zero
        self.v_cmd = cmd * np.array([self.V_XY, self.V_XY, self.V_Z])
        if cmd[2] > 0:
            self.landed = False

    def substep(self, dt: float) -> None:
        from ..crazyflie import A_XY, A_Z, GEAR, KP_HOLD, KV, M_RP, M_Y, T_MAX

        d = self.d
        ids = self.acts
        p, v = self.pos(), self.vel()
        vc = np.clip(KP_HOLD * (self.hold_p - p), -.5, .5) if self.holding else self.v_cmd
        if not self.landed and vc[2] <= 0 and self.touching_surface() and float(np.linalg.norm(v)) < .6:
            self.landed = True
        if self.landed:
            d.ctrl[ids] = 0.0
            self.yaw_des = self.yaw()
            self.hold_p = p
            return
        acc = KV * (vc - v)
        nxy = float(np.linalg.norm(acc[:2]))
        if nxy > A_XY:
            acc[:2] *= A_XY / nxy
        acc[2] = float(np.clip(acc[2], -A_Z, A_Z))
        f = self.mass * (acc + np.array([0, 0, 9.81]))
        self.yaw_des = wrap(self.yaw_des + self.yaw_rate * dt)
        zb = f / np.linalg.norm(f)
        xc = np.array([math.cos(self.yaw_des), math.sin(self.yaw_des), 0.0])
        yb = np.cross(zb, xc)
        yb /= np.linalg.norm(yb)
        xb = np.cross(yb, zb)
        Rd = np.column_stack([xb, yb, zb])
        R = self.R()
        E = .5 * (Rd.T @ R - R.T @ Rd)
        eR = np.array([E[2, 1], E[0, 2], E[1, 0]])
        w = self.omega()
        J = self.J
        wn = np.array([25.0, 25.0, 10.0])
        tau = -(J * wn ** 2) * eR - (2 * .9 * wn * J) * (w - R.T @ Rd @ np.array([0, 0, self.yaw_rate])) + np.cross(w, J * w)
        d.ctrl[ids[0]] = float(np.clip(f @ R[:, 2], 0, T_MAX))
        d.ctrl[ids[1]] = float(np.clip(-tau[0] / GEAR, -M_RP, M_RP))
        d.ctrl[ids[2]] = float(np.clip(-tau[1] / GEAR, -M_RP, M_RP))
        d.ctrl[ids[3]] = float(np.clip(-tau[2] / GEAR, -M_Y, M_Y))

    def state(self) -> dict:
        p, v = self.pos(), self.vel()
        return {"pos": r3(p), "velocity_m_s": r3(v), "yaw_deg": round(math.degrees(self.yaw()), 1),
                "motors": "off (landed)" if self.landed else "on"}

    def public(self) -> dict:
        out = super().public()
        out["workspace"] = "a 27 g, 9 cm quadrotor; it cannot carry or grasp anything; the forward camera looks ahead and slightly down"
        return out

    def footprint(self) -> float:
        return .12

    # ---- skills ------------------------------------------------------------------------------------------------
    def _goto_cmd(self, target) -> np.ndarray:
        from ..crazyflie import KP_GOTO, LEAD

        p, v = self.pos(), self.vel()
        c = KP_GOTO * (np.asarray(target) - (p + LEAD * v))
        sp = np.linalg.norm(c[:2])
        if sp > .8:
            c[:2] *= .8 / sp
        c[2] = np.clip(c[2], -.6, .6)
        return c

    def _fly(self, target, tol: float = .05, settle: int = 6, cap: int = 590):
        ok = 0
        for _ in range(cap):
            c = self._goto_cmd(target)
            far = np.linalg.norm(self.pos() - np.asarray(target)) > tol or np.linalg.norm(self.vel()) > .15
            ok = 0 if far else ok + 1
            if ok >= settle:
                break
            if np.allclose(c, 0):
                c = np.full(3, 1e-6)
            yield [*np.clip(c, -1, 1), 0.0]
        err = float(np.linalg.norm(self.pos() - np.asarray(target)))
        return {"reached": err < max(tol * 2, .08), "error_m": round(err, 3), "pos": r3(self.pos())}

    def takeoff(self, z: float = 1.0):
        p = self.pos()
        res = yield from self._fly([p[0], p[1], z], cap=190)
        return res

    def fly_to(self, x: float, y: float, z: float):
        if self.landed and z > self.pos()[2] + .02:
            yield [0, 0, .3, 0]
        res = yield from self._fly([x, y, z])
        return res

    def land(self):
        xy = self.pos()[:2].copy()
        for _ in range(290):
            if self.landed and float(np.linalg.norm(self.vel())) < .05:
                break
            p, v = self.pos(), self.vel()
            yield [*np.clip(1.5 * (xy - p[:2]) - .3 * v[:2], -1, 1), -.3, 0.0]
        return {"landed": bool(self.landed), "pos": r3(self.pos())}

    def turn(self, yaw_deg: float):
        if self.landed:
            return {"error": "the drone is landed (motors off); take off first"}
        for _ in range(190):
            e = wrap(math.radians(yaw_deg) - self.yaw())
            if abs(e) < math.radians(2) and abs(self.omega()[2]) < .2:
                break
            yield [1e-6, 0, 0, float(np.clip(e / self.YAW_RATE * 3, -1, 1))]
        return {"yaw_deg": round(math.degrees(self.yaw()), 1)}

    def look_at(self, x: float, y: float, z: float):
        p = self.pos()
        res = yield from self.turn(math.degrees(math.atan2(y - p[1], x - p[0])))
        for _ in range(20):
            yield self.hold()
        return res


DRIVERS = {c.key: c for c in (Panda, Aloha, Tiago, GoogleRobot, Go2, Crazyflie)}
