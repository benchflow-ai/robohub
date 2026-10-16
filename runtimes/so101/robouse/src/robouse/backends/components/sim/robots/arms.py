"""Surface-mounted arms: one Franka Panda, or ALOHA 2 (two arms)."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from ....embodied import ActionGroup, Skill, SkillArg, lookat_xyaxes, r3, wrap, yaw_of
from .base import GRASP_DOC, Driver, _delete, _yawq


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

        from ....menagerie.sim.ik import ArmIK

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

        from .....assets import asset_root

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

        from .....assets import asset_root

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
