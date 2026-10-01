"""Mobile manipulators on a wheeled base: PAL TIAGo and Google Robot."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from ....embodied import ActionGroup, Skill, SkillArg, lookat_xyaxes, r3, wrap, yaw_of
from .base import FloorRobot


class MobileManip(FloorRobot):
    """PAL TIAGo or the Google Robot (mobile_manip_sim.robots): base.twist, arm.ee_delta, gripper."""

    kind = "mobile_manipulator"
    family = "mobile_manipulator"
    grasp_mode = "front"
    arms = ["arm"]
    robot_key = "tiago"

    def __init__(self, layout, params=None):
        super().__init__(layout, params)
        from ....mobile_manip.sim.robots import ROBOTS

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
