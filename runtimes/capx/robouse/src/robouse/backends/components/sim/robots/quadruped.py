"""A legged base: Unitree Go2."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from ....embodied import ActionGroup, Skill, SkillArg, lookat_xyaxes, r3, wrap, yaw_of
from .base import PHYS_DT, FloorRobot, _delete


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
        from ....quadruped.sim.gait import CFGS

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

        from .....assets import robot_dir
        from ....quadruped.sim.gait import CFGS

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
        from ....quadruped.sim.gait import CFGS, Gait

        self.gait = Gait(CFGS["go2"], m, d, self.prefix, self.xml)
        self.body_geoms = {g for g in self.robot_geoms if m.geom_bodyid[g] == self.gait.base}

    def init_state(self) -> None:
        from ....quadruped.sim.gait import CFGS

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
