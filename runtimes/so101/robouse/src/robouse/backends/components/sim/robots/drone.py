"""A nano-quadrotor: Bitcraze Crazyflie 2."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from ....embodied import ActionGroup, Skill, SkillArg, lookat_xyaxes, r3, wrap, yaw_of
from .base import PHYS_DT, Driver, _delete


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

        from .....assets import robot_dir
        from ....crazyflie.backend import M_RP, M_Y

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
        from ....crazyflie.backend import A_XY, A_Z, GEAR, KP_HOLD, KV, M_RP, M_Y, T_MAX

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
        from ....crazyflie.backend import KP_GOTO, LEAD

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
