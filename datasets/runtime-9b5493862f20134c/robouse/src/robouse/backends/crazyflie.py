"""Crazyflie backend: one or three Bitcraze Crazyflie 2 nano-quadrotors (27 g, 9 cm; MuJoCo Menagerie
`bitcraze_crazyflie_2`, MIT) in an indoor flight arena.

Arena (metres, +x east, +y north, +z up, floor at z = 0): 8 x 6 m, 3 m high, walled on all sides. A full-height
partition at x = 1.5 splits it into a west hall and an east room; the only way through is a 0.36 x 0.36 m window.
The west hall has a home pad, three square racing gates (0.5 m openings) and three swarm pads; the east room has a
landing pad and a 0.9 m shelf with a 0.3 m landing platform on top. Hitting anything except the floor, a pad or the
platform top is a crash.

Model: the upstream cf2.xml (mass, inertia, meshes, collision hulls, thrust actuator 0-0.35 N) attached once per
drone. The upstream README says the actuator limits "are currently arbitrary"; the body-moment limits (gear 1e-5)
would give 0.4 rad/s^2 of roll authority, about 300x less than the real vehicle, so the moment ctrlrange is widened to
+-300 (roll, pitch: 3 mN m) and +-100 (yaw: 1 mN m), close to a real Crazyflie 2.1's motor-arm authority.

Controller (every 2 ms physics step, per drone), like the Crazyflie firmware's cascaded PID/Mellinger stack:
  velocity setpoint (or position hold when the setpoint is exactly zero) -> desired acceleration (gain 3 /s,
  horizontal limit 3 m/s^2) -> thrust vector -> desired attitude (thrust axis + heading) -> body moments from a
  geometric attitude controller (Lee et al. 2010) -> the thrust and moment actuators. Resting on a landing surface
  with a non-positive vertical setpoint, the motors are off (the firmware's landed state).

Action (one 50 ms step):
  single drone: velocity_setpoint [VX, VY, VZ, YAW_RATE], each in [-1, 1]: x 1 m/s, x 1 m/s, x 1 m/s, x 90 deg/s.
  swarm (3 drones): cf0.velocity_setpoint .. cf2.velocity_setpoint, [VX, VY, VZ] each (heading held at 0).
Skills: takeoff, goto, turn, land, hover (single); takeoff_all, goto, goto_all, land, land_all, hover (swarm).
"""
from __future__ import annotations

import math

import numpy as np

from .embodied import (ActionGroup, Budget, Embodiment, EmbodiedBackend, Oracle, Sensor, Skill, SkillArg, lookat_xyaxes, r3,
                       wrap, yaw_of)

PHYS_DT, CONTROL_DT = .002, .05
N_SUB = round(CONTROL_DT / PHYS_DT)
G = 9.81
V_XY, V_Z, YAW_RATE = 1.0, 1.0, math.radians(90)
KV, KP_HOLD, A_XY, A_Z = 3.0, 2.0, 3.0, 3.0
T_MAX = .35
M_RP, M_Y = 300.0, 100.0          # moment ctrlrange (x 1e-5 N m)
GEAR = 1e-5
LEAD, KP_GOTO = .45, 1.6          # goto skill: velocity = KP_GOTO * (target - (pos + LEAD * vel))

ARENA = dict(x=(-4.0, 4.0), y=(-3.0, 3.0), h=3.0)
PARTITION_X = 1.5
WINDOW = dict(c=(1.5, 1.8, 1.0), half=.18)
HOME = (-3.2, -2.3)
PADS = {"home": HOME, "A": (-1.5, -1.2), "B": (3.0, 1.8), "S0": (-2.6, -0.9), "S1": (-1.9, -2.1), "S2": (-1.2, -0.9)}
PAD_HALF = .15
SHELF = dict(c=(3.2, -2.0), half=(.35, .25), h=.9, pad_half=.15)
GATES = [dict(name="G1", c=(-3.0, 0.2, 0.8), normal=(0, 1)), dict(name="G2", c=(-1.6, 2.0, 1.4), normal=(1, 0)),
         dict(name="G3", c=(-0.4, 0.2, 1.0), normal=(0, -1))]
GATE_HALF, GATE_BAR = .25, .03
WAYPOINTS = [(-3.0, -1.0, 1.0), (-1.0, -2.0, 1.5), (0.5, 0.5, 0.6), (-2.0, 1.5, 2.0)]

TASKS: dict[str, dict] = {
    "crazyflie-waypoints": dict(n=1, kind="waypoints", land="A", max_steps=1400),
    "crazyflie-gates": dict(n=1, kind="gates", land="A", max_steps=1600),
    "crazyflie-window": dict(n=1, kind="land", land="B", max_steps=1400),
    "crazyflie-window-manual": dict(n=1, kind="land", land="B", max_steps=1400, skills=False),
    "crazyflie-shelf-landing": dict(n=1, kind="land", land="shelf", max_steps=1400),
    "crazyflie-swarm-formation": dict(n=3, kind="formation", center=(-1.9, -0.3, 1.2), side=.6, max_steps=1200),
    "crazyflie-swarm-swap": dict(n=3, kind="swap", max_steps=2000),
}
SWARM_START = ["S0", "S1", "S2"]
SWAP_GOAL = {0: "S2", 1: "S1", 2: "S0"}
MIN_SEP = .2


def _build(n: int):
    import mujoco

    from ..assets import robot_dir

    d = robot_dir("bitcraze_crazyflie_2")
    s = mujoco.MjSpec()
    s.option.timestep = PHYS_DT
    s.option.integrator = mujoco.mjtIntegrator.mjINT_RK4
    s.option.density, s.option.viscosity = 1.225, 1.8e-5
    s.visual.global_.offwidth, s.visual.global_.offheight = 1280, 960
    s.visual.headlight.ambient = [.4, .4, .4]
    s.visual.headlight.diffuse = [.5, .5, .5]
    s.visual.map.znear = .005
    s.stat.meansize = .05
    tex = s.add_texture(name="floor", type=mujoco.mjtTexture.mjTEXTURE_2D, builtin=mujoco.mjtBuiltin.mjBUILTIN_CHECKER,
                        rgb1=[.32, .34, .38], rgb2=[.28, .30, .34], width=256, height=256)
    s.add_material(name="floor", textures=["", "floor"], texrepeat=[16, 12], texuniform=False)
    s.add_texture(name="sky", type=mujoco.mjtTexture.mjTEXTURE_SKYBOX, builtin=mujoco.mjtBuiltin.mjBUILTIN_GRADIENT,
                  rgb1=[.8, .82, .85], rgb2=[.95, .95, .96], width=64, height=64)
    del tex
    w = s.worldbody
    BOX, PLANE, CYL = mujoco.mjtGeom.mjGEOM_BOX, mujoco.mjtGeom.mjGEOM_PLANE, mujoco.mjtGeom.mjGEOM_CYLINDER
    w.add_light(name="top", pos=[0, 0, 6], dir=[0, 0, -1], directional=True, diffuse=[.6, .6, .6], castshadow=False)
    w.add_geom(name="floor", type=PLANE, size=[4, 3, .1], material="floor")
    x0, x1 = ARENA["x"]
    y0, y1 = ARENA["y"]
    H = ARENA["h"]
    net = [.55, .6, .65, .25]
    w.add_geom(name="wall_w", type=BOX, pos=[x0 - .05, 0, H / 2], size=[.05, (y1 - y0) / 2, H / 2], rgba=net)
    w.add_geom(name="wall_e", type=BOX, pos=[x1 + .05, 0, H / 2], size=[.05, (y1 - y0) / 2, H / 2], rgba=net)
    w.add_geom(name="wall_s", type=BOX, pos=[0, y0 - .05, H / 2], size=[(x1 - x0) / 2, .05, H / 2], rgba=net)
    w.add_geom(name="wall_n", type=BOX, pos=[0, y1 + .05, H / 2], size=[(x1 - x0) / 2, .05, H / 2], rgba=net)
    w.add_geom(name="ceiling", type=BOX, pos=[0, 0, H + .05], size=[(x1 - x0) / 2, (y1 - y0) / 2, .05], rgba=[0, 0, 0, 0])
    # partition with a window
    px = PARTITION_X
    wx, wy, wz = WINDOW["c"]
    a = WINDOW["half"]
    part = [.78, .74, .68, 1]
    w.add_geom(name="partition_s", type=BOX, pos=[px, (y0 + wy - a) / 2, H / 2], size=[.05, (wy - a - y0) / 2, H / 2], rgba=part)
    w.add_geom(name="partition_n", type=BOX, pos=[px, (y1 + wy + a) / 2, H / 2], size=[.05, (y1 - wy - a) / 2, H / 2], rgba=part)
    w.add_geom(name="partition_lo", type=BOX, pos=[px, wy, (wz - a) / 2], size=[.05, a, (wz - a) / 2], rgba=part)
    w.add_geom(name="partition_hi", type=BOX, pos=[px, wy, (wz + a + H) / 2], size=[.05, a, (H - wz - a) / 2], rgba=part)
    w.add_geom(name="window_frame", type=BOX, pos=[px - .06, wy, wz - a - .015], size=[.015, a + .03, .015], rgba=[.9, .5, .1, 1],
               contype=0, conaffinity=0)
    # pads
    pad_rgba = {"home": [.2, .5, .9, 1], "A": [.15, .7, .75, 1], "B": [.9, .75, .1, 1], "S0": [.85, .25, .25, 1], "S1": [.25, .7, .3, 1],
                "S2": [.25, .35, .9, 1]}
    for k, (x, y) in PADS.items():
        w.add_geom(name=f"pad_{k}", type=BOX, pos=[x, y, .005], size=[PAD_HALF, PAD_HALF, .005], rgba=pad_rgba[k])
    # shelf with a landing platform
    sx, sy = SHELF["c"]
    hx, hy = SHELF["half"]
    w.add_geom(name="shelf", type=BOX, pos=[sx, sy, SHELF["h"] / 2 - .005], size=[hx, hy, SHELF["h"] / 2 - .005], rgba=[.45, .32, .2, 1])
    w.add_geom(name="pad_shelf", type=BOX, pos=[sx, sy, SHELF["h"]], size=[SHELF["pad_half"], SHELF["pad_half"], .005], rgba=[.95, .95, .95, 1])
    # gates: square frames of four bars
    for g in GATES:
        cx, cy, cz = g["c"]
        nx, ny = g["normal"]
        tx, ty = -ny, nx  # in-plane horizontal direction
        L = GATE_HALF + GATE_BAR
        col = [.95, .35, .1, 1]
        for i, (u, v, hu, hv) in enumerate(((0, L, L, GATE_BAR), (0, -L, L, GATE_BAR), (L, 0, GATE_BAR, L), (-L, 0, GATE_BAR, L))):
            size = [abs(tx) * hu + abs(nx) * .02, abs(ty) * hu + abs(ny) * .02, hv]
            w.add_geom(name=f"gate_{g['name']}_bar{i}", type=BOX, pos=[cx + tx * u, cy + ty * u, cz + v], size=size, rgba=col)
        for sgn in (-1, 1):  # posts from the floor to the lower bar
            px_, py_ = cx + tx * sgn * L, cy + ty * sgn * L
            h = cz - L
            w.add_geom(name=f"gate_{g['name']}_post{sgn}", type=CYL, pos=[px_, py_, h / 2], size=[.015, h / 2, 0], rgba=[.3, .3, .3, 1])
    w.add_camera(name="overview", pos=[-5.2, -4.6, 4.2], xyaxes=[float(v) for v in lookat_xyaxes([-5.2, -4.6, 4.2], [-.6, .2, .6]).split()],
                 fovy=58)
    w.add_camera(name="swarm", pos=[-1.9, -3.9, 2.3], xyaxes=[float(v) for v in lookat_xyaxes([-1.9, -3.9, 2.3], [-1.9, -1.0, .7]).split()],
                 fovy=55)
    w.add_camera(name="east_room", pos=[4.0, -3.0, 2.6], xyaxes=[float(v) for v in lookat_xyaxes([4.0, -3.0, 2.6], [2.6, 0.0, .6]).split()],
                 fovy=60)
    # drones
    for i in range(n):
        cf = mujoco.MjSpec.from_file(str(d / "cf2.xml"))
        for k in list(cf.keys):
            (cf.delete(k) if hasattr(cf, "delete") else k.delete())
        body = cf.body("cf2")
        body.add_camera(name="chase", pos=[-1.1, 0, .55], xyaxes=[0, -1, 0, .45, 0, .9], mode=mujoco.mjtCamLight.mjCAMLIGHT_TRACKCOM)
        body.add_camera(name="fpv", pos=[.03, 0, .01], xyaxes=[0, -1, 0, .17, 0, .98], fovy=80)
        for act in cf.actuators:
            if act.name in ("x_moment", "y_moment"):
                act.ctrlrange = [-M_RP, M_RP]
            elif act.name == "z_moment":
                act.ctrlrange = [-M_Y, M_Y]
        frame = w.add_frame(pos=[0, 0, 0])
        s.attach(cf, prefix=f"cf{i}/", frame=frame)
    return s.compile()


class CrazyflieBackend(EmbodiedBackend):
    name = "crazyflie"

    def __init__(self, spec: dict):
        super().__init__(spec)
        env = spec.get("env") or spec.get("id")
        if env not in TASKS:
            raise KeyError(f"unknown crazyflie task {env!r}")
        self.task_id, self.task = env, TASKS[env]
        self.n = self.task["n"]
        self.camera = spec.get("camera", "cf0/chase" if self.n == 1 else "swarm")
        self.decl = self._declare()

    def _declare(self) -> Embodiment:
        n = self.n
        kind = self.task["kind"]
        sensors = [Sensor("drones", "proprio", ["drones"], "per drone: pos, vel, yaw_deg, roll_deg, pitch_deg, landed_on, motors_on",
                          units="m, m/s, deg", frame="world"),
                   Sensor("arena", "world", ["arena"], "pads, gates, window, shelf platform, walls (public positions)", units="m", frame="world"),
                   *([Sensor("progress", "events", ["progress"], "waypoints reached / gates passed in order so far")]
                     if kind in ("waypoints", "gates") else []),
                   Sensor("safety_events", "events", ["safety_events"], "crashes and near misses so far (any one fails the task)"),
                   *[Sensor(f"camera:{c}", "camera", mount="body" if c.startswith("cf") else "world") for c in self._cameras()]]
        if n == 1:
            groups = [ActionGroup("body.velocity_setpoint", "velocity_setpoint", ["VX", "VY", "VZ", "YAW_RATE"], [-1] * 4, [1] * 4,
                                  "x 1 m/s (world x, y, z); x 90 deg/s",
                                  "zero = hold position; touching the floor, a pad or the platform while not climbing switches the motors off until VZ > 0",
                                  frame="world")]
            ID = []
        else:
            groups = [ActionGroup(f"cf{i}.velocity_setpoint", "velocity_setpoint", [f"VX{i}", f"VY{i}", f"VZ{i}"], [-1] * 3, [1] * 3,
                                  "x 1 m/s (world)", "zero = hold position; touching the floor or a pad while not climbing switches the motors off until VZ > 0",
                                  frame="world") for i in range(n)]
            ID = [SkillArg("id", "int", doc="drone index 0-2")]
        tol = SkillArg("tol", "float", "m", .05, doc="arrival tolerance")
        xyz = [SkillArg("x", unit="m"), SkillArg("y", unit="m"), SkillArg("z", unit="m")]
        if n == 1:
            skills = [Skill("takeoff", [SkillArg("z", unit="m", default=1.0)], "climb straight up to height Z", 200),
                      Skill("goto", xyz + [tol], "fly in a straight line to (X, Y, Z) and stop there; it does not avoid obstacles", 300),
                      Skill("turn", [SkillArg("yaw_deg", unit="deg")], "rotate in place to heading YAW_DEG (0 = +x, 90 = +y)", 80),
                      Skill("land", [], "descend straight down onto whatever is below and switch the motors off", 200),
                      Skill("hover", [SkillArg("seconds", unit="s", default=1.0)], "hold position", 100)]
        else:
            skills = [Skill("takeoff_all", [SkillArg("z", unit="m", default=1.0)], "all drones climb straight up to height Z", 200),
                      Skill("goto", ID + xyz + [tol], "fly drone ID in a straight line to (X, Y, Z) while the others hold position", 300),
                      Skill("goto_all", [SkillArg(f"{c}{i}", unit="m") for i in range(n) for c in "xyz"] + [tol],
                            "fly all drones at once, each in a straight line to its point (X0 Y0 Z0 X1 Y1 Z1 X2 Y2 Z2); no collision avoidance", 300),
                      Skill("land", ID, "drone ID descends straight down and switches its motors off", 200),
                      Skill("land_all", [], "all drones descend straight down and switch their motors off", 200),
                      Skill("hover", [SkillArg("seconds", unit="s", default=1.0)], "all drones hold position", 100)]
        return Embodiment(robot="Bitcraze Crazyflie 2" + (" x3" if n > 1 else ""), family="aerial", assets=["bitcraze_crazyflie_2"],
                          sensors=sensors, action_groups=groups, skills=skills, budget=Budget(self.task["max_steps"], CONTROL_DT),
                          cameras=self._cameras())

    def _cameras(self) -> list[str]:
        return (["cf0/chase", "cf0/fpv"] if self.n == 1 else ["swarm", "cf0/chase", "cf1/chase", "cf2/chase"]) + ["overview", "east_room"]

    # ---- episode -----------------------------------------------------------------------------------------------
    def reset(self, seed: int) -> None:
        import mujoco

        self._reset_renderers()
        self.model = m = _build(self.n)
        self.data = d = mujoco.MjData(m)
        starts = [PADS["home"]] if self.n == 1 else [PADS[k] for k in SWARM_START]
        self.bids, self.qadr, self.acts = [], [], []
        for i, (x, y) in enumerate(starts):
            bid = m.body(f"cf{i}/cf2").id
            jid = m.body_jntadr[bid]
            adr = m.jnt_qposadr[jid]
            d.qpos[adr:adr + 7] = [x, y, .02, 1, 0, 0, 0]
            self.bids.append(bid)
            self.qadr.append(adr)
            self.acts.append([m.actuator(f"cf{i}/{a}").id for a in ("body_thrust", "x_moment", "y_moment", "z_moment")])
        mujoco.mj_forward(m, d)
        # rest the drones on the floor: let them settle with motors off
        for _ in range(100):
            mujoco.mj_step(m, d)
        self.mass = float(m.body_subtreemass[self.bids[0]])
        self.J = m.body_inertia[self.bids[0]].copy()  # principal inertia (the upstream inertial frame is the body frame)
        self.v_cmd = np.zeros((self.n, 3))
        self.yaw_rate = np.zeros(self.n)
        self.yaw_des = np.array([self._yaw(i) for i in range(self.n)])
        self.hold = np.array([self._pos(i) for i in range(self.n)])
        self.holding = np.ones(self.n, dtype=bool)
        self.landed = np.ones(self.n, dtype=bool)
        gname = lambda g: mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or ""
        self.gname = [gname(g) for g in range(m.ngeom)]
        self.drone_of_geom = {g: i for i in range(self.n) for g in range(m.ngeom) if m.geom_bodyid[g] == self.bids[i]}
        self.surface = {g for g in range(m.ngeom) if self.gname[g] == "floor" or self.gname[g].startswith("pad_")}
        self.events, self._event_keys = [], set()
        self.next_wp, self.gates_passed = 0, 0
        self.gate_order_error = False
        self.prev = np.array([self._pos(i) for i in range(self.n)])
        self.min_sep = 9.0

    # ---- state helpers -----------------------------------------------------------------------------------------
    def _pos(self, i) -> np.ndarray:
        return self.data.xpos[self.bids[i]].copy()

    def _R(self, i) -> np.ndarray:
        return self.data.xmat[self.bids[i]].reshape(3, 3).copy()

    def _yaw(self, i) -> float:
        return yaw_of(self._R(i))

    def _vel(self, i) -> np.ndarray:
        k = self._dof(i)
        return self.data.qvel[k:k + 3].copy()  # free joint linear velocity, world frame

    def _dof(self, i) -> int:
        return int(self.model.jnt_dofadr[self.model.body_jntadr[self.bids[i]]])

    def _omega(self, i) -> np.ndarray:
        k = self._dof(i)
        return self.data.qvel[k + 3:k + 6].copy()  # free joint angular velocity is in the body frame

    def _on_surface(self, i) -> str | None:
        """The landing surface drone i touches; a pad or the platform wins over the floor."""
        d = self.data
        hit = None
        for c in d.contact[:d.ncon]:
            for a, b in ((c.geom1, c.geom2), (c.geom2, c.geom1)):
                if self.drone_of_geom.get(a) == i and b in self.surface:
                    if self.gname[b].startswith("pad_"):
                        return self.gname[b]
                    hit = self.gname[b]
        return hit

    # ---- control -----------------------------------------------------------------------------------------------
    def control_step(self, a: np.ndarray) -> None:
        import mujoco

        if self.n == 1:
            cmd = a[:3].reshape(1, 3)
            self.yaw_rate[0] = float(a[3]) * YAW_RATE
        else:
            cmd = a.reshape(self.n, 3)
        for i in range(self.n):
            zero = not np.any(cmd[i]) and (self.n > 1 or a[3] == 0)
            if zero and not self.holding[i]:
                self.hold[i] = self._pos(i) + .25 * self._vel(i)
            self.holding[i] = zero
            self.v_cmd[i] = cmd[i] * np.array([V_XY, V_XY, V_Z])
            if cmd[i][2] > 0:  # a climb command arms the motors
                self.landed[i] = False
        for _ in range(N_SUB):
            for i in range(self.n):
                self._inner(i)
            mujoco.mj_step(self.model, self.data)
            self._monitor()

    def _inner(self, i: int) -> None:
        d = self.data
        ids = self.acts[i]
        p, v = self._pos(i), self._vel(i)
        if self.holding[i]:
            vc = np.clip(KP_HOLD * (self.hold[i] - p), -.5, .5)
        else:
            vc = self.v_cmd[i]
        if not self.landed[i] and vc[2] <= 0 and self._on_surface(i) is not None:
            self.landed[i] = True  # touched down while not climbing: the firmware's landed state
        if self.landed[i]:
            d.ctrl[ids] = 0.0  # motors off until a climb command
            self.yaw_des[i] = self._yaw(i)
            self.hold[i] = p
            return
        acc = KV * (vc - v)
        nxy = float(np.linalg.norm(acc[:2]))
        if nxy > A_XY:
            acc[:2] *= A_XY / nxy
        acc[2] = float(np.clip(acc[2], -A_Z, A_Z))
        f = self.mass * (acc + np.array([0, 0, G]))
        self.yaw_des[i] = wrap(self.yaw_des[i] + self.yaw_rate[i] * PHYS_DT)
        zb = f / np.linalg.norm(f)
        xc = np.array([math.cos(self.yaw_des[i]), math.sin(self.yaw_des[i]), 0.0])
        yb = np.cross(zb, xc)
        yb /= np.linalg.norm(yb)
        xb = np.cross(yb, zb)
        Rd = np.column_stack([xb, yb, zb])
        R = self._R(i)
        E = .5 * (Rd.T @ R - R.T @ Rd)
        eR = np.array([E[2, 1], E[0, 2], E[1, 0]])
        w = self._omega(i)
        J = self.J
        wn = np.array([25.0, 25.0, 10.0])
        KR = J * wn ** 2
        KW = 2 * .9 * wn * J
        tau = -KR * eR - KW * (w - R.T @ Rd @ np.array([0, 0, self.yaw_rate[i]])) + np.cross(w, J * w)
        T = float(np.clip(f @ R[:, 2], 0, T_MAX))
        d.ctrl[ids[0]] = T
        d.ctrl[ids[1]] = float(np.clip(-tau[0] / GEAR, -M_RP, M_RP))
        d.ctrl[ids[2]] = float(np.clip(-tau[1] / GEAR, -M_RP, M_RP))
        d.ctrl[ids[3]] = float(np.clip(-tau[2] / GEAR, -M_Y, M_Y))

    # ---- monitor -----------------------------------------------------------------------------------------------
    def _monitor(self) -> None:
        d = self.data
        for c in d.contact[:d.ncon]:
            for a, b in ((c.geom1, c.geom2), (c.geom2, c.geom1)):
                i = self.drone_of_geom.get(a)
                if i is None or b in self.surface:
                    continue
                j = self.drone_of_geom.get(b)
                what = f"drone cf{j}" if j is not None else self.gname[b].replace("_", " ")
                self.event(f"crash:cf{i}:{what}", "crash", f"cf{i} hit {what}")
        P = np.array([self._pos(i) for i in range(self.n)])
        for i in range(self.n):
            p = P[i]
            if self.n == 1:
                self._progress(self.prev[i], p)
            for j in range(i + 1, self.n):
                sep = float(np.linalg.norm(P[i] - P[j]))
                self.min_sep = min(self.min_sep, sep)
                if sep < MIN_SEP:
                    self.event(f"near:{i}{j}", "near_miss", f"cf{i} and cf{j} came within {MIN_SEP} m of each other")
        self.prev = P

    def _progress(self, p0, p1) -> None:
        k = self.task["kind"]
        if k == "waypoints" and self.next_wp < len(WAYPOINTS):
            if np.linalg.norm(p1 - np.array(WAYPOINTS[self.next_wp])) < .1:
                self.next_wp += 1
        if k == "gates":
            for gi, g in enumerate(GATES):
                c = np.array(g["c"])
                n = np.array([*g["normal"], 0.0])
                s0, s1 = float((p0 - c) @ n), float((p1 - c) @ n)
                if s0 < 0 <= s1:
                    t = s0 / (s0 - s1)
                    x = p0 + t * (p1 - p0) - c
                    tang = np.array([-n[1], n[0], 0.0])
                    if abs(x @ tang) <= GATE_HALF and abs(x[2]) <= GATE_HALF:
                        if gi == self.gates_passed:
                            self.gates_passed += 1
                        elif gi > self.gates_passed:
                            self.gate_order_error = True

    # ---- observation -------------------------------------------------------------------------------------------
    def _landed_on(self, i) -> str | None:
        """Resting: touching a landing surface, slower than 0.1 m/s and tilted less than 20 degrees."""
        if float(np.linalg.norm(self._vel(i))) > .1 or self._R(i)[2, 2] < math.cos(math.radians(20)):
            return None
        s = self._on_surface(i)
        if s is None:
            return None
        return s.removeprefix("pad_") if s.startswith("pad_") else s

    def observe(self) -> dict:
        drones = []
        for i in range(self.n):
            R = self._R(i)
            drones.append({"id": i, "pos": r3(self._pos(i)), "vel": r3(self._vel(i)), "yaw_deg": round(math.degrees(self._yaw(i)), 1),
                           "roll_deg": round(math.degrees(math.atan2(R[2, 1], R[2, 2])), 1),
                           "pitch_deg": round(math.degrees(-math.asin(float(np.clip(R[2, 0], -1, 1)))), 1),
                           "landed_on": self._landed_on(i), "motors_on": not bool(self.landed[i])})
        st: dict = {"drones": drones, "time_s": round(float(self.data.time), 2)}
        st["arena"] = {
            "size": {"x": list(ARENA["x"]), "y": list(ARENA["y"]), "height": ARENA["h"]},
            "partition": {"x": PARTITION_X, "thickness": .1, "window_center": list(WINDOW["c"]), "window_size": 2 * WINDOW["half"],
                          "note": "full-height wall; the window is the only opening between the west hall and the east room"},
            "pads": {k: [x, y, .01] for k, (x, y) in PADS.items()}, "pad_size": 2 * PAD_HALF,
            "pad_note": "pad positions are the centre of each pad's top surface",
            "shelf": {"center_xy": list(SHELF["c"]), "top_z": round(SHELF["h"] - .01, 3), "top_size": [2 * SHELF["half"][0], 2 * SHELF["half"][1]],
                      "platform_top_center": [*SHELF["c"], SHELF["h"] + .005], "platform_size": 2 * SHELF["pad_half"],
                      "note": "only the platform is a landing surface; touching the rest of the shelf is a crash"},
            "gates": [{"name": g["name"], "center": list(g["c"]), "pass_direction": [*g["normal"], 0], "opening": 2 * GATE_HALF,
                       "frame_bar": 2 * GATE_BAR} for g in GATES],
        }
        k = self.task["kind"]
        if k == "waypoints":
            st["progress"] = {"waypoints_reached": self.next_wp, "waypoints": [list(w) for w in WAYPOINTS]}
        elif k == "gates":
            st["progress"] = {"gates_passed_in_order": self.gates_passed}
        st["safety_events"] = [e["detail"] for e in self.events]
        return st

    # ---- scoring -----------------------------------------------------------------------------------------------
    def success(self) -> bool:
        if self.events:
            return False
        t, k = self.task, self.task["kind"]
        if k == "waypoints":
            return self.next_wp == len(WAYPOINTS) and self._landed_on(0) == t["land"]
        if k == "gates":
            return self.gates_passed == len(GATES) and not self.gate_order_error and self._landed_on(0) == t["land"]
        if k == "land":
            return self._landed_on(0) == t["land"]
        if k == "formation":
            P = np.array([self._pos(i) for i in range(3)])
            if any(float(np.linalg.norm(self._vel(i))) > .1 for i in range(3)):
                return False
            if np.any(np.abs(P[:, 2] - t["center"][2]) > .05):
                return False
            if np.linalg.norm(P.mean(0)[:2] - np.array(t["center"][:2])) > .1:
                return False
            sides = [np.linalg.norm(P[i] - P[j]) for i, j in ((0, 1), (1, 2), (0, 2))]
            return all(abs(sd - t["side"]) <= .06 for sd in sides)
        if k == "swap":
            return all(self._landed_on(i) == SWAP_GOAL[i] for i in range(3))
        return False

    # ---- skills ------------------------------------------------------------------------------------------------
    def _vec(self, per: list[np.ndarray], yaw_rate: float = 0.0) -> list[float]:
        if self.n == 1:
            return [*np.clip(per[0], -1, 1), float(np.clip(yaw_rate, -1, 1))]
        return [float(x) for v in per for x in np.clip(v, -1, 1)]

    def _goto_cmd(self, i, target) -> np.ndarray:
        p, v = self._pos(i), self._vel(i)
        c = KP_GOTO * (np.asarray(target) - (p + LEAD * v))
        sp = np.linalg.norm(c[:2])
        if sp > .8:
            c[:2] *= .8 / sp
        c[2] = np.clip(c[2], -.6, .6)
        return c

    def _fly(self, targets: dict, tol: float, settle: int = 6):
        """Yield actions until every drone in `targets` is within tol of its point and slow, for `settle` steps."""
        ok = 0
        while True:
            per = [np.zeros(3) for _ in range(self.n)]
            done = True
            for i, tgt in targets.items():
                per[i] = self._goto_cmd(i, tgt)
                far = np.linalg.norm(self._pos(i) - np.asarray(tgt)) > tol or np.linalg.norm(self._vel(i)) > .15
                done = done and not far
                if np.allclose(per[i], 0):
                    per[i] = np.full(3, 1e-6)  # not exactly zero: keep flying rather than latching position hold
            ok = ok + 1 if done else 0
            if ok >= settle:
                break
            yield self._vec(per)
        return {"reached": True, "positions": {f"cf{i}": r3(self._pos(i)) for i in targets}}

    def skill_takeoff(self, z: float):
        p = self._pos(0)
        res = yield from self._fly({0: [p[0], p[1], z]}, .05)
        return res

    def skill_takeoff_all(self, z: float):
        res = yield from self._fly({i: [*self._pos(i)[:2], z] for i in range(self.n)}, .05)
        return res

    def skill_goto(self, x: float, y: float, z: float, tol: float, id: int = 0):
        if not 0 <= id < self.n:
            raise ValueError(f"id must be 0-{self.n - 1}")
        res = yield from self._fly({id: [x, y, z]}, max(tol, .02))
        return res

    def skill_goto_all(self, tol: float, **kw):
        targets = {i: [kw[f"x{i}"], kw[f"y{i}"], kw[f"z{i}"]] for i in range(self.n)}
        res = yield from self._fly(targets, max(tol, .02))
        return res

    def skill_turn(self, yaw_deg: float):
        if self.landed[0]:
            return {"error": "the drone is landed (motors off); take off first"}
        for _ in range(200):
            e = wrap(math.radians(yaw_deg) - self._yaw(0))
            if abs(e) < math.radians(2) and abs(self._omega(0)[2]) < .2:
                break
            yield [1e-6, 0, 0, float(np.clip(e / YAW_RATE * 3, -1, 1))]
        return {"yaw_deg": round(math.degrees(self._yaw(0)), 1)}

    def _land(self, ids):
        steps = 0
        while True:
            per = [np.zeros(3) for _ in range(self.n)]
            pending = False
            for i in ids:
                if self.landed[i] and float(np.linalg.norm(self._vel(i))) < .05:
                    continue
                pending = True
                p, v = self._pos(i), self._vel(i)
                hold_xy = self.hold[i][:2] if steps == 0 else self._land_xy[i]
                per[i] = np.array([*(1.5 * (hold_xy - p[:2]) - .3 * v[:2]), -.3])
            if steps == 0:
                self._land_xy = {i: self._pos(i)[:2].copy() for i in ids}
                per = [np.zeros(3) if i not in ids else np.array([0, 0, -.3]) for i in range(self.n)]
            if not pending:
                break
            steps += 1
            yield self._vec(per)
        return {"landed_on": {f"cf{i}": self._landed_on(i) for i in ids}}

    def skill_land(self, id: int = 0):
        if not 0 <= id < self.n:
            raise ValueError(f"id must be 0-{self.n - 1}")
        res = yield from self._land([id])
        return res

    def skill_land_all(self):
        res = yield from self._land(list(range(self.n)))
        return res

    def skill_hover(self, seconds: float):
        for _ in range(max(1, round(seconds / CONTROL_DT))):
            yield self.hold_action()
        return {}


# ------------------------------------------------------------------------------------------------------------------
# reference solutions (socket only)
# ------------------------------------------------------------------------------------------------------------------

def oracle_main(env: str) -> None:
    t = TASKS[env]

    def solve(o: Oracle) -> None:
        k = t["kind"]
        if k == "waypoints":
            o.skill("takeoff", 1.0)
            for w in WAYPOINTS:
                o.skill("goto", *w, "tol=0.04")
            x, y = PADS["A"]
            o.skill("goto", x, y, 0.6)
            o.skill("land")
        elif k == "gates":
            o.skill("takeoff", 0.8)
            for g in GATES:
                c = np.array(g["c"])
                n = np.array([*g["normal"], 0.0])
                o.skill("goto", *(c - .6 * n), "tol=0.03")
                o.skill("goto", *(c + .6 * n), "tol=0.05")
            x, y = PADS["A"]
            o.skill("goto", x, y, 0.8)
            o.skill("land")
        elif env == "crazyflie-window-manual":
            # no skills: a proportional controller on the predicted stopping point, through `robo act` only
            wx, wy, wz = WINDOW["c"]
            for tgt, tol in (((-3.2, -2.3, wz), .08), ((wx - .7, -1.0, wz), .1), ((wx - .7, wy, wz), .04), ((wx + .7, wy, wz), .08),
                             ((*PADS["B"], .6), .04)):
                for _ in range(200):
                    d = o.state()["drones"][0]
                    p, v = np.array(d["pos"]), np.array(d["vel"])
                    if np.linalg.norm(p - tgt) < tol and np.linalg.norm(v) < .1:
                        break
                    c = KP_GOTO * (np.array(tgt) - (p + LEAD * v))
                    c[:2] *= min(1.0, .8 / max(np.linalg.norm(c[:2]), 1e-9))
                    c[2] = np.clip(c[2], -.6, .6)
                    o.act([*c, 0.0], 2)
            for _ in range(100):
                d = o.state()["drones"][0]
                if d["landed_on"] and not d["motors_on"]:
                    break
                o.act([0, 0, -.3, 0], 5)
        elif env == "crazyflie-window":
            wx, wy, wz = WINDOW["c"]
            o.skill("takeoff", wz)
            o.skill("goto", wx - .7, -1.0, wz)  # east along y = -1, south of gate G3
            o.skill("goto", wx - .7, wy, wz, "tol=0.03")
            o.skill("goto", wx + .7, wy, wz, "tol=0.05")
            x, y = PADS["B"]
            o.skill("goto", x, y, 0.8)
            o.skill("land")
        elif env == "crazyflie-shelf-landing":
            wx, wy, wz = WINDOW["c"]
            o.skill("takeoff", wz)
            o.skill("goto", wx - .7, -1.0, wz)
            o.skill("goto", wx - .7, wy, wz, "tol=0.03")
            o.skill("goto", wx + .7, wy, wz, "tol=0.05")
            sx, sy = SHELF["c"]
            o.skill("goto", sx, sy, SHELF["h"] + .5, "tol=0.03")
            o.skill("land")
        elif k == "formation":
            cx, cy, cz = t["center"]
            r = t["side"] / math.sqrt(3)
            pts = [(cx + r * math.cos(a), cy + r * math.sin(a), cz) for a in (math.radians(210), math.radians(-30), math.radians(90))]
            o.skill("takeoff_all", cz)
            o.skill("goto_all", *[c for pt in pts for c in pt], "tol=0.02")
            o.skill("hover", 1.0)
        elif k == "swap":
            # deconflict in time and height: cf0 climbs out of the way, cf2 crosses low and lands on S0, then cf0 lands on S2
            x0, y0 = PADS["S0"]
            x2, y2 = PADS["S2"]
            o.skill("goto", 0, x0, y0, 1.5, "tol=0.05")
            o.skill("goto", 0, -1.9, -0.2, 1.5, "tol=0.05")
            o.skill("goto", 2, x2, y2, 0.7, "tol=0.05")
            o.skill("goto", 2, x0, y0, 0.7, "tol=0.03")
            o.skill("land", 2)
            o.skill("goto", 0, x2, y2, 0.7, "tol=0.03")
            o.skill("land", 0)

    Oracle().run(solve)
