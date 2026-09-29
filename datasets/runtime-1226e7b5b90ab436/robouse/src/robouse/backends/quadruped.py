"""Quadruped backend: Unitree Go2, Unitree Go1, Boston Dynamics Spot (optionally with its arm) and ANYbotics ANYmal C
(MuJoCo Menagerie, all BSD-3-Clause) walking in a small indoor plant ("the facility").

Facility (metres, +x east, +y north, +z up, floor at z = 0): 12 x 8 m (x -6..6, y -4..4) closed by 1.0 m walls. A
partition at x = 1 splits it into the west hall and the east plant room; its only opening is a 1.1 m doorway
(y 1.5..2.6) at the east end of a 1.6 m wide corridor (y 1.3..2.9, from x = -2.2). The west hall has the dock, a
barrier of crates and a valve with an inspection tag on the west wall. The east room has a pump with an asset tag, an
instrument board with a pressure gauge 1.5 m up, an emergency-stop button 0.9 m up on the east wall, and a 0.2 m high
mezzanine platform (reached by a 1.6 m ramp) with a control cabinet. Tasks add their own movable objects and floor
markings.

Robots: the upstream Menagerie models (masses, inertias, joint limits, actuators, foot contacts) on a free-floating
base. Nothing but the simulated foot contacts moves the base. The controller (quadruped_sim/gait.py) is a model-based
trot: gait clock, Raibert foot placement, per-leg inverse kinematics, attitude feedback and joint PD (torques for the
Go2, whose actuators are torque motors; position targets for the others; ANYmal's position gains are raised from
the upstream 100 to 300 N m/rad so that it can carry its 45 kg in a trot).

Action (one 50 ms step, 25 physics steps of 2 ms):
  base.twist [VX, VY, WZ]: body-frame velocity command (m/s, m/s, deg/s); zero = stop stepping and stand.
  body.pose [DHEIGHT, DPITCH]: change of the body height (m) and nose-up pitch (deg) setpoints; zero = keep them.
  arm.ee_delta [DX, DY, DZ] (Spot with arm only): gripper tip target change in the body frame (m); zero = hold.
Skills: walk_to, turn, stand, sit, look_at (+ reach, stow with the arm).
"""
from __future__ import annotations

import math

import numpy as np

from .embodied import (ActionGroup, Budget, Embodiment, EmbodiedBackend, Oracle, Sensor, Skill, SkillArg, lookat_xyaxes, r3,
                       wrap)
from .quadruped_sim.gait import CFGS, Gait

PHYS_DT, CONTROL_DT = .002, .05
N_SUB = round(CONTROL_DT / PHYS_DT)
MONITOR_EVERY = 5

# ---- facility ------------------------------------------------------------------------------------------------------
HALL = dict(x=(-6.0, 6.0), y=(-4.0, 4.0), wall_h=1.0)
PARTITION_X = 1.0
DOOR = dict(y=(1.5, 2.6))
CORRIDOR = dict(x=(-2.2, PARTITION_X), y=(1.3, 2.9))
DOCK = dict(c=(-4.8, -2.8), half=.45)
CRATES = [(-3.2, -3.65), (-3.2, -3.05), (-3.2, -2.45), (-3.2, -1.85), (-3.2, -1.25), (-1.9, -1.1)]  # 0.6 m cubes, 0.8 m high
CRATE_HALF, CRATE_H = .3, .8
MAT = dict(c=(-1.2, -2.8), half=.4)
VALVE = dict(pos=(-5.95, 0.0), tag=(-5.88, 0.0, .55), normal=(1, 0, 0))
PUMP = dict(c=(4.2, -2.6), r=.3, h=.6, tag=(3.89, -2.6, .35), normal=(-1, 0, 0))
BOARD = dict(x=5.93, y=(-.2, 1.2), h=1.9, gauge=(5.87, .5, 1.4), normal=(-1, 0, 0))
BUTTONS = {"estop": dict(c=(5.83, -1.2, .9), rgba=[.85, .05, .05, 1], label="red emergency-stop button"),
           "reset": dict(c=(5.83, -.85, .9), rgba=[.1, .7, .2, 1], label="green reset button")}  # c: centre of the cap's face
BUTTON_R, BUTTON_TRAVEL, BUTTON_PRESS = .045, .03, .015  # cap radius, travel, depth that counts as pressed (m)
PANEL = dict(y=(-1.4, -.65), z=(.72, 1.08))  # yellow button housing on the east wall
PLATFORM = dict(x=(4.4, 6.0), y=(1.6, 4.0), h=.2)
RAMP = dict(x=(2.8, 4.4), y=(2.4, 3.4))
CABINET = dict(x=(5.2, 5.9), y=(3.6, 4.0), h=1.0, tag=(5.55, 3.58, .75), normal=(0, -1, 0))
TAGS = {"valve": VALVE, "pump": PUMP, "gauge": BOARD, "cabinet": CABINET}
TAG_POS = {"valve": VALVE["tag"], "pump": PUMP["tag"], "gauge": BOARD["gauge"], "cabinet": CABINET["tag"]}
TAG_NORMAL = {k: TAGS[k]["normal"] for k in TAGS}
TAG_MAX_DEPTH = {"valve": 2.0, "pump": 2.0, "gauge": 1.6, "cabinet": 2.0}
INSPECT_FRAC, INSPECT_STEPS = .5, 20  # tag centre in the central half of the head image, held still 1 s

TASKS: dict[str, dict] = {
    "go2-crate-detour": dict(robot="go2", start=(*DOCK["c"], 0), kind="reach", goal=MAT["c"], tol=.35, max_steps=900),
    "go2-crate-detour-manual": dict(robot="go2", start=(*DOCK["c"], 0), kind="reach", goal=MAT["c"], tol=.35, max_steps=900,
                                    skills=False),
    "go2-push-box": dict(robot="go2", start=(1.8, 2.05, 0), kind="push", box=(3.2, 0.2), box_half=.2, box_mass=3.0,
                         zone=(3.2, -1.8), zone_half=.5, max_steps=1200),
    "go2-pump-inspection": dict(robot="go2", start=(*DOCK["c"], 90), kind="inspect", tags=["pump"], max_steps=1400),
    "go1-corridor-charger": dict(robot="go1", start=(*DOCK["c"], 0), kind="charge", pad=(2.2, -3.2), pad_half=.35, max_steps=1400),
    "go1-patrol": dict(robot="go1", start=(*DOCK["c"], 0), kind="patrol", checkpoints=[("A", (-1.2, -2.8)), ("B", (4.0, 1.0)),
                                                                                      ("C", (2.4, -3.0))], max_steps=2400),
    "spot-gauge-reading": dict(robot="spot", start=(-2.9, 2.1, 0), kind="inspect", tags=["gauge"], max_steps=1200),
    "spot-arm-estop": dict(robot="spot_arm", start=(2.0, -1.0, 0), kind="press", button="estop", max_steps=900),
    "spot-arm-reset": dict(robot="spot_arm", start=(2.0, 1.5, -90), kind="press", button="reset", forbid=["estop"], max_steps=900),
    "anymal-inspection-round": dict(robot="anymal", start=(*DOCK["c"], 0), kind="round", tags=["valve", "pump"], max_steps=2400),
    "anymal-mezzanine-cabinet": dict(robot="anymal", start=(2.0, 0.0, 90), kind="inspect", tags=["cabinet"], max_steps=1200,
                                     from_platform=True),
}

HEAD_CAM = {"go2": (.34, 0, .03), "go1": (.3, 0, .05), "spot": (.47, 0, .03), "spot_arm": (.47, 0, .03), "anymal": (.56, 0, .06)}
CHASE_SCALE = {"go2": 1.0, "go1": 1.0, "spot": 1.5, "spot_arm": 1.5, "anymal": 1.5}
PITCH_MAX = 20.0          # deg, body pitch setpoint limit
DH_STEP, DP_STEP = .02, 5.0  # body.pose limits per step (m, deg)
EE_STEP = .03             # arm.ee_delta limit per step (m)
HEIGHT_RANGE = (-.45, .05)  # body height setpoint range (fraction of the standing height)
WALK_MIN_HEIGHT = -.15    # while stepping, the body is kept at least this high (fraction)
FALL_TILT = math.radians(50)
STOW_Q = [0, -3.14, 3.06, 0, 0, 0, 0]
ARM_TIP = (.23, 0, -.03)  # gripper tip on arm_link_wr1 (closed jaw)


def _build(task: dict):
    import mujoco

    from ..assets import robot_dir

    cfg = CFGS[task["robot"]]
    xml = str(robot_dir(cfg.folder) / cfg.xml)
    s = mujoco.MjSpec.from_file(xml)
    for k in list(s.keys):
        k.delete()
    for lt in list(s.lights):  # the upstream spotlights cast noisy shadows in this scene; the facility has its own lights
        lt.delete()
    s.option.timestep = PHYS_DT
    s.visual.global_.offwidth, s.visual.global_.offheight = 1280, 960
    s.visual.headlight.ambient = [.35, .35, .35]
    s.visual.headlight.diffuse = [.55, .55, .55]
    s.visual.map.znear = .01
    s.add_texture(name="q_floor", type=mujoco.mjtTexture.mjTEXTURE_2D, builtin=mujoco.mjtBuiltin.mjBUILTIN_CHECKER,
                  rgb1=[.42, .44, .46], rgb2=[.38, .40, .42], width=256, height=256)
    s.add_material(name="q_floor", textures=["", "q_floor"], texrepeat=[12, 8], texuniform=False)
    s.add_texture(name="q_sky", type=mujoco.mjtTexture.mjTEXTURE_SKYBOX, builtin=mujoco.mjtBuiltin.mjBUILTIN_GRADIENT,
                  rgb1=[.82, .84, .87], rgb2=[.95, .95, .96], width=64, height=64)
    w = s.worldbody
    BOX, PLANE, CYL = mujoco.mjtGeom.mjGEOM_BOX, mujoco.mjtGeom.mjGEOM_PLANE, mujoco.mjtGeom.mjGEOM_CYLINDER
    w.add_light(name="q_top", pos=[0, 0, 8], dir=[0, 0, -1], directional=True, diffuse=[.55, .55, .55], castshadow=False)
    w.add_light(name="q_key", pos=[-3, -5, 6], dir=[.4, .6, -.7], directional=True, diffuse=[.3, .3, .3], castshadow=False)
    w.add_geom(name="floor", type=PLANE, size=[6.5, 4.5, .1], material="q_floor", friction=[1, .005, .0001])

    def box(name, c, half, rgba, collide=True):
        w.add_geom(name=name, type=BOX, pos=list(c), size=list(half), rgba=rgba,
                   contype=1 if collide else 0, conaffinity=1 if collide else 0)

    x0, x1 = HALL["x"]
    y0, y1 = HALL["y"]
    H = HALL["wall_h"]
    wall = [.80, .78, .74, 1]
    box("wall_w", (x0 - .05, 0, H / 2), (.05, (y1 - y0) / 2 + .1, H / 2), wall)
    box("wall_e", (x1 + .05, 0, H / 2), (.05, (y1 - y0) / 2 + .1, H / 2), wall)
    box("wall_s", (0, y0 - .05, H / 2), ((x1 - x0) / 2, .05, H / 2), wall)
    box("wall_n", (0, y1 + .05, H / 2), ((x1 - x0) / 2, .05, H / 2), wall)
    px, (d0, d1) = PARTITION_X, DOOR["y"]
    part = [.70, .72, .76, 1]
    box("partition_s", (px, (y0 + d0) / 2, H / 2), (.05, (d0 - y0) / 2, H / 2), part)
    box("partition_n", (px, (d1 + y1) / 2, H / 2), (.05, (y1 - d1) / 2, H / 2), part)
    box("door_frame", (px, (d0 + d1) / 2, H + .05), (.07, (d1 - d0) / 2 + .05, .05), [.35, .35, .38, 1], collide=False)
    cx0, cx1 = CORRIDOR["x"]
    for nm, yy in (("corridor_s", CORRIDOR["y"][0]), ("corridor_n", CORRIDOR["y"][1])):
        box(nm, ((cx0 + cx1) / 2, yy, H / 2), ((cx1 - cx0) / 2, .05, H / 2), part)
    # dock and mat (floor markings)
    dx, dy = DOCK["c"]
    box("dock", (dx, dy, .003), (DOCK["half"], DOCK["half"], .003), [.2, .45, .85, 1], collide=False)
    mx, my = MAT["c"]
    box("mat", (mx, my, .003), (MAT["half"], MAT["half"], .003), [.25, .7, .35, 1], collide=False)
    for i, (x, y) in enumerate(CRATES):
        box(f"crate_{i}", (x, y, CRATE_H / 2), (CRATE_HALF, CRATE_HALF, CRATE_H / 2), [.62, .45, .26, 1] if i % 2 else [.52, .37, .2, 1])
    # valve on the west wall
    vx, vy = VALVE["pos"]
    w.add_geom(name="valve_pipe", type=CYL, pos=[vx + .03, vy, .6], size=[.05, .6, 0], rgba=[.55, .55, .6, 1])
    w.add_geom(name="valve_wheel", type=CYL, pos=[vx + .12, vy, .85], size=[.12, .015, 0], euler=[0, math.pi / 2, 0], rgba=[.8, .1, .1, 1])
    _tag(w, "valve", VALVE["tag"], VALVE["normal"])
    # pump
    pcx, pcy = PUMP["c"]
    w.add_geom(name="pump_body", type=CYL, pos=[pcx, pcy, PUMP["h"] / 2], size=[PUMP["r"], PUMP["h"] / 2, 0], rgba=[.15, .35, .6, 1])
    w.add_geom(name="pump_motor", type=BOX, pos=[pcx + .45, pcy, .25], size=[.2, .15, .15], rgba=[.3, .3, .32, 1])
    _tag(w, "pump", PUMP["tag"], PUMP["normal"])
    # instrument board with a gauge
    by0, by1 = BOARD["y"]
    box("board", (BOARD["x"], (by0 + by1) / 2, BOARD["h"] / 2), (.03, (by1 - by0) / 2, BOARD["h"] / 2), [.25, .3, .28, 1])
    gx, gy, gz = BOARD["gauge"]
    w.add_geom(name="gauge_dial", type=CYL, pos=[gx + .02, gy, gz], size=[.11, .02, 0], euler=[0, math.pi / 2, 0], rgba=[.95, .95, .92, 1],
               contype=0, conaffinity=0)
    w.add_geom(name="gauge_needle", type=BOX, pos=[gx - .005, gy + .03, gz + .02], size=[.003, .06, .006], euler=[.6, 0, 0],
               rgba=[.8, .05, .05, 1], contype=0, conaffinity=0)
    w.add_geom(name="gauge_rim", type=CYL, pos=[gx + .025, gy, gz], size=[.125, .01, 0], euler=[0, math.pi / 2, 0], rgba=[.1, .1, .1, 1],
               contype=0, conaffinity=0)
    # emergency stop: a spring-loaded button on a housing on the east wall
    (py0_, py1_), (pz0, pz1) = PANEL["y"], PANEL["z"]
    box("button_panel", (x1 - .06, (py0_ + py1_) / 2, (pz0 + pz1) / 2), (.06, (py1_ - py0_) / 2, (pz1 - pz0) / 2), [.95, .8, .1, 1])
    for bn, bt in BUTTONS.items():
        bx_, by_, bz_ = bt["c"]
        btn = w.add_body(name=f"{bn}_button", pos=[bx_ + .025, by_, bz_])
        btn.add_joint(name=f"{bn}_slide", type=mujoco.mjtJoint.mjJNT_SLIDE, axis=[1, 0, 0], range=[0, BUTTON_TRAVEL], stiffness=150,
                      damping=5, springref=0)
        btn.add_geom(name=f"{bn}_cap", type=CYL, size=[BUTTON_R, .025, 0], euler=[0, math.pi / 2, 0], rgba=bt["rgba"], mass=.1)
        s.add_exclude(bodyname1="world", bodyname2=f"{bn}_button")  # the cap slides into the housing
    # mezzanine platform, ramp and cabinet
    (px0, px1), (py0, py1), ph = PLATFORM["x"], PLATFORM["y"], PLATFORM["h"]
    box("platform", ((px0 + px1) / 2, (py0 + py1) / 2, ph / 2), ((px1 - px0) / 2, (py1 - py0) / 2, ph / 2), [.5, .52, .56, 1])
    (rx0, rx1), (ry0, ry1) = RAMP["x"], RAMP["y"]
    L = rx1 - rx0
    ang = math.atan2(ph, L)
    hl, th = math.hypot(ph, L) / 2, .05
    w.add_geom(name="ramp", type=BOX, pos=[(rx0 + rx1) / 2 + th * math.sin(ang), (ry0 + ry1) / 2, ph / 2 - th * math.cos(ang)],
               size=[hl, (ry1 - ry0) / 2, th], euler=[0, -ang, 0], rgba=[.55, .57, .6, 1])
    for sgn, yy in ((-1, ry0), (1, ry1)):  # yellow edge lines
        w.add_geom(name=f"ramp_edge{sgn}", type=BOX, pos=[(rx0 + rx1) / 2, yy - sgn * .03, ph / 2 + .002],
                   size=[hl, .03, .002], euler=[0, -ang, 0], rgba=[.95, .8, .1, 1], contype=0, conaffinity=0)
    box("platform_edge", ((px0 + px1) / 2, py0 + .03, ph + .002), ((px1 - px0) / 2, .03, .002), [.95, .8, .1, 1], collide=False)
    (cx0_, cx1_), (cy0_, cy1_) = CABINET["x"], CABINET["y"]
    box("cabinet", ((cx0_ + cx1_) / 2, (cy0_ + cy1_) / 2, ph + CABINET["h"] / 2), ((cx1_ - cx0_) / 2, (cy1_ - cy0_) / 2, CABINET["h"] / 2),
        [.75, .75, .72, 1])
    _tag(w, "cabinet", CABINET["tag"], CABINET["normal"])
    # task objects
    k = task["kind"]
    if k == "push":
        zx, zy = task["zone"]
        zh = task["zone_half"]
        for i, (u, v, a, b) in enumerate(((0, zh, zh, .025), (0, -zh, zh, .025), (zh, 0, .025, zh), (-zh, 0, .025, zh))):
            box(f"zone_line{i}", (zx + u, zy + v, .003), (a, b, .003), [.95, .8, .1, 1], collide=False)
        bx, by = task["box"]
        bh = task["box_half"]
        b = w.add_body(name="box", pos=[bx, by, bh + .002])
        b.add_freejoint(name="box_free")
        b.add_geom(name="box_geom", type=BOX, size=[bh, bh, bh], rgba=[.85, .5, .15, 1], mass=task["box_mass"], friction=[.4, .005, .0001])
    if k == "charge":
        cx, cy = task["pad"]
        box("charger", (cx, cy, .004), (task["pad_half"], task["pad_half"], .004), [.95, .55, .1, 1], collide=False)
        box("charger_post", (cx, cy - task["pad_half"] - .05, .15), (.12, .04, .15), [.2, .2, .22, 1])
    if k == "patrol":
        for name, (x, y) in task["checkpoints"]:
            w.add_geom(name=f"checkpoint_{name}", type=CYL, pos=[x, y, .003], size=[.3, .003, 0], rgba=[.6, .3, .8, 1], contype=0, conaffinity=0)
    # cameras
    for name, pos, tgt, fovy in (("overview", (0, -8.5, 9.0), (0, -.3, 0), 60), ("west_hall", (-6.4, -4.4, 3.2), (-2.5, -.5, 0), 65),
                                 ("east_room", (6.4, -4.4, 3.2), (3.2, -.2, 0), 65)):
        w.add_camera(name=name, pos=list(pos), xyaxes=[float(v) for v in lookat_xyaxes(pos, tgt).split()], fovy=fovy)
    base = s.body(cfg.base)
    base.add_camera(name="head", pos=list(HEAD_CAM[cfg.key]), xyaxes=[0, -1, 0, 0, 0, 1], fovy=70)
    sc = CHASE_SCALE[cfg.key]
    off = np.array([-.8, -1.0, 2.1]) * sc
    base.add_camera(name="chase", pos=[float(v) for v in off], xyaxes=[float(v) for v in lookat_xyaxes(off, (0, 0, -.1 * sc)).split()],
                    mode=mujoco.mjtCamLight.mjCAMLIGHT_TRACKCOM, fovy=55)
    return s.compile(), cfg, xml


def _tag(w, name, pos, normal):
    """An inspection tag: a white plate with a black square and a coloured border, facing `normal`."""
    import mujoco

    n = np.asarray(normal, float)
    if abs(n[0]) > .5:
        size, off = [.005, .09, .09], np.array([.004 * n[0], 0, 0])
    else:
        size, off = [.09, .005, .09], np.array([0, .004 * n[1], 0])
    p = np.asarray(pos, float)
    B = mujoco.mjtGeom.mjGEOM_BOX
    w.add_geom(name=f"tag_{name}", type=B, pos=list(p - 2 * off), size=[s + (.01 if s > .01 else 0) for s in size], rgba=[.1, .6, .9, 1],
               contype=0, conaffinity=0)
    w.add_geom(name=f"tag_{name}_plate", type=B, pos=list(p - off), size=size, rgba=[.97, .97, .97, 1], contype=0, conaffinity=0)
    inner = [s * (.5 if s > .01 else 1) for s in size]
    w.add_geom(name=f"tag_{name}_mark", type=B, pos=list(p), size=inner, rgba=[.05, .05, .05, 1], contype=0, conaffinity=0)


class QuadrupedBackend(EmbodiedBackend):
    name = "quadruped"

    def __init__(self, spec: dict):
        super().__init__(spec)
        env = spec.get("env") or spec.get("id")
        if env not in TASKS:
            raise KeyError(f"unknown quadruped task {env!r}")
        self.task_id, self.task = env, TASKS[env]
        self.cfg = CFGS[self.task["robot"]]
        self.arm = bool(self.cfg.extra_arm)
        self.camera = spec.get("camera", "chase")
        self.decl = self._declare()

    # ---- declaration ---------------------------------------------------------------------------------------------
    def _cameras(self) -> list[str]:
        return ["chase", "head", "overview", "west_hall", "east_room"]

    def _declare(self) -> Embodiment:
        cfg = self.cfg
        vx, vy, wz = cfg.v_max
        wz_deg = round(math.degrees(wz))
        sensors = [Sensor("base_state", "proprio", ["robot"], "base position, heading, roll/pitch, body-frame velocity, gait state, "
                          "body pose setpoints, foot contacts, fallen flag", units="m, deg, m/s", frame="world"),
                   *([Sensor("arm_state", "proprio", ["arm"], "gripper tip position (world and body frame), arm joint angles",
                             units="m, deg", frame="world")] if self.arm else []),
                   Sensor("facility", "world", ["facility"], "walls, doorway, corridor, crates, dock, tags, pump, board, e-stop, "
                          "platform and ramp (public positions)", units="m", frame="world"),
                   Sensor("task_objects", "world", ["objects"], "movable objects and floor markings of this task", units="m",
                          frame="world"),
                   Sensor("progress", "events", ["progress", "safety_events"], "inspected tags, checkpoints, e-stop state; falls"),
                   Sensor("camera:chase", "camera", mount="body", doc="follows the robot from behind-left, fixed world orientation"),
                   Sensor("camera:head", "camera", mount="head", doc="forward-facing camera on the front of the base, 70 deg field of view"),
                   *[Sensor(f"camera:{c}", "camera", mount="world") for c in ("overview", "west_hall", "east_room")]]
        groups = [ActionGroup("base.twist", "base_twist", ["VX", "VY", "WZ"], [-vx, -vy, -wz_deg], [vx, vy, wz_deg], "m/s, m/s, deg/s",
                              "body-frame velocity command for the trot controller (x forward, y left, WZ counter-clockwise); "
                              "all zero = stop stepping and stand", frame="body"),
                  ActionGroup("body.pose", "body_pose_delta", ["DHEIGHT", "DPITCH"], [-DH_STEP, -DP_STEP], [DH_STEP, DP_STEP], "m, deg",
                              f"change of the body height and nose-up pitch setpoints per step (height {HEIGHT_RANGE[0]:+.0%}..{HEIGHT_RANGE[1]:+.0%} "
                              f"of standing height, pitch +-{PITCH_MAX:.0f} deg); zero = keep", frame="body")]
        if self.arm:
            groups.append(ActionGroup("arm.ee_delta", "ee_delta_pos", ["DX", "DY", "DZ"], [-EE_STEP] * 3, [EE_STEP] * 3, "m",
                                      "move the gripper-tip target in the body frame (x forward, y left, z up); zero = hold; "
                                      "the gripper stays closed", frame="body"))
        tol = SkillArg("tol", "float", "m", .2, doc="arrival tolerance")
        speed = SkillArg("speed", "float", "m/s", vx, doc="top walking speed")
        skills = [Skill("walk_to", [SkillArg("x", unit="m"), SkillArg("y", unit="m"), tol, speed],
                        "walk to (X, Y) at up to SPEED: turn towards it, trot along the straight line, stop; not obstacle-aware, "
                        "reports if blocked", 600),
                  Skill("turn", [SkillArg("yaw_deg", unit="deg")], "turn in place to heading YAW_DEG (0 = +x east, 90 = +y north)", 200),
                  Skill("stand", [], "stop stepping and return to the standing height and a level body", 80),
                  Skill("sit", [], "stop and lower the body to its lowest height (lie down); a velocity command raises it again", 80),
                  Skill("look_at", [SkillArg("x", unit="m"), SkillArg("y", unit="m"), SkillArg("z", unit="m"),
                                    SkillArg("seconds", unit="s", default=1.0)],
                        "turn to face (X, Y), pitch the body so the head camera points at height Z (within +-20 deg), then hold "
                        "still SECONDS (at most 3); reports where the point falls in the head image", 300)]
        if self.arm:
            skills += [Skill("reach", [SkillArg("x", unit="m"), SkillArg("y", unit="m"), SkillArg("z", unit="m"),
                                       SkillArg("tol", "float", "m", .02)],
                             "move the gripper tip in a straight line to the world point (X, Y, Z) (the base stands still); "
                             "stops at contact or when out of reach", 150),
                       Skill("stow", [], "fold the arm back onto the body", 100)]
        return Embodiment(robot=cfg.name, family="quadruped", assets=[cfg.folder], sensors=sensors, action_groups=groups, skills=skills,
                          budget=Budget(self.task["max_steps"], CONTROL_DT), cameras=self._cameras())

    # ---- episode -----------------------------------------------------------------------------------------------
    def reset(self, seed: int) -> None:
        import mujoco

        self._reset_renderers()
        self.model, cfg, xml = _build(self.task)
        m = self.model
        self.data = d = mujoco.MjData(m)
        self.gait = g = Gait(cfg, m, d, "", xml)
        x, y, yaw = self.task["start"]
        yaw = math.radians(yaw)
        d.qpos[g.q0:g.q0 + 7] = [x, y, g.h_nom + .01, math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]
        d.qpos[g.qadr.ravel()] = np.array(cfg.home)
        if self.arm:
            self.arm_j = [m.joint(j).id for j in cfg.extra_arm]
            self.arm_q = np.array([m.jnt_qposadr[j] for j in self.arm_j])
            self.arm_dof = np.array([m.jnt_dofadr[j] for j in self.arm_j])
            act_of = {int(m.actuator_trnid[a, 0]): a for a in range(m.nu)}
            self.arm_act = np.array([act_of[j] for j in self.arm_j])
            d.qpos[self.arm_q] = STOW_Q
            d.ctrl[self.arm_act] = STOW_Q
            km = g.kin.m
            self.k_arm_q = np.array([km.jnt_qposadr[km.joint(j).id] for j in cfg.extra_arm[:6]])
            self.k_arm_dof = np.array([km.jnt_dofadr[km.joint(j).id] for j in cfg.extra_arm[:6]])
            self.k_wr1 = km.body("arm_link_wr1").id
            self.arm_lo = km.jnt_range[[km.joint(j).id for j in cfg.extra_arm[:6]], 0]
            self.arm_hi = km.jnt_range[[km.joint(j).id for j in cfg.extra_arm[:6]], 1]
            self.arm_qk = np.array(STOW_Q[:6], float)
            self.stow_tip = self._arm_fk(self.arm_qk)[0]
            self.ee_target = self.stow_tip.copy()
            self.wr1 = m.body("arm_link_wr1").id
        mujoco.mj_forward(m, d)
        self.h_off, self.pitch_up = 0.0, 0.0
        g.h_target = g.height = g.h_nom
        for _ in range(150):  # settle on the feet
            g.step(PHYS_DT)
            mujoco.mj_step(m, d)
        self.base_geoms = {i for i in range(m.ngeom) if m.geom_bodyid[i] == g.base and (m.geom_contype[i] or m.geom_conaffinity[i])}
        self.robot_bodies = self._subtree(g.base)
        self.robot_geoms = {i for i in range(m.ngeom) if m.geom_bodyid[i] in self.robot_bodies}
        gname = lambda i: mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, i) or ""
        self.gname = [gname(i) for i in range(m.ngeom)]
        self.ground = {i for i in range(m.ngeom) if self.gname[i] in ("floor", "ramp", "platform")}
        self.static = {i for i in range(m.ngeom) if m.geom_bodyid[i] == 0 and (m.geom_contype[i] or m.geom_conaffinity[i])} - self.ground
        self.head = m.camera("head").id
        self.events, self._event_keys = [], set()
        self.inspected: list[str] = []
        self._settling = False
        self._view = {k: 0 for k in TAGS}
        self.next_cp = 0
        self.pressed: set[str] = set()
        self.bumps: set[str] = set()
        self.fallen = False
        if self.task["kind"] == "push":
            self.box_bid = m.body("box").id
        self.button_q = {b: m.jnt_qposadr[m.joint(f"{b}_slide").id] for b in BUTTONS}

    def _subtree(self, root: int) -> set[int]:
        m = self.model
        out = {root}
        for b in range(m.nbody):
            p = b
            while p > 0:
                if p == root:
                    out.add(b)
                    break
                p = m.body_parentid[p]
        return out

    # ---- arm kinematics (kinematic copy) ---------------------------------------------------------------------------
    def _arm_fk(self, q):
        import mujoco

        km, kd = self.gait.kin.m, self.gait.kin.d
        kd.qpos[self.k_arm_q] = q
        mujoco.mj_kinematics(km, kd)
        R = kd.xmat[self.k_wr1].reshape(3, 3)
        return kd.xpos[self.k_wr1] + R @ np.array(ARM_TIP), R

    def _arm_ik(self, target: np.ndarray, iters: int = 6) -> np.ndarray:
        """Gripper tip to `target` (body frame) with the jaw pointing forward (wrist x axis along body x), damped least
        squares with a pull towards the stowed posture in the null space."""
        import mujoco

        km, kd = self.gait.kin.m, self.gait.kin.d
        q = self.arm_qk.copy()
        jp, jr = np.zeros((3, km.nv)), np.zeros((3, km.nv))
        for _ in range(iters):
            p, R = self._arm_fk(q)
            mujoco.mj_comPos(km, kd)
            e_p = target - p
            e_r = np.cross(R[:, 0], np.array([1.0, 0, 0])) * .5
            mujoco.mj_jac(km, kd, jp, jr, p, self.k_wr1)
            J = np.vstack([jp[:, self.k_arm_dof], .3 * jr[:, self.k_arm_dof]])
            e = np.concatenate([e_p, .3 * e_r])
            JJ = J @ J.T + 1e-3 * np.eye(6)
            dq = J.T @ np.linalg.solve(JJ, e)
            q = np.clip(q + np.clip(dq, -.3, .3), self.arm_lo, self.arm_hi)
        self.arm_qk = q
        return q

    def _tip(self) -> np.ndarray:
        d = self.data
        return d.xpos[self.wr1] + d.xmat[self.wr1].reshape(3, 3) @ np.array(ARM_TIP)

    def _to_body(self, p) -> np.ndarray:
        g = self.gait
        return g.R().T @ (np.asarray(p, float) - self.data.xpos[g.base])

    def _to_world(self, p) -> np.ndarray:
        g = self.gait
        return self.data.xpos[g.base] + g.R() @ np.asarray(p, float)

    # ---- control -----------------------------------------------------------------------------------------------
    def control_step(self, a: np.ndarray) -> None:
        import mujoco

        g = self.gait
        vx, vy, wz_deg, dh, dp = [float(v) for v in a[:5]]
        twist = np.array([vx, vy, math.radians(wz_deg)])
        self.h_off = float(np.clip(self.h_off + dh, HEIGHT_RANGE[0] * g.h_nom, HEIGHT_RANGE[1] * g.h_nom))
        self.pitch_up = float(np.clip(self.pitch_up + dp, -PITCH_MAX, PITCH_MAX))
        if self.fallen:
            twist[:] = 0
        g.set_command(*twist)
        if np.any(np.abs(twist) > 1e-6):  # stepping raises a lowered body to at least 85 % of the standing height
            self.h_off = max(self.h_off, WALK_MIN_HEIGHT * g.h_nom)
        g.h_target = g.h_nom + self.h_off
        g.pitch_target = -math.radians(self.pitch_up)
        if self.arm:
            self.ee_target = self.ee_target + np.asarray(a[5:8], float)
            self.ee_target = self._clamp_reach(self.ee_target)
            q = self._arm_ik(self.ee_target)
            self.data.ctrl[self.arm_act[:6]] = q
            self.data.ctrl[self.arm_act[6]] = 0.0
        m, d = self.model, self.data
        for k in range(N_SUB):
            g.step(PHYS_DT)
            mujoco.mj_step(m, d)
            if k % MONITOR_EVERY == MONITOR_EVERY - 1:
                self._monitor_contacts()
        self._monitor_step()

    def _clamp_reach(self, t: np.ndarray) -> np.ndarray:
        """Keep the arm target within 1.0 m of the shoulder."""
        sh = np.array([.292, 0, .188])
        v = t - sh
        n = float(np.linalg.norm(v))
        if n > 1.0:
            t = sh + v / n
        return t

    # ---- monitors ----------------------------------------------------------------------------------------------
    def _monitor_contacts(self) -> None:
        d = self.data
        for c in d.contact[:d.ncon]:
            for a, b in ((c.geom1, c.geom2), (c.geom2, c.geom1)):
                if a in self.base_geoms and b in self.ground:
                    self._fall("the body touched the ground")
                elif a in self.robot_geoms and b in self.static:
                    self.bumps.add(self.gname[b] or "obstacle")
        roll, pitch, _ = self.gait.rpy()
        if abs(roll) > FALL_TILT or abs(pitch) > FALL_TILT:
            self._fall(f"the base tilted beyond {math.degrees(FALL_TILT):.0f} deg")
        for b, q in self.button_q.items():
            if b not in self.pressed and float(self.data.qpos[q]) > BUTTON_PRESS:
                self.pressed.add(b)
                if b in self.task.get("forbid", ()):
                    self.event(f"pressed:{b}", "damage", f"the {BUTTONS[b]['label']} was pressed: the plant shut down")

    def _fall(self, why: str) -> None:
        self.fallen = True
        self.event("fall", "fall", f"the robot fell: {why}")

    def hold_action(self) -> list[float]:
        # The episode server calls this once, right before the post-`done` settle steps: from then on no new progress
        # (inspections, checkpoints) is recorded, so the settle cannot complete a task.
        self._settling = True
        return super().hold_action()

    def _on_platform(self) -> bool:
        p = self.data.xpos[self.gait.base]
        return (PLATFORM["x"][0] <= p[0] <= PLATFORM["x"][1] and PLATFORM["y"][0] <= p[1] <= PLATFORM["y"][1]
                and p[2] > PLATFORM["h"] + .5 * self.gait.h_nom)

    def _monitor_step(self) -> None:
        t = self.task
        p = self.data.xpos[self.gait.base]
        if getattr(self, "_settling", False):
            return
        if t["kind"] == "patrol" and self.next_cp < len(t["checkpoints"]):
            _, c = t["checkpoints"][self.next_cp]
            if math.dist(p[:2], c) < .35:
                self.next_cp += 1
        for k in TAGS:
            if k in self.inspected:
                continue
            ok = self._tag_in_view(k) and self._still() and (self._on_platform() or not t.get("from_platform"))
            self._view[k] = self._view[k] + 1 if ok else 0
            if self._view[k] >= INSPECT_STEPS:
                self.inspected.append(k)

    def _still(self) -> bool:
        g = self.gait
        v = g.vel_heading()
        return not g.active and float(np.hypot(v[0], v[1])) < .1 and abs(v[2]) < .2 and not self.fallen

    def _project(self, p) -> tuple[float, float, float]:
        """(u, v, depth) of world point p in the head camera: u, v in [-1, 1] at the image border (u right, v up)."""
        d = self.data
        pos = d.cam_xpos[self.head]
        R = d.cam_xmat[self.head].reshape(3, 3)
        q = R.T @ (np.asarray(p) - pos)
        depth = -q[2]
        if depth <= 1e-6:
            return 9.0, 9.0, depth
        f = 1 / math.tan(math.radians(self.model.cam_fovy[self.head]) / 2)
        return q[0] / depth * f, q[1] / depth * f, depth

    def _clear(self, p) -> bool:
        import mujoco

        d = self.data
        pos = d.cam_xpos[self.head].copy()
        vec = np.asarray(p, float) - pos
        dist = float(np.linalg.norm(vec))
        gid = np.array([-1], dtype=np.int32)
        frac = mujoco.mj_ray(self.model, d, pos, vec / dist, None, 1, self.gait.base, gid)
        return frac < 0 or frac >= dist - .03 or self.gname[int(gid[0])].startswith("tag_")

    def _tag_in_view(self, k: str) -> bool:
        p = np.array(TAG_POS[k], float)
        u, v, depth = self._project(p)
        if not (.3 <= depth <= TAG_MAX_DEPTH[k] and abs(u) <= INSPECT_FRAC and abs(v) <= INSPECT_FRAC):
            return False
        to_cam = self.data.cam_xpos[self.head] - p
        if float(np.dot(to_cam / np.linalg.norm(to_cam), TAG_NORMAL[k])) < .5:  # within 60 deg of the tag's facing
            return False
        return self._clear(p)

    # ---- observation -------------------------------------------------------------------------------------------
    def _box_state(self):
        d = self.data
        b = self.box_bid
        R = d.xmat[b].reshape(3, 3)
        return d.xpos[b].copy(), math.degrees(math.atan2(R[1, 0], R[0, 0])), float(R[2, 2])

    def observe(self) -> dict:
        g, d = self.gait, self.data
        roll, pitch, yaw = g.rpy()
        v = g.vel_heading()
        st: dict = {"robot": {
            "name": self.cfg.name, "pos": r3(d.xpos[g.base]), "yaw_deg": round(math.degrees(yaw), 1),
            "roll_deg": round(math.degrees(roll), 1), "pitch_up_deg": round(-math.degrees(pitch), 1),
            "vel_body": r3(v[:2]), "yaw_rate_deg_s": round(math.degrees(v[2]), 1),
            "gait": "walking" if g.active else ("sitting" if self._sitting() else "standing"),
            "body_pose_setpoint": {"height_offset_m": round(self.h_off, 3), "pitch_up_deg": round(self.pitch_up, 1)},
            "standing_height_m": round(g.h_nom, 3), "feet_in_contact": [bool(x) for x in g.feet_in_contact()],
            "head_camera": {"pos": r3(d.cam_xpos[self.head]), "forward": r3(-d.cam_xmat[self.head].reshape(3, 3)[:, 2])},
            "fallen": self.fallen}, "time_s": round(float(d.time), 2)}
        if self.arm:
            tip = self._tip()
            st["arm"] = {"tip_pos": r3(tip), "tip_pos_body": r3(self._to_body(tip)), "tip_target_body": r3(self.ee_target),
                         "joints_deg": [round(math.degrees(float(x)), 1) for x in d.qpos[self.arm_q[:6]]]}
        st["facility"] = self._facility()
        st["objects"] = self._objects()
        st["progress"] = self._progress()
        st["safety_events"] = [e["detail"] for e in self.events]
        return st

    def _facility(self) -> dict:
        return {
            "hall": {"x": list(HALL["x"]), "y": list(HALL["y"]), "wall_height": HALL["wall_h"]},
            "partition": {"x": PARTITION_X, "thickness": .1, "doorway_y": list(DOOR["y"]),
                          "note": "wall between the west hall (x < 1) and the east plant room (x > 1); the doorway is the only opening"},
            "corridor": {"x": list(CORRIDOR["x"]), "walls_at_y": list(CORRIDOR["y"]), "wall_thickness": .1,
                         "note": "open at its west end (x = -2.2), leads east to the doorway"},
            "dock": {"center": list(DOCK["c"]), "size": [2 * DOCK["half"]] * 2},
            "mat": {"center": list(MAT["c"]), "size": [2 * MAT["half"]] * 2},
            "crates": [{"center": [x, y], "size": [2 * CRATE_HALF, 2 * CRATE_HALF], "height": CRATE_H} for x, y in CRATES],
            "tags": {k: {"pos": list(TAG_POS[k]), "faces": list(TAG_NORMAL[k])} for k in TAGS},
            "valve": {"pos": [*VALVE["pos"], .6], "note": "pipe and red hand wheel on the west wall"},
            "pump": {"center": list(PUMP["c"]), "radius": PUMP["r"], "height": PUMP["h"], "motor_east_side": True},
            "instrument_board": {"x": BOARD["x"], "y": list(BOARD["y"]), "height": BOARD["h"], "gauge_center": list(BOARD["gauge"])},
            "buttons": {b: {"cap_face_center": list(v["c"]), "faces": [-1, 0, 0], "cap_diameter": 2 * BUTTON_R, "travel_m": BUTTON_TRAVEL,
                            "what": v["label"]} for b, v in BUTTONS.items()},
            "button_panel": {"x": HALL["x"][1], "y": list(PANEL["y"]), "z": list(PANEL["z"]), "note": "yellow housing on the east wall"},
            "platform": {"x": list(PLATFORM["x"]), "y": list(PLATFORM["y"]), "height": PLATFORM["h"]},
            "ramp": {"x": list(RAMP["x"]), "y": list(RAMP["y"]), "rises_to": PLATFORM["h"], "note": "rises eastwards onto the platform"},
            "cabinet": {"x": list(CABINET["x"]), "y": list(CABINET["y"]), "on": "platform", "height": CABINET["h"]},
        }

    def _objects(self) -> dict:
        t, out = self.task, {}
        if t["kind"] == "push":
            p, yaw, up = self._box_state()
            out["box"] = {"pos": r3(p), "yaw_deg": round(yaw, 1), "size": [2 * t["box_half"]] * 3, "mass_kg": t["box_mass"],
                          "upright": up > .94}
            out["zone"] = {"center": list(t["zone"]), "size": [2 * t["zone_half"]] * 2}
        if t["kind"] == "charge":
            out["charger_pad"] = {"center": list(t["pad"]), "size": [2 * t["pad_half"]] * 2,
                                  "post": "charging post on the pad's south edge"}
        if t["kind"] == "patrol":
            out["checkpoints"] = {n: list(c) for n, c in t["checkpoints"]}
        return out

    def _progress(self) -> dict:
        t = self.task
        out: dict = {"inspected_tags": list(self.inspected)}
        if t["kind"] == "patrol":
            out["checkpoints_reached_in_order"] = self.next_cp
        if self.arm:
            out["buttons_pressed"] = sorted(self.pressed)
            out["button_depth_m"] = {b: round(float(self.data.qpos[q]), 4) for b, q in self.button_q.items()}
        if self.bumps:
            out["touched_obstacles"] = sorted(self.bumps)
        return out

    # ---- scoring -----------------------------------------------------------------------------------------------
    def _sitting(self) -> bool:
        """Lying down: stopped with the body-height setpoint at its lowest."""
        return not self.gait.active and self.h_off <= HEIGHT_RANGE[0] * self.gait.h_nom + 1e-3

    def _upright_standing(self) -> bool:
        return not self.fallen and self._still()

    def _in_box(self, p, c, half) -> bool:
        return abs(p[0] - c[0]) <= half and abs(p[1] - c[1]) <= half

    def success(self) -> bool:
        if self.events or self.fallen:
            return False
        t, k = self.task, self.task["kind"]
        p = self.data.xpos[self.gait.base]
        if k == "reach":
            return math.dist(p[:2], t["goal"]) <= t["tol"] and self._upright_standing()
        if k == "inspect":
            return all(x in self.inspected for x in t["tags"])
        if k == "round":
            return all(x in self.inspected for x in t["tags"]) and self._in_box(p, DOCK["c"], DOCK["half"]) and self._upright_standing()
        if k == "patrol":
            return self.next_cp == len(t["checkpoints"]) and self._in_box(p, DOCK["c"], DOCK["half"]) and self._upright_standing()
        if k == "charge":
            return self._in_box(p, t["pad"], t["pad_half"]) and self._sitting() and p[2] <= .7 * self.gait.h_nom
        if k == "push":
            bp, _, up = self._box_state()
            if up < .94 or not self._box_inside(t):
                return False
            return self._upright_standing()
        if k == "press":
            return t["button"] in self.pressed and self._upright_standing()
        return False

    def _box_inside(self, t) -> bool:
        """Every corner of the box's footprint lies inside the zone."""
        d = self.data
        R = d.xmat[self.box_bid].reshape(3, 3)
        c = d.xpos[self.box_bid]
        h = t["box_half"]
        for sx in (-1, 1):
            for sy in (-1, 1):
                q = c + R @ np.array([sx * h, sy * h, 0])
                if not self._in_box(q, t["zone"], t["zone_half"]):
                    return False
        return True

    # ---- skills ------------------------------------------------------------------------------------------------
    def _act(self, twist=(0, 0, 0), dh=0.0, dp=0.0, ee=(0, 0, 0)) -> list[float]:
        vx, vy, wz = self.cfg.v_max
        out = [float(np.clip(twist[0], -vx, vx)), float(np.clip(twist[1], -vy, vy)),
               float(np.clip(twist[2], -math.degrees(wz), math.degrees(wz))), float(np.clip(dh, -DH_STEP, DH_STEP)),
               float(np.clip(dp, -DP_STEP, DP_STEP))]
        if self.arm:
            out += [float(np.clip(x, -EE_STEP, EE_STEP)) for x in ee]
        return out

    def _pose_now(self):
        g = self.gait
        p = self.data.xpos[g.base]
        return float(p[0]), float(p[1]), g.yaw()

    def _stop(self, n: int = 40):
        for _ in range(n):
            if not self.gait.active:
                break
            yield self._act()

    def skill_walk_to(self, x: float, y: float, tol: float, speed: float):
        tol = max(float(tol), .05)
        vxm = float(np.clip(speed, .05, self.cfg.v_max[0]))
        hist: list[tuple[float, float]] = []
        why = ""
        steps = 0
        while True:
            px, py, yaw = self._pose_now()
            dx, dy = x - px, y - py
            dist = math.hypot(dx, dy)
            if dist <= tol:
                break
            if self.fallen:
                why = "the robot fell"
                break
            hist.append((px, py))
            if len(hist) > 60 and math.dist(hist[-1], hist[-61]) < .08:
                why = "blocked: moved less than 8 cm in the last 3 s"
                break
            c, s = math.cos(yaw), math.sin(yaw)
            bx, by = c * dx + s * dy, -s * dx + c * dy  # target in the body frame
            e = math.atan2(by, bx)
            if dist < .5:  # close: side-step onto the point without turning
                sp = min(vxm, .9 * dist + .05)
                yield self._act((sp * bx / dist, sp * by / dist, 0))
            elif abs(e) > math.radians(35):
                yield self._act((0, 0, math.degrees(2.0 * e)))
            else:
                sp = min(vxm, .8 * dist + .1)
                yield self._act((sp * math.cos(e), 0, math.degrees(2.0 * e)))
            steps += 1
        yield from self._stop()
        px, py, yaw = self._pose_now()
        dist = math.hypot(x - px, y - py)
        out = {"reached": dist <= tol * 1.5 and not self.fallen, "distance_m": round(dist, 3), "pos": [round(px, 3), round(py, 3)],
               "yaw_deg": round(math.degrees(yaw), 1)}
        if why:
            out["stopped"] = why
        if self.bumps:
            out["touched_obstacles"] = sorted(self.bumps)
        return out

    def skill_turn(self, yaw_deg: float):
        tgt = math.radians(yaw_deg)
        for _ in range(160):
            e = wrap(tgt - self.gait.yaw())
            if abs(e) < math.radians(3) or self.fallen:
                break
            yield self._act((0, 0, math.degrees(float(np.clip(2.5 * e, -2, 2)))))
        yield from self._stop()
        return {"yaw_deg": round(math.degrees(self.gait.yaw()), 1)}

    def skill_stand(self):
        g = self.gait
        yield from self._stop()
        for _ in range(60):
            dh = -self.h_off
            dp = -self.pitch_up
            if abs(dh) < 1e-4 and abs(dp) < .05:
                break
            yield self._act(dh=dh, dp=dp)
        yield self._act()
        return {"gait": "standing", "height_m": round(float(self.data.xpos[g.base][2]), 3)}

    def skill_sit(self):
        g = self.gait
        yield from self._stop()
        low = HEIGHT_RANGE[0] * g.h_nom
        for _ in range(60):
            if self.h_off <= low + 1e-4:
                break
            yield self._act(dh=-DH_STEP, dp=-self.pitch_up)
        for _ in range(10):  # let the body come down
            yield self._act()
        return {"gait": "sitting", "height_m": round(float(self.data.xpos[g.base][2]), 3)}

    def skill_look_at(self, x: float, y: float, z: float, seconds: float):
        px, py, _ = self._pose_now()
        if math.hypot(x - px, y - py) > .05:
            res = yield from self.skill_turn(math.degrees(math.atan2(y - py, x - px)))
            del res
        for _ in range(20):
            cam = self.data.cam_xpos[self.head]
            want = math.degrees(math.atan2(z - cam[2], math.hypot(x - cam[0], y - cam[1])))
            want = float(np.clip(want, -PITCH_MAX, PITCH_MAX))
            e = want - self.pitch_up
            if abs(e) < .5:
                break
            yield self._act(dp=e)
        for _ in range(max(1, round(min(float(seconds), 3.0) / CONTROL_DT))):
            yield self._act()
        u, v, depth = self._project((x, y, z))
        in_img = depth > 0 and abs(u) <= 1 and abs(v) <= 1
        return {"pitch_up_deg": round(self.pitch_up, 1), "in_head_image": bool(in_img),
                "image_offset": {"right": round(float(u), 2), "up": round(float(v), 2)} if in_img else None,
                "distance_m": round(float(np.linalg.norm(np.array([x, y, z]) - self.data.cam_xpos[self.head])), 2)}

    def skill_reach(self, x: float, y: float, z: float, tol: float):
        if not self.arm:
            raise ValueError("this robot has no arm")
        yield from self._stop()
        hist: list[np.ndarray] = []
        why = ""
        for _ in range(140):
            tgt_b = self._to_body((x, y, z))  # world target, re-expressed as the body sways
            tip_b = self._to_body(self._tip())
            err = tgt_b - tip_b
            if float(np.linalg.norm(err)) <= tol:
                break
            n = float(np.linalg.norm(err))
            new = self.ee_target + err * min(1.0, .02 / n)
            dev = new - tip_b  # the target may lead the measured tip by at most 6 cm (bounded push on contact)
            nd = float(np.linalg.norm(dev))
            if nd > .06:
                new = tip_b + dev * .06 / nd
            hist.append(tip_b)
            if len(hist) > 10 and float(np.linalg.norm(hist[-1] - hist[-11])) < .004:
                why = "the tip stopped moving (blocked or out of reach)"
                break
            yield self._act(ee=new - self.ee_target)
        tip = self._tip()
        dist = float(np.linalg.norm(np.array([x, y, z]) - tip))
        contacts = sorted({self.gname[b] for c in self.data.contact[:self.data.ncon] for a, b in ((c.geom1, c.geom2), (c.geom2, c.geom1))
                           if self.model.geom_bodyid[a] in self._arm_bodies() and b not in self.robot_geoms and self.gname[b]})
        out = {"reached": dist <= tol * 1.5, "distance_m": round(dist, 3), "tip_pos": r3(tip), "touching": contacts}
        if why:
            out["stopped"] = why
        return out

    def _arm_bodies(self) -> set[int]:
        if not hasattr(self, "_armb"):
            self._armb = self._subtree(self.model.body("arm_link_sh0").id)
        return self._armb

    def skill_stow(self):
        if not self.arm:
            raise ValueError("this robot has no arm")
        for _ in range(90):
            step = self.stow_tip - self.ee_target
            n = float(np.linalg.norm(step))
            if n < 1e-3:
                break
            if n > .025:
                step *= .025 / n
            yield self._act(ee=step)
        for _ in range(10):
            yield self._act()
        return {"tip_pos_body": r3(self._to_body(self._tip()))}


# ------------------------------------------------------------------------------------------------------------------
# reference solutions (socket only)
# ------------------------------------------------------------------------------------------------------------------

def push_box(o: Oracle, t: dict, nose: float) -> None:
    """Push the box towards the zone centre in short straight pushes, re-aligning behind it after each one."""
    zone = np.array(t["zone"], float)
    bh = t["box_half"]
    stand = bh + nose + .12  # base-to-box-centre distance when lined up behind it
    for _ in range(8):
        b = np.array(o.state()["objects"]["box"]["pos"][:2])
        dvec = zone - b
        dist = float(np.linalg.norm(dvec))
        if np.all(np.abs(dvec) < t["zone_half"] - bh - .12):
            break
        u = dvec / dist
        behind = b - u * stand
        r = np.array(o.state()["robot"]["pos"][:2])
        if float(np.dot(r - b, u)) > -.3:  # on the wrong side: go round the box first
            n = np.array([-u[1], u[0]])
            side = b + n * (.8 if float(np.dot(r - b, n)) >= 0 else -.8)
            o.skill("walk_to", *(side - u * .2), .12)
            o.skill("walk_to", *(b - u * (stand + .4)), .1)
        o.skill("walk_to", *behind - u * .1, .06)
        o.skill("turn", math.degrees(math.atan2(u[1], u[0])))
        push = min(dist, .6)
        o.skill("walk_to", *(b + u * push - u * (bh + nose - .02)), .06, .2)
        r = np.array(o.state()["robot"]["pos"][:2])
        o.skill("walk_to", *(r - u * .35), .08)  # step back from the box


def drive_to(o: Oracle, x: float, y: float, tol: float = .2, vmax: float = .5) -> None:
    """Heading-controlled walk to (x, y) using only observations and `robo act` (for tasks without skills)."""
    for _ in range(400):
        r = o.state()["robot"]
        px, py = r["pos"][:2]
        yaw = math.radians(r["yaw_deg"])
        dx, dy = x - px, y - py
        dist = math.hypot(dx, dy)
        if dist <= tol:
            return
        e = wrap(math.atan2(dy, dx) - yaw)
        if abs(e) > math.radians(35):
            a = [0, 0, float(np.clip(math.degrees(2 * e), -60, 60))]
        else:
            a = [min(vmax, .8 * dist + .1) * math.cos(e), 0, float(np.clip(math.degrees(2 * e), -60, 60))]
        o.act(a + [0, 0], 2)


def oracle_main(env: str) -> None:
    t = TASKS[env]

    def walk(o: Oracle, pts, tol=.2):
        for x, y in pts:
            o.skill("walk_to", x, y, tol)

    def solve(o: Oracle) -> None:
        if env == "go2-crate-detour-manual":  # no skills: a waypoint follower on `robo act`
            for pt in [(-4.2, -.3), (-2.55, -.3), (-2.55, -2.2), MAT["c"]]:
                drive_to(o, *pt, tol=.1 if pt == MAT["c"] else .2)
            o.act([0, 0, 0, 0, 0], 12)
        elif env == "go2-crate-detour":
            walk(o, [(-4.2, -.3), (-2.55, -.3), (-2.55, -2.2)])
            o.skill("walk_to", *MAT["c"], .1)
        elif env == "go2-push-box":
            push_box(o, t, nose=HEAD_CAM["go2"][0])
        elif env == "go2-pump-inspection":
            tx, ty, tz = PUMP["tag"]
            walk(o, [(-3.2, 2.05), (1.6, 2.05), (2.4, -1.5), (tx - 1.0, ty)], .15)
            o.skill("look_at", tx, ty, tz, 1.5)
        elif env == "go1-corridor-charger":
            cx, cy = t["pad"]
            walk(o, [(-3.2, 2.05), (1.6, 2.05), (cx, cy + 1.0)], .15)
            o.skill("walk_to", cx, cy + .05, .08)
            o.skill("sit")
        elif env == "go1-patrol":
            walk(o, [(-4.2, -.3), (-2.55, -.3), (-2.55, -2.2)], .15)
            o.skill("walk_to", *t["checkpoints"][0][1], .15)
            walk(o, [(-2.55, -2.2), (-2.55, -.3), (-2.8, 2.05), (1.6, 2.05)], .15)
            o.skill("walk_to", *t["checkpoints"][1][1], .15)
            o.skill("walk_to", *t["checkpoints"][2][1], .15)
            walk(o, [(1.8, 2.05), (-2.8, 2.05), (-4.2, -.3)], .2)
            o.skill("walk_to", *DOCK["c"], .1)
        elif env == "spot-gauge-reading":
            gx, gy, gz = BOARD["gauge"]
            walk(o, [(1.8, 2.05), (gx - 1.3, gy)], .1)
            o.skill("look_at", gx, gy, gz, 1.5)
        elif env in ("spot-arm-estop", "spot-arm-reset"):
            bx, by, bz = BUTTONS[t["button"]]["c"]
            o.skill("walk_to", bx - 1.1, by, .05)
            o.skill("turn", 0)
            o.skill("reach", bx - .1, by, bz, .015)
            o.skill("reach", bx + .03, by, bz, .01)   # push the cap in by 2-3 cm
            o.skill("reach", bx - .15, by, bz, .02)
            o.skill("stow")
        elif env == "anymal-inspection-round":
            vx_, vy_, vz_ = VALVE["tag"]
            walk(o, [(-4.6, -.3), (vx_ + 1.4, vy_)], .12)
            o.skill("look_at", vx_, vy_, vz_, 1.5)
            px_, py_, pz_ = PUMP["tag"]
            walk(o, [(-2.9, 2.05), (1.6, 2.05), (2.4, -1.4), (px_ - 1.4, py_)], .15)
            o.skill("look_at", px_, py_, pz_, 1.5)
            walk(o, [(2.2, -1.4), (1.8, 2.05), (-2.9, 2.05), (-4.6, -.3)], .2)
            o.skill("walk_to", *DOCK["c"], .1)
        elif env == "anymal-mezzanine-cabinet":
            cx, cy, cz = CABINET["tag"]
            walk(o, [(2.0, 2.9)], .1)
            o.skill("turn", 0)
            walk(o, [(4.9, 2.9), (cx, cy - 1.1)], .1)
            o.skill("look_at", cx, cy, cz, 1.5)

    Oracle().run(solve)
