"""Scene builder of the so101 suite: a small hobby desk scaled to the SO-101 and Koch v1.1 arms, one arm or two, and
the task's props. Everything is built with MjSpec; each arm is its MuJoCo Menagerie MJCF attached unchanged
(kinematics, actuators, gains, gripper contact settings as shipped).

Frames (metres): the desk top is z = 0. One arm: its base is the origin and the world frame is the arm's base frame
(+x forward, away from the base; +y left; +z up). Two arms (the StationeryBench rig): both bases stand at the desk's
far edge facing the same way, at y = +BIMANUAL_Y (left) and y = -BIMANUAL_Y (right); the world frame is the rig frame,
midway between the bases, with the same axes, so each arm's base frame is the world frame shifted by its base
position.

The desk, the props (cubes, cups, trays, drawer unit, push-button switch, ruler, puck, box, zone and marker) and the
cameras are BenchFlow's.
"""

from __future__ import annotations

import math

import numpy as np

DESK_W, DESK_D, DESK_T = 0.45, 0.35, 0.03  # width (y), depth (x), thickness (m)
DESK_X0 = -0.07  # back edge of the desk in the single-arm frame (the base stands 7 cm in from it)
BIMANUAL_Y = 0.14  # each base's distance from the rig's centre line
PHYS_DT = 0.002

# Per robot: Menagerie folder and file, the five arm joints (agent order), the gripper joint, how the gripper's raw
# joint angle maps to the 0-100 scale (0 = closed, 100 = fully open), the tool point, the gripper's approach and jaw
# axes in the tool body's frame, the mount rotation, joint limits (deg) and the home and rest poses.
ROBOTS = {
    "so101": dict(
        name="SO-101 follower arm",
        folder="robotstudio_so101",
        xml="so101.xml",
        licence="Apache-2.0",
        joints=["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"],
        gripper="gripper",
        grip_q=(-0.174533, 1.7453292),  # raw joint angle at 0 and at 100 (the joint's range; LeRobot RANGE_0_100)
        tool_body="gripper",
        moving_body="moving_jaw_so101_v1",
        tip_r=0.00075,  # radius of the jaw-tip spheres
        tool=None,  # the upstream site `gripperframe` (LeRobot's gripper_frame_link: between the fingertips)
        tool_site="gripperframe",
        approach=(0.0, 0.0, -1.0),  # fingertips point along -z of the gripper body
        jaw=(1.0, 0.0, 0.0),  # from the fixed jaw toward the moving jaw
        fixed_tip="fixed_jaw_sph_tip1",
        moving_tip="moving_jaw_sph_tip1",
        mount_quat=(1.0, 0.0, 0.0, 0.0),
        # the real so101 profile's limits (robouse.real.robots.SO101): URDF limits shaved, inside the calibrated stops
        low=[-108.0, -98.0, -94.8, -93.0, -155.2],
        high=[108.0, 98.0, 94.8, 93.0, 160.8],
        home=[0.0, -60.0, 60.0, 60.0, 0.0],
        park=[0.0, -98.0, 86.0, 70.0, 0.0],
        ik_seeds=[[0.0, -30.0, 30.0, 70.0], [0.0, 0.0, 40.0, 50.0], [0.0, 20.0, 0.0, 70.0], [0.0, -60.0, 60.0, 80.0]],
        wrist_cam="wrist_cam",
    ),
    "koch": dict(
        name="Koch v1.1 low-cost robot arm",
        folder="low_cost_robot_arm",
        xml="low_cost_robot_arm.xml",
        licence="Apache-2.0",
        joints=["base_rotation", "pitch", "elbow", "wrist_pitch", "wrist_roll"],
        gripper="gripper",
        grip_q=(0.032, -1.60),  # closed at the upper end of the joint's range, open at the lower
        tool_body="gripper_static_finger",
        moving_body="gripper_moving_finger",
        tip_r=0.002,  # half the thickness of the finger pads
        tool=(-0.066, -0.004, 0.0005),  # where a 2.5 cm cube sits in the closed jaws (a site added here)
        tool_site=None,
        approach=(-1.0, 0.0, 0.0),
        jaw=(0.0, -1.0, 0.0),
        fixed_tip="gripper_static_finger_pad_1",
        moving_tip="gripper_moving_finger_pad_1",
        # the upstream model reaches along its -x; turned half a turn so it faces +x like the SO-101
        mount_quat=(0.0, 0.0, 0.0, 1.0),
        low=[-124.0, -88.0, -88.0, -112.0, -178.0],
        high=[124.0, 32.0, 81.0, 112.0, 178.0],
        home=[0.0, -30.0, 40.0, 60.0, 0.0],
        park=[0.0, 0.0, 80.0, 80.0, 0.0],
        ik_seeds=[[0.0, -30.0, 40.0, 60.0], [0.0, -60.0, 20.0, 80.0], [0.0, 0.0, 60.0, 40.0], [0.0, -45.0, 0.0, 90.0]],
        wrist_cam=None,
    ),
}

COLORS = {
    "red": [0.86, 0.16, 0.14, 1],
    "blue": [0.13, 0.36, 0.86, 1],
    "green": [0.16, 0.66, 0.30, 1],
    "white": [0.93, 0.93, 0.90, 1],
    "yellow": [0.96, 0.78, 0.16, 1],
    "grey": [0.55, 0.57, 0.60, 1],
}
TRAY_RGBA = {"red": [0.80, 0.34, 0.30, 1], "blue": [0.32, 0.47, 0.82, 1], "grey": [0.58, 0.60, 0.63, 1]}
WOOD = [0.72, 0.58, 0.42, 1]

# prop dimensions (m) and masses (kg)
CUBE = 0.025
CUBE_MASS = 0.012
CUP_R, CUP_H, CUP_WALL, CUP_MASS = 0.025, 0.045, 0.003, 0.06  # 5 cm across the outside
TRAY_L, TRAY_W, TRAY_H, TRAY_WALL, TRAY_MASS = 0.12, 0.08, 0.016, 0.004, 0.15
RULER_L, RULER_W, RULER_T, RULER_MASS = 0.15, 0.024, 0.012, 0.010
PUCK_R, PUCK_H, PUCK_MASS = 0.03, 0.015, 0.04
BOX_L, BOX_W, BOX_H, BOX_MASS = 0.15, 0.06, 0.04, 0.06
ZONE_R = 0.04
DRAWER_W, DRAWER_H, DRAWER_D, DRAWER_TRAVEL = 0.06, 0.035, 0.07, 0.06
KNOB_R, KNOB_L, KNOB_HEAD_R, KNOB_HEAD_T = 0.0045, 0.022, 0.0085, 0.005
BUTTON_R, BUTTON_TRAVEL, BUTTON_BASE, BUTTON_CAP_H = 0.011, 0.008, (0.045, 0.045, 0.022), 0.016
LED_OFF, LED_ON = [0.12, 0.12, 0.12, 1], [1.0, 0.95, 0.55, 1]

OBJ_FRICTION = [0.9, 0.005, 0.0001]


def yaw_quat(yaw_deg: float) -> list[float]:
    h = math.radians(yaw_deg) / 2
    return [math.cos(h), 0.0, 0.0, math.sin(h)]


def tilt_deg(xmat) -> float:
    """Angle between a body's z axis and world z."""
    return math.degrees(math.acos(float(np.clip(np.asarray(xmat).reshape(3, 3)[2, 2], -1, 1))))


def arm_bases(rig: str) -> dict[str, tuple[float, float, float]]:
    """Arm prefix -> base position in the world frame."""
    if rig == "bimanual":
        return {"left": (0.0, BIMANUAL_Y, 0.0), "right": (0.0, -BIMANUAL_Y, 0.0)}
    return {"arm": (0.0, 0.0, 0.0)}


def desk_extent(rig: str) -> tuple[float, float, float, float]:
    """(x0, x1, y0, y1) of the desk top."""
    return (DESK_X0, DESK_X0 + DESK_D, -DESK_W / 2, DESK_W / 2)


def _xyaxes(pos, target):
    from ..embodied import lookat_xyaxes

    return [float(v) for v in lookat_xyaxes(pos, target).split()]


def build(robot: str, rig: str, props: list[dict]):
    """Compile the scene: desk, arm(s), props and cameras. Returns the MjModel."""
    import mujoco

    from ...assets import robot_dir

    R = ROBOTS[robot]
    s = mujoco.MjSpec()
    s.compiler.degree = False
    s.option.timestep = PHYS_DT
    s.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    s.option.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
    s.option.impratio = 10
    s.option.iterations = 30
    s.option.ls_iterations = 30
    s.option.enableflags |= mujoco.mjtEnableBit.mjENBL_MULTICCD  # several contacts for flat convex faces (puck, cup)
    s.option.noslip_iterations = 3  # no creep of a fingertip pinch under a load away from the jaws (ruler, box)
    s.visual.global_.offwidth, s.visual.global_.offheight = 1280, 1280
    s.visual.headlight.ambient = [0.38, 0.38, 0.38]
    s.visual.headlight.diffuse = [0.50, 0.50, 0.50]
    s.visual.headlight.specular = [0.05, 0.05, 0.05]
    s.visual.map.znear = 0.005
    s.visual.quality.shadowsize = 2048
    s.stat.meansize = 0.03
    s.stat.extent = 0.6
    s.stat.center = [0.12, 0.0, 0.05]
    BOX, PLANE = mujoco.mjtGeom.mjGEOM_BOX, mujoco.mjtGeom.mjGEOM_PLANE
    s.add_texture(
        name="floor",
        type=mujoco.mjtTexture.mjTEXTURE_2D,
        builtin=mujoco.mjtBuiltin.mjBUILTIN_CHECKER,
        rgb1=[0.34, 0.35, 0.38],
        rgb2=[0.30, 0.31, 0.34],
        width=256,
        height=256,
    )
    s.add_material(name="floor", textures=["", "floor"], texrepeat=[10, 10])
    s.add_texture(
        name="sky",
        type=mujoco.mjtTexture.mjTEXTURE_SKYBOX,
        builtin=mujoco.mjtBuiltin.mjBUILTIN_GRADIENT,
        rgb1=[0.94, 0.95, 0.97],
        rgb2=[0.66, 0.70, 0.76],
        width=512,
        height=3072,
    )
    s.add_texture(
        name="mat",
        type=mujoco.mjtTexture.mjTEXTURE_2D,
        builtin=mujoco.mjtBuiltin.mjBUILTIN_CHECKER,
        rgb1=[0.20, 0.42, 0.30],
        rgb2=[0.18, 0.39, 0.28],
        mark=mujoco.mjtMark.mjMARK_EDGE,
        markrgb=[0.42, 0.62, 0.48],
        width=200,
        height=200,
    )
    s.add_material(name="mat", textures=["", "mat"], texrepeat=[9, 7], texuniform=False)
    w = s.worldbody
    w.add_light(
        name="key", pos=[0.6, -0.5, 1.6], dir=[-0.4, 0.3, -1], directional=True, diffuse=[0.55] * 3, castshadow=True
    )
    w.add_light(
        name="fill", pos=[-0.4, 0.6, 1.2], dir=[0.3, -0.4, -1], directional=True, diffuse=[0.22] * 3, castshadow=False
    )
    x0, x1, y0, y1 = desk_extent(rig)
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    w.add_geom(
        name="floor", type=PLANE, pos=[0, 0, -0.74], size=[2, 2, 0.1], material="floor", contype=0, conaffinity=0
    )
    w.add_geom(
        name="desk",
        type=BOX,
        pos=[cx, cy, -DESK_T / 2],
        size=[DESK_D / 2, DESK_W / 2, DESK_T / 2],
        rgba=WOOD,
        friction=[0.8, 0.005, 0.0001],
    )
    # a cutting mat on the desk (visual; 1 cm grid)
    w.add_geom(
        name="mat",
        type=BOX,
        pos=[cx + 0.01, cy, 0.0002],
        size=[DESK_D / 2 - 0.03, DESK_W / 2 - 0.02, 0.0002],
        material="mat",
        contype=0,
        conaffinity=0,
        group=1,
    )
    for sx in (-1, 1):
        for sy in (-1, 1):
            w.add_geom(
                name=f"desk_leg{sx}{sy}",
                type=BOX,
                pos=[cx + sx * (DESK_D / 2 - 0.025), cy + sy * (DESK_W / 2 - 0.025), -0.38],
                size=[0.015, 0.015, 0.36],
                rgba=[0.45, 0.36, 0.26, 1],
                contype=0,
                conaffinity=0,
            )

    # cameras: front (from the operator's side, looking back at the arm), top (straight down), side (three-quarter
    # view used for recordings); the SO-101's wrist camera comes with its model
    tgt = [0.13, 0.0, 0.02]
    w.add_camera(name="front", pos=[0.58, 0.0, 0.34], xyaxes=_xyaxes([0.58, 0.0, 0.34], tgt), fovy=42)
    w.add_camera(name="top", pos=[0.115, 0.0, 0.78], xyaxes=[0, -1, 0, 1, 0, 0], fovy=40)
    side = [0.50, -0.42, 0.36] if rig == "single" else [0.52, -0.46, 0.40]
    w.add_camera(name="side", pos=side, xyaxes=_xyaxes(side, [0.10, 0.0, 0.03]), fovy=45)

    # arm(s): the upstream MJCF attached at each base
    for pre, base in arm_bases(rig).items():
        spec = mujoco.MjSpec.from_file(str(robot_dir(R["folder"]) / R["xml"]))
        frame = w.add_frame(pos=list(base), quat=list(R["mount_quat"]))
        s.attach(spec, prefix=f"{pre}/", frame=frame)
        if R["tool"] is not None:
            s.body(f"{pre}/{R['tool_body']}").add_site(
                name=f"{pre}/tool", pos=list(R["tool"]), size=[0.004, 0, 0], group=4
            )
        # a small pad under the base, so the arm reads as clamped to the desk
        w.add_geom(
            name=f"{pre}_clamp",
            type=BOX,
            pos=[base[0] - 0.005, base[1], 0.001],
            size=[0.05, 0.045, 0.001],
            rgba=[0.15, 0.15, 0.16, 1],
            contype=0,
            conaffinity=0,
            group=1,
        )

    for p in props:
        _add_prop(s, w, p)
    return s.compile()


def hollow_ring(body, name, *, r_in, height, wall, n, rgba, z0=0.0, friction=None):
    """Thin boxes around a vertical axis at the body origin (a cup or holder wall)."""
    import mujoco

    r = r_in + wall / 2
    half_w = r * math.tan(math.pi / n) + wall / 2
    for i in range(n):
        a = 2 * math.pi * i / n
        g = body.add_geom(
            name=f"{name}_wall{i}",
            type=mujoco.mjtGeom.mjGEOM_BOX,
            pos=[r * math.cos(a), r * math.sin(a), z0 + height / 2],
            size=[wall / 2, half_w, height / 2],
            quat=[math.cos(a / 2), 0, 0, math.sin(a / 2)],
            rgba=rgba,
        )
        if friction is not None:
            g.friction = friction


def _add_prop(s, w, p: dict) -> None:
    import mujoco

    BOX, CYL, SPH = mujoco.mjtGeom.mjGEOM_BOX, mujoco.mjtGeom.mjGEOM_CYLINDER, mujoco.mjtGeom.mjGEOM_SPHERE
    k, n = p["kind"], p["name"]
    x, y = p["pos"][:2]
    yaw = float(p.get("yaw", 0.0))
    if k == "cube":
        b = w.add_body(name=n, pos=[x, y, CUBE / 2 + 0.0005], quat=yaw_quat(yaw))
        b.add_freejoint(name=n)
        b.add_geom(
            name=n,
            type=BOX,
            size=[CUBE / 2] * 3,
            mass=CUBE_MASS,
            rgba=COLORS[p["color"]],
            friction=OBJ_FRICTION,
            condim=4,
            priority=0,
        )
    elif k == "cup":
        b = w.add_body(name=n, pos=[x, y, 0.0005])
        b.add_freejoint(name=n)
        rgba = COLORS.get(p.get("color", "white"), COLORS["white"])
        hollow_ring(b, n, r_in=CUP_R - CUP_WALL, height=CUP_H, wall=CUP_WALL, n=16, rgba=rgba)
        b.add_geom(name=f"{n}_bottom", type=CYL, pos=[0, 0, 0.002], size=[CUP_R, 0.002, 0], rgba=rgba)
        for g in b.geoms:
            g.mass = CUP_MASS / 17
    elif k == "tray":
        b = w.add_body(name=n, pos=[x, y, 0.0005], quat=yaw_quat(yaw))
        b.add_freejoint(name=n)
        rgba = TRAY_RGBA[p.get("color", "grey")]
        L, W, H, T = TRAY_L, TRAY_W, TRAY_H, TRAY_WALL
        parts = [
            ("floor", [0, 0, 0.002], [L / 2, W / 2, 0.002]),
            ("wall_xp", [L / 2 - T / 2, 0, H / 2], [T / 2, W / 2, H / 2]),
            ("wall_xn", [-L / 2 + T / 2, 0, H / 2], [T / 2, W / 2, H / 2]),
            ("wall_yp", [0, W / 2 - T / 2, H / 2], [L / 2, T / 2, H / 2]),
            ("wall_yn", [0, -W / 2 + T / 2, H / 2], [L / 2, T / 2, H / 2]),
        ]
        for part, pos, size in parts:
            b.add_geom(
                name=f"{n}_{part}", type=BOX, pos=pos, size=size, rgba=rgba, mass=TRAY_MASS / 5, friction=OBJ_FRICTION
            )
    elif k == "ruler":
        b = w.add_body(name=n, pos=[x, y, RULER_T / 2 + 0.0005], quat=yaw_quat(yaw))
        b.add_freejoint(name=n)
        b.add_geom(
            name=n,
            type=BOX,
            size=[RULER_L / 2, RULER_W / 2, RULER_T / 2],
            mass=RULER_MASS,
            rgba=[0.93, 0.80, 0.36, 1],
            friction=OBJ_FRICTION,
            condim=4,
        )
        for i in range(15):  # centimetre ticks (visual)
            ln = 0.008 if i % 5 == 0 else 0.005
            b.add_geom(
                name=f"{n}_tick{i}",
                type=BOX,
                pos=[-RULER_L / 2 + 0.005 + 0.01 * i, RULER_W / 2 - ln / 2, RULER_T / 2 + 0.0002],
                size=[0.0004, ln / 2, 0.0002],
                rgba=[0.15, 0.12, 0.08, 1],
                contype=0,
                conaffinity=0,
                group=1,
            )
    elif k == "puck":
        b = w.add_body(name=n, pos=[x, y, PUCK_H / 2 + 0.0005])
        b.add_freejoint(name=n)
        b.add_geom(
            name=n,
            type=CYL,
            size=[PUCK_R, PUCK_H / 2, 0],
            mass=PUCK_MASS,
            rgba=[0.12, 0.12, 0.13, 1],
            friction=[0.35, 0.005, 0.0001],
            condim=4,
        )
    elif k == "box":
        b = w.add_body(name=n, pos=[x, y, BOX_H / 2 + 0.0005], quat=yaw_quat(yaw))
        b.add_freejoint(name=n)
        b.add_geom(
            name=n,
            type=BOX,
            size=[BOX_W / 2, BOX_L / 2, BOX_H / 2],
            mass=BOX_MASS,
            rgba=[0.78, 0.62, 0.42, 1],
            friction=[1.0, 0.005, 0.0001],
            condim=4,
        )
        b.add_geom(
            name=f"{n}_tape",
            type=BOX,
            pos=[0, 0, BOX_H / 2 + 0.0002],
            size=[0.012, BOX_L / 2, 0.0002],
            rgba=[0.86, 0.80, 0.64, 1],
            contype=0,
            conaffinity=0,
            group=1,
        )
    elif k == "zone":
        w.add_geom(
            name=n,
            type=CYL,
            pos=[x, y, 0.0006],
            size=[ZONE_R, 0.0005, 0],
            rgba=[0.95, 0.85, 0.20, 0.55],
            contype=0,
            conaffinity=0,
            group=1,
        )
    elif k == "marker":
        z = p["pos"][2]
        w.add_geom(
            name=n,
            type=SPH,
            pos=[x, y, z],
            size=[0.006, 0, 0],
            rgba=[0.95, 0.25, 0.70, 0.85],
            contype=0,
            conaffinity=0,
            group=1,
        )
        w.add_geom(
            name=f"{n}_post",
            type=CYL,
            pos=[x, y, z / 2],
            size=[0.0012, z / 2, 0],
            rgba=[0.95, 0.25, 0.70, 0.35],
            contype=0,
            conaffinity=0,
            group=1,
        )
        w.add_geom(
            name=f"{n}_spot",
            type=CYL,
            pos=[x, y, 0.0006],
            size=[0.008, 0.0005, 0],
            rgba=[0.95, 0.25, 0.70, 0.6],
            contype=0,
            conaffinity=0,
            group=1,
        )
    elif k == "drawer":
        _add_drawer(s, w, n, x, y)
    elif k == "button":
        _add_button(s, w, n, x, y, p.get("color", "red"))
    else:
        raise ValueError(f"unknown prop kind {k!r}")


def _add_drawer(s, w, n: str, fx: float, y: float) -> None:
    """A small drawer unit fixed to the desk. Its front faces the arm (-x); `fx` is the x of the closed drawer's front
    face, `y` its centre. The drawer slides out toward -x on a joint `n` (0 = closed, DRAWER_TRAVEL fully out)."""
    import mujoco

    BOX, CYL = mujoco.mjtGeom.mjGEOM_BOX, mujoco.mjtGeom.mjGEOM_CYLINDER
    T = 0.004
    W, H, D = DRAWER_W, DRAWER_H, DRAWER_D
    carcass = [0.82, 0.80, 0.76, 1]
    cab = w.add_body(name=f"{n}_cabinet", pos=[fx, y, 0])
    ow, oh = W / 2 + T + 0.002, H + T + 0.004  # carcass outer half-width, height
    cab.add_geom(
        name=f"{n}_top", type=BOX, pos=[D / 2 + 0.002, 0, oh - T / 2], size=[D / 2 + 0.004, ow, T / 2], rgba=carcass
    )
    cab.add_geom(name=f"{n}_back", type=BOX, pos=[D + 0.004, 0, oh / 2], size=[T / 2, ow, oh / 2], rgba=carcass)
    for sy in (-1, 1):
        cab.add_geom(
            name=f"{n}_side{'l' if sy > 0 else 'r'}",
            type=BOX,
            pos=[D / 2 + 0.002, sy * (ow - T / 2), oh / 2],
            size=[D / 2 + 0.004, T / 2, oh / 2],
            rgba=carcass,
        )
    dr = cab.add_body(name=n, pos=[0, 0, 0])
    dr.add_joint(
        name=n,
        type=mujoco.mjtJoint.mjJNT_SLIDE,
        axis=[-1, 0, 0],
        range=[0, DRAWER_TRAVEL],
        limited=True,
        damping=3.0,
        frictionloss=0.15,
        armature=0.01,
    )
    wood = [0.62, 0.45, 0.30, 1]
    dr.add_geom(
        name=f"{n}_front", type=BOX, pos=[T / 2, 0, H / 2 + 0.002], size=[T / 2, W / 2, H / 2], rgba=wood, mass=0.02
    )
    dr.add_geom(
        name=f"{n}_floor",
        type=BOX,
        pos=[D / 2, 0, 0.004],
        size=[D / 2 - 0.001, W / 2 - 0.001, 0.002],
        rgba=wood,
        mass=0.015,
    )
    for sy in (-1, 1):
        dr.add_geom(
            name=f"{n}_wall{'l' if sy > 0 else 'r'}",
            type=BOX,
            pos=[D / 2, sy * (W / 2 - 0.002), H / 2 - 0.004],
            size=[D / 2 - 0.001, 0.0015, H / 2 - 0.006],
            rgba=wood,
            mass=0.005,
        )
    dr.add_geom(
        name=f"{n}_rear",
        type=BOX,
        pos=[D - 0.002, 0, H / 2 - 0.004],
        size=[0.0015, W / 2 - 0.002, H / 2 - 0.006],
        rgba=wood,
        mass=0.005,
    )
    kz = H / 2 + 0.002
    brass = [0.80, 0.66, 0.30, 1]
    dr.add_geom(
        name=f"{n}_knob",
        type=CYL,
        pos=[-KNOB_L / 2, 0, kz],
        size=[KNOB_R, KNOB_L / 2, 0],
        quat=[math.cos(math.pi / 4), 0, math.sin(math.pi / 4), 0],
        rgba=brass,
        mass=0.004,
        friction=[1.0, 0.005, 0.0001],
        condim=4,
    )
    dr.add_geom(
        name=f"{n}_knob_head",
        type=CYL,
        pos=[-KNOB_L - KNOB_HEAD_T / 2, 0, kz],
        size=[KNOB_HEAD_R, KNOB_HEAD_T / 2, 0],
        quat=[math.cos(math.pi / 4), 0, math.sin(math.pi / 4), 0],
        rgba=brass,
        mass=0.004,
        friction=[1.0, 0.005, 0.0001],
        condim=4,
    )


def _add_button(s, w, n: str, x: float, y: float, color: str = "red") -> None:
    """A momentary push button fixed to the desk: a cap of `color` on a sprung vertical joint `n` (0 = up, negative =
    pressed in) and an LED. The backend reads full presses from the cap's travel and lights the LED (the button panel
    of the backend's `_press`)."""
    import mujoco

    BOX, CYL, SPH = mujoco.mjtGeom.mjGEOM_BOX, mujoco.mjtGeom.mjGEOM_CYLINDER, mujoco.mjtGeom.mjGEOM_SPHERE
    bx, by, bz = BUTTON_BASE
    base = w.add_body(name=f"{n}_base", pos=[x, y, 0])
    base.add_geom(
        name=f"{n}_housing", type=BOX, pos=[0, 0, bz / 2], size=[bx / 2, by / 2, bz / 2], rgba=[0.20, 0.21, 0.23, 1]
    )
    base.add_geom(
        name=f"{n}_led",
        type=SPH,
        pos=[bx / 2 - 0.007, by / 2 - 0.007, bz],
        size=[0.003, 0, 0],
        rgba=LED_OFF,
        contype=0,
        conaffinity=0,
        group=1,
    )
    cap = base.add_body(name=n, pos=[0, 0, bz])
    cap.add_joint(
        name=n,
        type=mujoco.mjtJoint.mjJNT_SLIDE,
        axis=[0, 0, 1],
        range=[-BUTTON_TRAVEL, 0],
        limited=True,
        stiffness=150.0,
        springref=0.004,
        damping=2.0,
        armature=0.002,
    )
    cap.add_geom(
        name=f"{n}_cap",
        type=CYL,
        pos=[0, 0, BUTTON_CAP_H / 2],
        size=[BUTTON_R, BUTTON_CAP_H / 2, 0],
        rgba=COLORS[color],
        mass=0.005,
        friction=[1.0, 0.005, 0.0001],
    )
