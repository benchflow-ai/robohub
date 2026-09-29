"""Scene builder for the dexhand suite: a table, a 6-DoF positioner hanging above it, one Menagerie hand on the
positioner's flange, and the task's objects. Everything is built with MjSpec; the hand is the upstream XML attached
unchanged (kinematics, actuators, gains, contact and friction settings as shipped).

World frame (metres): +x away from the positioner's home (forward), +y to the left, +z up; the floor is z = 0 and the
table top is z = TABLE_Z.

Positioner: three prismatic joints (x, y, z) and three revolute joints in yaw-pitch-roll order (z, then the yawed y,
then the pitched x). The hand's forearm/palm axis is the positioner's x axis, so ROLL turns the hand about its own
long axis (pronation / supination), PITCH tips the fingers down (+) or up (-), YAW turns about the vertical. At zero
roll/pitch/yaw the palm faces down and the fingers point along +x. The joints are driven by torque/force motors with
force limits (see POSITIONER); the PD law with gravity feed-forward lives in the backend, like a robot arm's joint
controller.
"""
from __future__ import annotations

import math

import numpy as np

TABLE_Z = 0.40
TABLE_C = (0.10, 0.0)
TABLE_HALF = (0.45, 0.35)

# positioner joints: name, type, axis, range (m or rad), motor force limit (N or N m)
POSITIONER = [
    ("pos_x", "slide", (1, 0, 0), (-0.45, 0.45), 200.0),
    ("pos_y", "slide", (0, 1, 0), (-0.40, 0.40), 200.0),
    ("pos_z", "slide", (0, 0, 1), (0.42, 1.00), 300.0),
    ("pos_yaw", "hinge", (0, 0, 1), (math.radians(-190), math.radians(190)), 40.0),
    ("pos_pitch", "hinge", (0, 1, 0), (math.radians(-60), math.radians(100)), 40.0),
    ("pos_roll", "hinge", (1, 0, 0), (math.radians(-190), math.radians(190)), 40.0),
]

# Per hand: Menagerie folder, how it is mounted on the flange, which bodies make up each finger, the fingertip geoms,
# the grasp point (a site in front of the palm) and the upstream solver options kept for that hand.
HANDS = {
    "shadow": dict(
        folder="shadow_hand", robot="Shadow Dexterous Hand E3M5 (right)", licence="Apache-2.0",
        mount_quat=[0, 1, 0, 0], mount_pos=[-0.213, 0.0, 0.010],  # flange at the wrist joint; palm down, fingers +x
        act_prefix="rh_A_", palm_body="rh_palm", grasp_site=None,  # upstream `grasp_site` is used
        impratio=10.0,
        fingers={"thumb": ["rh_thbase", "rh_thproximal", "rh_thhub", "rh_thmiddle", "rh_thdistal"],
                 "index": ["rh_ffknuckle", "rh_ffproximal", "rh_ffmiddle", "rh_ffdistal"],
                 "middle": ["rh_mfknuckle", "rh_mfproximal", "rh_mfmiddle", "rh_mfdistal"],
                 "ring": ["rh_rfknuckle", "rh_rfproximal", "rh_rfmiddle", "rh_rfdistal"],
                 "little": ["rh_lfmetacarpal", "rh_lfknuckle", "rh_lfproximal", "rh_lfmiddle", "rh_lfdistal"],
                 "palm": ["rh_forearm", "rh_wrist", "rh_palm"]},
        tips={"thumb": "rh_thdistal", "index": "rh_ffdistal", "middle": "rh_mfdistal", "ring": "rh_rfdistal", "little": "rh_lfdistal"},
    ),
    "leap": dict(
        folder="leap_hand", robot="LEAP Hand (right)", licence="MIT",
        mount_quat=[0, 1, 0, 0], mount_pos=[0.05, 0.035, 0.087],  # flange at the centre of the back of the palm
        act_prefix="", act_suffix="_act", palm_body="palm", grasp_site=(-0.02, -0.035, -0.067),
        impratio=100.0,
        fingers={"thumb": ["th_mp", "th_bs", "th_px", "th_ds"], "index": ["if_bs", "if_px", "if_md", "if_ds"],
                 "middle": ["mf_bs", "mf_px", "mf_md", "mf_ds"], "ring": ["rf_bs", "rf_px", "rf_md", "rf_ds"], "palm": ["palm"]},
        tips={"thumb": "th_ds", "index": "if_ds", "middle": "mf_ds", "ring": "rf_ds"},
    ),
}

CUBE_FACES = {"red": (1, 0, 0), "orange": (-1, 0, 0), "green": (0, 1, 0), "blue": (0, -1, 0), "white": (0, 0, 1), "yellow": (0, 0, -1)}
FACE_RGBA = {"red": [.85, .15, .15, 1], "orange": [.95, .55, .1, 1], "green": [.2, .7, .25, 1], "blue": [.15, .3, .85, 1],
             "white": [.95, .95, .95, 1], "yellow": [.95, .85, .15, 1]}

CONTAINER_BOTTOM = .015             # fixed containers (bowl, stand, pen cup) have a 15 mm solid bottom
OBJ_FRICTION = [1.2, 0.01, 0.0005]   # objects are rubber-coated / grippy (only object friction is raised; hands are upstream)


def _xyaxes(pos, target):
    from ..embodied import lookat_xyaxes

    return [float(v) for v in lookat_xyaxes(pos, target).split()]


def hollow_cylinder(body, name, *, pos, r_in, height, wall, n=16, rgba, bottom=True, mass=None, friction=None, bottom_t=0.0):
    """A cup/holder/bowl wall made of n thin boxes around a vertical axis (plus an optional bottom disc)."""
    import mujoco

    BOX, CYL = mujoco.mjtGeom.mjGEOM_BOX, mujoco.mjtGeom.mjGEOM_CYLINDER
    cx, cy, cz = pos
    r = r_in + wall / 2
    half_w = r * math.tan(math.pi / n) + wall / 2
    geoms = []
    for i in range(n):
        a = 2 * math.pi * i / n
        g = body.add_geom(name=f"{name}_wall{i}", type=BOX, pos=[cx + r * math.cos(a), cy + r * math.sin(a), cz + height / 2],
                          size=[wall / 2, half_w, height / 2], quat=[math.cos(a / 2), 0, 0, math.sin(a / 2)], rgba=rgba)
        geoms.append(g)
    if bottom:
        t = max(wall, bottom_t)
        g = body.add_geom(name=f"{name}_bottom", type=CYL, pos=[cx, cy, cz + t / 2], size=[r_in + wall, t / 2, 0], rgba=rgba)
        geoms.append(g)
    for g in geoms:
        if mass is not None:
            g.mass = mass / len(geoms)
        if friction is not None:
            g.friction = friction
    return geoms


def build(hand: str, task: dict):
    """Compile the scene for one task. Returns the MjModel."""
    import mujoco

    from ...assets import robot_dir

    H = HANDS[hand]
    s = mujoco.MjSpec()
    s.compiler.degree = False
    s.option.timestep = 0.002
    s.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    s.option.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
    s.option.impratio = H["impratio"]
    s.option.noslip_iterations = int(task.get("noslip", 0))
    s.visual.global_.offwidth, s.visual.global_.offheight = 1280, 960
    s.visual.headlight.ambient = [.35, .35, .35]
    s.visual.headlight.diffuse = [.55, .55, .55]
    s.visual.map.znear = .01
    s.stat.meansize = .05
    s.stat.extent = 1.0
    s.add_texture(name="floor", type=mujoco.mjtTexture.mjTEXTURE_2D, builtin=mujoco.mjtBuiltin.mjBUILTIN_CHECKER,
                  rgb1=[.30, .32, .36], rgb2=[.26, .28, .32], width=256, height=256)
    s.add_material(name="floor", textures=["", "floor"], texrepeat=[8, 8])
    w = s.worldbody
    BOX, PLANE, CAP = mujoco.mjtGeom.mjGEOM_BOX, mujoco.mjtGeom.mjGEOM_PLANE, mujoco.mjtGeom.mjGEOM_CAPSULE
    w.add_light(name="key", pos=[0.4, -0.6, 2.2], dir=[-0.15, 0.3, -1], directional=True, diffuse=[.55, .55, .55], castshadow=True)
    w.add_geom(name="floor", type=PLANE, size=[2, 2, .1], material="floor")
    tx, ty = TABLE_C
    hx, hy = TABLE_HALF
    w.add_geom(name="table", type=BOX, pos=[tx, ty, TABLE_Z - .04], size=[hx, hy, .04], rgba=[.66, .55, .42, 1], friction=[1.0, .005, .0001])
    for sx in (-1, 1):
        for sy in (-1, 1):
            w.add_geom(name=f"table_leg{sx}{sy}", type=BOX, pos=[tx + sx * (hx - .04), ty + sy * (hy - .04), (TABLE_Z - .08) / 2],
                       size=[.025, .025, (TABLE_Z - .08) / 2], rgba=[.4, .33, .25, 1])

    # cameras
    tgt = [0.10, 0.0, TABLE_Z + 0.10]
    for name, pos, fovy in (("front", [0.56, -0.44, 0.76], 50), ("side", [0.12, -0.62, 0.60], 50), ("top", [0.12, 0.0, 1.15], 45),
                            ("back", [-0.40, 0.30, 0.85], 50)):
        w.add_camera(name=name, pos=pos, xyaxes=_xyaxes(pos, tgt if name != "top" else [0.12, 0.0, 0.0]), fovy=fovy)

    # positioner (the hand hangs from it)
    parent = w.add_body(name="positioner_base", pos=[0, 0, 0])
    for jn, jt, axis, rng, _f in POSITIONER:
        b = parent.add_body(name=jn.replace("pos_", "carriage_"), pos=[0, 0, 0])
        b.mass = 0.2
        b.inertia = [1e-3, 1e-3, 1e-3]
        j = b.add_joint(name=jn, type=mujoco.mjtJoint.mjJNT_SLIDE if jt == "slide" else mujoco.mjtJoint.mjJNT_HINGE, axis=list(axis),
                        range=list(rng), limited=True, damping=2.0 if jt == "slide" else 0.5, armature=0.01)
        del j
        parent = b
    flange = parent
    flange.name = "flange"
    # visual only: the positioner's last link, behind the hand along its long axis
    back = -0.23 if hand == "shadow" else -0.02
    flange.add_geom(name="arm_link", type=CAP, fromto=[back, 0, 0, back - 0.30, 0, 0], size=[.03, 0, 0], rgba=[.55, .57, .6, 1],
                    contype=0, conaffinity=0, group=1)
    for jn, _jt, _axis, _rng, f in POSITIONER:
        a = s.add_actuator(name=f"{jn}_motor", target=jn, trntype=mujoco.mjtTrn.mjTRN_JOINT)
        a.ctrllimited = True
        a.ctrlrange = [-f, f]

    hs = mujoco.MjSpec.from_file(str(robot_dir(H["folder"]) / "right_hand.xml"))
    frame = flange.add_frame(pos=H["mount_pos"], quat=H["mount_quat"])
    s.attach(hs, prefix="h/", frame=frame)
    if H["grasp_site"] is not None:
        s.body("h/" + H["palm_body"]).add_site(name="h/grasp_site", pos=list(H["grasp_site"]), size=[.008, 0, 0], group=4)

    _add_objects(s, w, task)
    return s.compile()


def _add_objects(s, w, task: dict) -> None:
    import mujoco

    BOX, CYL, SPH, CAP = (mujoco.mjtGeom.mjGEOM_BOX, mujoco.mjtGeom.mjGEOM_CYLINDER, mujoco.mjtGeom.mjGEOM_SPHERE,
                          mujoco.mjtGeom.mjGEOM_CAPSULE)
    Z = TABLE_Z
    for ob in task.get("objects", []):
        k = ob["kind"]
        if k == "ball":
            b = w.add_body(name=ob["name"], pos=[*ob["pos"][:2], ob["pos"][2] if len(ob["pos"]) > 2 else Z + ob["r"] + .001])
            b.add_freejoint(name=ob["name"])
            b.add_geom(name=ob["name"], type=SPH, size=[ob["r"], 0, 0], mass=ob["mass"], rgba=ob.get("rgba", [.9, .75, .1, 1]),
                       friction=ob.get("friction", OBJ_FRICTION), condim=6, priority=1)
        elif k == "stand":  # a small ring the ball rests in so it does not roll away
            n, r = 10, ob["r"]
            for i in range(n):
                a = 2 * math.pi * i / n
                w.add_geom(name=f"{ob['name']}_{i}", type=CAP, fromto=[ob["pos"][0] + r * math.cos(a), ob["pos"][1] + r * math.sin(a), Z + .006,
                                                                       ob["pos"][0] + r * math.cos(a + 2 * math.pi / n),
                                                                       ob["pos"][1] + r * math.sin(a + 2 * math.pi / n), Z + .006],
                           size=[.006, 0, 0], rgba=[.3, .3, .35, 1])
        elif k == "cube":
            h = ob["half"]
            b = w.add_body(name=ob["name"], pos=[*ob["pos"][:2], ob["pos"][2] if len(ob["pos"]) > 2 else Z + h + .001],
                           quat=ob.get("quat", [1, 0, 0, 0]))
            b.add_freejoint(name=ob["name"])
            b.add_geom(name=ob["name"], type=BOX, size=[h, h, h], mass=ob["mass"], rgba=[.85, .85, .85, 1], friction=ob.get("friction", OBJ_FRICTION),
                       condim=4, priority=1)
            for face, n in CUBE_FACES.items():
                n = np.array(n, float)
                size = [h * .82 if abs(n[i]) < .5 else .0008 for i in range(3)]
                b.add_geom(name=f"{ob['name']}_{face}", type=BOX, pos=list(n * (h + .0004)), size=size, rgba=FACE_RGBA[face],
                           contype=0, conaffinity=0, mass=0)
        elif k == "cup":
            b = w.add_body(name=ob["name"], pos=[*ob["pos"][:2], Z + .001])
            b.add_freejoint(name=ob["name"])
            hollow_cylinder(b, ob["name"], pos=(0, 0, 0), r_in=ob["r_in"], height=ob["height"], wall=ob["wall"], n=16, rgba=ob.get("rgba", [.2, .55, .8, 1]),
                            mass=ob["mass"], friction=OBJ_FRICTION)
        elif k == "container":  # fixed to the table (bowl, pen holder)
            hollow_cylinder(w, ob["name"], pos=(*ob["pos"][:2], Z), r_in=ob["r_in"], height=ob["height"], wall=ob["wall"], n=ob.get("n", 20),
                            rgba=ob.get("rgba", [.85, .85, .8, 1]), bottom_t=CONTAINER_BOTTOM)
        elif k == "balls":
            cx, cy = ob["pos"][:2]
            for i in range(ob["n"]):
                a = 2 * math.pi * i / max(1, ob["n"] - 1)
                rr = 0 if i == 0 else ob["ring"]
                bz = ob["z0"] + (0 if i == 0 else ob["r"] * 1.2)
                b = w.add_body(name=f"ball{i}", pos=[cx + rr * math.cos(a), cy + rr * math.sin(a), bz])
                b.add_freejoint(name=f"ball{i}")
                b.add_geom(name=f"ball{i}", type=SPH, size=[ob["r"], 0, 0], mass=ob["mass"], rgba=ob.get("rgba", [.95, .35, .2, 1]),
                           friction=[.8, .005, .0002], condim=4)
        elif k == "pen":
            b = w.add_body(name=ob["name"], pos=list(ob["pos"]), quat=ob.get("quat", [1, 0, 0, 0]))
            b.add_freejoint(name=ob["name"])
            b.add_geom(name=ob["name"], type=CAP, size=[ob["r"], ob["half_len"], 0], mass=ob["mass"], rgba=[.1, .1, .12, 1],
                       friction=OBJ_FRICTION, condim=4, priority=1)
            b.add_geom(name=f"{ob['name']}_cap", type=CYL, pos=[0, 0, ob["half_len"] - .01], size=[ob["r"] + .0008, .018, 0],
                       rgba=[.85, .1, .1, 1], contype=0, conaffinity=0, mass=0)
        elif k == "valve":  # hand wheel on a vertical shaft, turning with friction
            px, py = ob["pos"][:2]
            hz = Z + ob["height"]
            w.add_geom(name=f"{ob['name']}_pipe", type=CYL, pos=[px, py, Z + ob["height"] / 2 - .01], size=[.022, ob["height"] / 2 - .01, 0],
                       rgba=[.45, .47, .5, 1])
            b = w.add_body(name=ob["name"], pos=[px, py, hz])
            b.add_joint(name=ob["name"], type=mujoco.mjtJoint.mjJNT_HINGE, axis=[0, 0, 1], damping=ob["damping"], frictionloss=ob["frictionloss"],
                        armature=.002)
            R = ob["r"]
            b.add_geom(name=f"{ob['name']}_hub", type=CYL, size=[.018, .012, 0], rgba=[.75, .1, .1, 1], mass=.05, friction=OBJ_FRICTION)
            for i in range(3):
                a = 2 * math.pi * i / 3
                b.add_geom(name=f"{ob['name']}_spoke{i}", type=CAP, fromto=[0, 0, 0, R * math.cos(a), R * math.sin(a), 0], size=[.007, 0, 0],
                           rgba=[.8, .12, .12, 1], mass=.02, friction=OBJ_FRICTION)
            n = 18
            for i in range(n):
                a0, a1 = 2 * math.pi * i / n, 2 * math.pi * (i + 1) / n
                b.add_geom(name=f"{ob['name']}_rim{i}", type=CAP, fromto=[R * math.cos(a0), R * math.sin(a0), 0, R * math.cos(a1), R * math.sin(a1), 0],
                           size=[.008, 0, 0], rgba=[.8, .12, .12, 1], mass=.01, friction=OBJ_FRICTION)
            b.add_geom(name=f"{ob['name']}_mark", type=BOX, pos=[R, 0, .009], size=[.012, .006, .002], rgba=[.95, .95, .95, 1], contype=0,
                       conaffinity=0, mass=0)
        elif k == "buttons":  # spring-loaded push buttons on a panel
            px, py = ob["pos"][:2]
            ph = ob["panel_h"]
            n = len(ob["colors"])
            span = ob["spacing"] * (n - 1)
            w.add_geom(name="panel", type=BOX, pos=[px, py, Z + ph / 2], size=[.045, span / 2 + .045, ph / 2], rgba=[.25, .26, .3, 1])
            for i, (cname, rgba) in enumerate(ob["colors"]):
                y = py - span / 2 + i * ob["spacing"]
                b = w.add_body(name=f"button_{cname}", pos=[px, y, Z + ph + ob["travel"] + .004])
                b.add_joint(name=f"button_{cname}", type=mujoco.mjtJoint.mjJNT_SLIDE, axis=[0, 0, 1], range=[-ob["travel"], 0], limited=True,
                            stiffness=ob["stiffness"], springref=.01, damping=1.0, solref_limit=[.004, 1])
                b.add_geom(name=f"button_{cname}", type=CYL, size=[ob["r"], .006, 0], rgba=rgba, mass=.01, friction=OBJ_FRICTION)
                w.add_geom(name=f"collar_{cname}", type=CYL, pos=[px, y, Z + ph + .002], size=[ob["r"] + .006, .002, 0], rgba=[.15, .15, .17, 1],
                           contype=0, conaffinity=0)
        else:
            raise ValueError(f"unknown object kind {k!r}")
