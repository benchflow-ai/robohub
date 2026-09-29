"""Scene construction for the humanoid suite (MjSpec): the Menagerie humanoid on a support stand, a floor, a table
and the task's fixtures and objects.

Collision filtering: robot collision geoms get contype 2 / conaffinity 1, the world and objects contype 1 /
conaffinity 3, so the robot touches the table and objects but its own links do not collide with each other (the
Menagerie models ship without self-collision for the same reason).
"""
from __future__ import annotations

import functools

import mujoco
import numpy as np

from ...assets import robot_dir
from ..embodied import lookat_xyaxes
from .robots import ROBOTS

BOX, CYL, SPH, CAP, PLANE = (mujoco.mjtGeom.mjGEOM_BOX, mujoco.mjtGeom.mjGEOM_CYLINDER, mujoco.mjtGeom.mjGEOM_SPHERE,
                             mujoco.mjtGeom.mjGEOM_CAPSULE, mujoco.mjtGeom.mjGEOM_PLANE)
PHYS_DT = .002
ROBOT_CT, ROBOT_CA = 2, 1
WORLD_CT, WORLD_CA = 1, 3


@functools.lru_cache(maxsize=None)
def _stand_qpos(key: str) -> tuple:
    c = ROBOTS[key]
    m = mujoco.MjModel.from_xml_path(str(robot_dir(c["folder"]) / c["xml"]))
    q = m.key(c["keyframe"]).qpos if c["keyframe"] else np.zeros(m.nq)
    return tuple((m.joint(j).name, float(q[m.jnt_qposadr[j]])) for j in range(m.njnt) if m.jnt_type[j] == mujoco.mjtJoint.mjJNT_HINGE)


def stand_qpos(key: str) -> dict[str, float]:
    """Joint name -> standing value (the Menagerie keyframe, or zeros)."""
    out = dict(_stand_qpos(key))
    out.update(ROBOTS[key]["stand"])
    return out


def _robot_spec(key: str, root_z: float | None):
    c = ROBOTS[key]
    s = mujoco.MjSpec.from_file(str(robot_dir(c["folder"]) / c["xml"]))
    root = s.body(c["root"])
    for j in list(root.joints):
        j.delete()  # fixed base: the root is welded to the world (mounted on the stand)
    for k in list(s.keys):
        k.delete()
    for lt in list(s.worldbody.lights):
        lt.delete()
    if root_z is not None:
        root.pos = [0, 0, root_z]
    pitch = c.get("root_pitch", 0.0)
    if pitch:  # mounted leaning forward (rotation about y); the hips compensate so the legs stay vertical
        root.quat = [float(np.cos(pitch / 2)), 0.0, float(np.sin(pitch / 2)), 0.0]
    for g in s.geoms:
        if g.group == 3:
            g.contype, g.conaffinity = ROBOT_CT, ROBOT_CA
            g.condim = 4 if key == "g1" else 3
            g.friction = [1.0, .005, .0001]
        else:
            g.contype, g.conaffinity = 0, 0
    for side, (body, off) in c["ee"].items():
        s.body(body).add_site(name=f"ee_{side}", pos=list(off), size=[.012, 0, 0], rgba=[1, .2, .2, .0], group=5)
    return s


@functools.lru_cache(maxsize=None)
def _stand_kinematics(key: str):
    """One kinematics pass in the standing pose: (root height that puts the lowest foot collision point 3 mm above
    the floor, {body name: (world position, rotation)} with the root at that height)."""
    s = _robot_spec(key, None)
    m = s.compile()
    d = mujoco.MjData(m)
    for n, v in stand_qpos(key).items():
        d.qpos[m.joint(n).qposadr[0]] = v
    mujoco.mj_forward(m, d)
    zmin = 9.0
    for g in range(m.ngeom):
        if m.geom_group[g] != 3:
            continue
        if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH:
            mid = m.geom_dataid[g]
            v = m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid] + m.mesh_vertnum[mid]]
            pts = d.geom_xpos[g] + v @ d.geom_xmat[g].reshape(3, 3).T
            zmin = min(zmin, float(pts[:, 2].min()))
        else:
            c, r = m.geom_aabb[g][:3], m.geom_aabb[g][3:]
            corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * r + c
            pts = d.geom_xpos[g] + corners @ d.geom_xmat[g].reshape(3, 3).T
            zmin = min(zmin, float(pts[:, 2].min()))
    root_z = float(d.xpos[m.body(ROBOTS[key]["root"]).id][2])
    new_z = root_z - zmin + .003
    frames = {m.body(b).name: (d.xpos[b] + np.array([0, 0, new_z - root_z]), d.xmat[b].reshape(3, 3).copy()) for b in range(1, m.nbody)}
    return new_z, frames


def root_height(key: str) -> float:
    return _stand_kinematics(key)[0]


def new_scene(key: str):
    """The robot on its stand in an empty room: returns the MjSpec to which the task adds its table and objects."""
    s = _robot_spec(key, root_height(key))
    s.option.timestep = PHYS_DT
    s.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    s.option.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
    s.option.impratio = 10
    s.visual.global_.offwidth, s.visual.global_.offheight = 1280, 960
    s.visual.headlight.ambient = [.35, .35, .35]
    s.visual.headlight.diffuse = [.55, .55, .55]
    s.visual.map.znear = .01
    s.add_texture(name="floor", type=mujoco.mjtTexture.mjTEXTURE_2D, builtin=mujoco.mjtBuiltin.mjBUILTIN_CHECKER,
                  rgb1=[.55, .55, .58], rgb2=[.48, .48, .52], width=256, height=256)
    s.add_material(name="floor", textures=["", "floor"], texrepeat=[8, 8], texuniform=True, reflectance=.05)
    s.add_texture(name="sky", type=mujoco.mjtTexture.mjTEXTURE_SKYBOX, builtin=mujoco.mjtBuiltin.mjBUILTIN_GRADIENT,
                  rgb1=[.85, .87, .9], rgb2=[.97, .97, .98], width=64, height=64)
    s.add_material(name="wood", rgba=[.62, .46, .3, 1])
    s.add_material(name="stand", rgba=[.18, .19, .21, 1])
    w = s.worldbody
    w.add_light(name="key", pos=[1.5, -1.0, 3.5], dir=[-.3, .2, -1], directional=True, diffuse=[.55, .55, .55], castshadow=True)
    w.add_light(name="fill", pos=[-1, 1.5, 3], dir=[.3, -.3, -1], directional=True, diffuse=[.25, .25, .25], castshadow=False)
    w.add_geom(name="floor", type=PLANE, size=[4, 4, .1], material="floor", contype=WORLD_CT, conaffinity=WORLD_CA)
    # support stand: a floor plate, a post behind the robot and a bracket to the root body (visual; the root is fixed)
    rz = root_height(key)
    w.add_geom(name="stand_plate", type=BOX, pos=[-.38, 0, .01], size=[.16, .16, .01], material="stand", contype=0, conaffinity=0)
    w.add_geom(name="stand_post", type=CYL, pos=[-.38, 0, rz / 2], size=[.03, rz / 2, 0], material="stand", contype=0, conaffinity=0)
    w.add_geom(name="stand_bracket", type=BOX, pos=[-.25, 0, rz], size=[.13, .025, .025], material="stand", contype=0, conaffinity=0)
    return s


def add_table(s, x0: float, x1: float, y0: float, y1: float, h: float, name: str = "table", rgba=None):
    """A table whose top spans [x0, x1] x [y0, y1] at height h (4 cm thick top, four legs)."""
    w = s.worldbody
    cx, cy, hx, hy = (x0 + x1) / 2, (y0 + y1) / 2, (x1 - x0) / 2, (y1 - y0) / 2
    kw = dict(contype=WORLD_CT, conaffinity=WORLD_CA)
    if rgba is None:
        w.add_geom(name=f"{name}_top", type=BOX, pos=[cx, cy, h - .02], size=[hx, hy, .02], material="wood", **kw)
    else:
        w.add_geom(name=f"{name}_top", type=BOX, pos=[cx, cy, h - .02], size=[hx, hy, .02], rgba=rgba, **kw)
    for sx in (-1, 1):
        for sy in (-1, 1):
            w.add_geom(name=f"{name}_leg{sx}{sy}", type=BOX, pos=[cx + sx * (hx - .04), cy + sy * (hy - .04), (h - .04) / 2],
                       size=[.025, .025, (h - .04) / 2], rgba=[.35, .26, .18, 1], **kw)


def add_static(s, name: str, type_, pos, size, rgba, collide: bool = True, quat=None):
    kw = dict(contype=WORLD_CT, conaffinity=WORLD_CA) if collide else dict(contype=0, conaffinity=0)
    g = s.worldbody.add_geom(name=name, type=type_, pos=list(pos), size=list(size), rgba=list(rgba), **kw)
    if quat is not None:
        g.quat = list(quat)
    return g


def add_object(s, name: str, type_, pos, size, mass: float, rgba, friction: float = 1.0, quat=None, extra=()):
    """A free rigid object: one main geom plus optional `extra` geoms (type, pos, size, mass) in the body frame."""
    b = s.worldbody.add_body(name=name, pos=list(pos))
    if quat is not None:
        b.quat = list(quat)
    b.add_freejoint(name=f"{name}_free")
    kw = dict(rgba=list(rgba), friction=[friction, .005, .0001], condim=4, contype=WORLD_CT, conaffinity=WORLD_CA, solref=[.004, 1])
    b.add_geom(name=name, type=type_, size=list(size), mass=mass, **kw)
    for i, (t, p, sz, ms) in enumerate(extra):
        b.add_geom(name=f"{name}_part{i}", type=t, pos=list(p), size=list(sz), mass=ms, **kw)
    return b


def add_cameras(s, key: str, look_at, front_pos, head_body: str | None, head_offset=(.1, 0, .05)):
    w = s.worldbody
    w.add_camera(name="front", pos=list(front_pos), xyaxes=[float(v) for v in lookat_xyaxes(front_pos, look_at).split()], fovy=50)
    top = [look_at[0] + .05, look_at[1], look_at[2] + 1.3]
    w.add_camera(name="top", pos=top, xyaxes=[0, -1, 0, 1, 0, 0], fovy=55)
    side = [look_at[0] + .1, look_at[1] - 1.6, look_at[2] + .5]
    w.add_camera(name="side", pos=side, xyaxes=[float(v) for v in lookat_xyaxes(side, look_at).split()], fovy=50)
    if head_body:
        # a head camera: fixed to the head (or upper torso) body, aimed at the workspace in the standing pose
        pos, R = _stand_kinematics(key)[1][head_body]
        cw = pos + R @ np.array(head_offset, float)
        xy = np.array([float(v) for v in lookat_xyaxes(cw, look_at).split()]).reshape(2, 3)
        local = (R.T @ xy.T).T.ravel()
        s.body(head_body).add_camera(name="head", pos=list(head_offset), xyaxes=[float(v) for v in local], fovy=70)
