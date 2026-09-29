"""Task scenes and success rules of the humanoid suite.

Each task is a `Task` subclass: which robot, the table and fixtures it adds to the scene (`build`), the public scene
facts for `robo observe` (`landmarks`), and the success rule judged from the final simulated state (`check`, which
returns (ok, reason)). World frame: metres, +x forward from the robot, +y to the robot's left, +z up; the robot's
stand is at the origin and the floor at z = 0.
"""
from __future__ import annotations

import math

import mujoco
import numpy as np

from ..embodied import r3
from . import scene as S


def tilt_deg(xmat: np.ndarray) -> float:
    """Angle between the body's z axis and world z."""
    return math.degrees(math.acos(float(np.clip(xmat.reshape(3, 3)[2, 2], -1, 1))))


def add_cabinet(s, *, front_x: float, cy: float, width: float, depth: float, height: float, base_z: float, handle: str,
                handle_y: float = 0.0, travel: float = .16, handle_frac: float = .5):
    """A drawer cabinet standing on a surface at base_z, opening toward the robot (-x).

    Shell: top, bottom, two sides and a back (static). Drawer: a front panel, a tray (bottom, back, two side walls
    5 cm high) and a handle, on a slide joint `drawer_slide` (axis -x, 0 = closed, up to `travel` m open, 6 N s/m
    damping, 1.5 N friction). handle "vertical": a vertical bar (8 cm long, 1.6 cm thick) on two 6 cm standoffs,
    for a wrap grasp; "hook": a horizontal bar (14 cm long) on standoffs 6 cm deep, so a flat hand fits behind it.
    """
    w = s.worldbody
    kw = dict(contype=S.WORLD_CT, conaffinity=S.WORLD_CA)
    shell = [.86, .86, .83, 1]
    t = .012
    x0, x1 = front_x, front_x + depth
    cx = (x0 + x1) / 2
    hw = width / 2
    top = base_z + height
    w.add_geom(name="cab_top", type=S.BOX, pos=[cx, cy, top - t / 2], size=[depth / 2, hw, t / 2], rgba=shell, **kw)
    w.add_geom(name="cab_bottom", type=S.BOX, pos=[cx, cy, base_z + t / 2], size=[depth / 2, hw, t / 2], rgba=shell, **kw)
    for sgn in (-1, 1):
        w.add_geom(name=f"cab_side{sgn}", type=S.BOX, pos=[cx, cy + sgn * (hw - t / 2), base_z + height / 2],
                   size=[depth / 2, t / 2, height / 2], rgba=shell, **kw)
    w.add_geom(name="cab_back", type=S.BOX, pos=[x1 - t / 2, cy, base_z + height / 2], size=[t / 2, hw, height / 2], rgba=shell, **kw)
    # drawer body (origin at the closed front face)
    d = w.add_body(name="drawer", pos=[x0 - .001, cy, base_z + t + .002])
    d.add_joint(name="drawer_slide", type=mujoco.mjtJoint.mjJNT_SLIDE, axis=[-1, 0, 0], range=[0, travel], damping=6,
                frictionloss=1.5, armature=.05)
    wood = [.55, .38, .24, 1]
    iw = hw - t - .004              # tray half width
    dd = depth - t - .01            # tray depth
    fh = height - 2 * t - .006      # front panel height
    d.add_geom(name="drawer_front", type=S.BOX, pos=[-.006, 0, fh / 2], size=[.006, hw + .004, fh / 2 + .004], rgba=wood, mass=.4, **kw)
    d.add_geom(name="drawer_floor", type=S.BOX, pos=[dd / 2, 0, .004], size=[dd / 2, iw, .004], rgba=wood, mass=.3, **kw)
    d.add_geom(name="drawer_back", type=S.BOX, pos=[dd - .005, 0, .03], size=[.005, iw, .03], rgba=wood, mass=.1, **kw)
    for sgn in (-1, 1):
        d.add_geom(name=f"drawer_wall{sgn}", type=S.BOX, pos=[dd / 2, sgn * (iw - .005), .03], size=[dd / 2, .005, .03], rgba=wood,
                   mass=.1, **kw)
    metal = [.25, .25, .27, 1]
    hz = fh * handle_frac
    if handle == "vertical":
        # a tall vertical bar: from low on the drawer face to 11 cm above the cabinet top, on two standoffs
        so = .05
        zb, zt = .01, height - t - .002 + .11
        d.add_geom(name="handle", type=S.CYL, pos=[-.012 - so, handle_y, (zb + zt) / 2], size=[.008, (zt - zb) / 2, 0], rgba=metal,
                   mass=.08, **kw)
        for i, zp in enumerate((.02, fh - .015)):
            d.add_geom(name=f"handle_post{i}", type=S.BOX, pos=[-.012 - so / 2, handle_y, zp], size=[so / 2, .006, .006],
                       rgba=metal, mass=.02, **kw)
        hz = height - t - .002 + .06    # the grip section above the cabinet top
    else:
        so = .065
        d.add_geom(name="handle", type=S.CYL, pos=[-.012 - so, handle_y, hz], size=[.009, .07, 0], quat=[.7071068, .7071068, 0, 0],
                   rgba=metal, mass=.05, **kw)
        for sy in (-1, 1):
            d.add_geom(name=f"handle_post{sy}", type=S.BOX, pos=[-.012 - so / 2, handle_y + sy * .064, hz], size=[so / 2, .006, .006],
                       rgba=metal, mass=.02, **kw)
    return dict(front_x=x0, cy=cy, width=width, depth=depth, height=height, base_z=base_z, tray_half_width=iw, tray_depth=dd, handle=handle,
                handle_standoff=so, handle_z=base_z + t + .002 + hz, handle_y=cy + handle_y, travel=travel)


HANDLE_SHAPE = {"vertical": "vertical bar, 1.6 cm thick, 5 cm in front of the drawer face, rising 11 cm above the cabinet top; "
                            "`center` is the middle of the part above the cabinet top",
                "hook": "horizontal bar along y, 14 cm long, 1.8 cm thick, 6.5 cm in front of the drawer face"}


BUTTON_RGBA = {"green": (.2, .75, .3, 1), "blue": (.2, .4, .9, 1), "yellow": (.95, .8, .1, 1), "white": (.95, .95, .95, 1),
               "red": (.9, .1, .1, 1), "orange": (1, .5, .1, 1)}


def add_button_panel(s, *, face_x: float, y_range, z_range, base_z: float, buttons: dict, radius: float = .025):
    """A vertical panel facing the robot (-x) with spring-loaded push buttons.

    Each button is a cylinder cap (radius `radius`, protruding 1.5 cm) on a slide joint `btn_<name>` along +x, 0 to
    12 mm travel, 400 N/m return spring, and counts as pressed at 8 mm. `buttons`: name -> (y, z) of its centre.
    """
    w = s.worldbody
    kw = dict(contype=S.WORLD_CT, conaffinity=S.WORLD_CA)
    y0, y1 = y_range
    z0, z1 = z_range
    th = .03
    w.add_geom(name="panel", type=S.BOX, pos=[face_x + th / 2, (y0 + y1) / 2, (z0 + z1) / 2], size=[th / 2, (y1 - y0) / 2, (z1 - z0) / 2],
               rgba=[.35, .37, .4, 1], **kw)
    if z0 > base_z + .001:  # two legs down to the surface it stands on
        for yl in (y0 + .03, y1 - .03):
            w.add_geom(name=f"panel_leg{yl:+.2f}", type=S.BOX, pos=[face_x + th / 2, yl, (base_z + z0) / 2],
                       size=[th / 2, .02, (z0 - base_z) / 2], rgba=[.3, .3, .33, 1], **kw)
    for n, (y, z) in buttons.items():
        b = w.add_body(name=f"button_{n}", pos=[face_x - .0075, y, z])
        b.add_joint(name=f"btn_{n}", type=mujoco.mjtJoint.mjJNT_SLIDE, axis=[1, 0, 0], range=[0, .012], stiffness=400, damping=4,
                    springref=0, armature=.01)
        # buttons collide with the robot only (contype 4 / conaffinity 2), so they can sink into the panel
        b.add_geom(name=f"button_{n}", type=S.CYL, size=[radius, .0075, 0], quat=[.7071068, 0, .7071068, 0], rgba=list(BUTTON_RGBA[n]),
                   mass=.02, contype=4, conaffinity=2)
        # a dark bezel ring on the panel face around the button (visual)
        w.add_geom(name=f"bezel_{n}", type=S.CYL, pos=[face_x - .001, y, z], size=[radius + .008, .001, 0], quat=[.7071068, 0, .7071068, 0],
                   rgba=[.12, .12, .13, 1], contype=0, conaffinity=0)


class Task:
    robot = ""
    title = ""
    max_steps = 800
    hand_pose = "default"          # 7-DoF arms: the fixed hand orientation the arm controller holds
    home = {"left": (0.25, 0.2, 0.9), "right": (0.25, -0.2, 0.9)}
    objects: list[str] = []        # free objects reported under `objects`
    fragile: list[str] = []        # objects that must not be touched by the robot or knocked over
    table = dict(x0=.2, x1=.7, y0=-.5, y1=.5, h=.7)
    front_cam = (1.15, .7, 1.35)
    look = (.32, 0, .8)

    def build(self, s) -> None:
        t = self.table
        S.add_table(s, t["x0"], t["x1"], t["y0"], t["y1"], t["h"])

    def landmarks(self, be) -> dict:
        t = self.table
        return {"table": {"top_height": t["h"], "x_range": [t["x0"], t["x1"]], "y_range": [t["y0"], t["y1"]]}}

    def progress(self, be) -> dict | None:
        return None

    def check(self, be) -> tuple[bool, str]:
        raise NotImplementedError

    # helpers -----------------------------------------------------------------------------------------------------
    @staticmethod
    def in_disc(p, c, r) -> bool:
        return float(np.hypot(p[0] - c[0], p[1] - c[1])) <= r


# ---------------------------------------------------------------------------------------------------------------------
# Unitree G1 with Dex3-1 hands
# ---------------------------------------------------------------------------------------------------------------------

G1_TABLE = dict(x0=.17, x1=.62, y0=-.45, y1=.45, h=.72)


class G1CanToPlate(Task):
    robot = "g1"
    title = "Put the can on the plate"
    max_steps = 500
    table = G1_TABLE
    home = {"left": (0.2, 0.2, 0.94), "right": (0.2, -0.2, 0.94)}
    objects = ["can"]
    CAN = dict(pos=(.31, -.15), r=.026, h=.12)
    PLATE = dict(pos=(.33, .06), r=.085)

    def build(self, s):
        super().build(s)
        h = self.table["h"]
        px, py = self.PLATE["pos"]
        S.add_static(s, "plate", S.CYL, (px, py, h + .006), (self.PLATE["r"], .006, 0), (.93, .93, .9, 1))
        c = self.CAN
        S.add_object(s, "can", S.CYL, (c["pos"][0], c["pos"][1], h + c["h"] / 2 + .001), (c["r"], c["h"] / 2, 0), .18, (.8, .12, .1, 1))

    def landmarks(self, be):
        L = super().landmarks(be)
        L["plate"] = {"center": [*self.PLATE["pos"], self.table["h"] + .012], "radius": self.PLATE["r"]}
        return L

    def object_info(self, name):
        return {"diameter": 2 * self.CAN["r"], "height": self.CAN["h"]}

    def check(self, be):
        p = be.obj_pos("can")
        if be.obj_tilt("can") > 15:
            return False, "the can is not upright"
        if not self.in_disc(p, self.PLATE["pos"], self.PLATE["r"] - .015):
            return False, "the can is not on the plate"
        if abs(p[2] - (self.table["h"] + .012 + self.CAN["h"] / 2)) > .01:
            return False, "the can is not resting on the plate"
        if be.robot_touching("can"):
            return False, "the robot is still holding the can"
        return True, "the can stands on the plate"


class G1PassAcross(Task):
    robot = "g1"
    title = "Pass the bottle to the other side"
    max_steps = 900
    table = G1_TABLE
    home = {"left": (0.2, 0.2, 0.94), "right": (0.2, -0.2, 0.94)}
    objects = ["bottle", "glass"]
    fragile = ["glass"]
    BOTTLE = dict(pos=(.32, -.24), r=.025, h=.13)
    COASTER = dict(pos=(.32, .24), r=.06)
    GLASS = dict(pos=(.33, -.1), r=.03, h=.12)

    def build(self, s):
        super().build(s)
        h = self.table["h"]
        px, py = self.COASTER["pos"]
        S.add_static(s, "coaster", S.CYL, (px, py, h + .004), (self.COASTER["r"], .004, 0), (.2, .45, .8, 1))
        b = self.BOTTLE
        S.add_object(s, "bottle", S.CYL, (b["pos"][0], b["pos"][1], h + b["h"] / 2 + .001), (b["r"], b["h"] / 2, 0), .2, (.15, .6, .3, 1))
        g = self.GLASS
        S.add_object(s, "glass", S.CYL, (g["pos"][0], g["pos"][1], h + g["h"] / 2 + .001), (g["r"], g["h"] / 2, 0), .15, (.8, .9, .95, .45))

    def object_info(self, name):
        if name == "glass":
            return {"diameter": 2 * self.GLASS["r"], "height": self.GLASS["h"], "note": "a tall drinking glass (fragile): do not touch it"}
        return {"diameter": 2 * self.BOTTLE["r"], "height": self.BOTTLE["h"]}

    def landmarks(self, be):
        L = super().landmarks(be)
        L["coaster"] = {"center": [*self.COASTER["pos"], self.table["h"] + .008], "radius": self.COASTER["r"]}
        return L

    def check(self, be):
        p = be.obj_pos("bottle")
        if be.obj_tilt("bottle") > 15:
            return False, "the bottle is not upright"
        if not self.in_disc(p, self.COASTER["pos"], self.COASTER["r"]):
            return False, "the bottle is not on the coaster"
        if abs(p[2] - (self.table["h"] + .008 + self.BOTTLE["h"] / 2)) > .005:
            return False, "the bottle is not resting on the coaster"
        if be.robot_touching("bottle"):
            return False, "the robot is still holding the bottle"
        return True, "the bottle stands on the coaster"


class G1DrawerStow(Task):
    robot = "g1"
    title = "Stow the soap in the drawer"
    max_steps = 1200
    table = G1_TABLE
    home = {"left": (0.2, 0.2, 0.94), "right": (0.2, -0.2, 0.94)}
    objects = ["soap"]
    BOX = dict(pos=(.29, .25), half=(.022, .022, .03))
    CAB = dict(front_x=.38, cy=0.0, width=.26, depth=.24, height=.10, handle="vertical", handle_y=-.05, travel=.16)
    front_cam = (1.0, -.75, 1.45)
    def build(self, s):
        super().build(s)
        h = self.table["h"]
        self.cab = add_cabinet(s, base_z=h, **self.CAB)
        b = self.BOX
        S.add_object(s, "soap", S.BOX, (b["pos"][0], b["pos"][1], h + b["half"][2] + .001), b["half"], .1, (.55, .8, .95, 1))

    def opening(self, be) -> float:
        return float(be.data.qpos[be.model.joint("drawer_slide").qposadr[0]])

    def object_info(self, name):
        return {"size": [2 * v for v in self.BOX["half"]], "note": "a bar of soap in a box"}

    def landmarks(self, be):
        L = super().landmarks(be)
        c = self.cab
        o = self.opening(be)
        L["cabinet"] = {"front_x": c["front_x"], "y_range": [c["cy"] - c["width"] / 2, c["cy"] + c["width"] / 2],
                        "top_height": c["base_z"] + c["height"], "note": "the drawer slides out toward the robot (-x)"}
        L["drawer"] = {"opening_m": round(o, 3), "max_opening_m": c["travel"],
                       "handle": {"center": r3([c["front_x"] - .013 - c["handle_standoff"] - o, c["handle_y"], c["handle_z"]]),
                                  "shape": HANDLE_SHAPE[c["handle"]]},
                       "tray_inside": {"x_range": r3([c["front_x"] - o, c["front_x"] - o + c["tray_depth"] - .012]),
                                       "y_range": r3([c["cy"] - c["tray_half_width"] + .015, c["cy"] + c["tray_half_width"] - .015]),
                                       "floor_height": round(c["base_z"] + .012 + .002 + .008, 3), "wall_height": .06,
                                       "note": "only the part in front of the cabinet (x < front_x) is open from above"}}
        return L

    def progress(self, be):
        return {"drawer_opening_m": round(self.opening(be), 3), "soap_in_drawer": self.box_in_drawer(be)}

    def box_in_drawer(self, be) -> bool:
        c = self.cab
        p = be.obj_pos("soap")
        rel = p - be.data.xpos[be.model.body("drawer").id]
        return bool(.0 <= rel[0] <= c["tray_depth"] - .012 and abs(rel[1]) <= c["tray_half_width"] - .015 and rel[2] < .07)

    def check(self, be):
        if not self.box_in_drawer(be):
            return False, "the soap is not inside the drawer"
        if self.opening(be) > .015:
            return False, "the drawer is not closed"
        if be.robot_touching("soap"):
            return False, "the robot is still touching the soap"
        return True, "the soap is in the closed drawer"


# ---------------------------------------------------------------------------------------------------------------------
# Unitree H1 (no hands: each forearm ends in a round fist)
# ---------------------------------------------------------------------------------------------------------------------

H1_TABLE = dict(x0=.18, x1=.68, y0=-.5, y1=.5, h=.92)


class H1BoxToShelf(Task):
    robot = "h1"
    title = "Lift the box onto the shelf with both arms"
    max_steps = 700
    table = H1_TABLE
    home = {"left": (0.25, 0.3, 1.12), "right": (0.25, -0.3, 1.12)}
    front_cam = (1.6, -1.0, 1.75)
    look = (.35, 0, 1.05)
    objects = ["box"]
    BOX = dict(pos=(.34, .17), half=(.08, .11, .08), mass=.5)
    SHELF = dict(center=(.36, -.25), half=(.14, .13), height=.1)

    def build(self, s):
        super().build(s)
        h = self.table["h"]
        sx, sy = self.SHELF["center"]
        hx, hy = self.SHELF["half"]
        S.add_static(s, "shelf", S.BOX, (sx, sy, h + self.SHELF["height"] / 2), (hx, hy, self.SHELF["height"] / 2), (.3, .45, .65, 1))
        b = self.BOX
        S.add_object(s, "box", S.BOX, (b["pos"][0], b["pos"][1], h + b["half"][2] + .001), b["half"], b["mass"], (.75, .55, .3, 1))

    def object_info(self, name):
        b = self.BOX
        return {"size": [2 * v for v in b["half"]], "mass_kg": b["mass"]}

    def landmarks(self, be):
        L = super().landmarks(be)
        sx, sy = self.SHELF["center"]
        hx, hy = self.SHELF["half"]
        L["shelf"] = {"top_center": [sx, sy, self.table["h"] + self.SHELF["height"]], "top_size": [2 * hx, 2 * hy]}
        return L

    def check(self, be):
        p = be.obj_pos("box")
        sx, sy = self.SHELF["center"]
        hx, hy = self.SHELF["half"]
        top = self.table["h"] + self.SHELF["height"]
        if be.obj_tilt("box") > 8:
            return False, "the box is not sitting flat"
        if abs(p[0] - sx) > hx or abs(p[1] - sy) > hy:
            return False, "the box's centre is not over the shelf top"
        if abs(p[2] - (top + self.BOX["half"][2])) > .01:
            return False, "the box is not resting on the shelf"
        if be.robot_touching("box"):
            return False, "the robot is still touching the box"
        return True, "the box rests on the shelf"


class H1PanelSequence(Task):
    robot = "h1"
    title = "Press the panel buttons in order"
    max_steps = 600
    table = H1_TABLE
    home = {"left": (0.25, 0.3, 1.12), "right": (0.25, -0.3, 1.12)}
    front_cam = (-.35, .95, 1.95)
    look = (.4, 0, 1.1)
    FACE_X = .43
    BUTTONS = {"green": (.18, 1.2), "blue": (.18, 1.04), "yellow": (-.18, 1.2), "white": (-.18, 1.04), "red": (0.0, 1.12)}
    SEQUENCE = ["yellow", "green", "white", "blue"]
    FORBIDDEN = "red"

    def build(self, s):
        super().build(s)
        add_button_panel(s, face_x=self.FACE_X, y_range=(-.36, .36), z_range=(.96, 1.3), base_z=self.table["h"], buttons=self.BUTTONS)

    def landmarks(self, be):
        L = super().landmarks(be)
        L["panel"] = {"face_x": self.FACE_X, "note": "vertical panel facing the robot; buttons protrude 1.5 cm and travel 12 mm (+x); "
                                                     "a button counts as pressed at 8 mm",
                      "buttons": {n: {"center": [round(self.FACE_X - .015, 3), y, z], "diameter": .05} for n, (y, z) in self.BUTTONS.items()}}
        return L

    def depth(self, be, n) -> float:
        return float(be.data.qpos[be.model.joint(f"btn_{n}").qposadr[0]])

    def monitor(self, be):
        st = be.task_state
        down = st.setdefault("down", set())
        log = st.setdefault("log", [])
        for n in self.BUTTONS:
            d = self.depth(be, n)
            if d > .008 and n not in down:
                down.add(n)
                log.append(n)
                if n == self.FORBIDDEN:
                    be.event("pressed:red", "safety", "the red emergency-stop button was pressed")
            elif d < .004 and n in down:
                down.discard(n)

    def progress(self, be):
        return {"pressed_so_far": list(be.task_state.get("log", []))}

    def check(self, be):
        log = be.task_state.get("log", [])
        if log != self.SEQUENCE:
            return False, f"pressed {log}, expected {self.SEQUENCE}"
        return True, "the buttons were pressed in the required order"


# ---------------------------------------------------------------------------------------------------------------------
# Apptronik Apollo (rigid hand plates)
# ---------------------------------------------------------------------------------------------------------------------

APOLLO_TABLE = dict(x0=.16, x1=.76, y0=-.5, y1=.5, h=.88)


class ApolloCarryTray(Task):
    robot = "apollo"
    title = "Carry the tray with the cup to the mat"
    max_steps = 700
    table = APOLLO_TABLE
    hand_pose = "palms_in"
    home = {"left": (0.3, 0.3, 1.05), "right": (0.3, -0.3, 1.05)}
    front_cam = (1.3, -.8, 1.6)
    look = (.4, 0, .95)
    objects = ["tray", "cup"]
    TRAY = dict(pos=(.3, 0.0), half=(.1, .16, .006), rim=.07, wall=.006, mass=.4)
    CUP = dict(r=.03, h=.09, mass=.15)
    MAT = dict(center=(.56, 0.0), half=(.14, .21))

    def build(self, s):
        super().build(s)
        h = self.table["h"]
        mx, my = self.MAT["center"]
        S.add_static(s, "mat", S.BOX, (mx, my, h + .001), (*self.MAT["half"], .001), (.2, .5, .75, 1), collide=False)
        T = self.TRAY
        hx, hy, hz = T["half"]
        w, R = T["wall"] / 2, T["rim"] / 2
        rims = [(S.BOX, (0, sg * (hy - w), hz + R), (hx, w, R), .03) for sg in (-1, 1)] + \
               [(S.BOX, (sg * (hx - w), 0, hz + R), (w, hy - 2 * w, R), .02) for sg in (-1, 1)]
        S.add_object(s, "tray", S.BOX, (T["pos"][0], T["pos"][1], h + hz + .001), T["half"], T["mass"], (.85, .85, .8, 1), extra=rims)
        c = self.CUP
        S.add_object(s, "cup", S.CYL, (T["pos"][0], T["pos"][1], h + 2 * hz + c["h"] / 2 + .002), (c["r"], c["h"] / 2, 0), c["mass"],
                     (.9, .35, .2, 1))

    def object_info(self, name):
        T = self.TRAY
        if name == "tray":
            return {"size": [2 * T["half"][0], 2 * T["half"][1], round(2 * T["half"][2] + T["rim"], 3)], "mass_kg": T["mass"] + .1,
                    "note": "flat tray with 7 cm high sides all round"}
        return {"size": [2 * self.CUP["r"], 2 * self.CUP["r"], self.CUP["h"]], "note": "a cup standing loose on the tray"}

    def landmarks(self, be):
        L = super().landmarks(be)
        mx, my = self.MAT["center"]
        L["mat"] = {"center": [mx, my, self.table["h"]], "size": [2 * v for v in self.MAT["half"]]}
        return L

    def monitor(self, be):
        tray = be.obj_geoms["tray"]
        cup = be.obj_geoms["cup"]
        for a, b in be.contacts():
            if (a in cup and b not in tray) or (b in cup and a not in tray):
                be.event("cup_off", "dropped", "the cup touched something other than the tray (fell out or was hit)")

    def check(self, be):
        p = be.obj_pos("tray")
        mx, my = self.MAT["center"]
        hx, hy = self.MAT["half"]
        T = self.TRAY
        if abs(p[0] - mx) > hx - T["half"][0] or abs(p[1] - my) > hy - T["half"][1]:
            return False, "the tray is not entirely on the mat"
        if be.obj_tilt("tray") > 5 or p[2] > self.table["h"] + T["half"][2] + .01:
            return False, "the tray is not resting flat on the table"
        if be.obj_tilt("cup") > 10:
            return False, "the cup is not standing upright"
        if be.robot_touching("tray") or be.robot_touching("cup"):
            return False, "the robot is still touching the tray"
        return True, "the tray with the cup rests on the mat"


class ApolloBoxIntoBin(Task):
    robot = "apollo"
    title = "Put the box into the bin with both hands"
    max_steps = 700
    table = APOLLO_TABLE
    hand_pose = "palms_in"
    home = {"left": (0.3, 0.3, 1.05), "right": (0.3, -0.3, 1.05)}
    front_cam = (1.3, .8, 1.6)
    look = (.4, 0, .95)
    objects = ["box"]
    BOX = dict(pos=(.3, 0.0), half=(.07, .07, .08), mass=.6)
    BIN = dict(center=(.54, 0.0), inner_half=(.1, .12), wall=.012, height=.07)

    def build(self, s):
        super().build(s)
        h = self.table["h"]
        bx, by = self.BIN["center"]
        ix, iy = self.BIN["inner_half"]
        w, H = self.BIN["wall"], self.BIN["height"]
        col = (.25, .45, .35, 1)
        S.add_static(s, "bin_floor", S.BOX, (bx, by, h + .004), (ix + w, iy + w, .004), col)
        for sg in (-1, 1):
            S.add_static(s, f"bin_wall_x{sg}", S.BOX, (bx + sg * (ix + w / 2), by, h + H / 2), (w / 2, iy + w, H / 2), col)
            S.add_static(s, f"bin_wall_y{sg}", S.BOX, (bx, by + sg * (iy + w / 2), h + H / 2), (ix, w / 2, H / 2), col)
        b = self.BOX
        S.add_object(s, "box", S.BOX, (b["pos"][0], b["pos"][1], h + b["half"][2] + .001), b["half"], b["mass"], (.8, .3, .25, 1))

    def object_info(self, name):
        b = self.BOX
        return {"size": [2 * v for v in b["half"]], "mass_kg": b["mass"]}

    def landmarks(self, be):
        L = super().landmarks(be)
        bx, by = self.BIN["center"]
        ix, iy = self.BIN["inner_half"]
        L["bin"] = {"center": [bx, by, self.table["h"]], "inside_size": [2 * ix, 2 * iy], "wall_height": self.BIN["height"],
                    "floor_height": round(self.table["h"] + .008, 3)}
        return L

    def check(self, be):
        p = be.obj_pos("box")
        bx, by = self.BIN["center"]
        ix, iy = self.BIN["inner_half"]
        if abs(p[0] - bx) > ix or abs(p[1] - by) > iy:
            return False, "the box is not inside the bin"
        if p[2] > self.table["h"] + .008 + self.BOX["half"][2] + .02:
            return False, "the box is not resting on the bin floor"
        if be.robot_touching("box"):
            return False, "the robot is still touching the box"
        return True, "the box is in the bin"


# ---------------------------------------------------------------------------------------------------------------------
# Booster T1 (4-DoF arms ending in rigid cylindrical hands)
# ---------------------------------------------------------------------------------------------------------------------

T1_TABLE = dict(x0=.16, x1=.6, y0=-.45, y1=.45, h=.56)


class T1PushPucks(Task):
    robot = "t1"
    title = "Push each can into its matching zone"
    max_steps = 700
    table = T1_TABLE
    home = {"left": (0.2, 0.2, 0.72), "right": (0.2, -0.2, 0.72)}
    front_cam = (1.2, .7, 1.25)
    look = (.3, 0, .6)
    objects = ["red_can", "blue_can"]
    PUCKS = {"red_can": ((.2, -.1), (.85, .15, .15, 1)), "blue_can": ((.2, .1), (.15, .3, .85, 1))}
    ZONES = {"red_can": ((.32, -.2), (.95, .35, .3, 1)), "blue_can": ((.33, .05), (.35, .55, 1, 1))}
    R, H, ZR = .03, .06, .04

    def build(self, s):
        super().build(s)
        h = self.table["h"]
        for n, ((x, y), rgba) in self.ZONES.items():
            S.add_static(s, f"zone_{n}", S.CYL, (x, y, h + .0005), (self.ZR, .0005, 0), rgba, collide=False)
        for n, ((x, y), rgba) in self.PUCKS.items():
            S.add_object(s, n, S.CYL, (x, y, h + self.H / 2 + .001), (self.R, self.H / 2, 0), .15, rgba, friction=.35)

    def object_info(self, name):
        return {"diameter": 2 * self.R, "height": self.H}

    def landmarks(self, be):
        L = super().landmarks(be)
        L["zones"] = {n.replace("_can", ""): {"center": [x, y, self.table["h"]], "radius": self.ZR} for n, ((x, y), _) in self.ZONES.items()}
        return L

    def check(self, be):
        for n, ((x, y), _) in self.ZONES.items():
            p = be.obj_pos(n)
            if not self.in_disc(p, (x, y), self.ZR):
                return False, f"the {n.replace('_', ' ')}'s centre is not in its zone"
            if be.obj_tilt(n) > 15:
                return False, f"the {n.replace('_', ' ')} is not standing"
        return True, "both cans stand in their zones"


class T1BoxOntoStep(Task):
    robot = "t1"
    title = "Lift the box onto the step with both hands"
    max_steps = 600
    table = T1_TABLE
    home = {"left": (0.2, 0.2, 0.72), "right": (0.2, -0.2, 0.72)}
    front_cam = (1.2, -.7, 1.25)
    look = (.3, 0, .6)
    objects = ["box"]
    BOX = dict(pos=(.22, 0.0), half=(.05, .07, .05), mass=.3)
    STEP = dict(center=(.36, 0.0), half=(.07, .12), height=.06)

    def build(self, s):
        super().build(s)
        h = self.table["h"]
        sx, sy = self.STEP["center"]
        hx, hy = self.STEP["half"]
        S.add_static(s, "step", S.BOX, (sx, sy, h + self.STEP["height"] / 2), (hx, hy, self.STEP["height"] / 2), (.35, .5, .35, 1))
        b = self.BOX
        S.add_object(s, "box", S.BOX, (b["pos"][0], b["pos"][1], h + b["half"][2] + .001), b["half"], b["mass"], (.75, .55, .3, 1))

    def object_info(self, name):
        b = self.BOX
        return {"size": [2 * v for v in b["half"]], "mass_kg": b["mass"]}

    def landmarks(self, be):
        L = super().landmarks(be)
        sx, sy = self.STEP["center"]
        hx, hy = self.STEP["half"]
        L["step"] = {"top_center": [sx, sy, self.table["h"] + self.STEP["height"]], "top_size": [2 * hx, 2 * hy]}
        return L

    def check(self, be):
        p = be.obj_pos("box")
        sx, sy = self.STEP["center"]
        hx, hy = self.STEP["half"]
        top = self.table["h"] + self.STEP["height"]
        if be.obj_tilt("box") > 8:
            return False, "the box is not sitting flat"
        if abs(p[0] - sx) > hx or abs(p[1] - sy) > hy:
            return False, "the box's centre is not over the step"
        if abs(p[2] - (top + self.BOX["half"][2])) > .01:
            return False, "the box is not resting on the step"
        if be.robot_touching("box"):
            return False, "the robot is still touching the box"
        return True, "the box rests on the step"

TASKS: dict[str, Task] = {
    "g1-can-to-plate": G1CanToPlate(),
    "g1-pass-across": G1PassAcross(),
    "g1-drawer-stow": G1DrawerStow(),
    "h1-box-to-shelf": H1BoxToShelf(),
    "h1-panel-sequence": H1PanelSequence(),
    "apollo-carry-tray": ApolloCarryTray(),
    "apollo-box-into-bin": ApolloBoxIntoBin(),
    "t1-push-cans": T1PushPucks(),
    "t1-box-onto-step": T1BoxOntoStep(),
}
