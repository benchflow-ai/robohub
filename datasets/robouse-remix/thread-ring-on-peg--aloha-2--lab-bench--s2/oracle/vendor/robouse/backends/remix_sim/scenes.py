"""Remix scenes: environment, fixtures, work surfaces, object slots and the placements (mounts) robots can use.

Each scene is a `Layout`: rooms or open ground, furniture as static boxes, named work surfaces with their heights and
the sides a floor robot can reach them from, and the placements it offers:

  arm       a surface-mounted single arm (base pose on a work surface) and the part of the surface it works on
  bimanual  two arm bases side by side on a work surface, and their shared work area
  floor     a start pose and the navigable area for wheeled and legged robots
  airspace  a start pad and a ceiling for drones

`build(spec, variant)` adds everything to an MjSpec. `variant` (from the episode seed) picks the lighting and the
floor and surface textures (randomisation that never changes geometry).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

from ..embodied import lookat_xyaxes
from .world import box


@dataclass
class Surface:
    name: str
    label: str
    c: tuple[float, float]
    half: tuple[float, float]
    top: float
    approach: tuple[str, ...] = ()   # sides a floor robot can reach it from: "s", "n", "w", "e"
    rgba: tuple = (.55, .4, .26, 1)
    legs: bool = True                # a table on four legs; False: a solid block (counter, bench, rack)

    def contains(self, x: float, y: float, margin: float = 0.0) -> bool:
        return abs(x - self.c[0]) <= self.half[0] - margin and abs(y - self.c[1]) <= self.half[1] - margin

    def public(self) -> dict:
        return {"label": self.label, "top_center": [self.c[0], self.c[1], self.top], "top_size_m": [2 * self.half[0], 2 * self.half[1]],
                **({"reachable_from": [SIDE_NAMES[s] for s in self.approach]} if self.approach else {})}


SIDE_NAMES = {"s": "south (-y)", "n": "north (+y)", "w": "west (-x)", "e": "east (+x)"}
SIDE_VEC = {"s": (0, -1), "n": (0, 1), "w": (-1, 0), "e": (1, 0)}


@dataclass
class Obstacle:
    name: str
    c: tuple[float, float]
    half: tuple[float, float]
    h: float
    label: str = ""
    rgba: tuple = (.7, .7, .72, 1)


@dataclass
class ArmMount:
    surface: str
    bases: dict            # arm name -> (x, y, yaw_deg): "arm" for a single arm, "left" / "right" for a pair
    work: tuple            # (x0, x1, y0, y1) on the surface where task objects go


@dataclass
class Layout:
    id: str
    title: str
    scenario: str
    bounds: tuple                       # (x0, x1, y0, y1) of the floor area (walls or a fence around it)
    ceiling: float
    surfaces: dict[str, Surface]
    obstacles: list[Obstacle]
    mounts: dict                        # "arm" / "bimanual" -> ArmMount, "floor" -> dict(start=(x, y, yaw)), "airspace" -> dict(pad=(x, y))
    floor_slots: list = field(default_factory=list)    # free floor areas (x0, x1, y0, y1) for floor objects and mats
    tag_spots: list = field(default_factory=list)      # (x, y, z, nx, ny, label, post)
    object_kinds: list = field(default_factory=list)   # object kinds this scene stocks
    containers: list = field(default_factory=list)     # container kinds it stocks
    walls: bool = True
    floor_rgba: tuple = (.6, .6, .6)
    wall_rgba: tuple = (.92, .91, .88, .35)
    cams: list = field(default_factory=list)          # (name, pos, target, fovy)
    decor: list = field(default_factory=list)         # extra visual boxes (name, c, half, rgba)
    ground_texture: str = "checker"

    def public(self) -> dict:
        x0, x1, y0, y1 = self.bounds
        out = {"scene": self.id, "scenario": self.scenario, "floor_area": {"x_range": [x0, x1], "y_range": [y0, y1]},
               "surfaces": {k: s.public() for k, s in self.surfaces.items()},
               "obstacles": {o.name: {"label": o.label or o.name.replace("_", " "), "center": list(o.c),
                                      "size_m": [2 * o.half[0], 2 * o.half[1], o.h]} for o in self.obstacles}}
        out["walls"] = (f"walls enclose x {x0:g}..{x1:g}, y {y0:g}..{y1:g} ({self.ceiling:g} m to the ceiling)" if self.walls else
                        f"open ground; stay inside x {x0:g}..{x1:g}, y {y0:g}..{y1:g}")
        return out

    def footprints(self) -> list[tuple]:
        """Every static footprint on the floor (x0, x1, y0, y1, height): surfaces and obstacles."""
        out = [(s.c[0] - s.half[0], s.c[0] + s.half[0], s.c[1] - s.half[1], s.c[1] + s.half[1], s.top) for s in self.surfaces.values()]
        out += [(o.c[0] - o.half[0], o.c[0] + o.half[0], o.c[1] - o.half[1], o.c[1] + o.half[1], o.h) for o in self.obstacles]
        return out

    # ---- building ------------------------------------------------------------------------------------------
    def build(self, s, variant: int = 0, mount: str | None = None) -> None:
        import mujoco

        s.visual.global_.offwidth, s.visual.global_.offheight = 1280, 960
        s.visual.headlight.ambient = [.42, .42, .42]
        s.visual.headlight.diffuse = [.4, .4, .4]
        s.visual.headlight.specular = [0, 0, 0]
        s.visual.map.znear = .01
        rng_tint = [(1.0, 1.0, 1.0), (1.04, 1.0, .94), (.94, .97, 1.04)][variant % 3]
        f1 = [min(1.0, c * t) for c, t in zip(self.floor_rgba, rng_tint)]
        f2 = [c * .93 for c in f1]
        builtin = mujoco.mjtBuiltin.mjBUILTIN_CHECKER if self.ground_texture == "checker" else mujoco.mjtBuiltin.mjBUILTIN_FLAT
        s.add_texture(name="floor", type=mujoco.mjtTexture.mjTEXTURE_2D, builtin=builtin, rgb1=f1, rgb2=f2, width=256, height=256,
                      **({"mark": mujoco.mjtMark.mjMARK_RANDOM, "random": .04, "markrgb": [c * .85 for c in f1]}
                         if self.ground_texture == "grass" else {}))
        x0, x1, y0, y1 = self.bounds
        s.add_material(name="floor", textures=["", "floor"], texrepeat=[max(1, round(x1 - x0) * 2), max(1, round(y1 - y0) * 2)])
        s.add_texture(name="sky", type=mujoco.mjtTexture.mjTEXTURE_SKYBOX, builtin=mujoco.mjtBuiltin.mjBUILTIN_GRADIENT,
                      rgb1=[.82, .86, .92] if not self.walls else [.85, .86, .88], rgb2=[.97, .97, .98], width=64, height=64)
        w = s.worldbody
        light_dir = [[0, 0, -1], [.3, .2, -1], [-.25, .3, -1]][variant % 3]
        w.add_light(name="sun", pos=[0, 0, 8], dir=light_dir, directional=True, diffuse=[.5, .5, .5], castshadow=False)
        cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
        w.add_geom(name="floor", type=mujoco.mjtGeom.mjGEOM_PLANE, pos=[cx, cy, 0], size=[(x1 - x0) / 2 + 3, (y1 - y0) / 2 + 3, .1],
                   material="floor", friction=[1, .005, .0001])
        T, H = .1, min(self.ceiling, 2.6)
        if self.walls:  # drawn see-through (collisions are unchanged), so outside cameras can look in
            box(w, "wall_west", (x0 - T / 2, cy, H / 2), (T / 2, (y1 - y0) / 2 + T, H / 2), self.wall_rgba)
            box(w, "wall_east", (x1 + T / 2, cy, H / 2), (T / 2, (y1 - y0) / 2 + T, H / 2), self.wall_rgba)
            box(w, "wall_south", (cx, y0 - T / 2, H / 2), ((x1 - x0) / 2, T / 2, H / 2), self.wall_rgba)
            box(w, "wall_north", (cx, y1 + T / 2, H / 2), ((x1 - x0) / 2, T / 2, H / 2), self.wall_rgba)
            box(w, "ceiling", (cx, cy, self.ceiling + .05), ((x1 - x0) / 2, (y1 - y0) / 2, .05), (0, 0, 0, 0))
        else:  # a low fence marks the boundary (visual only)
            post = (.55, .45, .32, 1)
            box(w, "fence_w", (x0, cy, .3), (.02, (y1 - y0) / 2, .02), post, collide=False)
            box(w, "fence_e", (x1, cy, .3), (.02, (y1 - y0) / 2, .02), post, collide=False)
            box(w, "fence_s", (cx, y0, .3), ((x1 - x0) / 2, .02, .02), post, collide=False)
            box(w, "fence_n", (cx, y1, .3), ((x1 - x0) / 2, .02, .02), post, collide=False)
        for sname, sf in self.surfaces.items():
            _surface(w, sname, sf)
        for o in self.obstacles:
            box(w, o.name, (o.c[0], o.c[1], o.h / 2), (o.half[0], o.half[1], o.h / 2), o.rgba)
        for name, c, half, rgba in self.decor:
            box(w, name, c, half, rgba, collide=False)
        for m in ("arm", "bimanual"):
            am = self.mounts.get(m) if m == mount else None
            if am:
                top = self.surfaces[am.surface].top
                for side, (bx, by, _) in am.bases.items():
                    box(w, f"mount_plate_{m}_{side}", (bx, by, top + .0075), (.1, .1, .0075), (.3, .33, .37, 1))
        for name, pos, tgt, fovy in self.cams:
            w.add_camera(name=name, pos=list(pos), xyaxes=[float(v) for v in lookat_xyaxes(pos, tgt).split()], fovy=fovy)


def _surface(w, name: str, sf: Surface) -> None:
    (cx, cy), (hx, hy), h = sf.c, sf.half, sf.top
    if sf.legs:
        box(w, f"{name}_top", (cx, cy, h - .02), (hx, hy, .02), sf.rgba)
        for i, (dx, dy) in enumerate(((1, 1), (1, -1), (-1, 1), (-1, -1))):
            box(w, f"{name}_leg{i}", (cx + dx * (hx - .04), cy + dy * (hy - .04), (h - .04) / 2), (.025, .025, (h - .04) / 2), sf.rgba)
    else:
        box(w, f"{name}_top", (cx, cy, h - .015), (hx, hy, .015), sf.rgba)
        body = tuple(min(1.0, c * 1.25) for c in sf.rgba[:3]) + (1,)
        box(w, f"{name}_body", (cx, cy, (h - .03) / 2), (hx - .01, hy - .01, (h - .03) / 2), body)


def _cam(name, pos, tgt, fovy=60):
    return (name, pos, tgt, fovy)


# ------------------------------------------------------------------------------------------------------------------
# the scenes
# ------------------------------------------------------------------------------------------------------------------

WOOD = (.58, .42, .27, 1)
LIGHT_WOOD = (.74, .62, .46, 1)
STEEL = (.62, .64, .67, 1)
LAMINATE = (.82, .82, .8, 1)


def tabletop() -> Layout:
    table = Surface("table", "the table", (0.0, 0.0), (1.0, .4), .75, approach=("s",), rgba=WOOD)
    return Layout(
        id="tabletop", title="Tabletop", scenario="tabletop",
        bounds=(-2.5, 2.5, -2.2, 1.6), ceiling=2.6,
        surfaces={"table": table},
        obstacles=[],
        mounts={"arm": ArmMount("table", {"arm": (0.0, .3, -90.0)}, (-.4, .4, -.32, -.02)),
                "bimanual": ArmMount("table", {"left": (-.4, .2, -90.0), "right": (.4, .2, -90.0)}, (-.75, .75, -.15, .05)),
                "floor": dict(start=(0.0, -1.5, 90.0)),
                "airspace": dict(pad=(-1.8, -1.7))},
        floor_slots=[(-2.2, 2.2, -1.9, -.8), (-2.2, -1.3, -.8, 1.3), (1.3, 2.2, -.8, 1.3)],
        tag_spots=[(2.45, -.4, .55, -1, 0, "on the east wall", False), (-2.45, -.4, .55, 1, 0, "on the west wall", False)],
        object_kinds=["block", "small_block", "large_block", "tall_block", "can", "cup", "ring", "crate", "carton"],
        containers=["bowl", "tray"],
        floor_rgba=(.55, .56, .58),
        cams=[_cam("overview", (0, -2.1, 2.3), (0, -.2, .6), 62), _cam("front", (0, -1.25, 1.35), (0, -.05, .75), 55),
              _cam("top", (0, -.1, 2.0), (0, -.1, .75), 50)],
    )


def kitchen() -> Layout:
    counter = Surface("counter", "the kitchen counter", (-.4, 1.85), (1.5, .35), .9, approach=("s",), rgba=(.36, .36, .4, 1), legs=False)
    dining = Surface("dining_table", "the dining table", (.4, -.7), (.6, .4), .75, approach=("s", "n", "w", "e"), rgba=WOOD)
    return Layout(
        id="kitchen-counter", title="Kitchen counter", scenario="household-kitchen",
        bounds=(-2.5, 2.5, -2.2, 2.45), ceiling=2.5,
        surfaces={"counter": counter, "dining_table": dining},
        obstacles=[Obstacle("fridge", (1.95, 1.95), (.35, .35), 1.8, "the fridge", (.9, .9, .92, 1)),
                   Obstacle("chair", (-.55, -.7), (.2, .2), .45, "a chair", (.45, .32, .2, 1))],
        mounts={"arm": ArmMount("counter", {"arm": (-.4, 2.05, -90.0)}, (-.85, .05, 1.56, 1.73)),
                "bimanual": ArmMount("counter", {"left": (-.8, 2.0, -90.0), "right": (0.0, 2.0, -90.0)}, (-1.15, .35, 1.6, 1.85)),
                "floor": dict(start=(-.6, .35, 0.0)),
                "airspace": dict(pad=(-2.0, -1.7))},
        floor_slots=[(-2.2, -1.2, -1.9, -.9), (1.3, 2.2, -1.9, -.9), (1.2, 2.2, .1, .9)],
        tag_spots=[(2.45, .6, .7, -1, 0, "on the east wall next to the fridge", False),
                   (-2.45, -.2, .7, 1, 0, "on the west wall", False)],
        object_kinds=["block", "tall_block", "can", "cup", "crate", "carton"],
        containers=["bowl", "tray", "tote"],
        floor_rgba=(.66, .6, .52), wall_rgba=(.95, .93, .86,.35),
        decor=[("backsplash", (-.4, 2.44, 1.2), (1.5, .01, .3), (.85, .88, .9, 1)),
               ("stove", (.75, 1.8, .902), (.25, .22, .002), (.12, .12, .12, 1)),
               ("wall_cabinets", (-.4, 2.3, 1.95), (1.5, .15, .3), (.8, .78, .72, 1))],
        cams=[_cam("overview", (0, -2.0, 2.35), (0, .2, .5), 70), _cam("counter_cam", (-.4, .45, 1.7), (-.4, 1.75, .9), 62),
              _cam("top", (0, -.1, 4.6), (0, -.1, 0), 60)],
    )


def workshop() -> Layout:
    bench = Surface("workbench", "the workbench", (0.0, 1.25), (1.0, .35), .9, approach=("s",), rgba=LIGHT_WOOD, legs=False)
    cart = Surface("parts_cart", "the parts cart", (1.7, -.6), (.35, .3), .8, approach=("s", "w", "n"), rgba=STEEL)
    return Layout(
        id="workshop-bench", title="Workshop bench", scenario="workshop-assembly",
        bounds=(-2.6, 2.6, -2.2, 1.95), ceiling=2.8,
        surfaces={"workbench": bench, "parts_cart": cart},
        obstacles=[Obstacle("tool_cabinet", (-2.15, .2), (.3, .45), 1.5, "the tool cabinet", (.75, .2, .15, 1)),
                   Obstacle("drill_press", (-1.2, -1.2), (.25, .25), 1.2, "the drill press", (.3, .35, .4, 1))],
        mounts={"arm": ArmMount("workbench", {"arm": (0.0, 1.52, -90.0)}, (-.42, .42, 1.0, 1.2)),
                "bimanual": ArmMount("workbench", {"left": (-.4, 1.5, -90.0), "right": (.4, 1.5, -90.0)}, (-.75, .75, 1.1, 1.35)),
                "floor": dict(start=(.3, -1.4, 90.0)),
                "airspace": dict(pad=(2.1, -1.8))},
        floor_slots=[(-2.2, -.2, -1.8, .4), (.2, 2.2, -1.8, .4)],
        tag_spots=[(2.55, .6, .6, -1, 0, "on the east wall", False), (-1.84, .2, .7, 1, 0, "on the front of the tool cabinet", False)],
        object_kinds=["block", "small_block", "large_block", "tall_block", "bolt", "ring", "crate", "carton"],
        containers=["tote", "tray"],
        floor_rgba=(.5, .52, .54), wall_rgba=(.86, .87, .85,.35),
        decor=[("pegboard", (0.0, 1.93, 1.35), (1.0, .015, .35), (.8, .68, .5, 1))],
        cams=[_cam("overview", (0, -2.0, 2.4), (0, .3, .6), 68), _cam("bench_cam", (0, .1, 1.75), (0, 1.2, .9), 62),
              _cam("top", (0, -.2, 4.6), (0, -.2, 0), 60)],
    )


def warehouse() -> Layout:
    station = Surface("packing_station", "the packing station", (2.2, -1.6), (.7, .35), .85, approach=("s",), rgba=LIGHT_WOOD)
    rack_lo = Surface("rack_a", "the low shelf of rack A", (-1.5, 1.2), (1.2, .3), .6, approach=("s",), rgba=(.25, .4, .65, 1), legs=False)
    rack_hi = Surface("rack_b", "the middle shelf of rack B", (1.8, 1.2), (1.0, .3), .95, approach=("s",), rgba=(.25, .4, .65, 1), legs=False)
    return Layout(
        id="warehouse", title="Warehouse aisle", scenario="warehouse",
        bounds=(-3.5, 3.5, -2.6, 2.0), ceiling=4.0,
        surfaces={"packing_station": station, "rack_a": rack_lo, "rack_b": rack_hi},
        obstacles=[Obstacle("pallet_stack", (-.6, -1.2), (.5, .4), .9, "a stack of pallets", (.6, .45, .28, 1)),
                   Obstacle("column", (.3, .2), (.15, .15), 4.0, "a steel column", (.55, .56, .6, 1))],
        mounts={"arm": ArmMount("packing_station", {"arm": (2.2, -1.32, -90.0)}, (1.8, 2.6, -1.85, -1.64)),
                "floor": dict(start=(-2.6, -1.8, 0.0)),
                "airspace": dict(pad=(-3.0, -2.2))},
        floor_slots=[(-3.2, -1.8, -.6, .5), (-.2, 1.2, -2.3, -1.3), (1.0, 3.2, -.6, .3)],
        tag_spots=[(3.45, -.2, .8, -1, 0, "on the east wall", False), (.3, .04, .9, 0, -1, "on the steel column", False),
                   (-1.5, .88, 1.1, 0, -1, "on rack A above its low shelf", False)],
        object_kinds=["parcel", "tall_block", "can", "block", "crate", "carton"],
        containers=["tote"],
        floor_rgba=(.62, .62, .6), wall_rgba=(.8, .82, .84,.35),
        decor=[("aisle_line_s", (0, -.35, .001), (3.4, .03, .001), (.95, .8, .1, 1)),
               ("rack_a_upright_w", (-2.7, 1.2, 1.5), (.04, .3, .9), (.2, .3, .5, 1)),
               ("rack_a_upright_e", (-.3, 1.2, 1.5), (.04, .3, .9), (.2, .3, .5, 1))],
        cams=[_cam("overview", (0, -2.5, 3.2), (0, .0, .4), 72), _cam("station_cam", (2.2, -2.5, 1.7), (2.2, -1.5, .85), 60),
              _cam("top", (0, -.3, 6.5), (0, -.3, 0), 62)],
    )


def outdoor() -> Layout:
    return Layout(
        id="outdoor-field", title="Outdoor field", scenario="outdoor-field",
        bounds=(-5.0, 5.0, -4.0, 4.0), ceiling=6.0, walls=False,
        surfaces={},
        obstacles=[Obstacle("shed", (-3.5, 2.6), (1.0, .8), 2.2, "the tool shed", (.55, .35, .25, 1)),
                   Obstacle("rock", (1.2, 1.4), (.35, .3), .5, "a boulder", (.45, .45, .44, 1)),
                   Obstacle("solar_panel", (3.2, -2.4), (.9, .5), 1.1, "the solar panel array", (.15, .2, .35, 1)),
                   Obstacle("tree", (-1.4, -2.6), (.2, .2), 3.0, "a tree trunk", (.35, .25, .15, 1))],
        mounts={"floor": dict(start=(-3.8, -3.2, 45.0)), "airspace": dict(pad=(-4.2, -3.4))},
        floor_slots=[(-2.5, -.5, -1.5, .5), (.5, 2.8, -1.2, .4), (2.6, 4.4, .6, 3.0), (-1.0, .8, 1.8, 3.4)],
        tag_spots=[(3.2, -1.88, .85, 0, 1, "on the solar panel array", False), (-3.5, 1.78, 1.1, 0, -1, "on the shed door", False),
                   (4.2, 2.2, .7, -1, 0, "on a post near the east fence", True)],
        object_kinds=["carton", "crate"],
        containers=[],
        floor_rgba=(.36, .52, .28), ground_texture="grass",
        decor=[("path", (0, -.2, .001), (4.8, .35, .001), (.62, .55, .42, 1))],
        cams=[_cam("overview", (0, -6.2, 5.0), (0, 0, 0), 70), _cam("top", (0, 0, 10.5), (0, 0, 0), 55)],
    )


def lab() -> Layout:
    bench = Surface("lab_bench", "the lab bench", (0.0, 1.2), (.9, .35), .9, approach=("s",), rgba=(.9, .9, .9, 1), legs=False)
    side = Surface("side_bench", "the side bench", (1.8, -.4), (.35, .6), .9, approach=("w",), rgba=(.9, .9, .9, 1), legs=False)
    return Layout(
        id="lab-bench", title="Lab bench", scenario="lab",
        bounds=(-2.4, 2.4, -2.0, 1.85), ceiling=2.7,
        surfaces={"lab_bench": bench, "side_bench": side},
        obstacles=[Obstacle("fume_hood", (-1.75, 1.2), (.55, .38), 2.0, "the fume hood", (.8, .82, .86, 1)),
                   Obstacle("freezer", (-2.0, -1.4), (.3, .35), 1.6, "the sample freezer", (.9, .9, .95, 1))],
        mounts={"arm": ArmMount("lab_bench", {"arm": (0.0, 1.47, -90.0)}, (-.4, .4, .95, 1.15)),
                "bimanual": ArmMount("lab_bench", {"left": (-.4, 1.45, -90.0), "right": (.4, 1.45, -90.0)}, (-.75, .75, 1.05, 1.3)),
                "floor": dict(start=(0.0, -1.3, 90.0)),
                "airspace": dict(pad=(-.8, -1.6))},
        floor_slots=[(-1.6, 1.3, -1.7, .5)],
        tag_spots=[(2.35, .7, .9, -1, 0, "on the east wall above the side bench", False)],
        object_kinds=["tube", "block", "small_block", "cup", "ring"],
        containers=["tote", "tray", "bin"],
        floor_rgba=(.78, .8, .8), wall_rgba=(.95, .96, .97,.35),
        decor=[("biohazard_sign", (2.39, -.4, 1.3), (.005, .12, .12), (.95, .75, .1, 1))],
        cams=[_cam("overview", (0, -1.9, 2.3), (0, .4, .7), 68), _cam("bench_cam", (0, .2, 1.7), (0, 1.15, .9), 62),
              _cam("top", (0, -.2, 4.4), (0, -.2, 0), 60)],
    )


SCENES = {"tabletop": tabletop, "kitchen-counter": kitchen, "workshop-bench": workshop, "warehouse": warehouse,
          "outdoor-field": outdoor, "lab-bench": lab}


def get(scene_id: str) -> Layout:
    if scene_id not in SCENES:
        raise KeyError(f"unknown scene {scene_id!r}; scenes: {', '.join(SCENES)}")
    return SCENES[scene_id]()


def approach_pose(sf: Surface, x: float, y: float, side: str, standoff: float) -> tuple[float, float, float]:
    """Floor-robot pose (x, y, yaw_deg) facing the surface edge `side`, `standoff` metres from the point (x, y) along
    the approach direction."""
    vx, vy = SIDE_VEC[side]
    if side in ("s", "n"):
        edge = sf.c[1] + vy * sf.half[1]
        return x, edge + vy * max(0.0, standoff - abs(edge - y)), math.degrees(math.atan2(-vy, 0))
    edge = sf.c[0] + vx * sf.half[0]
    return edge + vx * max(0.0, standoff - abs(edge - x)), y, math.degrees(math.atan2(0, -vx))
