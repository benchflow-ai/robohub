"""Task templates: a goal, a physical success check and a reference strategy, parameterised by scene and embodiment.

A template turns (scene layout, embodiment driver, seed) into a concrete *instance*: the task objects and fixtures
with their poses, the goal, the instruction text and a step budget. The instance is written into the task folder's
`robouse.instance.layout`, so a composed task is fully determined by its folder. `check(b)` judges the episode from
the simulated state (b is the ComposedBackend); `solve(r, inst)` is the reference solution, which drives the robot only
through the episode socket with the embodiment's declared skills (strategies.Robo).

Templates declare what they need through `variants`: each variant lists the placements it works in and the
capabilities the embodiment must have. The component resolver (robouse.components) checks the same declarations
statically, so a combination that cannot be generated is rejected with the reason before any simulation runs.
"""
from __future__ import annotations

import math
import re
from dataclasses import dataclass, field

import numpy as np

from . import world as W
from .scenes import SIDE_VEC, Layout


class Incompatible(Exception):
    """This template cannot be instantiated for this scene and embodiment (the message says why)."""


# ------------------------------------------------------------------------------------------------------------------
# generation context
# ------------------------------------------------------------------------------------------------------------------

# Front grasps (a level gripper) need tall objects: the finger pads are 4 cm tall around the grasp point.
FRONT_MIN_HEIGHT = .07
FRONT_MIN_WIDTH = .04
CONTAINER_MAX_H = .08     # tallest object a top grasp puts into a container     # thinner things (bolts, tubes) slip and topple in a level grasp
FLOOR_REACH = {"pal-tiago": dict(h=(.5, .85), standoff=.62, band=(.08, .24), body=.33, gz_max=.97),
               "google-robot": dict(h=(.35, 1.0), standoff=.62, band=(.06, .2), body=.36, gz_max=.99)}


def graspable(kind: str, grasp_mode: str | None, max_grip: float) -> bool:
    """Can a gripper with this grasp mode and widest usable grip pick this kind up for pick-and-place tasks?"""
    k = W.KINDS[kind]
    if not k.get("graspable", True) or k["shape"] in ("knife", "ring"):
        return False  # rings are grasped by their wall (thread-ring-on-peg asks for them explicitly)
    return grasp_fits(W.grip_width(kind), 2 * W.half_height(kind), grasp_mode, max_grip)


def grasp_fits(width: float, height: float, grasp_mode: str | None, max_grip: float) -> bool:
    if not grasp_mode or width > max_grip:
        return False
    if grasp_mode == "front":
        return height >= FRONT_MIN_HEIGHT and width >= FRONT_MIN_WIDTH
    return True


@dataclass
class Region:
    surface: str
    x0: float
    x1: float
    y0: float
    y1: float
    z: float
    side: str = ""      # floor robots: the side of the surface they reach it from

    def area(self) -> float:
        return max(0.0, self.x1 - self.x0) * max(0.0, self.y1 - self.y0)


@dataclass
class Ctx:
    layout: Layout
    drv: type            # driver class (robots.DRIVERS[...])
    rng: np.random.Generator
    mods: dict = field(default_factory=dict)
    placed: W.Placed = field(default_factory=W.Placed)

    @property
    def caps(self) -> set:
        return set(self.drv.caps)

    @property
    def mount(self) -> str:
        return self.drv.mount

    def graspable(self, kind: str) -> bool:
        return "grasp" in self.caps and graspable(kind, self.drv.grasp_mode, self.drv.max_grip)

    def kinds(self, candidates=None, graspable=True, into_container=False) -> list[str]:
        ks = [k for k in self.layout.object_kinds if (candidates is None or k in candidates)]
        if into_container and self.drv.grasp_mode == "top":  # tall things tip over when set down in a bowl or tote
            ks = [k for k in ks if 2 * W.half_height(k) <= CONTAINER_MAX_H]
        return [k for k in ks if self.graspable(k)] if graspable else ks

    # ---- regions where manipulable things go --------------------------------------------------------------------
    def regions(self) -> list[Region]:
        L = self.layout
        if self.mount in ("arm", "bimanual"):
            am = L.mounts.get(self.mount)
            if not am:
                return []
            x0, x1, y0, y1 = am.work
            return [Region(am.surface, x0, x1, y0, y1, L.surfaces[am.surface].top)]
        if self.mount == "floor" and "grasp" in self.caps:
            fr = FLOOR_REACH[self.drv.key]
            out = []
            for sname, sf in L.surfaces.items():
                if not fr["h"][0] <= sf.top <= fr["h"][1]:
                    continue
                for side in sf.approach:
                    r = self._band(sname, sf, side, fr)
                    if r and r.area() > .02:
                        out.append(r)
            return out
        return []

    def container_regions(self, regs: list, ck: str, kinds: list) -> list:
        """Floor robots with a level grasp drop objects in from above the rim: keep the regions where that height is
        within the arm's reach for every object kind in play."""
        if self.mount != "floor":
            return regs
        fr = FLOOR_REACH[self.drv.key]
        grip = max(max(.03, min(W.half_height(k), .055)) for k in kinds)
        out = [r for r in regs if r.z + W.CONTAINERS[ck]["h"] + .01 + grip <= fr["gz_max"]]
        if not out:
            raise Incompatible(f"{self.drv.name} cannot lift objects over a {ck} rim on any surface here")
        return out

    def _band(self, sname, sf, side, fr) -> Region | None:
        (cx, cy), (hx, hy) = sf.c, sf.half
        lo, hi = fr["band"]
        m = .1
        if side == "s":
            r = Region(sname, cx - hx + m, cx + hx - m, cy - hy + lo, min(cy + hy - .05, cy - hy + hi), sf.top, side)
        elif side == "n":
            r = Region(sname, cx - hx + m, cx + hx - m, max(cy - hy + .05, cy + hy - hi), cy + hy - lo, sf.top, side)
        elif side == "w":
            r = Region(sname, cx - hx + lo, min(cx + hx - .05, cx - hx + hi), cy - hy + m, cy + hy - m, sf.top, side)
        else:
            r = Region(sname, max(cx - hx + .05, cx + hx - hi), cx + hx - lo, cy - hy + m, cy + hy - m, sf.top, side)
        # keep only the part whose approach pose is free floor
        xs = np.linspace(r.x0, r.x1, 9)
        ys = np.linspace(r.y0, r.y1, 5)
        ok = [(x, y) for x in xs for y in ys if self.approach_free(sname, x, y, side, fr)]
        if not ok:
            return None
        ax, ay = zip(*ok)
        return Region(sname, min(ax), max(ax), min(ay), max(ay), sf.top, side)

    def approach_free(self, sname, x, y, side, fr) -> bool:
        p = approach_point(self.layout.surfaces[sname], x, y, side, fr["standoff"], fr["body"])
        return floor_free(self.layout, p[0], p[1], fr["body"] + .12)

    def spot(self, regions: list[Region], kind: str, tries: int = 400, same_region: Region | None = None,
             avoid: list | None = None) -> tuple[Region, float, float]:
        """A free spot for an object of `kind` (or a fixture footprint radius) in one of the regions."""
        r_obj = W.footprint_radius(kind) if kind in W.KINDS else float(kind_radius(kind))
        regs = [same_region] if same_region else regions
        for _ in range(tries):
            reg = regs[int(self.rng.integers(len(regs)))]
            if reg.x1 - reg.x0 < 2 * r_obj or reg.y1 - reg.y0 < 0:
                continue
            x = float(self.rng.uniform(reg.x0 + r_obj, reg.x1 - r_obj)) if reg.x1 - reg.x0 > 2 * r_obj else (reg.x0 + reg.x1) / 2
            y = float(self.rng.uniform(reg.y0, reg.y1)) if reg.y1 > reg.y0 else reg.y0
            if not self.layout.surfaces[reg.surface].contains(x, y, r_obj * .7):
                continue
            if avoid and any(math.hypot(x - ax, y - ay) < ar for ax, ay, ar in avoid):
                continue
            if reg.side and not self.approach_free(reg.surface, x, y, reg.side, FLOOR_REACH[self.drv.key]):
                continue  # a floor robot must be able to park in front of it
            if self.placed.free(reg.surface, x, y, r_obj):
                self.placed.add(reg.surface, x, y, r_obj)
                return reg, round(x, 3), round(y, 3)
        raise Incompatible(f"not enough free space on {', '.join(sorted({r.surface for r in regs}))} for this task")


HAND_CLEARANCE = .07  # extra room around tall fixtures so a gripper's hand and fingers fit next to them


def kind_radius(kind: str) -> float:
    c = W.CONTAINERS.get(kind)
    if c:
        r = (c["r"] + c["t"]) if c.get("round") else math.hypot(*c["half"]) + c["t"]
        return r + (HAND_CLEARANCE if c["h"] > .03 else .02)
    if kind == "peg":
        return W.PEG["base_half"] * 1.5 + HAND_CLEARANCE
    if kind == "zone":
        return .1
    return .05


def approach_point(sf, x: float, y: float, side: str, standoff: float, body: float) -> tuple[float, float, float]:
    """Floor-robot base pose (x, y, yaw_deg) facing the surface across edge `side`, with the object `standoff` ahead."""
    vx, vy = SIDE_VEC[side]
    if side in ("s", "n"):
        edge = sf.c[1] + vy * sf.half[1]
        d = max(standoff, abs(edge - y) + body + .06)
        return x, y + vy * d, math.degrees(math.atan2(-vy, 0.0))
    edge = sf.c[0] + vx * sf.half[0]
    d = max(standoff, abs(edge - x) + body + .06)
    return x + vx * d, y, math.degrees(math.atan2(0.0, -vx))


def floor_free(L: Layout, x: float, y: float, r: float) -> bool:
    x0, x1, y0, y1 = L.bounds
    if not (x0 + r <= x <= x1 - r and y0 + r <= y <= y1 - r):
        return False
    for fx0, fx1, fy0, fy1, _ in L.footprints():
        if fx0 - r < x < fx1 + r and fy0 - r < y < fy1 + r:
            return False
    return True


def reachable(L: Layout, start, goal, r: float, boxes: list | None = None, res: float = .05) -> bool:
    """Is there a path for a disc of radius r from start to goal on the floor (4-connected grid), keeping clear of the
    scene's footprints and the extra boxes (x0, x1, y0, y1)?"""
    from collections import deque

    x0, x1, y0, y1 = L.bounds
    nx, ny = int((x1 - x0) / res) + 1, int((y1 - y0) / res) + 1
    X, Y = np.meshgrid(x0 + np.arange(nx) * res, y0 + np.arange(ny) * res, indexing="ij")
    blocked = (X < x0 + r) | (X > x1 - r) | (Y < y0 + r) | (Y > y1 - r)
    for fx0, fx1, fy0, fy1, *_ in L.footprints() + list(boxes or []):
        blocked |= (X > fx0 - r) & (X < fx1 + r) & (Y > fy0 - r) & (Y < fy1 + r)
    cell = lambda p: (int(np.clip(round((p[0] - x0) / res), 0, nx - 1)), int(np.clip(round((p[1] - y0) / res), 0, ny - 1)))
    s, g = cell(start), cell(goal)
    if blocked[g]:
        return False
    blocked[s] = False
    seen = {s}
    q = deque([s])
    while q:
        c = q.popleft()
        if c == g:
            return True
        for d in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            n = (c[0] + d[0], c[1] + d[1])
            if 0 <= n[0] < nx and 0 <= n[1] < ny and not blocked[n] and n not in seen:
                seen.add(n)
                q.append(n)
    return False


def floor_spot(ctx: Ctx, r: float, avoid: list | None = None, tries: int = 600) -> tuple[float, float]:
    """A free floor point with clearance r from furniture, walls, the robot start and `avoid` circles."""
    L = ctx.layout
    start = L.mounts.get("floor", {}).get("start") or (*L.mounts.get("airspace", {}).get("pad", (0, 0)), 0)
    slots = L.floor_slots
    for _ in range(tries):
        x0, x1, y0, y1 = slots[int(ctx.rng.integers(len(slots)))]
        x, y = x0 + (x1 - x0) * float(ctx.rng.random()), y0 + (y1 - y0) * float(ctx.rng.random())
        if not floor_free(L, x, y, r):
            continue
        if math.hypot(x - start[0], y - start[1]) < r + .6:
            continue
        if avoid and any(math.hypot(x - a, y - b) < r + rb + .15 for a, b, rb in avoid):
            continue
        return round(x, 3), round(y, 3)
    raise Incompatible(f"no free floor area in {L.id} for this task")


def pick_colors(ctx: Ctx, n: int, pool=("red", "blue", "green", "yellow", "orange", "purple")) -> list[str]:
    idx = ctx.rng.permutation(len(pool))[:n]
    return [pool[i] for i in idx]


def label(o: dict) -> str:
    return f"{o['color']} {W.KINDS[o['kind']]['label']}"


def flabel(f: dict) -> str:
    base = W.CONTAINERS[f["kind"]]["label"] if f["kind"] in W.CONTAINERS else f["kind"]
    return f"{f['color']} {base}" if f.get("color") else base


# ------------------------------------------------------------------------------------------------------------------
# templates
# ------------------------------------------------------------------------------------------------------------------

class Template:
    id = ""
    title = ""
    capabilities: tuple = ()     # capability tags (see docs/components.md)
    difficulty = "medium"
    variants: tuple = ()         # ({"mounts": [...], "caps": [...]}, ...): any one variant must match
    needs: dict = {}             # scene needs: {"object_kinds_any": [...], "containers": n, "surfaces": n, "floor_slots": n, "tags": n}
    perturbable = True           # has a primary object the `push` perturbation can move
    safety_levels = ("direct", "indirect", "privacy")

    def variant(self, drv) -> dict | None:
        for v in self.variants:
            if drv.mount in v["mounts"] and set(v["caps"]) <= set(drv.caps):
                return v
        return None

    def generate(self, ctx: Ctx) -> dict:
        raise NotImplementedError

    def check(self, b) -> tuple[bool, dict]:
        raise NotImplementedError

    def solve(self, r, inst: dict) -> None:
        raise NotImplementedError

    @staticmethod
    def primary(inst: dict) -> str | None:
        return inst.get("goal", {}).get("object")


# ---- 1. put in container ----------------------------------------------------------------------------------------

class PutInContainer(Template):
    id = "put-in-container"
    title = "Put an object in a container"
    capabilities = ("pick-and-place",)
    difficulty = "easy"
    variants = ({"mounts": ["arm", "bimanual", "floor"], "caps": ["reach", "grasp"]},)
    needs = {"containers": 1}

    def generate(self, ctx: Ctx) -> dict:
        regs = ctx.regions()
        if not regs:
            raise Incompatible("no work surface this embodiment can reach")
        kinds = ctx.kinds(into_container=True)
        if not kinds:
            raise Incompatible("the scene stocks no object this gripper can pick up")
        conts = [c for c in ctx.layout.containers if c != "bin" and (ctx.drv.grasp_mode != "front" or W.CONTAINERS[c]["h"] <= .05)]
        if not conts:
            raise Incompatible("the scene has no container that fits on a work surface" +
                               (" and is low enough (5 cm) to drop into from a level front grasp" if ctx.drv.grasp_mode == "front" else ""))
        ck = conts[int(ctx.rng.integers(len(conts)))]
        cols = pick_colors(ctx, 3)
        creg, cx, cy = ctx.spot(ctx.container_regions(regs, ck, kinds), ck)
        cont = {"name": ck, "kind": ck, "color": "white" if ck == "bowl" else "grey", "pos": [cx, cy, creg.z], "on": creg.surface}
        objs = []
        target_kind = kinds[int(ctx.rng.integers(len(kinds)))]
        reg, x, y = ctx.spot(regs, target_kind, same_region=creg if ctx.mount in ("arm",) else None)
        objs.append({"name": f"{cols[0]}_{target_kind}", "kind": target_kind, "color": cols[0], "pos": [x, y, reg.z],
                     "yaw": float(ctx.rng.uniform(-12, 12)), "on": reg.surface})
        for c in cols[1:1 + int(ctx.mods.get("distractors", 1))]:
            k = kinds[int(ctx.rng.integers(len(kinds)))]
            reg2, x2, y2 = ctx.spot(regs, k)
            objs.append({"name": f"{c}_{k}", "kind": k, "color": c, "pos": [x2, y2, reg2.z], "yaw": float(ctx.rng.uniform(-12, 12)),
                         "on": reg2.surface})
        t = objs[0]
        return {"objects": objs, "fixtures": [cont],
                "goal": {"object": t["name"], "container": cont["name"]},
                "task": f"Put the {label(t)} in the {flabel(cont)}. Leave the other objects where they are.",
                "success": (f"the {label(t)} rests inside the {flabel(cont)} (its centre within the container's inner footprint and "
                            f"below its rim), no part of the robot touches it, and no other object is inside the {flabel(cont)}"),
                "steps": 700 if ctx.mount != "floor" else 2400}

    def check(self, b) -> tuple[bool, dict]:
        g = b.inst["goal"]
        ok_in = b.in_container(g["object"], g["container"])
        others = [o["name"] for o in b.inst["objects"] if o["name"] != g["object"] and b.in_container(o["name"], g["container"], loose=True)]
        return ok_in and not others, {"in_container": ok_in, "others_inside": others}

    def solve(self, r, inst: dict) -> None:
        g = inst["goal"]
        r.pick(g["object"])
        r.place_in(g["object"], g["container"])


# ---- 2. stack in order ------------------------------------------------------------------------------------------

SIZE_ORDER = ["large_block", "block", "small_block"]


class StackInOrder(Template):
    id = "stack-in-order"
    title = "Stack blocks in a stated order"
    capabilities = ("spatial-reasoning", "pick-and-place")
    difficulty = "medium"
    variants = ({"mounts": ["arm", "bimanual"], "caps": ["reach", "grasp", "top-grasp"]},)
    needs = {"object_kinds_any": ["block"]}

    def generate(self, ctx: Ctx) -> dict:
        regs = ctx.regions()
        if not regs:
            raise Incompatible("no work surface this embodiment can reach")
        by_size = all(k in ctx.layout.object_kinds and ctx.graspable(k) for k in SIZE_ORDER)
        if "block" not in ctx.layout.object_kinds or not ctx.graspable("block"):
            raise Incompatible("the scene stocks no blocks this gripper can pick up")
        cols = pick_colors(ctx, 3)
        kinds = list(SIZE_ORDER) if by_size else ["block"] * 3
        order = list(range(3))
        # the base pad: a zone mark on the surface
        creg, zx, zy = ctx.spot(regs, "zone")
        zone = {"name": "base_mark", "kind": "zone", "color": "black", "pos": [zx, zy, creg.z], "half": [.045, .045], "on": creg.surface}
        objs = []
        idx = ctx.rng.permutation(3)
        for i in idx:
            reg, x, y = ctx.spot(regs, kinds[i])
            objs.append({"name": f"{cols[i]}_{kinds[i]}", "kind": kinds[i], "color": cols[i], "pos": [x, y, reg.z],
                         "yaw": float(ctx.rng.uniform(-12, 12)), "on": reg.surface})
        names = [f"{cols[i]}_{kinds[i]}" for i in order]  # bottom -> top
        if by_size:
            rule = ("Build a tower of the three blocks on the black mark, ordered by size: the largest block at the bottom and "
                    "the smallest on top.")
        else:
            rule = (f"Build a tower of the three blocks on the black mark: the {cols[0]} block at the bottom, the {cols[2]} block "
                    f"on top, and the remaining block in the middle.")
        return {"objects": objs, "fixtures": [zone], "goal": {"order": names, "mark": "base_mark", "object": names[0]},
                "task": rule,
                "success": ("the bottom block rests on the surface with its centre on the black mark (within 3 cm), each other block "
                            "rests on the one below it (centres within 2.5 cm horizontally, touching), every block is level (tilted "
                            "less than 15 degrees) and the robot touches none of them"),
                "steps": 1400}

    def check(self, b) -> tuple[bool, dict]:
        g = b.inst["goal"]
        names = g["order"]
        mark = b.fixture(g["mark"])
        p = [b.obj_pos(n) for n in names]
        det = {}
        det["base_on_mark"] = float(np.linalg.norm(p[0][:2] - np.asarray(mark["pos"][:2]))) <= .03 and b.touches_surface(names[0])
        det["stacked"] = all(float(np.linalg.norm(p[i + 1][:2] - p[i][:2])) <= .025 and b.touching_objects(names[i + 1], names[i])
                             and p[i + 1][2] > p[i][2] for i in range(2))
        det["level"] = all(b.obj_tilt(n) < 15 for n in names)
        det["released"] = not any(b.touch_robot(n) for n in names)
        return all(det.values()), det

    def solve(self, r, inst: dict) -> None:
        g = inst["goal"]
        mark = next(f for f in inst["fixtures"] if f["name"] == g["mark"])
        below = None
        for n in g["order"]:
            r.pick(n)
            if below is None:
                r.place_on_point(n, mark["pos"][0], mark["pos"][1], mark["pos"][2])
            else:
                bp = r.obj(below)["pos"]
                r.place_on_point(n, bp[0], bp[1], r.obj_top(below))
            below = n


# ---- 3. thread a ring onto a peg --------------------------------------------------------------------------------

class ThreadRing(Template):
    id = "thread-ring-on-peg"
    title = "Thread a ring onto the right peg"
    capabilities = ("topological-reasoning", "spatial-reasoning", "contact-rich")
    difficulty = "hard"
    variants = ({"mounts": ["arm", "bimanual"], "caps": ["reach", "grasp", "top-grasp", "precise"]},)
    needs = {"object_kinds_any": ["ring"]}

    def generate(self, ctx: Ctx) -> dict:
        regs = ctx.regions()
        if not regs:
            raise Incompatible("no work surface this embodiment can reach")
        if "ring" not in ctx.layout.object_kinds:
            raise Incompatible("the scene stocks no rings")
        pcols = pick_colors(ctx, 2, ("red", "blue", "green", "yellow"))
        pegs = []
        for c in pcols:
            reg, x, y = ctx.spot(regs, "peg")
            pegs.append({"name": f"{c}_peg", "kind": "peg", "color": c, "pos": [x, y, reg.z], "on": reg.surface})
        reg, x, y = ctx.spot(regs, "ring")
        ring = {"name": "ring", "kind": "ring", "color": "orange", "pos": [x, y, reg.z], "yaw": 0.0, "on": reg.surface}
        # the right peg is named by a spatial relation to a landmark block
        breg, bx, by = ctx.spot(regs, "block")
        block = {"name": "purple_block", "kind": "block", "color": "purple", "pos": [bx, by, breg.z], "yaw": 0.0, "on": breg.surface}
        d = [math.hypot(p["pos"][0] - bx, p["pos"][1] - by) for p in pegs]
        if abs(d[0] - d[1]) < .08:
            raise Incompatible("resample")
        target = pegs[int(np.argmin(d))]
        return {"objects": [ring, block], "fixtures": pegs, "goal": {"object": "ring", "peg": target["name"]},
                "task": "Put the ring over the peg that stands closer to the purple block, so that the peg passes through the ring's hole.",
                "success": ("the peg that is closer to the purple block passes through the ring's hole (the peg's axis crosses the "
                            "ring's plane inside its hole, below the peg's top), and the robot does not touch the ring"),
                "steps": 900}

    def check(self, b) -> tuple[bool, dict]:
        g = b.inst["goal"]
        peg = b.fixture(g["peg"])
        k = W.KINDS["ring"]
        c = b.obj_pos("ring")
        R = b.obj_R("ring")
        n = R[:, 2]
        px, py, pz = peg["pos"]
        top = pz + W.PEG["base_h"] + W.PEG["h"]
        det = {"threaded": False}
        if abs(n[2]) > .2:
            t = float(np.dot(c - np.array([px, py, 0.0]), n) / n[2])  # the peg axis (px, py, z) meets the ring plane at z = t
            hit = np.array([px, py, t])
            rr = float(np.linalg.norm(hit - c))
            det["axis_offset_m"] = round(rr, 4)
            det["threaded"] = bool(rr < k["R"] - k["r"] - W.PEG["r"] + .002 and pz < t < top)
        det["released"] = not b.touch_robot("ring")
        return det["threaded"] and det["released"], det

    def solve(self, r, inst: dict) -> None:
        g = inst["goal"]
        peg = next(f for f in inst["fixtures"] if f["name"] == g["peg"])
        r.thread_ring("ring", peg)


# ---- 4. sort by colour ------------------------------------------------------------------------------------------

class SortByColor(Template):
    id = "sort-by-color"
    title = "Sort objects into containers by colour"
    capabilities = ("long-horizon", "pick-and-place")
    difficulty = "medium"
    variants = ({"mounts": ["arm", "bimanual", "floor"], "caps": ["reach", "grasp"]},)
    needs = {"containers": 1}

    def generate(self, ctx: Ctx) -> dict:
        regs = ctx.regions()
        if not regs:
            raise Incompatible("no work surface this embodiment can reach")
        kinds = ctx.kinds(into_container=True)
        conts = [c for c in ctx.layout.containers if c in ("bowl", "tote", "tray") and (ctx.drv.grasp_mode != "front" or W.CONTAINERS[c]["h"] <= .05)]
        if not kinds or not conts:
            raise Incompatible("the scene has no graspable objects or no containers for sorting")
        ck = next(c for c in ("bowl", "tray", "tote") if c in conts)
        cols = pick_colors(ctx, 2, ("red", "blue", "green", "yellow"))
        fx = []
        for c in cols:
            reg, x, y = ctx.spot(ctx.container_regions(regs, ck, kinds), ck)
            fx.append({"name": f"{c}_{ck}", "kind": ck, "color": c, "pos": [x, y, reg.z], "on": reg.surface})
        objs = []
        n_each = 2 if ctx.mount != "floor" else 1
        for c in cols:
            for i in range(n_each):
                k = kinds[int(ctx.rng.integers(len(kinds)))]
                reg, x, y = ctx.spot(regs, k)
                objs.append({"name": f"{c}_{k}_{i + 1}", "kind": k, "color": c, "pos": [x, y, reg.z],
                             "yaw": float(ctx.rng.uniform(-12, 12)), "on": reg.surface})
        order = ctx.rng.permutation(len(objs))
        objs = [objs[i] for i in order]
        return {"objects": objs, "fixtures": fx,
                "goal": {"assign": {o["name"]: f"{o['color']}_{ck}" for o in objs}, "object": objs[0]["name"]},
                "task": f"Sort the objects by colour: put every {cols[0]} object into the {cols[0]} {ck} and every {cols[1]} object into the "
                        f"{cols[1]} {ck}.",
                "success": f"every object rests inside the {ck} of its own colour and no part of the robot touches any of them",
                "steps": 2400 if ctx.mount != "floor" else 4000}

    def check(self, b) -> tuple[bool, dict]:
        det = {n: b.in_container(n, c) for n, c in b.inst["goal"]["assign"].items()}
        return all(det.values()), det

    def solve(self, r, inst: dict) -> None:
        for n, c in inst["goal"]["assign"].items():
            r.pick(n)
            r.place_in(n, c)


# ---- 5. push to zone (contact-rich) ----------------------------------------------------------------------------

class PushToZone(Template):
    id = "push-to-zone"
    title = "Push an object that cannot be grasped into a marked zone"
    capabilities = ("contact-rich",)
    difficulty = "medium"
    variants = ({"mounts": ["arm", "bimanual"], "caps": ["reach"], "object": "crate"},
                {"mounts": ["floor"], "caps": ["legged"], "object": "carton"})
    needs = {"object_kinds_any": ["crate", "carton"]}

    def generate(self, ctx: Ctx) -> dict:
        v = self.variant(ctx.drv)
        kind = v["object"]
        if kind not in ctx.layout.object_kinds:
            raise Incompatible(f"the scene stocks no {W.KINDS[kind]['label']} to push")
        if kind == "crate":
            regs = ctx.regions()
            if not regs:
                raise Incompatible("no work surface this embodiment can reach")
            reg = regs[0]
            for _ in range(200):
                ctx.placed = W.Placed()
                _, zx, zy = ctx.spot(regs, "zone")
                _, cx, cy = ctx.spot(regs, "crate")
                if .18 <= math.hypot(zx - cx, zy - cy) <= .35:
                    break
            else:
                raise Incompatible("the work area is too small to push the crate into a zone")
            zone = {"name": "target_zone", "kind": "zone", "color": "green", "pos": [zx, zy, reg.z], "half": [.1, .08], "on": reg.surface}
            obj = {"name": "crate", "kind": "crate", "color": "cardboard", "pos": [cx, cy, reg.z], "yaw": 0.0, "on": reg.surface}
            steps = 900
        else:
            for _ in range(300):
                zx, zy = floor_spot(ctx, .6)
                cx, cy = floor_spot(ctx, .72, avoid=[(zx, zy, .55)])  # room on every side to walk round and re-align
                d = math.hypot(zx - cx, zy - cy)
                if not 1.2 <= d <= 2.6:
                    continue
                ux, uy = (zx - cx) / d, (zy - cy) / d
                # the robot lines up behind the carton: that spot must be free floor, reachable from the start around the carton
                behind = (cx - ux * .8, cy - uy * .8)
                k = W.footprint_radius("carton")
                start = ctx.layout.mounts["floor"]["start"][:2]
                if (floor_free(ctx.layout, *behind, .5) and floor_free(ctx.layout, cx - ux * .5, cy - uy * .5, .3)
                        and reachable(ctx.layout, start, behind, .42, [(cx - k, cx + k, cy - k, cy + k)])):
                    break
            else:
                raise Incompatible("no free floor for the carton, the zone and the robot behind the carton")
            zone = {"name": "target_zone", "kind": "mat", "color": "green", "pos": [zx, zy, 0.0], "half": [.55, .55], "on": "floor"}
            obj = {"name": "carton", "kind": "carton", "color": "cardboard", "pos": [cx, cy, 0.0], "yaw": float(ctx.rng.uniform(-20, 20)),
                   "on": "floor"}
            steps = 2400
        return {"objects": [obj], "fixtures": [zone], "goal": {"object": obj["name"], "zone": "target_zone"},
                "task": f"The {W.KINDS[kind]['label']} is too wide to grasp. Push it into the green zone.",
                "success": (f"the centre of the {W.KINDS[kind]['label']} is inside the green zone, it is upright (tilted less than 15 "
                            "degrees) and nothing of the robot touches it"),
                "steps": steps}

    def check(self, b) -> tuple[bool, dict]:
        g = b.inst["goal"]
        z = b.fixture(g["zone"])
        p = b.obj_pos(g["object"])
        inside = abs(p[0] - z["pos"][0]) <= z["half"][0] and abs(p[1] - z["pos"][1]) <= z["half"][1]
        det = {"in_zone": bool(inside), "upright": b.obj_tilt(g["object"]) < 15, "released": not b.touch_robot(g["object"])}
        return all(det.values()), det

    def solve(self, r, inst: dict) -> None:
        g = inst["goal"]
        z = next(f for f in inst["fixtures"] if f["name"] == g["zone"])
        r.push_into(g["object"], z)


# ---- 6. go to a landmark ----------------------------------------------------------------------------------------

class GoToLandmark(Template):
    id = "go-to-landmark"
    title = "Go to the mat described by a spatial relation"
    capabilities = ("spatial-reasoning", "navigation")
    difficulty = "easy"
    variants = ({"mounts": ["floor"], "caps": ["locomote"]}, {"mounts": ["airspace"], "caps": ["fly"]})
    needs = {"floor_slots": 1}
    perturbable = False

    def generate(self, ctx: Ctx) -> dict:
        L = ctx.layout
        landmarks = [(o.label or o.name, o.c) for o in L.obstacles] + [(s.label, s.c) for s in L.surfaces.values()]
        if not landmarks:
            raise Incompatible("the scene has no landmark to describe a place by")
        fly = ctx.mount == "airspace"
        cols = pick_colors(ctx, 3, ("red", "blue", "green", "yellow"))
        half = .2 if fly else .3
        clear = .2 if fly else .45
        mats = []
        pts = []
        for c in cols:
            x, y = floor_spot(ctx, clear, avoid=pts)
            pts.append((x, y, half))
            mats.append({"name": f"{c}_{'pad' if fly else 'mat'}", "kind": "pad" if fly else "mat", "color": c, "pos": [x, y, 0.0],
                         "half": [half, half], "on": "floor"})
        for _ in range(40):
            lm_label, lm = landmarks[int(ctx.rng.integers(len(landmarks)))]
            d = sorted((math.hypot(m["pos"][0] - lm[0], m["pos"][1] - lm[1]), i) for i, m in enumerate(mats))
            if d[1][0] - d[0][0] > .5:
                break
        else:
            raise Incompatible("resample")
        target = mats[d[0][1]]
        what = "landing pad" if fly else "mat"
        if fly:
            task = f"Three coloured landing pads lie on the floor. Take off, fly to the pad closest to {lm_label} and land on it."
            succ = ("the drone rests on that pad with its motors off (its centre above the pad), and it never touched anything but the "
                    "floor, a pad or a surface top")
        else:
            task = f"Three coloured mats lie on the floor. Go to the mat closest to {lm_label} and stop on it."
            succ = "the robot's base centre is on that mat and the robot is standing still (not stepping or driving)"
        return {"objects": [], "fixtures": mats, "goal": {"target": target["name"], "landmark": lm_label},
                "task": task + f" (Closest means the smallest distance between the {what}'s centre and the centre of {lm_label}.)",
                "success": succ, "steps": 1200}

    def check(self, b) -> tuple[bool, dict]:
        f = b.fixture(b.inst["goal"]["target"])
        x, y, _ = b.driver.base_pose()
        on = abs(x - f["pos"][0]) <= f["half"][0] and abs(y - f["pos"][1]) <= f["half"][1]
        det = {"on_target": bool(on), "still": b.robot_still()}
        if b.driver.mount == "airspace":
            det["landed"] = bool(b.driver.landed)
        return all(det.values()), det

    def solve(self, r, inst: dict) -> None:
        f = next(f for f in inst["fixtures"] if f["name"] == inst["goal"]["target"])
        r.travel_to(f["pos"][0], f["pos"][1])
        if r.fly:
            r.skill("land")


# ---- 7. inspect a tag (perception) ------------------------------------------------------------------------------

class InspectTag(Template):
    id = "inspect-tag"
    title = "Read the number on an inspection tag"
    capabilities = ("perception", "partial-observability", "navigation")
    difficulty = "medium"
    variants = ({"mounts": ["floor"], "caps": ["locomote", "head-camera"]}, {"mounts": ["airspace"], "caps": ["fly", "fpv-camera"]})
    needs = {"tags": 1}
    perturbable = False

    def generate(self, ctx: Ctx) -> dict:
        L = ctx.layout
        if not L.tag_spots:
            raise Incompatible("the scene has no place for an inspection tag")
        x, y, z, nx, ny, where, post = L.tag_spots[int(ctx.rng.integers(len(L.tag_spots)))]
        reading = int(ctx.rng.integers(100, 1000))
        tag = {"name": "tag", "kind": "tag", "color": "yellow", "pos": [x, y, z], "normal": [nx, ny], "reading": reading,
               "label": f"the yellow tag {where}", "post": post}
        return {"objects": [], "fixtures": [tag], "goal": {"tag": "tag", "reading": reading},
                "task": (f"A yellow inspection tag hangs {where}; a three-digit number is printed on it. Go and look at it with your "
                         "camera, read the number, and report it in the `robo done` message (for example `robo done \"the tag reads 123\"`)."),
                "success": ("the tag was seen by the robot's own camera (its centre in the middle 70 % of the image, facing the camera, "
                            "at most 2.5 m away, not blocked) for at least one second, and the `robo done` message contains the number and "
                            "no other three-digit number"),
                "steps": 1400}

    def check(self, b) -> tuple[bool, dict]:
        return b.tag_seen("tag"), {"seen": b.tag_seen("tag")}

    def judge_text(self, b, text: str) -> bool:
        """The message names the tag's number and no other three-digit number (listing candidates is not reading)."""
        return set(re.findall(r"(?<!\d)\d{3}(?!\d)", text)) == {str(b.inst["goal"]["reading"])}

    def solve(self, r, inst: dict) -> None:
        r.inspect("tag")
        r.done_text = f"the tag reads {r.privileged_reading('tag')}"


# ---- 8. fetch and deliver (mobile manipulation) ------------------------------------------------------------------

class FetchDeliver(Template):
    id = "fetch-and-deliver"
    title = "Fetch an object from one surface and deliver it to another"
    capabilities = ("long-horizon", "navigation", "pick-and-place")
    difficulty = "hard"
    variants = ({"mounts": ["floor"], "caps": ["locomote", "grasp", "reach"]},)
    needs = {"surfaces": 2}

    def generate(self, ctx: Ctx) -> dict:
        regs = ctx.regions()
        by_surface = {}
        for r_ in regs:
            by_surface.setdefault(r_.surface, []).append(r_)
        if len(by_surface) < 2:
            raise Incompatible("fetch-and-deliver needs two work surfaces this embodiment can reach; the scene has "
                               f"{len(by_surface)}")
        kinds = ctx.kinds()
        if not kinds:
            raise Incompatible("the scene stocks no object this gripper can pick up")
        names = list(by_surface)
        best = None
        for a in names:
            for bname in names:
                if a == bname:
                    continue
                sa, sb = ctx.layout.surfaces[a], ctx.layout.surfaces[bname]
                dd = math.hypot(sa.c[0] - sb.c[0], sa.c[1] - sb.c[1])
                if best is None or dd > best[0] or (dd == best[0] and ctx.rng.random() < .5):
                    best = (dd, a, bname)
        _, src, dst = best
        k = kinds[int(ctx.rng.integers(len(kinds)))]
        col = pick_colors(ctx, 1)[0]
        reg, x, y = ctx.spot(by_surface[src], k)
        obj = {"name": f"{col}_{k}", "kind": k, "color": col, "pos": [x, y, reg.z], "yaw": 0.0, "on": src}
        dreg, dx, dy = ctx.spot(by_surface[dst], "zone")
        zone = {"name": "drop_zone", "kind": "zone", "color": "green", "pos": [dx, dy, dreg.z], "half": [.1, .1], "on": dst}
        S = ctx.layout.surfaces
        return {"objects": [obj], "fixtures": [zone], "goal": {"object": obj["name"], "zone": "drop_zone", "surface": dst, "from": src},
                "task": f"Bring the {label(obj)} from {S[src].label} to {S[dst].label} and leave it standing on the green mark there.",
                "success": (f"the {label(obj)} stands on {S[dst].label} (touching its top, tilted less than 20 degrees) with its centre on "
                            "the green mark, and no part of the robot touches it"),
                "steps": 3600}

    def check(self, b) -> tuple[bool, dict]:
        g = b.inst["goal"]
        z = b.fixture(g["zone"])
        p = b.obj_pos(g["object"])
        det = {"on_surface": b.touches_surface(g["object"], g["surface"]),
               "on_mark": abs(p[0] - z["pos"][0]) <= z["half"][0] and abs(p[1] - z["pos"][1]) <= z["half"][1],
               "upright": b.obj_tilt(g["object"]) < 20, "released": not b.touch_robot(g["object"])}
        return all(det.values()), det

    def solve(self, r, inst: dict) -> None:
        g = inst["goal"]
        z = next(f for f in inst["fixtures"] if f["name"] == g["zone"])
        r.pick(g["object"])
        r.place_on_point(g["object"], z["pos"][0], z["pos"][1], z["pos"][2], surface=g["surface"])


# ---- 9. bimanual relay ------------------------------------------------------------------------------------------

class BimanualRelay(Template):
    id = "bimanual-relay"
    title = "Move an object across a span no single arm reaches"
    capabilities = ("long-horizon", "bimanual-coordination")
    difficulty = "medium"
    variants = ({"mounts": ["bimanual"], "caps": ["bimanual", "grasp"]},)
    needs = {}

    def generate(self, ctx: Ctx) -> dict:
        L = ctx.layout
        am = L.mounts.get("bimanual")
        if not am:
            raise Incompatible("the scene has no bimanual station")
        kinds = ctx.kinds()
        if not kinds:
            raise Incompatible("the scene stocks no object this gripper can pick up")
        x0, x1, y0, y1 = am.work
        z = L.surfaces[am.surface].top
        left_first = bool(ctx.rng.random() < .5)
        sx = x0 + .05 if left_first else x1 - .05
        gx = x1 - .05 if left_first else x0 + .05
        k = kinds[int(ctx.rng.integers(len(kinds)))]
        col = pick_colors(ctx, 1)[0]
        yy = float(ctx.rng.uniform(y0 + .03, y1 - .03))
        obj = {"name": f"{col}_{k}", "kind": k, "color": col, "pos": [round(sx, 3), round(yy, 3), z], "yaw": 0.0, "on": am.surface}
        gy = float(ctx.rng.uniform(y0 + .03, y1 - .03))
        zone = {"name": "goal_mark", "kind": "zone", "color": "green", "pos": [round(gx, 3), round(gy, 3), z], "half": [.06, .06],
                "on": am.surface}
        return {"objects": [obj], "fixtures": [zone], "goal": {"object": obj["name"], "zone": "goal_mark"},
                "task": (f"Move the {label(obj)} onto the green mark at the other end of the station. The two points are too far apart "
                         "for one arm to reach both."),
                "success": (f"the {label(obj)} rests on the surface with its centre on the green mark (a 12 cm square), level (tilted less "
                            "than 15 degrees), and neither arm touches it"),
                "steps": 1400}

    def check(self, b) -> tuple[bool, dict]:
        g = b.inst["goal"]
        z = b.fixture(g["zone"])
        p = b.obj_pos(g["object"])
        det = {"on_mark": abs(p[0] - z["pos"][0]) <= z["half"][0] and abs(p[1] - z["pos"][1]) <= z["half"][1],
               "on_surface": b.touches_surface(g["object"]), "level": b.obj_tilt(g["object"]) < 15,
               "released": not b.touch_robot(g["object"])}
        return all(det.values()), det

    def solve(self, r, inst: dict) -> None:
        g = inst["goal"]
        z = next(f for f in inst["fixtures"] if f["name"] == g["zone"])
        r.relay(g["object"], z)


TEMPLATES: dict[str, Template] = {t.id: t for t in (PutInContainer(), StackInOrder(), ThreadRing(), SortByColor(), PushToZone(),
                                                     GoToLandmark(), InspectTag(), FetchDeliver(), BimanualRelay())}


def get(tid: str) -> Template:
    if tid not in TEMPLATES:
        raise KeyError(f"unknown task template {tid!r}; templates: {', '.join(TEMPLATES)}")
    return TEMPLATES[tid]
