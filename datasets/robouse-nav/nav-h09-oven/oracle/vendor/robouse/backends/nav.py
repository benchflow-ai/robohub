"""Embodied navigation in indoor homes: object-goal navigation for a mobile base, in the protocol of Dimensional's
"Can Jev Nav?" study (research.dimensional.org/system-one-navigation, dimensionalOS/dimos, Apache-2.0).

Geometry-only simulator (numpy): the home is a set of 2-D boxes (walls and furniture) and labelled objects; the robot
is a 0.25 m radius holonomic base moved kinematically at 10 Hz like an agent on a navigation mesh: a move that would
intersect an obstacle slides along it or stops. No rendering engine or scene assets are needed, so it runs anywhere.

Observation (`robo observe`): a robot-frame WorldState in spatial language, as in dimos' TypeSafe agent: the goal and
visible objects with 8-way bearings (ahead, ahead_left, left, behind_left, behind, behind_right, right, ahead_right),
distance words (touching < 0.5 m, near < 1.5 m, mid < 4 m, far) and metres, whether the way to the target is blocked and
by what, free space along 8 directions, and recent motion (moved, turned, target closer, stuck). No world coordinates
are given unless the task sets `nav.world_coords`.

Action (`robo act VX VY WZ`): body-frame velocity (m/s, m/s, deg/s; up to 0.5, 0.3, 46), 0.1 s per step. Skills:
`drive X Y YAW [SECONDS]` with Dimensional's discrete choices (x: forward|none|backward, y: left|none|right,
yaw: turn_left|none|turn_right; 0.5 m/s and 0.8 rad/s, held for SECONDS, default 0.5 = one 2 Hz decision), `turn DEG`,
`forward METRES`.

Success: after `robo done`, the base is within `success_radius` (1.0 m) of the target object's footprint with line of
sight to it. Metrics (Dimensional's definitions): SPL = S * l / max(p, l), SoftSPL = max(0, 1 - d_final / l) * l / max(p, l)
with l the geodesic shortest path and d_final the remaining geodesic distance, arrival, collisions (a linear command of
at least 0.1 m/s achieving under 20 % of it for 0.5 s), time to target, path ratio p / l, excess turning (rad per metre
driven beyond the turning of the reference path).
"""
from __future__ import annotations

import heapq
import math

import numpy as np

from .base import ActionSpec, StepInfo
from .embodied import ActionGroup, Budget, Embodiment, EmbodiedBackend, Sensor, Skill, SkillArg

DT = 0.1
RADIUS = 0.25
CELL = 0.1
V_LIN, V_LAT, W_MAX = 0.5, 0.3, math.degrees(0.8)
BEARINGS = ["ahead", "ahead_left", "left", "behind_left", "behind", "behind_right", "right", "ahead_right"]


def bearing_word(deg: float) -> str:
    d = (deg + 180) % 360 - 180
    if abs(d) <= 15:
        return "ahead"
    return BEARINGS[int(((d + 22.5) % 360) // 45)] if abs(d) > 15 else "ahead"


def distance_word(m: float) -> str:
    return "touching" if m < 0.5 else "near" if m < 1.5 else "mid" if m < 4.0 else "far"


def _box_dist(p, box) -> float:
    cx, cy, sx, sy = box[:4]
    dx = max(abs(p[0] - cx) - sx / 2, 0.0)
    dy = max(abs(p[1] - cy) - sy / 2, 0.0)
    return math.hypot(dx, dy)


def _seg_hits_box(a, b, box, pad: float = 0.0) -> float | None:
    """Parameter t in [0, 1] where segment a->b first enters the (padded) box, or None (slab test)."""
    cx, cy, sx, sy = box[:4]
    lo = (cx - sx / 2 - pad, cy - sy / 2 - pad)
    hi = (cx + sx / 2 + pad, cy + sy / 2 + pad)
    t0, t1 = 0.0, 1.0
    for i in range(2):
        d = b[i] - a[i]
        if abs(d) < 1e-12:
            if a[i] < lo[i] or a[i] > hi[i]:
                return None
            continue
        ta, tb = (lo[i] - a[i]) / d, (hi[i] - a[i]) / d
        if ta > tb:
            ta, tb = tb, ta
        t0, t1 = max(t0, ta), min(t1, tb)
        if t0 > t1:
            return None
    return t0


class NavBackend(EmbodiedBackend):
    name = "nav"

    def __init__(self, spec: dict):
        super().__init__(spec)
        n = spec["nav"]
        self.nav = n
        sc = n["scene"]
        self.walls = [list(map(float, w[:4])) for w in sc["walls"]]
        self.objects = [dict(o, box=list(map(float, o["box"][:4]))) for o in sc.get("objects", [])]
        self.target = dict(n["target"], box=list(map(float, n["target"]["box"][:4])))
        self.bounds = list(map(float, sc["bounds"]))  # x0, y0, x1, y1
        self.radius = float(n.get("success_radius", 1.0))
        self.obstacles = self.walls + [o["box"] for o in self.objects if not o.get("passable")]
        self.world_coords = bool(n.get("world_coords", False))
        self.camera = "local"
        self._field = None
        self.decl = Embodiment(
            robot=n.get("robot", "mobile base (0.25 m radius, holonomic)"), family="mobile_base", assets=[],
            sensors=[Sensor("world_state", "world", ["goal", "objects", "way_to_target", "free_space", "robot"],
                            doc="robot-frame spatial-language WorldState"),
                     Sensor("camera:local", "camera", mount="body", doc="egocentric top-down view of what the robot can see")],
            action_groups=[ActionGroup("base.twist", "base_twist", ["vx", "vy", "wz"], [-V_LIN, -V_LAT, -W_MAX], [V_LIN, V_LAT, W_MAX],
                                       units="m/s, m/s, deg/s", frame="body", doc="body-frame velocity: x forward, y left, wz counter-clockwise")],
            skills=[Skill("drive", [SkillArg("x", "enum", choices=["forward", "none", "backward"]),
                                    SkillArg("y", "enum", choices=["left", "none", "right"]),
                                    SkillArg("yaw", "enum", choices=["turn_left", "none", "turn_right"]),
                                    SkillArg("seconds", "float", "s", 0.5)],
                          "Dimensional's discrete drive command (0.5 m/s, 0.8 rad/s) held for SECONDS", max_steps=50),
                    Skill("turn", [SkillArg("deg", "float", "deg")], "turn in place by DEG (counter-clockwise positive)", max_steps=80),
                    Skill("forward", [SkillArg("metres", "float", "m")], "drive straight ahead (stops at obstacles)", max_steps=120)],
            budget=Budget(max_steps=int(spec.get("max_steps", 1200)), control_dt=DT), cameras=["local"])

    # ---- geometry -----------------------------------------------------------------------------------------------------
    def _free(self, p) -> bool:
        x0, y0, x1, y1 = self.bounds
        if not (x0 + RADIUS <= p[0] <= x1 - RADIUS and y0 + RADIUS <= p[1] <= y1 - RADIUS):
            return False
        return all(_box_dist(p, b) >= RADIUS for b in self.obstacles)

    def _los(self, a, b, box_to_ignore=None) -> bool:
        for bx in self.walls + [o["box"] for o in self.objects]:
            if box_to_ignore is not None and bx is box_to_ignore:
                continue
            t = _seg_hits_box(a, b, bx)
            if t is not None and t < 1.0 - 1e-6:
                return False
        return True

    def _target_point(self, p):
        cx, cy, sx, sy = self.target["box"]
        return (min(max(p[0], cx - sx / 2), cx + sx / 2), min(max(p[1], cy - sy / 2), cy + sy / 2))

    def arrived(self, p=None) -> bool:
        p = self.pos if p is None else p
        if _box_dist(p, self.target["box"]) > self.radius:
            return False
        return self._los(p, self._target_point(p), self._target_box_ref())

    def _target_box_ref(self):
        for o in self.objects:
            if o.get("id") == self.target.get("id") and o["box"] == self.target["box"]:
                return o["box"]
        return None

    def _raycast(self, p, ang, max_d=6.0) -> tuple[float, str]:
        e = (p[0] + max_d * math.cos(ang), p[1] + max_d * math.sin(ang))
        best, what = 1.0, "open"
        for w in self.walls:
            t = _seg_hits_box(p, e, w)
            if t is not None and t < best:
                best, what = t, "wall"
        for o in self.objects:
            t = _seg_hits_box(p, e, o["box"])
            if t is not None and t < best:
                best, what = t, o["label"]
        return best * max_d, what

    # ---- geodesic distance field to the goal region ------------------------------------------------------------------
    def _grid(self):
        x0, y0, x1, y1 = self.bounds
        nx, ny = int(math.ceil((x1 - x0) / CELL)), int(math.ceil((y1 - y0) / CELL))
        xs = x0 + (np.arange(nx) + 0.5) * CELL
        ys = y0 + (np.arange(ny) + 0.5) * CELL
        X, Y = np.meshgrid(xs, ys, indexing="ij")
        free = (X >= x0 + RADIUS) & (X <= x1 - RADIUS) & (Y >= y0 + RADIUS) & (Y <= y1 - RADIUS)
        for b in self.obstacles:
            cx, cy, sx, sy = b
            dx = np.maximum(np.abs(X - cx) - sx / 2, 0)
            dy = np.maximum(np.abs(Y - cy) - sy / 2, 0)
            free &= np.hypot(dx, dy) >= RADIUS
        return xs, ys, free

    def _distance_field(self):
        if self._field is not None:
            return self._field
        xs, ys, free = self._grid()
        cx, cy, sx, sy = self.target["box"]
        dist = np.full(free.shape, np.inf)
        heap = []
        near = np.argwhere(free)
        tbox = self._target_box_ref()
        for i, j in near:
            p = (xs[i], ys[j])
            if _box_dist(p, self.target["box"]) <= self.radius and self._los(p, self._target_point(p), tbox):
                dist[i, j] = 0.0
                heap.append((0.0, int(i), int(j)))
        heapq.heapify(heap)
        steps = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0), (1, 1, 1.4142), (1, -1, 1.4142), (-1, 1, 1.4142), (-1, -1, 1.4142)]
        nx, ny = free.shape
        while heap:
            d, i, j = heapq.heappop(heap)
            if d > dist[i, j]:
                continue
            for di, dj, c in steps:
                a, b = i + di, j + dj
                if 0 <= a < nx and 0 <= b < ny and free[a, b]:
                    if di and dj and not (free[i + di, j] and free[i, j + dj]):
                        continue
                    nd = d + c * CELL
                    if nd < dist[a, b]:
                        dist[a, b] = nd
                        heapq.heappush(heap, (nd, a, b))
        self._field = (xs, ys, dist)
        return self._field

    def geodesic(self, p) -> float:
        xs, ys, dist = self._distance_field()
        i = int(np.clip(np.searchsorted(xs, p[0]) - 1, 0, len(xs) - 1))
        j = int(np.clip(np.searchsorted(ys, p[1]) - 1, 0, len(ys) - 1))
        best = np.inf
        for a in range(max(0, i - 2), min(len(xs), i + 3)):  # nearest reachable cell around p
            for b in range(max(0, j - 2), min(len(ys), j + 3)):
                if np.isfinite(dist[a, b]):
                    best = min(best, dist[a, b] + math.hypot(xs[a] - p[0], ys[b] - p[1]))
        return float(best)

    # ---- lifecycle ----------------------------------------------------------------------------------------------------
    def reset(self, seed: int) -> None:
        self.events, self._event_keys = [], set()
        self.pos = tuple(map(float, self.nav["spawn"][:2]))
        if not self._free(self.pos):  # imported scenes: a spawn inside the 2-D footprints moves to the nearest free point
            best = None
            for r in np.arange(0.05, 1.01, 0.05):
                for a in np.linspace(0, 2 * math.pi, 24, endpoint=False):
                    q = (self.pos[0] + r * math.cos(a), self.pos[1] + r * math.sin(a))
                    if self._free(q):
                        best = q
                        break
                if best:
                    break
            if best:
                self.spawn_shift_m = round(math.hypot(best[0] - self.pos[0], best[1] - self.pos[1]), 3)
                self.pos = (float(best[0]), float(best[1]))
        self.yaw = math.radians(float(self.nav.get("spawn_yaw_deg", 0.0)))
        self.t = 0.0
        self.path = [self.pos]
        self.turn_total = 0.0
        self.collisions = 0
        self._stall = 0.0
        self._bumped = False
        self.first_arrival = None
        self.last_cmd = (0.0, 0.0, 0.0)
        self.hist: list[tuple] = [(0.0, self.pos, self.yaw)]
        self.l_ref = float(self.nav.get("geodesic_m") or self.geodesic(self.pos))
        if not math.isfinite(self.l_ref):
            raise ValueError("the target cannot be reached from the spawn point")

    def _move(self, vx, vy, wz_deg) -> None:
        self.yaw += math.radians(wz_deg) * DT
        self.turn_total += abs(math.radians(wz_deg)) * DT
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        dx, dy = (vx * c - vy * s) * DT, (vx * s + vy * c) * DT
        p0 = self.pos
        for cand in ((p0[0] + dx, p0[1] + dy), (p0[0] + dx, p0[1]), (p0[0], p0[1] + dy)):  # full move, then slide
            if self._free(cand):
                self.pos = cand
                break
        moved = math.hypot(self.pos[0] - p0[0], self.pos[1] - p0[1])
        cmd = math.hypot(vx, vy)
        if cmd >= 0.1 and moved < 0.2 * cmd * DT:
            self._stall += DT
            if self._stall >= 0.5 and not self._bumped:
                self.collisions += 1
                self._bumped = True
                self.event(f"bump:{self.collisions}", "collision", f"blocked at ({self.pos[0]:.2f}, {self.pos[1]:.2f})")
        else:
            self._stall, self._bumped = 0.0, False
        self.t += DT
        self.path.append(self.pos)
        self.hist.append((self.t, self.pos, self.yaw))
        if self.first_arrival is None and self.arrived():
            self.first_arrival = self.t

    def control_step(self, a) -> None:
        vx, vy, wz = (float(x) for x in a)
        self.last_cmd = (vx, vy, wz)
        self._move(vx, vy, wz)

    def step(self, action) -> StepInfo:
        a = self.action_spec.clip(action)
        self.control_step(a)
        return StepInfo(success=self.arrived())

    def success(self) -> bool:
        return self.arrived()

    def hold_action(self):
        return [0.0, 0.0, 0.0]

    def metrics_point(self):
        return [self.pos[0], self.pos[1], 0.0]

    # ---- skills -------------------------------------------------------------------------------------------------------
    def skill_drive(self, x: str, y: str, yaw: str, seconds: float = 0.5):
        vx = {"forward": V_LIN, "none": 0.0, "backward": -V_LIN}[x]
        vy = {"left": V_LAT, "none": 0.0, "right": -V_LAT}[y]
        wz = {"turn_left": W_MAX, "none": 0.0, "turn_right": -W_MAX}[yaw]
        for _ in range(max(1, int(round(float(seconds) / DT)))):
            yield [vx, vy, wz]
        return {"drove_s": float(seconds)}

    def skill_turn(self, deg: float):
        n = max(1, int(math.ceil(abs(deg) / (W_MAX * DT))))
        for _ in range(n):
            yield [0.0, 0.0, deg / (n * DT)]
        return {"turned_deg": deg}

    def skill_forward(self, metres: float):
        start = self.pos
        for _ in range(max(1, int(math.ceil(abs(metres) / (V_LIN * DT))))):
            before = self.pos
            yield [math.copysign(V_LIN, metres), 0.0, 0.0]
            if math.hypot(self.pos[0] - before[0], self.pos[1] - before[1]) < 0.01:
                return {"moved_m": round(math.hypot(self.pos[0] - start[0], self.pos[1] - start[1]), 2), "stopped": "blocked"}
        return {"moved_m": round(math.hypot(self.pos[0] - start[0], self.pos[1] - start[1]), 2)}

    # ---- observation ----------------------------------------------------------------------------------------------------
    def _rel(self, q) -> tuple[float, float]:
        dx, dy = q[0] - self.pos[0], q[1] - self.pos[1]
        ang = math.degrees(math.atan2(dy, dx) - self.yaw)
        return (ang + 180) % 360 - 180, math.hypot(dx, dy)

    def observe(self) -> dict:
        tp = self._target_point(self.pos)
        gdeg, gdist = self._rel(tp)
        tbox = self._target_box_ref()
        goal = {"label": self.target["label"], "bearing": bearing_word(gdeg), "bearing_deg": round(gdeg, 1),
                "distance": distance_word(gdist), "distance_m": round(gdist, 2), "visible": self._los(self.pos, tp, tbox),
                "arrived": self.arrived()}
        objs = []
        for o in self.objects:
            q = self._target_point_of(o["box"])
            deg, dist = self._rel(q)
            if dist <= 6.0 and self._los(self.pos, q, o["box"]):
                objs.append({"label": o["label"], "bearing": bearing_word(deg), "bearing_deg": round(deg, 1),
                             "distance": distance_word(dist), "distance_m": round(dist, 2),
                             "width_m": round(max(o["box"][2], o["box"][3]), 2), "target": o["box"] == self.target["box"]})
        objs.sort(key=lambda o: o["distance_m"])
        d_ahead, what = self._raycast(self.pos, self.yaw + math.radians(gdeg), max_d=max(0.1, gdist))
        way = {"blocked": what != "open" and d_ahead < gdist - 0.05, "blocked_by": None if what == "open" else what,
               "clear_m": round(d_ahead, 2)}
        if way["blocked"]:
            way["open_sides"] = [side for side, off in (("left", 45), ("right", -45))
                                 if self._raycast(self.pos, self.yaw + math.radians(gdeg + off), 1.5)[0] >= 1.49]
        free = {}
        for k, name in enumerate(BEARINGS):
            d, _ = self._raycast(self.pos, self.yaw + math.radians(45 * k), 6.0)
            free[name] = {"clear_m": round(max(0.0, d - RADIUS), 2),
                          "state": "blocked" if d - RADIUS < 0.5 else "open" if d - RADIUS >= 1.5 else "narrow"}
        t_back, recent = self.t - 8.0, self.hist[0]
        for h in self.hist:
            if h[0] >= t_back:
                recent = h
                break
        moved = math.hypot(self.pos[0] - recent[1][0], self.pos[1] - recent[1][1])
        closer = self.geodesic(recent[1]) - self.geodesic(self.pos)
        vx, vy, wz = self.last_cmd
        motion = "stopped" if max(abs(vx), abs(vy), abs(wz)) < 1e-6 else ("forward" if vx > 0 else "backward" if vx < 0 else "turning")
        stuck = len(self.hist) > 30 and math.hypot(self.pos[0] - self.hist[-31][1][0], self.pos[1] - self.hist[-31][1][1]) < 0.15 and motion != "stopped"
        out = {"goal": goal, "objects": objs[:12], "way_to_target": way, "free_space": free,
               "robot": {"motion": motion, "last_drive": {"vx": vx, "vy": vy, "wz_deg_s": wz},
                         "recent": {"window_s": 8, "moved_m": round(moved, 2), "turned_deg": round(math.degrees(self.yaw - recent[2]), 1),
                                    "target_closer_m": round(closer, 2) if math.isfinite(closer) else None,
                                    "pattern": "stuck" if stuck else "progressing" if closer > 0.3 else "not_progressing"},
                         "collisions": self.collisions},
               "time_s": round(self.t, 1)}
        if self.world_coords:
            out["robot"]["pos"] = [round(self.pos[0], 2), round(self.pos[1], 2)]
            out["robot"]["yaw_deg"] = round(math.degrees(self.yaw), 1)
        return out

    def _target_point_of(self, box):
        cx, cy, sx, sy = box
        p = self.pos
        return (min(max(p[0], cx - sx / 2), cx + sx / 2), min(max(p[1], cy - sy / 2), cy + sy / 2))

    # ---- metrics ------------------------------------------------------------------------------------------------------
    def episode_metrics(self) -> dict:
        p = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(self.path, self.path[1:]))
        l = self.l_ref
        s = 1.0 if self.arrived() else 0.0
        d_final = self.geodesic(self.pos)
        ref = self.nav.get("reference") or []
        ref_turn = 0.0
        for a, b, c in zip(ref, ref[1:], ref[2:]):
            h1, h2 = math.atan2(b[1] - a[1], b[0] - a[0]), math.atan2(c[1] - b[1], c[0] - b[0])
            ref_turn += abs((h2 - h1 + math.pi) % (2 * math.pi) - math.pi)
        return {"arrived": int(s), "spl": round(s * l / max(p, l), 4),
                "soft_spl": round(max(0.0, 1 - (d_final if math.isfinite(d_final) else l) / l) * l / max(p, l), 4),
                "collisions": self.collisions, "time_to_target_s": self.first_arrival,
                "path_ratio": round(p / l, 3) if l > 0 else None,
                "excess_turning_rad_per_m": round(max(0.0, self.turn_total - ref_turn) / p, 3) if p > 0.05 else None,
                "geodesic_m": round(l, 2), "path_m": round(p, 2),
                "remaining_geodesic_m": round(d_final, 2) if math.isfinite(d_final) else None}

    def judge(self, outcome: str, text: str = "") -> bool:
        return outcome == "done" and self.arrived()

    # ---- rendering ----------------------------------------------------------------------------------------------------
    def _canvas(self, px_per_m: float, bounds):
        from PIL import Image, ImageDraw

        x0, y0, x1, y1 = bounds
        W, H = int((x1 - x0) * px_per_m), int((y1 - y0) * px_per_m)
        W, H = (W + 15) // 16 * 16, (H + 15) // 16 * 16
        img = Image.new("RGB", (W, H), (245, 243, 238))
        dr = ImageDraw.Draw(img)
        P = lambda x, y: ((x - x0) * px_per_m, H - (y - y0) * px_per_m)  # noqa: E731
        return img, dr, P

    def render(self, width: int = 480, height: int = 480) -> np.ndarray:
        if self.camera == "overview":
            return self.render_overview()
        # egocentric local view: only what the robot can see (ray fan) and visible objects, ahead is up
        from PIL import Image, ImageDraw

        S, R = 320, 6.0
        img = Image.new("RGB", (S, S), (40, 40, 44))
        dr = ImageDraw.Draw(img)
        k = S / (2 * R)
        c = (S / 2, S / 2)
        fan = [c]
        for i in range(0, 361, 3):
            ang = self.yaw + math.radians(i)
            d, _ = self._raycast(self.pos, ang, R)
            rel = math.radians(i) + math.pi / 2
            fan.append((c[0] + d * k * math.cos(rel), c[1] - d * k * math.sin(rel)))
        dr.polygon(fan, fill=(235, 232, 225))
        for o in self.objects:
            q = self._target_point_of(o["box"])
            deg, dist = self._rel(q)
            if dist <= R and self._los(self.pos, q, o["box"]):
                rel = math.radians(deg) + math.pi / 2
                x, y = c[0] + dist * k * math.cos(rel), c[1] - dist * k * math.sin(rel)
                col = (200, 60, 50) if o["box"] == self.target["box"] else (70, 110, 170)
                dr.ellipse([x - 5, y - 5, x + 5, y + 5], fill=col)
                dr.text((x + 6, y - 6), o["label"], fill=(20, 20, 20))
        dr.polygon([(c[0], c[1] - 10), (c[0] - 6, c[1] + 6), (c[0] + 6, c[1] + 6)], fill=(30, 150, 80))
        return np.asarray(img)

    def render_overview(self) -> np.ndarray:
        x0, y0, x1, y1 = self.bounds
        ppm = min(40.0, 640.0 / max(x1 - x0, y1 - y0))
        img, dr, P = self._canvas(ppm, self.bounds)
        for w in self.walls:
            cx, cy, sx, sy = w
            a, b = P(cx - sx / 2, cy + sy / 2), P(cx + sx / 2, cy - sy / 2)
            dr.rectangle([a, b], fill=(60, 60, 60))
        for o in self.objects:
            cx, cy, sx, sy = o["box"]
            a, b = P(cx - sx / 2, cy + sy / 2), P(cx + sx / 2, cy - sy / 2)
            tgt = o["box"] == self.target["box"]
            dr.rectangle([a, b], fill=(220, 90, 70) if tgt else (150, 170, 200))
        if self.target["box"] not in [o["box"] for o in self.objects]:
            cx, cy, sx, sy = self.target["box"]
            dr.rectangle([P(cx - sx / 2, cy + sy / 2), P(cx + sx / 2, cy - sy / 2)], fill=(220, 90, 70))
        if len(self.path) > 1:
            dr.line([P(*q) for q in self.path], fill=(30, 150, 80), width=2)
        x, y = P(*self.pos)
        r = RADIUS * ppm
        dr.ellipse([x - r, y - r, x + r, y + r], outline=(20, 110, 60), width=2)
        dr.line([(x, y), (x + 1.5 * r * math.cos(self.yaw), y - 1.5 * r * math.sin(self.yaw))], fill=(20, 110, 60), width=2)
        return np.asarray(img)

    def render_video(self) -> np.ndarray:
        return self.render_overview()

    def mj_model_data(self):
        return None


def oracle_main(env: str) -> None:
    """Reference solution: follow the geodesic distance field downhill with the body-velocity action (turn toward the
    next waypoint, drive forward), then stop and claim done. It reads only its own privileged state via the oracle token
    (world coordinates), which agents do not get."""
    import json
    from pathlib import Path

    from ..agent_cli import _send

    # env = oracle/nav_spec.json (the task's robouse block, written by the generator; JSON so the reference solution
    # needs no YAML parser in the agent image)
    spec = json.loads(Path(env).read_text())
    b = NavBackend(spec)
    b.reset(0)
    xs, ys, dist = b._distance_field()

    def act(v):
        r = _send({"op": "act", "action": v, "repeat": 1})
        if not r.get("ok") or "episode" in (r.get("result") or {}):
            raise StopIteration
        return r

    try:
        for _ in range(int(spec.get("max_steps", 1200)) - 2):
            b_pos = b.pos
            if b.arrived():
                break
            i = int(np.clip(np.searchsorted(xs, b_pos[0]) - 1, 0, len(xs) - 1))
            j = int(np.clip(np.searchsorted(ys, b_pos[1]) - 1, 0, len(ys) - 1))
            if not np.isfinite(dist[i, j]):  # snap to the nearest reachable cell
                near = [(abs(a - i) + abs(c - j), a, c) for a in range(max(0, i - 5), min(len(xs), i + 6))
                        for c in range(max(0, j - 5), min(len(ys), j + 6)) if np.isfinite(dist[a, c])]
                if near:
                    _, i, j = min(near)
            path = [(i, j)]
            for _k in range(40):  # walk the distance field downhill: the geodesic path from here
                a0, c0 = path[-1]
                if dist[a0, c0] == 0:
                    break
                nb = [(dist[a, c], a, c) for a in range(max(0, a0 - 1), min(len(xs), a0 + 2))
                      for c in range(max(0, c0 - 1), min(len(ys), c0 + 2)) if (a, c) != (a0, c0)]
                d_best, a1, c1 = min(nb)
                if not d_best < dist[a0, c0]:
                    break
                path.append((a1, c1))
            look = 4
            for k in range(min(len(path) - 1, 12), 0, -1):  # farthest of the next 1.2 m of path reachable in a straight line
                q = (xs[path[k][0]], ys[path[k][1]])
                if all(_seg_hits_box(b_pos, q, o, RADIUS * 0.95) is None for o in b.obstacles):
                    look = k
                    break
            else:
                look = 1
            k = min(look, len(path) - 1)
            tgt = (xs[path[k][0]], ys[path[k][1]])
            head = math.atan2(tgt[1] - b_pos[1], tgt[0] - b_pos[0])
            err = (head - b.yaw + math.pi) % (2 * math.pi) - math.pi
            wz = max(-W_MAX, min(W_MAX, math.degrees(err) / DT * 0.5))
            vx = V_LIN if abs(err) < math.radians(30) else 0.0
            act([vx, 0.0, wz])
            b.control_step([vx, 0.0, wz])  # mirror the server's kinematics
        _send({"op": "done", "text": "oracle arrived"})
    except StopIteration:
        return
