"""Driving backend: a passenger car on MetaDrive's procedural roads (MetaDrive 0.4.3, Apache-2.0,
github.com/metadriverse/metadrive): Bullet vehicle dynamics, road maps built from blocks (straights, curves, roundabout,
intersections, on-ramps), IDM traffic and static road objects (cones, warning triangles, broken-down cars).

MetaDrive brings numpy 2, opencv, pygame, panda3d and gymnasium 1.x, so the simulator runs in its own virtualenv (default
~/.cache/robouse/metadrive-venv, override with ROBOUSE_METADRIVE_PYTHON) as a subprocess (worker.py) that speaks
JSON lines. This module is the trusted side: the embodiment declaration, the skills (closed-loop generators), the
success check and the reference solutions. Setup: docs/suites/driving.md.

Action (one 0.1 s step = one MetaDrive step): vehicle.controls [STEER, THROTTLE] in [-1, 1], MetaDrive's own action.
STEER x 40 deg front-wheel angle (positive = left); THROTTLE > 0 drives (engine force), < 0 brakes, 0 coasts. The hold
value (settle after `robo done`) is [0, -1]: full brake.

Skills: follow_route SPEED_KMH METERS (lane keeping by a Stanley controller on the route's lane centre line plus PI cruise
control; it does not brake for traffic or obstacles), change_lane left|right, stop [METERS], wait SECONDS.

Success (every task): after `robo done` and the 10-step settle (full brake, wheels straight), the car is at rest
(< 2 km/h) inside the destination zone (MetaDrive's arrive_dest region: from 5 m before the end of the route's final
road to 5 m past it, any lane of that road), and at no time did it collide with a vehicle or object, touch the road
edge or sidewalk, go out of road (off the lanes or onto a solid lane line: MetaDrive's out_of_road rule), or cut off
another driver (a vehicle behind in the car's lane closing in with a time gap under 1 s). The roundabout task adds a
give-way rule (`yield_rule`, published in the observation as `route.give_way`).

Each task's scene is fixed (map blocks, map seed, scripted traffic timetable); the episode seed (`--seed`) is ignored.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

from ... import config
from ...workers.client import StdioWorker
from ..base import StepInfo
from ..embodied import ActionGroup, Budget, EmbodiedBackend, Embodiment, Oracle, Sensor, Skill, SkillArg

WORKER = Path(__file__).with_name("worker.py")
CONTROL_DT = 0.1
MAX_KMH = 80.0  # MetaDrive's default car: no engine force above 80 km/h

# Road names are MetaDrive node pairs: ">", ">>", ">>>" are the start block; "<i><B><socket>_<road>_" belongs to block i of
# type B (S straight, C curve, O roundabout, X intersection, r on-ramp). Lane 0 is the leftmost lane (next to the centre line).
_MAIN = [
    ((">", ">>"), 0.0, 10.0),
    ((">>", ">>>"), 10.0, 50.0),
    ((">>>", "1S0_0_"), 50.0, 121.3),
    (("1S0_0_", "2r0_0_"), 121.3, 179.9),
]


def _main_road(x: float) -> list:
    """Ramp-merge map: the main-road road containing x (the main road runs along +x from x = 0)."""
    return list(next(r for r, a, b in _MAIN if a <= x < b))


def _main_road_s(x: float) -> float:
    return x - next(a for r, a, b in _MAIN if a <= x < b)


DENSE = dict(
    gap_m=5.0, headway_s=1.0
)  # scripted streams: IDM with a 5 m + 1 s desired gap (MetaDrive's default: 10 m + 1.5 s)

TASKS: dict[str, dict] = {
    # lane keeping on a winding two-lane road (tight S-bends), no traffic
    "driving-winding-road": dict(map="CSCC", seed=0, lanes=2, spawn=[">", ">>", 1], max_steps=1000),
    # three-lane highway: a slow truck and a slow car ahead, faster cars coming from behind in the left lane; the budget
    # is too short to stay behind the truck
    "driving-highway-overtake": dict(
        map="SSSSSSSS",
        seed=1,
        lanes=3,
        spawn=[">", ">>", 2],
        max_steps=500,
        vehicles=[
            dict(type="xl", lane=[">>>", "1S0_0_", 2], s=0, speed_kmh=30),
            dict(type="m", lane=[">>>", "1S0_0_", 1], s=30, speed_kmh=34),
            dict(type="l", lane=["2S0_0_", "3S0_0_", 2], s=20, speed_kmh=32),
            dict(type="m", lane=["3S0_0_", "4S0_0_", 1], s=10, speed_kmh=36),
            dict(type="m", lane=[">", ">>", 0], s=0, speed_kmh=70),
            dict(type="s", lane=[">", ">>", 0], s=0, speed_kmh=72, t=4.0),
        ],
    ),
    # a broken-down car with a warning triangle blocks the right lane; cars pass in the left lane
    "driving-broken-down-car": dict(
        map="SCS",
        seed=2,
        lanes=2,
        spawn=[">", ">>", 1],
        max_steps=800,
        vehicles=[
            dict(type="m", lane=[">>>", "1S0_0_", 1], s=45, broken_down=True),
            dict(type="s", lane=[">", ">>", 0], s=0, speed_kmh=50, t=2.0),
            dict(type="m", lane=[">", ">>", 0], s=0, speed_kmh=50, t=5.0),
            dict(type="l", lane=[">", ">>", 0], s=0, speed_kmh=48, t=8.0),
        ],
        objects=[dict(kind="warning_triangle", lane=[">>>", "1S0_0_", 1], s=33)],
    ),
    # a construction zone on a curve closes the two right lanes of three; a slower car drives in the open lane
    "driving-cone-zone": dict(
        map="SCS",
        seed=4,
        lanes=3,
        spawn=[">", ">>", 2],
        max_steps=900,
        cones=dict(road=["1S0_0_", "2C0_0_"], s=30.0, length=45.0),
        vehicles=[dict(type="m", lane=[">>>", "1S0_0_", 0], s=20, speed_kmh=35)],
    ),
    # a two-lane roundabout; circulating cars (they do not yield) pass the entry in a steady stream, then a gap
    "driving-roundabout": dict(
        map="SOS",
        seed=0,
        lanes=2,
        spawn=[">", ">>", 1],
        max_steps=1200,
        yield_rule=dict(road="1S0_0_->2O0_0_", point=[136.3, -5.1], window_s=4.0),
        vehicles=[
            dict(
                type=("m", "s", "l", "m", "s")[i], lane=road + [1], s=st, speed_kmh=30, dest="2O0_3_", **DENSE
            )  # circulating
            for i, (road, st) in enumerate(
                (
                    (["2O3_1_", "2O0_0_"], 8.0),
                    (["2O3_0_", "2O3_1_"], 18.0),
                    (["2O3_0_", "2O3_1_"], 0.5),
                    (["2O2_1_", "2O3_0_"], 10.0),
                    (["2O2_0_", "2O2_1_"], 20.0),
                )
            )
        ]
        + [
            dict(
                type=("m", "s", "l", "m")[i % 4],
                lane=["-2O2_3_", "-2O2_2_", 1],
                s=0,
                speed_kmh=30,
                t=2.1 * i,
                dest="2O0_3_",
                **DENSE,
            )
            for i in range(10)
        ],
    ),  # entering from the north
    # unprotected left turn at a four-way intersection (no traffic lights) across a stream of oncoming cars
    "driving-intersection-left-turn": dict(
        map="SXS",
        seed=0,
        lanes=2,
        spawn=[">", ">>", 0],
        max_steps=900,
        vehicles=[
            dict(
                type=("m", "s", "l", "m")[i % 4],
                lane=["-2X1_1_", "-2X1_0_", i % 2],
                s=0,
                speed_kmh=40,
                t=3.0 + 1.0 * i,
                dest="->",
                **DENSE,
            )
            for i in range(24)
        ]
        + [
            dict(type=ty, lane=["-2X1_1_", "-2X1_0_", ln], s=0, speed_kmh=40, t=t, dest="->")
            for ty, ln, t in (("s", 0, 48.0), ("m", 1, 51.0))
        ],
    ),
    # join a highway from an on-ramp: a short acceleration lane and a long platoon in the right lane
    "driving-ramp-merge": dict(
        map="SrS",
        seed=0,
        lanes=3,
        spawn=["2r1_0_", "2r1_1_", 0],
        spawn_s=2.0,
        max_steps=800,
        vehicles=[
            dict(
                type=("m", "l", "m", "s", "xl", "m")[k % 6],
                lane=_main_road(175.0 - 20.0 * k) + [2],
                s=_main_road_s(175.0 - 20.0 * k),
                speed_kmh=50,
            )
            for k in range(9)
        ]
        + [
            dict(type=("s", "m", "l", "m", "m")[k], lane=[">", ">>", 2], s=0.0, speed_kmh=50, t=0.35 + 1.44 * k)
            for k in range(5)
        ]
        + [dict(type="m", lane=[">>>", "1S0_0_", 1], s=30, speed_kmh=62)],
    ),
}


def _cone_taper(road, s0, length, lanes, width=3.5, closed=2):
    """Cones closing the `closed` rightmost lanes of `road`: a taper from the right edge, a straight row, no exit taper
    (the zone runs to the end of the road). Positions are (lane, station, lateral offset to the right of that lane)."""
    out = []
    last = lanes - 1
    taper = 18.0
    n = 7
    for i in range(n):  # taper: from the right edge to the left line of the leftmost closed lane
        frac = i / (n - 1)
        off = (0.5 * width + 0.1) - frac * (
            closed * width - 0.2
        )  # offset from the centre of the rightmost lane (+ = right)
        out.append(dict(kind="cone", lane=[*road, last], s=s0 + frac * taper, lat=off))
    k = 0
    s = s0 + taper + 3.0
    while s < s0 + taper + length:
        out.append(dict(kind="cone", lane=[*road, last], s=s, lat=0.5 * width - closed * width + 0.3))
        s += 3.0
        k += 1
    out.append(dict(kind="barrier", lane=[*road, last], s=s0 + taper + 2.0, lat=0.0))
    return out


def task_config(tid: str) -> dict:
    t = dict(TASKS[tid])
    if "cones" in t:
        c = t.pop("cones")
        t["objects"] = list(t.get("objects", [])) + _cone_taper(c["road"], c["s"], c["length"], t["lanes"])
    return t


class DrivingBackend(EmbodiedBackend):
    name = "driving"
    image_flipped = False

    def __init__(self, spec: dict):
        super().__init__(spec)
        env = spec.get("env") or spec.get("id")
        if env not in TASKS:
            raise KeyError(f"unknown driving task {env!r}")
        self.task_id, self.task = env, task_config(env)
        self.camera = spec.get("camera", "topdown")
        self.render_size = tuple(spec.get("render_size", (480, 480)))
        self.decl = self._declare()
        # run from the worker's own folder so no stray module (e.g. a profile.py in the cwd) shadows the stdlib
        self.worker = StdioWorker(
            "metadrive",
            config.sim_python("metadrive", "metadrive"),
            WORKER,
            setup="docs/suites/driving.md",
            env={
                "PYGAME_HIDE_SUPPORT_PROMPT": "1",
                "SDL_VIDEODRIVER": "dummy",
                "SDL_AUDIODRIVER": "dummy",
                "AUDIODEV": "null",
            },
        )
        self._state = None
        self._new_events: list[dict] = []
        self.last_judge: dict = {}

    def _declare(self) -> Embodiment:
        sensors = [
            Sensor(
                "ego",
                "proprio",
                ["ego"],
                "position (m, map frame), heading_deg (0 = +x, counter-clockwise), speed_kmh, last steering and throttle",
                frame="world",
            ),
            Sensor(
                "lane",
                "world",
                ["lane"],
                "current road, lane index (0 = leftmost) and count, lateral offset from the lane centre (m, + = left), lane line types, road shape",
            ),
            Sensor(
                "route",
                "world",
                ["route"],
                "navigation: distance travelled and remaining, route_completion, next roads (distance, lanes, shape), destination",
            ),
            Sensor(
                "vehicles",
                "world",
                ["vehicles"],
                "vehicles within 60 m: position in the car's frame (ahead, left), distance along the route, road, lane, speed, heading",
                frame="body",
            ),
            Sensor(
                "obstacles",
                "world",
                ["obstacles"],
                "cones, warning triangles and barriers within 60 m: position in the car's frame, distance along the route, lane",
                frame="body",
            ),
            Sensor("events", "events", ["events"], "collisions, leaving the road, crossing a solid line"),
            Sensor(
                "camera:topdown",
                "camera",
                mount="world",
                doc="top-down view following the car, map +y up, 5 px per metre",
            ),
            Sensor("camera:route_map", "camera", mount="world", doc="top-down view of the whole route"),
        ]
        groups = [
            ActionGroup(
                "vehicle.controls",
                "vehicle_controls",
                ["STEER", "THROTTLE"],
                [-1.0, -1.0],
                [1.0, 1.0],
                "STEER x 40 deg front-wheel angle (+ = left); THROTTLE > 0 engine, < 0 brake",
                "MetaDrive's own car action; zero throttle coasts, so the hold value brakes",
                hold=[0.0, -1.0],
                frame="body",
            )
        ]
        speed = SkillArg("speed_kmh", "float", "km/h", doc=f"cruise speed, 0 to {MAX_KMH:.0f}")
        skills = [
            Skill(
                "follow_route",
                [speed, SkillArg("meters", "float", "m", doc="distance to drive before the skill returns")],
                "keep the current lane along the navigation route at a set speed (lane-centred steering plus cruise control). "
                "It does NOT brake for vehicles or obstacles; it reports the distance driven, the speed and the nearest vehicle or object ahead in the lane",
                400,
            ),
            Skill(
                "change_lane",
                [
                    SkillArg("direction", "enum", choices=["left", "right"]),
                    SkillArg(
                        "speed_kmh", "float", "km/h", -1.0, doc="speed during the change; -1 keeps the current speed"
                    ),
                ],
                "steer into the adjacent lane and centre in it; it does not check for traffic or solid lines",
                80,
            ),
            Skill(
                "stop",
                [SkillArg("meters", "float", "m", 0.0, doc="come to rest after about this distance; 0 = brake hard")],
                "brake to a standstill while keeping the lane; reports where the car stopped",
                200,
            ),
            Skill("wait", [SkillArg("seconds", "float", "s", 1.0)], "stay stopped (brakes on) for SECONDS", 100),
        ]
        return Embodiment(
            robot="MetaDrive sedan (Bullet vehicle dynamics)",
            family="wheeled",
            assets=[],
            sensors=sensors,
            action_groups=groups,
            skills=skills,
            budget=Budget(int(self.task["max_steps"]), CONTROL_DT),
            cameras=["topdown", "route_map"],
        )

    # ---- worker ------------------------------------------------------------------------------------------------
    def _call(self, cmd: str, **kw):
        return self.worker.call(cmd, **kw)

    # ---- episode -----------------------------------------------------------------------------------------------
    def reset(self, seed: int) -> None:
        self.meta = self._call("make", task=self.task)
        self._state = None
        self.events, self._event_keys = [], set()
        self._new_events = []

    def control_step(self, a: np.ndarray) -> None:
        r = self._call("step", action=[float(a[0]), float(a[1])])
        self._state = None
        for e in r["new_events"]:
            if e["key"] not in self._event_keys:
                self._event_keys.add(e["key"])
                ev = {"key": e["key"], "kind": e["kind"], "detail": e["detail"], "t": e["t"]}
                self.events.append(ev)
                self._new_events.append(ev)

    def step(self, action) -> StepInfo:
        a = self.action_spec.clip(action)
        self.control_step(a)
        return StepInfo(success=False)

    def observe(self) -> dict:
        if self._state is None:
            self._state = self._call("observe")
        return json.loads(json.dumps(self._state))

    def judge_detail(self) -> dict:
        """Why the episode scored what it did: zone, speed at the end, events (also kept in self.last_judge)."""
        try:
            j = self._call("judge")
        except RuntimeError as e:
            j = {"success": False, "error": str(e)}
        j["success"] = bool(j.get("success")) and not self.events
        self.last_judge = j
        return j

    def success(self) -> bool:
        return bool(self.judge_detail()["success"])  # False (never raises) when the worker has died

    def judge(self, outcome: str, text: str = "") -> bool:
        detail = self.judge_detail()
        return outcome == "done" and bool(detail["success"])

    def mj_model_data(self):
        return None

    def render(self, width: int | None = None, height: int | None = None) -> np.ndarray:
        w, h = (width, height) if width and height else self.render_size
        return self._call("render", width=int(w), height=int(h), camera=self.camera or "topdown")

    def render_record(self, width: int, height: int) -> np.ndarray:
        # the follow camera at 12 px per metre (the agent's 320 px view is 5 px per metre): the cars stay legible
        return self._call("render", width=int(width), height=int(height), camera=self.camera or "topdown", scale=12.0)

    def close(self) -> None:
        self.worker.close()

    # ---- skills ------------------------------------------------------------------------------------------------
    def _ahead(self) -> dict | None:
        """Nearest vehicle or object ahead in the car's lane (from the public observation)."""
        st = self.observe()
        best = None
        for kind, items in (("vehicle", st["vehicles"]), ("object", st["obstacles"])):
            for o in items:
                if o.get("in_my_lane") and o.get("route_ahead_m", -1) > 0:
                    if best is None or o["route_ahead_m"] < best["route_ahead_m"]:
                        best = {
                            "what": kind,
                            **{
                                k: o[k]
                                for k in ("id", "kind", "route_ahead_m", "speed_kmh", "gap_m", "broken_down")
                                if k in o
                            },
                        }
        return best

    def _events_since(self, n0: int) -> list[str]:
        return [f"{e['key']}: {e['detail']}" for e in self.events[n0:]]

    def skill_follow_route(self, speed_kmh: float, meters: float):
        v = float(np.clip(speed_kmh, 0.0, MAX_KMH))
        if meters <= 0:
            return {"error": "METERS must be positive"}
        n0 = len(self.events)
        f = self._call("follow", speed_kmh=v, reset=True)
        odo0, note = f["odometer_m"], ""
        while True:
            if f["odometer_m"] - odo0 >= meters:
                break
            if f["remaining_m"] <= 0.0:
                note = "reached the end of the route; the car is still moving"
                break
            yield f["action"]
            if len(self.events) > n0:
                note = "stopped early: " + "; ".join(self._events_since(n0))
                break
            f = self._call("follow", speed_kmh=v)
        out = {
            "driven_m": round(f["odometer_m"] - odo0, 1),
            "speed_kmh": f["speed_kmh"],
            "lane": f["lane"],
            "remaining_m": f["remaining_m"],
            "ahead_in_lane": self._ahead(),
        }
        if note:
            out["note"] = note
        return out

    def skill_change_lane(self, direction: str, speed_kmh: float = -1.0):
        st = self.observe()
        k, n = st["lane"]["index"], st["lane"]["count"]
        tk = k - 1 if direction == "left" else k + 1
        if not 0 <= tk < n:
            return {"error": f"there is no lane to the {direction} (lane {k} of {n}, 0 = leftmost)"}
        v = st["ego"]["speed_kmh"] if speed_kmh < 0 else float(np.clip(speed_kmh, 0.0, MAX_KMH))
        n0 = len(self.events)
        f = self._call("follow", speed_kmh=v, lane=tk, reset=True)
        steps, done = 0, False
        while steps < 80:
            if abs(f["target_lateral_m"]) < 0.3 and abs(f["heading_error_deg"]) < 3.0 and f["lane"] == tk:
                done = True
                break
            yield f["action"]
            steps += 1
            if len(self.events) > n0:
                break
            f = self._call("follow", speed_kmh=v, lane=tk)
        out = {
            "completed": done,
            "lane": f["lane"],
            "lane_count": f["lane_count"],
            "speed_kmh": f["speed_kmh"],
            "offset_from_target_lane_m": f["target_lateral_m"],
            "ahead_in_lane": self._ahead(),
        }
        if len(self.events) > n0:
            out["events"] = self._events_since(n0)
        return out

    def skill_stop(self, meters: float = 0.0):
        n0 = len(self.events)
        f = self._call("follow", speed_kmh=0.0, reset=True, brake=-1.0)
        odo0, v0 = f["odometer_m"], f["speed_kmh"] / 3.6
        a = v0 * v0 / (2 * meters) if meters > 0.5 and v0 > 0.1 else None
        rest = 0
        while rest < 3:
            if f["speed_kmh"] < 0.5:
                rest += 1
            if a is None:
                thr = -1.0
            else:
                d = f["odometer_m"] - odo0
                vt = math.sqrt(max(0.0, v0 * v0 - 2 * a * d))
                thr = float(np.clip(-a / 8.0 + 0.6 * (vt - f["speed_kmh"] / 3.6), -1.0, 0.3))
                if vt < 0.5:
                    thr = -1.0
            yield [f["action"][0], thr]
            f = self._call("follow", speed_kmh=0.0, brake=thr)
        out = {
            "stopped": f["speed_kmh"] < 1.0,
            "stopping_distance_m": round(f["odometer_m"] - odo0, 1),
            "remaining_m": f["remaining_m"],
            "in_destination_zone": self.observe()["route"]["in_destination_zone"],
        }
        if len(self.events) > n0:
            out["events"] = self._events_since(n0)
        return out

    def skill_wait(self, seconds: float = 1.0):
        n = max(1, int(round(seconds / CONTROL_DT)))
        for _ in range(n):
            yield [0.0, -1.0]
        return {"waited_s": round(n * CONTROL_DT, 1), "speed_kmh": self.observe()["ego"]["speed_kmh"]}


# ---- reference solutions (socket only, public observation) ----------------------------------------------------------
# A scripted driver made of the public skills: it reads `robo observe`, drives in short follow_route chunks and decides
# speed, lane changes and waiting itself, like an agent would.


def _curve_limit(st: dict, lookahead: float = 45.0, a_lat: float = 2.2) -> float:
    """Speed (km/h) for the tightest curve on the current road or starting within `lookahead` metres."""
    shapes = [(0.0, st["lane"]["shape"])] + [(r["starts_in_m"], r["shape"]) for r in st["route"]["next_roads"]]
    v = 1e9
    for d, sh in shapes:
        if d > lookahead or not sh.startswith("curve"):
            continue
        radius = float(sh.split("radius ")[1].split(" m")[0])
        v = min(v, math.sqrt(a_lat * radius) * 3.6)
    return v


def _in_lane(st: dict, lane: int, lo: float, hi: float) -> list[dict]:
    """Vehicles and objects on the route in `lane`, between lo and hi metres ahead along the route."""
    return [
        o
        for o in st["vehicles"] + st["obstacles"]
        if o.get("on_route") and o.get("lane") == lane and lo < o["route_ahead_m"] < hi
    ]


def _lead(st: dict, lane: int | None = None, rng: float = 70.0) -> dict | None:
    k = st["lane"]["index"] if lane is None else lane
    ahead = _in_lane(st, k, 0.0, rng)
    return min(ahead, key=lambda o: o["route_ahead_m"]) if ahead else None


def _gap_ok(st: dict, lane: int, v_kmh: float, front: float = 12.0) -> bool:
    """A lane change into `lane` is safe: nothing alongside, nothing close ahead, and anything behind is far enough back
    for its closing speed (3 s time gap)."""
    v = v_kmh / 3.6
    for o in st["vehicles"] + st["obstacles"]:
        if not o.get("on_route") or o.get("lane") != lane:
            continue
        d = o["route_ahead_m"]
        ov = o.get("speed_kmh", 0.0) / 3.6
        if -7.0 < d < front + max(0.0, v - ov) * 2.0:
            return False
        if d <= -7.0 and (-d - 7.0) < max(0.0, ov - v) * 3.0 + 4.0:
            return False
    return True


def _speed_for(st: dict, cruise: float) -> float:
    """Cruise speed limited by curves ahead and by the vehicle or object ahead in the lane (keep a 2 s gap)."""
    v = min(cruise, _curve_limit(st))
    lead = _lead(st)
    if lead is not None:
        d = lead.get("gap_m", lead["route_ahead_m"] - 3.0)
        lv = lead.get("speed_kmh", 0.0)
        want = 6.0 + 2.0 * lv / 3.6
        if d < want + 25.0:
            v = min(v, max(0.0, lv + (d - want) * 0.8))
    return v


def _step_drive(o: Oracle, st: dict, cruise: float, chunk: float = 5.0) -> dict:
    v = _speed_for(st, cruise)
    if v < 3.0:
        if st["ego"]["speed_kmh"] > 3.0:
            lead = _lead(st)
            gap = lead.get("gap_m", lead["route_ahead_m"] - 3.0) if lead else 10.0
            o.skill("stop", max(1.0, gap - 5.0))
        else:
            o.skill("wait", 0.5)
    else:
        o.skill("follow_route", round(v, 1), chunk)
    return o.state()


def _arrive(o: Oracle, st: dict, cruise: float) -> None:
    """Drive on and come to rest inside the destination zone (the last 5 m of the route)."""
    while st["route"]["remaining_m"] > 22.0:
        st = _step_drive(o, st, cruise, chunk=min(5.0, st["route"]["remaining_m"] - 21.0))
    if st["ego"]["speed_kmh"] < 12.0 and st["route"]["remaining_m"] > 6.0:
        o.skill("follow_route", 12, st["route"]["remaining_m"] - 5.0)
        st = o.state()
    o.skill("stop", max(1.0, st["route"]["remaining_m"] - 1.5))


def _pass_blockage(o: Oracle, st: dict, cruise: float, pass_to: int, blocked_speed: float = 5.0) -> dict:
    """Change into lane `pass_to` (one lane at a time) as soon as the gap is safe; until then follow or stop behind."""
    while st["lane"]["index"] != pass_to:
        k = st["lane"]["index"]
        tgt = k - 1 if pass_to < k else k + 1
        v = max(st["ego"]["speed_kmh"], 15.0)
        if _gap_ok(st, tgt, v):
            o.skill("change_lane", "left" if tgt < k else "right", round(min(v, cruise), 1))
        else:
            st = _step_drive(o, st, cruise, chunk=3.0)
        st = o.state()
    return st


def _stop_before(o: Oracle, st: dict, road: str, cruise: float) -> dict:
    """Drive until `road` is the next road on the route and stop about 2.5 m before its start (a stop line)."""
    while st["lane"]["road"] != road:
        to_line = next((r["starts_in_m"] for r in st["route"]["next_roads"] if r["road"] == road), None)
        if to_line is not None and to_line < 30.0:
            o.skill("stop", max(1.0, to_line - 2.5))
            return o.state()
        st = _step_drive(o, st, cruise)
    return st


def _yield(o: Oracle, horizon_s: float) -> None:
    """Wait until no vehicle is within 14 m or closing in on the car within `horizon_s` seconds."""
    while True:
        st = o.state()
        danger = False
        for v in st["vehicles"]:
            spd = v.get("speed_kmh", 0.0) / 3.6
            hr = math.radians(v.get("heading_rel_deg", 0.0))
            closing = v["rel_ahead_m"] * math.cos(hr) + v["rel_left_m"] * math.sin(hr) < 0  # moving towards the car
            if v["distance_m"] < 14.0 or (closing and spd > 0.5 and v["distance_m"] < 14.0 + spd * horizon_s):
                danger = True
        if not danger:
            return
        o.skill("wait", 0.5)


def _eta_min(st: dict, point) -> float:
    """Shortest time (s) any listed vehicle needs to reach `point` (map frame), from the public observation."""
    ex, ey = st["ego"]["pos"]
    h = math.radians(st["ego"]["heading_deg"])
    best = 1e9
    for v in st["vehicles"]:
        px = ex + v["rel_ahead_m"] * math.cos(h) - v["rel_left_m"] * math.sin(h)
        py = ey + v["rel_ahead_m"] * math.sin(h) + v["rel_left_m"] * math.cos(h)
        vh = h + math.radians(v.get("heading_rel_deg", 0.0))
        spd = v.get("speed_kmh", 0.0) / 3.6
        dx, dy = px - point[0], py - point[1]
        dist = math.hypot(dx, dy)
        closing = -(dx * math.cos(vh) + dy * math.sin(vh)) * spd / max(dist, 1e-6)
        if dist < 8.0 or (closing > 0.3 and dist < 18.0):
            best = 0.0
        elif closing > 0.3:
            best = min(best, dist / closing)
    return best


def solution(env: str):
    def winding(o: Oracle) -> None:
        st = o.state()
        _arrive(o, st, 55.0)

    def highway(o: Oracle) -> None:
        st = o.state()
        cruise = 75.0
        while st["route"]["remaining_m"] > 60.0:
            lead = _lead(st, rng=45.0)
            k = st["lane"]["index"]
            if (
                lead is not None
                and lead.get("speed_kmh", 0) < cruise - 15
                and k > 0
                and _gap_ok(st, k - 1, st["ego"]["speed_kmh"])
            ):
                o.skill("change_lane", "left", max(35.0, st["ego"]["speed_kmh"]))
                st = o.state()
                continue
            st = _step_drive(o, st, cruise)
        _arrive(o, st, cruise)

    def blocked_lane(cruise: float):
        def solve(o: Oracle) -> None:
            st = o.state()
            while st["route"]["remaining_m"] > 30.0:
                block = [x for x in _in_lane(st, st["lane"]["index"], 0.0, 45.0) if x.get("speed_kmh", 0.0) < 1.0]
                if block and st["lane"]["index"] > 0:
                    st = _pass_blockage(o, st, cruise, st["lane"]["index"] - 1)
                    continue
                st = _step_drive(o, st, cruise)
            _arrive(o, st, cruise)

        return solve

    def roundabout(o: Oracle) -> None:
        st = _stop_before(o, o.state(), "1S0_0_->2O0_0_", 30.0)
        gw = st["route"]["give_way"]
        # give way: wait until no vehicle will reach the conflict point within the rule's window plus the time the car
        # needs to get there from the stop line (about 3 s)
        while _eta_min(o.state(), gw["point"]) < gw["window_s"] + 3.5:
            o.skill("wait", 0.5)
        o.skill("follow_route", 22, 30)
        _arrive(o, o.state(), 30.0)

    def intersection(o: Oracle) -> None:
        _stop_before(o, o.state(), "1S0_0_->2X2_0_", 40.0)
        _yield(o, horizon_s=5.0)
        o.skill("follow_route", 22, 32)
        _arrive(o, o.state(), 40.0)

    def ramp(o: Oracle) -> None:
        # stop near the end of the ramp and wait until the right lane is clear from 80 m back to 10 m ahead
        st = _stop_before(o, o.state(), "2r1_3_->2r0_0_", 25.0)
        while True:
            busy = [
                v
                for v in st["vehicles"]
                if v.get("lane") == 2
                and str(v.get("road", "")).split("->")[0] in (">>", ">>>", "1S0_0_", "2r0_0_")
                and -80.0 < v["rel_ahead_m"] < 12.0
                and v.get("speed_kmh", 0) > 5
            ]
            if not busy:
                break
            o.skill("wait", 0.5)
            st = o.state()
        while not (st["lane"]["count"] == 4 and st["lane"]["index"] == 3):
            o.skill("follow_route", 50, 4)
            st = o.state()
        # acceleration lane: merge left into the first safe gap
        while st["lane"]["index"] == 3:
            if _gap_ok(st, 2, st["ego"]["speed_kmh"], front=8.0):
                o.skill("change_lane", "left", max(40.0, st["ego"]["speed_kmh"]))
            else:
                o.skill("follow_route", 35, 3)
            st = o.state()
        _arrive(o, st, 55.0)

    table = {
        "driving-winding-road": winding,
        "driving-highway-overtake": highway,
        "driving-broken-down-car": blocked_lane(45.0),
        "driving-cone-zone": blocked_lane(45.0),
        "driving-roundabout": roundabout,
        "driving-intersection-left-turn": intersection,
        "driving-ramp-merge": ramp,
    }
    return table[env]


def oracle_main(env: str) -> None:
    Oracle().run(solution(env))
