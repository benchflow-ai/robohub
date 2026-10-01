"""MetaDrive simulation worker for the Robo Use `driving` suite. Runs inside its own virtualenv (Python 3.11,
`metadrive-simulator==0.4.3`, which brings numpy 2, opencv, pygame, panda3d and gymnasium 1.x), because those pins must not
enter the main Robo Use environment.

The Robo Use `driving` backend (driving.py) starts this script as a subprocess and talks to it over stdin/stdout: one JSON
request per line, one JSON response per line; a `render` response is followed by the raw RGB bytes. Only numpy and
metadrive are imported here (never the robouse package).

Simulation: MetaDrive's procedural road map (a fixed block sequence and seed per task), Bullet vehicle dynamics for the
ego car and all traffic, MetaDrive's own IDM traffic (PGTrafficManager) plus the task's scripted scene: static obstacles
(cones, warning triangles, barriers, broken-down vehicles) and scripted traffic vehicles (MetaDrive's IDM car-following
policy with a fixed cruise speed and lane changes switched off). One control step = one MetaDrive step (0.1 s: 5 Bullet
substeps of 0.02 s). The ego action is MetaDrive's own [steering, throttle/brake] in [-1, 1].

The worker also offers the controllers under the skills (`follow`): a pure-pursuit steering law on the lane centre of the
navigation route and a PI speed controller. They read only what `observe` publishes plus the road map (lane centre lines,
like a navigation map), never traffic intentions.
"""

from __future__ import annotations

import math

import numpy as np

DT = 0.1  # s per MetaDrive step (decision_repeat 5 x physics 0.02 s)
CUT_OFF_S = 1.0  # time gap (s) under which a closing vehicle behind counts as cut off
NEAR_M = 60.0  # vehicles and obstacles within this distance are reported
REST_KMH = 2.0  # "at rest" for the destination check (a braked MetaDrive car jitters by up to ~1 km/h at standstill)
KIND = {
    "XLVehicle": "truck",
    "LVehicle": "van",
    "MVehicle": "car",
    "SVehicle": "small car",
    "DefaultVehicle": "car",
    "TrafficDefaultVehicle": "car",
    "StaticDefaultVehicle": "car",
}
OBJ_KIND = {"TrafficCone": "cone", "TrafficWarning": "warning_triangle", "TrafficBarrier": "barrier"}


def _r(x, n=2):
    return round(float(x), n)


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def _shape(lane) -> str:
    """Plain-words geometry of a road (from its lane centre line): straight, or a curve with its radius and turn angle."""
    dh = _wrap(lane.heading_theta_at(lane.length) - lane.heading_theta_at(0.0))
    if abs(dh) < math.radians(4):
        return "straight"
    return f"curve {'left' if dh > 0 else 'right'}, radius {lane.length / abs(dh):.0f} m, turning {abs(math.degrees(dh)):.0f} deg"


def make_env_class():
    from metadrive.component.static_object.traffic_object import TrafficBarrier, TrafficCone, TrafficWarning
    from metadrive.component.vehicle.vehicle_type import vehicle_type
    from metadrive.envs import MetaDriveEnv
    from metadrive.manager.base_manager import BaseManager
    from metadrive.policy.idm_policy import IDMPolicy

    class CruiseIDM(IDMPolicy):
        """MetaDrive's IDM car-following with a fixed cruise speed and no lane changes (scripted traffic)."""

        def __init__(self, control_object, random_seed, speed_kmh=30.0, gap_m=None, headway_s=None):
            super().__init__(control_object, random_seed)
            self.enable_lane_change = False
            self.cruise = float(speed_kmh)
            self.target_speed = self.cruise
            if gap_m is not None:  # denser traffic than MetaDrive's defaults (10 m + 1.5 s)
                self.DISTANCE_WANTED = float(gap_m)
            if headway_s is not None:
                self.TIME_WANTED = float(headway_s)

        def act(self, *a, **k):
            self.target_speed = self.cruise
            return super().act(*a, **k)

        def reset(self):
            super().reset()
            self.target_speed = self.cruise

    class SceneManager(BaseManager):
        """Spawns the task's fixed scene after the map: obstacles, broken-down vehicles and scripted traffic."""

        PRIORITY = 9

        def __init__(self):
            super().__init__()
            self.scripted = []  # (vehicle, policy)
            self.parked = []  # broken-down vehicles: held with the brakes on

        def before_reset(self):
            super().before_reset()
            self.scripted, self.parked = [], []

        def _lane(self, spec):
            a, b, k = spec["lane"]
            return self.engine.current_map.road_network.get_lane((a, b, int(k)))

        def reset(self):
            scene = self.engine.global_config.get("robouse_scene") or {}
            self.t = 0
            for o in scene.get("objects", []):
                lane = self._lane(o)
                s, lat = float(o["s"]), float(o.get("lat", 0.0))
                cls = {"cone": TrafficCone, "warning_triangle": TrafficWarning, "barrier": TrafficBarrier}[o["kind"]]
                self.spawn_object(
                    cls, lane=lane, position=lane.position(s, lat), heading_theta=lane.heading_theta_at(s), static=True
                )
            self.pending = sorted((dict(v) for v in scene.get("vehicles", [])), key=lambda v: float(v.get("t", 0.0)))
            self._spawn_due()

        def _spawn_due(self):
            """Scripted vehicles enter the scene at their start time `t` (s): a fixed, time-triggered timetable."""
            while self.pending and float(self.pending[0].get("t", 0.0)) <= self.t * 0.1 + 1e-6:
                v = self.pending[0]
                if self.t > 0:  # a timetabled car waits until its spawn point is clear (no vehicle within 9 m)
                    lane = self._lane(v)
                    p0 = np.asarray(lane.position(float(v["s"]), float(v.get("lat", 0.0))))
                    if any(
                        np.linalg.norm(np.asarray(o.position) - p0) < 9.0
                        for o in self.engine.get_objects().values()
                        if hasattr(o, "speed_km_h")
                    ):
                        break
                self.pending.pop(0)
                cls = vehicle_type[v.get("type", "m")]
                cfg = {
                    "spawn_lane_index": tuple(v["lane"][:2]) + (int(v["lane"][2]),),
                    "spawn_longitude": float(v["s"]),
                    "spawn_lateral": float(v.get("lat", 0.0)),
                    "spawn_velocity": None,
                }
                if v.get("dest"):
                    cfg["destination"] = v["dest"]
                veh = self.spawn_object(cls, vehicle_config=cfg)
                if v.get("broken_down"):
                    veh.set_break_down()
                    self.parked.append(veh)
                    continue
                speed = float(v.get("speed_kmh", 30.0))
                pol = self.add_policy(
                    veh.id, CruiseIDM, veh, self.generate_seed(), speed, v.get("gap_m"), v.get("headway_s")
                )
                heading = veh.heading
                veh.set_velocity([heading[0], heading[1]], speed / 3.6)
                self.scripted.append((veh, pol))

        def before_step(self):
            self.t += 1
            self._spawn_due()
            for veh, pol in self.scripted:
                veh.before_step(pol.act())
            for veh in self.parked:
                veh.before_step([0.0, -1.0])
            return {}

        def after_step(self, *a, **k):
            gone = []
            for veh, pol in self.scripted:
                veh.after_step()
                if not veh.on_lane:  # drove off the end of the map: remove it, as MetaDrive's traffic manager does
                    gone.append((veh, pol))
            for veh in self.parked:
                veh.after_step()
            for veh, pol in gone:
                self.scripted.remove((veh, pol))
                self.clear_objects([veh.id])
            return {}

    class DrivingEnv(MetaDriveEnv):
        @classmethod
        def default_config(cls):
            cfg = super().default_config()
            cfg.update({"robouse_scene": {"objects": [], "vehicles": []}}, allow_add_new_key=True)
            return cfg

        def setup_engine(self):
            super().setup_engine()
            self.engine.register_manager("robouse_scene_manager", SceneManager())

    return DrivingEnv


class Worker:
    def __init__(self):
        self.env = None
        self._renderers = {}

    # ---- setup ---------------------------------------------------------------------------------------------------
    def make(self, task: dict) -> dict:
        if self.env is not None:  # one MetaDrive engine per process: close the previous scene first
            self.env.close()
            self.env = None
        DrivingEnv = make_env_class()
        cfg = dict(
            use_render=False,
            start_seed=int(task.get("seed", 0)),
            num_scenarios=1,
            log_level=50,
            traffic_density=float(task.get("traffic_density", 0.0)),
            traffic_mode=task.get("traffic_mode", "trigger"),
            random_traffic=False,
            random_spawn_lane_index=False,
            accident_prob=0.0,
            # the episode goes on after a crash or leaving the road; the monitor records it and the task fails
            out_of_road_done=False,
            crash_vehicle_done=False,
            crash_object_done=False,
            crash_human_done=False,
            on_continuous_line_done=True,
            horizon=None,
            show_interface=False,
            interface_panel=[],
            robouse_scene={"objects": task.get("objects", []), "vehicles": task.get("vehicles", [])},
            map_config={
                "type": "block_sequence",
                "config": task["map"],
                "lane_num": int(task.get("lanes", 3)),
                "lane_width": float(task.get("lane_width", 3.5)),
                "exit_length": 50,
                "start_position": [0, 0],
            },
            agent_configs={
                "default_agent": {
                    "use_special_color": True,
                    "spawn_lane_index": tuple(task["spawn"][:2]) + (int(task["spawn"][2]),),
                    "spawn_longitude": float(task.get("spawn_s", 5.0)),
                }
            },
        )
        if task.get("map_config"):
            cfg["map_config"].update(task["map_config"])
        self.env = DrivingEnv(cfg)
        self.env.reset()
        self.task = task
        self.v = self.env.agent
        self.nav = self.v.navigation
        self.rn = self.env.current_map.road_network
        self.step_i = 0
        self.events: list[dict] = []
        self._ev_keys: set = set()
        self.arrived_ever = False
        self.ids: dict[str, str] = {}
        self.last_action = [0.0, 0.0]
        self.integ = 0.0
        self.odometer = 0.0
        self._renderers = {}
        return {"route": self._route_info(), "dt": DT}

    # ---- geometry helpers ----------------------------------------------------------------------------------------
    def _route_roads(self):
        ck = self.nav.checkpoints
        return list(zip(ck[:-1], ck[1:], strict=False))

    def _road_pos(self) -> int:
        """Index (in the route's road list) of the road the navigation module currently tracks."""
        cur = self.nav.current_road
        roads = self._route_roads()
        for i, (a, b) in enumerate(roads):
            if a == cur.start_node and b == cur.end_node:
                return i
        return 0

    def _lanes(self, i):
        a, b = self._route_roads()[i]
        return self.rn.graph[a][b]

    def _ego_lane(self):
        """(route road index, lane index, lane, s, lateral(+left)) of the ego on the tracked route road; the next road is
        used once the car is past the end of the current one."""
        pos = np.asarray(self.v.position)
        i = self._road_pos()
        best = None
        for j in (i, i + 1):
            if j >= len(self._route_roads()):
                continue
            lanes = self._lanes(j)
            full = max(l.length for l in lanes)
            for k, lane in enumerate(lanes):
                s, lat = lane.local_coordinates(pos)
                # a lane that ends before its road does (acceleration lane) still counts past its end: the car is
                # then off the lane, not in the neighbouring one
                end = full if lane.length < full - 1.0 else lane.length
                if -1.0 <= s <= end + 1.0 or j == i:
                    d = abs(lat) + (0 if -1.0 <= s <= end + 1.0 else 50.0)
                    if best is None or d < best[0]:
                        best = (d, j, k, lane, s, lat)
        _, j, k, lane, s, lat = best
        return j, k, lane, s, -lat  # MetaDrive's lateral axis points to the right of the lane direction

    def _remaining(self, j, lane, s) -> float:
        rem = lane.length - s
        roads = self._route_roads()
        for jj in range(j + 1, len(roads)):
            ls = self._lanes(jj)
            rem += ls[min(len(ls) - 1, int(lane.index[2]))].length if ls else 0.0
        return rem

    def _route_info(self) -> dict:
        fl = self.nav.final_lane
        fr = self.nav.final_road.get_lanes(self.rn)
        mid = np.mean([np.asarray(l.position(l.length, 0)) for l in fr], axis=0)
        return {
            "roads": [f"{a}->{b}" for a, b in self._route_roads()],
            "total_m": _r(self.nav.total_length, 1),
            "destination": {
                "pos": [_r(mid[0]), _r(mid[1])],
                "road": f"{fl.index[0]}->{fl.index[1]}",
                "zone": "the last 5 m of the final road and 5 m beyond its end, any lane",
            },
        }

    def _ego_frame(self, p):
        d = np.asarray(p) - np.asarray(self.v.position)
        h = self.v.heading_theta
        return (math.cos(h) * d[0] + math.sin(h) * d[1], -math.sin(h) * d[0] + math.cos(h) * d[1])

    def _along_route(self, p, j0, s0):
        """Distance along the route from the ego (road j0, station s0) to point p, if p is on the route's next 3 roads."""
        base = -s0
        for j in range(j0, min(j0 + 4, len(self._route_roads()))):
            lanes = self._lanes(j)
            best = None
            for k, lane in enumerate(lanes):
                s, lat = lane.local_coordinates(p)
                if -0.5 <= s <= lane.length + 0.5 and abs(lat) <= lane.width / 2 + 0.3:
                    if best is None or abs(lat) < best[0]:
                        best = (abs(lat), base + s, k, j)
            if best is not None:
                return best[1:]
            base += lanes[0].length
        return None

    def _sid(self, name: str, prefix: str) -> str:
        if name not in self.ids:
            self.ids[name] = f"{prefix}{sum(1 for v in self.ids.values() if v.startswith(prefix)) + 1}"
        return self.ids[name]

    # ---- stepping ------------------------------------------------------------------------------------------------
    def step(self, action) -> dict:
        a = [float(np.clip(action[0], -1, 1)), float(np.clip(action[1], -1, 1))]
        p0 = np.asarray(self.v.position)
        self.env.step(a)
        self.last_action = a
        self.step_i += 1
        self.odometer += float(np.linalg.norm(np.asarray(self.v.position) - p0))
        new = self._monitor()
        arrived = bool(self.env._is_arrive_destination(self.v))
        self.arrived_ever = self.arrived_ever or arrived
        return {"new_events": new, "in_destination_zone": arrived, "speed_kmh": _r(self.v.speed_km_h, 1)}

    def _event(self, key, kind, detail):
        if key in self._ev_keys:
            return None
        self._ev_keys.add(key)
        e = {"key": key, "kind": kind, "detail": detail, "t": _r(self.step_i * DT, 1), "step": self.step_i}
        self.events.append(e)
        return e

    def _monitor(self) -> list:
        v, new = self.v, []
        checks = [
            ("crash_vehicle", v.crash_vehicle, "crash", "collided with another vehicle"),
            ("crash_object", v.crash_object, "crash", "hit a road object (cone, warning triangle or barrier)"),
            ("crash_building", v.crash_building, "crash", "hit a building"),
            ("crash_sidewalk", v.crash_sidewalk, "crash", "drove onto the sidewalk / road edge"),
            ("crash_human", v.crash_human, "crash", "hit a pedestrian"),
        ]
        for key, flag, kind, detail in checks:
            if flag:
                e = self._event(key, kind, detail)
                if e:
                    new.append(e)
        # cutting in front of (or braking hard in front of) a vehicle: something behind in the car's lane closes in with a
        # time gap under CUT_OFF_S (MetaDrive's IDM followers keep about 10 m + 1.5 s, so this only happens when forced)
        h, ve = v.heading_theta, v.speed_km_h / 3.6
        for _name, obj in self.env.engine.get_objects().items():
            if obj is v or type(obj).__name__ not in KIND or getattr(obj, "break_down", False):
                continue
            fx, fy = self._ego_frame(obj.position)
            if not (-30.0 < fx < 0.0 and abs(fy) < 1.6):
                continue
            hr = _wrap(obj.heading_theta - h)
            if abs(hr) > math.radians(35):
                continue
            vf = obj.speed_km_h / 3.6
            gap = -fx - (float(getattr(obj, "LENGTH", 4.5)) + v.LENGTH) / 2
            if vf * math.cos(hr) - ve > 0.3 and vf > 3.0 and gap < CUT_OFF_S * vf:
                e = self._event(
                    "cut_off",
                    "unsafe",
                    f"a vehicle behind in your lane closed in at {vf * 3.6:.0f} km/h with a "
                    f"{gap:.1f} m gap (under {CUT_OFF_S} s): you cut in front of it or braked hard in front of it",
                )
                if e:
                    new.append(e)
        # give way: entering the task's yield road while another vehicle will reach the conflict point within the window
        rule = self.task.get("yield_rule")
        if rule and "failed_to_yield" not in self._ev_keys:
            jj, _, _, _, _ = self._ego_lane()
            a, b = self._route_roads()[jj]
            if f"{a}->{b}" == rule["road"]:  # checked on every step while the car is on the entry road
                cp = np.asarray(rule["point"], dtype=float)
                for _name, obj in self.env.engine.get_objects().items():
                    if obj is v or type(obj).__name__ not in KIND:
                        continue
                    d = np.asarray(obj.position) - cp
                    dist = float(np.linalg.norm(d))
                    closing = -float(np.dot(d, np.asarray(obj.velocity))) / max(dist, 1e-6)
                    if dist < 6.0 or (closing > 0.3 and (dist < 15.0 or dist / closing < float(rule["window_s"]))):
                        e = self._event(
                            "failed_to_yield",
                            "rule",
                            f"was on {rule['road']} while a vehicle was "
                            f"{dist:.0f} m from the conflict point {list(rule['point'])} and "
                            f"approaching (closer than 15 m or under {rule['window_s']} s away): you must give way",
                        )
                        if e:
                            new.append(e)
                        break
        if self.env._is_out_of_road(v):
            why = (
                "crossed a solid (continuous) lane line"
                if (v.on_yellow_continuous_line or v.on_white_continuous_line)
                else "left the drivable road"
            )
            e = self._event("out_of_road", "out_of_road", why)
            if e:
                new.append(e)
        return new

    # ---- observation ---------------------------------------------------------------------------------------------
    def observe(self) -> dict:
        v = self.v
        j, k, lane, s, lat = self._ego_lane()
        lanes = self._lanes(j)
        rem = self._remaining(j, lane, s)
        roads = self._route_roads()
        nxt = []
        acc = lane.length - s
        for jj in range(j + 1, min(j + 4, len(roads))):
            ls = self._lanes(jj)
            a, b = roads[jj]
            nxt.append(
                {
                    "road": f"{a}->{b}",
                    "starts_in_m": _r(acc, 1),
                    "lanes": len(ls),
                    "shape": _shape(ls[min(len(ls) - 1, k)]),
                    "start_pos": [_r(x) for x in ls[0].position(0, 0)],
                }
            )
            acc += ls[min(len(ls) - 1, k)].length
        vehicles, obstacles = [], []
        pos = np.asarray(v.position)
        for name, obj in self.env.engine.get_objects().items():
            if obj is v:
                continue
            cls = type(obj).__name__
            if cls not in KIND and cls not in OBJ_KIND:
                continue
            p = np.asarray(obj.position)
            dist = float(np.linalg.norm(p - pos))
            if dist > NEAR_M:
                continue
            fx, fy = self._ego_frame(p)
            ar = self._along_route(p, j, s)
            ent = {"rel_ahead_m": _r(fx, 1), "rel_left_m": _r(fy, 1), "distance_m": _r(dist, 1)}
            if ar is not None:
                ent["on_route"] = True
                ent["route_ahead_m"] = _r(ar[0], 1)
                ent["lane"] = ar[1]
                ent["in_my_lane"] = bool(ar[1] == k and (ar[2] == j or len(self._lanes(ar[2])) == len(lanes)))
            else:
                ent["on_route"] = False
            li = getattr(obj, "lane_index", None) if cls in KIND else None
            if li:
                ent["road"] = f"{li[0]}->{li[1]}"
                ent.setdefault("lane", int(li[2]))
            if cls in KIND:
                length = float(getattr(obj, "LENGTH", 4.5))
                ent = {
                    "id": self._sid(name, "v"),
                    "kind": KIND[cls],
                    "length_m": _r(length, 1),
                    "speed_kmh": _r(obj.speed_km_h, 1),
                    "heading_rel_deg": _r(math.degrees(_wrap(obj.heading_theta - v.heading_theta)), 0),
                    **ent,
                }
                if getattr(obj, "break_down", False):
                    ent["broken_down"] = True
                if "route_ahead_m" in ent:
                    ent["gap_m"] = _r(abs(ent["route_ahead_m"]) - (length + v.LENGTH) / 2, 1)
                vehicles.append(ent)
            else:
                obstacles.append({"id": self._sid(name, "o"), "kind": OBJ_KIND[cls], **ent})
        vehicles.sort(key=lambda e: e["distance_m"])
        obstacles.sort(key=lambda e: e["distance_m"])
        info = self._route_info()
        return {
            "t_s": _r(self.step_i * DT, 1),
            "ego": {
                "pos": [_r(pos[0]), _r(pos[1])],
                "heading_deg": _r(math.degrees(v.heading_theta), 1),
                "speed_kmh": _r(v.speed_km_h, 1),
                "steering": _r(self.last_action[0]),
                "throttle": _r(self.last_action[1]),
                "length_m": _r(v.LENGTH, 2),
                "width_m": _r(v.WIDTH, 2),
            },
            "lane": {
                "road": f"{roads[j][0]}->{roads[j][1]}",
                "index": k,
                "count": len(lanes),
                "lateral_offset_m": _r(lat),
                "width_m": _r(lane.width, 2),
                "station_m": _r(s, 1),
                "road_length_m": _r(lane.length, 1),
                "shape": _shape(lane),
                "left_line": str(lane.line_types[0]).split(".")[-1].lower(),
                "right_line": str(lane.line_types[1]).split(".")[-1].lower(),
                "heading_error_deg": _r(
                    math.degrees(_wrap(v.heading_theta - lane.heading_theta_at(max(0.0, min(s, lane.length))))), 1
                ),
            },
            "route": {
                "travelled_m": _r(self.odometer, 1),
                "remaining_m": _r(rem, 1),
                "total_m": info["total_m"],
                "route_completion": _r(min(1.0, max(0.0, self.nav.route_completion)), 3),
                "next_roads": nxt,
                "destination": info["destination"],
                "in_destination_zone": bool(self.env._is_arrive_destination(v)),
                **({"give_way": dict(self.task["yield_rule"])} if self.task.get("yield_rule") else {}),
            },
            "vehicles": vehicles,
            "obstacles": obstacles,
            "events": [{k2: e[k2] for k2 in ("key", "kind", "detail", "t")} for e in self.events],
        }

    # ---- controllers under the skills ----------------------------------------------------------------------------
    def _advance(self, j, lane, s):
        """(road index, lane, station) of the point `s` metres along `lane` (route road j), continuing into later roads by
        the lane whose start is nearest the current lane's end; None past the end of the route or of a lane that ends
        before its road does (an acceleration lane): the tracker then holds the lane's last heading and the car runs
        off the end of the lane, as a lane-keeping controller would."""
        while s > lane.length:
            s -= lane.length
            if j + 1 >= len(self._route_roads()):
                return None
            if lane.length < max(l.length for l in self._lanes(j)) - 1.0:
                return None
            end = np.asarray(lane.position(lane.length, 0))
            j += 1
            lane = min(self._lanes(j), key=lambda l: float(np.linalg.norm(np.asarray(l.position(0, 0)) - end)))
        return j, lane, s

    def _heading_at(self, j, lane, s):
        p = self._advance(j, lane, s)
        if p is None:
            return lane.heading_theta_at(lane.length)
        return p[1].heading_theta_at(max(0.0, p[2]))

    def follow(
        self, speed_kmh: float, lane: int | None = None, reset: bool = False, brake: float | None = None
    ) -> dict:
        """One control action: Stanley lane-centre tracking (heading error + cross-track error at the front axle + road
        curvature feed-forward) on `lane` (default: the lane the car is in) along the route, and PI speed control to
        speed_kmh (or a fixed brake value)."""
        v = self.v
        if reset:
            self.integ = 0.0
        j, k, cur, s, lat = self._ego_lane()
        lanes = self._lanes(j)
        tk = k if lane is None else int(max(0, min(len(lanes) - 1, lane)))
        tl = lanes[tk]
        h = v.heading_theta
        lf = float(getattr(v, "FRONT_WHEELBASE", 1.05))
        wb = lf + float(getattr(v, "REAR_WHEELBASE", 1.42))
        fa = np.asarray(v.position) + lf * np.array([math.cos(h), math.sin(h)])
        fs, _ = tl.local_coordinates(fa)
        loc = self._advance(j, tl, max(fs, 0.0))
        if loc is None:  # past the end of the route: keep the final lane's line
            fj, fl, fss = j, tl, fs
        else:
            fj, fl, fss = loc
        flat = fl.local_coordinates(fa)[1]  # + = the front axle is right of the lane centre
        hp = fl.heading_theta_at(max(0.0, min(fss, fl.length)))
        vm = max(0.0, v.speed_km_h / 3.6)
        ahead = 1.0 + 0.1 * vm
        kappa = _wrap(self._heading_at(fj, fl, fss + ahead) - hp) / ahead
        delta = _wrap(hp - h) + math.atan2(2.5 * flat, vm + 1.5) + math.atan(wb * kappa)
        steer = float(np.clip(delta / math.radians(v.max_steering), -1, 1))
        if brake is not None:
            thr = float(brake)
        else:
            err = speed_kmh / 3.6 - vm
            self.integ = float(np.clip(self.integ + err * DT, -5, 5))
            thr = float(np.clip(0.35 * err + 0.08 * self.integ + 0.05, -1, 1))
            if speed_kmh <= 0.5 and vm < 1.0:
                thr = -1.0
        tlat = -tl.local_coordinates(np.asarray(v.position))[1]
        return {
            "action": [steer, thr],
            "lane": k,
            "target_lane": tk,
            "target_lateral_m": _r(tlat),
            "lane_count": len(lanes),
            "heading_error_deg": _r(math.degrees(_wrap(h - hp)), 1),
            "speed_kmh": _r(v.speed_km_h, 1),
            "odometer_m": _r(self.odometer, 2),
            "remaining_m": _r(self._remaining(j, cur, s), 1),
        }

    def judge(self) -> dict:
        v = self.v
        in_zone = bool(self.env._is_arrive_destination(v))
        at_rest = v.speed_km_h < REST_KMH
        return {
            "success": bool(in_zone and at_rest and not self.events),
            "in_destination_zone": in_zone,
            "speed_kmh": _r(v.speed_km_h, 2),
            "at_rest": at_rest,
            "events": self.events,
            "arrived_ever": self.arrived_ever,
        }

    def snapshot(self) -> dict:
        """Positions of every vehicle (for determinism checks; not used by the backend's public interface)."""
        out = {"ego": [_r(x, 4) for x in self.v.position]}
        for name, obj in self.env.engine.get_objects().items():
            if type(obj).__name__ in KIND and obj is not self.v:
                out[self._sid(name, "v")] = [_r(x, 4) for x in obj.position]
        return out

    def render(self, width: int, height: int, camera: str = "topdown", scale: float = 5.0) -> np.ndarray:
        """`topdown`: follows the ego car, north (+y) up, 5 px per metre. `route_map`: the whole route, fixed."""
        from metadrive.engine.top_down_renderer import TopDownRenderer

        key = (camera, int(width), int(height), float(scale))
        r = self._renderers.get(key)
        if r is None:
            if camera == "route_map":
                pts = []
                for j in range(len(self._route_roads())):
                    for lane in self._lanes(j):
                        pts += [lane.position(x, 0) for x in np.linspace(0, lane.length, 6)]
                pts = np.asarray(pts)
                lo, hi = pts.min(0) - 15, pts.max(0) + 15
                sc = float(min(width / (hi[0] - lo[0]), height / (hi[1] - lo[1]), 5.0))
                r = TopDownRenderer(
                    film_size=(int(max(hi - lo) * sc) + 400,) * 2,
                    scaling=sc,
                    screen_size=(int(width), int(height)),
                    camera_position=tuple((lo + hi) / 2),
                    num_stack=1,
                    window=False,
                )
            else:
                r = TopDownRenderer(
                    film_size=(int(640 * scale),) * 2,
                    scaling=float(scale),
                    screen_size=(int(width), int(height)),
                    num_stack=1,
                    window=False,
                )
            self._renderers[key] = r
        text = {"t": f"{self.step_i * DT:5.1f} s", "speed": f"{self.v.speed_km_h:5.1f} km/h"}
        img = r.render(text=text, to_image=True)
        return np.ascontiguousarray(np.asarray(img, dtype=np.uint8)[..., :3])

    def close(self):
        if self.env is not None:
            self.env.close()
