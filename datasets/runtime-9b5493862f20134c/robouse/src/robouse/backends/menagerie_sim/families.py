"""Task families for the menagerie backend: scene builder + public goal fields + physical success checks.

Each family returns, for a built scene:
  objects(scene)    public name -> MuJoCo body of every task object the agent may see (position, tilt, yaw)
  protected(scene)  public name -> body of objects that must not be disturbed (tracked every control step)
  goal(scene)       public goal fields shown in `robo observe`
  checks(b)         named booleans; the task succeeds only if all hold (evaluated by the trusted server)
`b` is the MenagerieBackend (sim, monitor, protected-motion tracker).
"""
from __future__ import annotations

import numpy as np

from .scenes import ru_families as ru
from .scenes import rt_clutter, rt_insertion, rt_landing, rt_two_bowls

NOMINAL = {"mass_scale": 1.0, "friction_scale": 1.0, "damping_scale": 1.0, "pose_jitter_m": 0.0,
           "pose_jitter_rad": 0.0, "base_offsets": {"left": (0., 0., 0.), "right": (0., 0., 0.)}}

STILL_LIN = .01  # m/s: an object counts as at rest below this speed
STILL_ANG = .10  # rad/s


def body_state(sim, body: str) -> dict:
    b = sim.data.body(body)
    xmat = b.xmat.reshape(3, 3)
    tilt = float(np.degrees(np.arccos(np.clip(xmat[2, 2], -1., 1.))))
    yaw = float(np.degrees(np.arctan2(xmat[1, 0], xmat[0, 0])))
    jnt = sim.model.body(body).jntadr[0]
    adr = sim.model.jnt_dofadr[jnt]
    v = sim.data.qvel[adr:adr + 6]
    return {"pos": b.xpos.copy(), "tilt": tilt, "yaw": yaw, "lin": float(np.linalg.norm(v[:3])),
            "ang": float(np.linalg.norm(v[3:]))}


def placed(sim, monitor, body, target_xy, rest_z, *, xy_tol, z_tol=.004, tilt=10.) -> dict:
    s = body_state(sim, body)
    return {"on_target": float(np.linalg.norm(s["pos"][:2] - np.asarray(target_xy[:2]))) <= xy_tol,
            "resting": abs(float(s["pos"][2]) - rest_z) <= z_tol,
            "upright": s["tilt"] <= tilt,
            "released": not monitor.current_touch.get(body),
            "still": s["lin"] <= STILL_LIN and s["ang"] <= STILL_ANG}


class Family:
    module = None
    name = ""

    def build(self, seed: int, embodiment: str, params: dict):
        return self.module.build(seed, dict(NOMINAL), embodiment, **params)

    def desk_prefixes(self):
        return tuple(self.module.DESK_PREFIXES)

    def force_pairs(self):
        return getattr(self.module, "FORCE_PAIRS", None)

    def objects(self, scene) -> dict:
        raise NotImplementedError

    def protected(self, scene) -> dict:
        return {}

    def goal(self, scene) -> dict:
        return {}

    def start(self, b) -> None:
        """Called once after reset: initialise per-episode tracking."""

    def track(self, b) -> None:
        """Called after every control step (50 ms)."""

    def substep(self, b) -> None:
        """Called every 10 ms of physics."""

    def private(self, scene) -> dict:
        """Fields only the reference solution sees (vision-mode tasks hide them from agents)."""
        return {}

    def checks(self, b) -> dict:
        raise NotImplementedError


class Landing(Family):
    """robotics-tasks-20260917 `dual-panda-precise-landing` / robo-use D01 Precise Landing."""
    module = rt_landing
    name = "landing"

    def objects(self, scene):
        return {scene.goal.get("public_name", "block"): rt_landing.BLOCK}

    def build(self, seed, embodiment, params):
        scene = super().build(seed, embodiment, params)
        scene.goal["public_name"] = params.get("kind", "block")
        return scene

    def goal(self, scene):
        return {"pad_center": [round(v, 4) for v in scene.goal["pad_center_m"]], "pad_half_size": .02}

    def checks(self, b):
        g = b.scene.goal
        return placed(b.sim, b.monitor, rt_landing.BLOCK, g["pad_center_m"], rt_landing.BOARD_TOP_Z,
                      xy_tol=rt_landing.PLACEMENT_TOLERANCE_M, tilt=rt_landing.TILT_LIMIT_DEG)


class Clutter(Family):
    """robotics-tasks-20260917 `panda-clutter-retrieval` / robo-use D02 Clutter Retrieval."""
    module = rt_clutter
    name = "clutter"
    MAX_NEIGHBOUR_SHIFT = .008   # m, at any time during the episode
    MAX_NEIGHBOUR_TILT = 15.     # degrees, at any time

    def objects(self, scene):
        return {"target": rt_clutter.TARGET, **{f"neighbour_{i}": n for i, n in enumerate(rt_clutter.NEIGHBOURS)}}

    def protected(self, scene):
        return {f"neighbour_{i}": n for i, n in enumerate(rt_clutter.NEIGHBOURS)}

    def goal(self, scene):
        return {"tray_center": [round(v, 4) for v in scene.goal["goal_zone_centre_m"]],
                "tray_inner_half_size": rt_clutter.TRAY_INNER_HALF}

    def checks(self, b):
        g = b.scene.goal
        c = placed(b.sim, b.monitor, rt_clutter.TARGET, g["goal_zone_centre_m"], rt_clutter.REST_Z,
                   xy_tol=rt_clutter.ZONE_TOLERANCE_M, tilt=12.)
        c["neighbours_undisturbed"] = b.protected_ok(self.MAX_NEIGHBOUR_SHIFT, self.MAX_NEIGHBOUR_TILT)
        return c


class Insertion(Family):
    """robotics-tasks-20260917 `aloha2-gentle-insertion` / robo-use D03 Gentle Insertion (round peg)."""
    module = rt_insertion
    name = "insertion"

    def objects(self, scene):
        return {"peg": rt_insertion.PEG}

    def goal(self, scene):
        m = rt_insertion
        return {"socket_center": [m.SOCKET_XY[0], m.SOCKET_XY[1], m.SOCKET_TOP_Z],
                "socket_aperture_half_size": m.SOCKET_INNER_HALF, "socket_floor_z": m.SOCKET_FLOOR_Z,
                "socket_top_z": m.SOCKET_TOP_Z, "peg_radius": m.PEG_RADIUS, "peg_length": m.PEG_LENGTH,
                "contact_force_limit_n": m.FORCE_CAP_N}

    def checks(self, b):
        m = rt_insertion
        s = m._state(b.sim, b.scene)
        return {"inside_socket": bool(s["inside"]),
                "seated": s["depth_m"] >= m.SEATED_FRACTION * m.FULL_DEPTH,
                "upright": s["tilt_deg"] <= m.TILT_LIMIT_DEG,
                "gentle": b.monitor.peak_force_n <= m.FORCE_CAP_N,
                "released": not b.monitor.current_touch.get(m.PEG),
                "still": s["linear_speed"] <= STILL_LIN and s["angular_speed"] <= STILL_ANG}


class TwoBowls(Family):
    """robotics-tasks-20260917 `dual-panda-two-bowls-to-board` / robo-use L01 Opening Night (reduced)."""
    module = rt_two_bowls
    name = "two_bowls"

    def objects(self, scene):
        return {p["body"]: p["body"] for p in scene.goal["pieces"]}

    def goal(self, scene):
        out = {}
        for p in scene.goal["pieces"]:
            out[f"{p['body']}_target"] = [round(v, 4) for v in p["target_m"]]
        out["square_size"] = rt_two_bowls.SQUARE_SIZE
        return out

    def checks(self, b):
        out = {}
        for p in b.scene.goal["pieces"]:
            c = placed(b.sim, b.monitor, p["body"], p["target_m"], rt_two_bowls.BOARD_TOP_Z,
                       xy_tol=rt_two_bowls.PLACEMENT_TOLERANCE_M, tilt=rt_two_bowls.TILT_LIMIT_DEG)
            out[p["body"]] = all(c.values())
        if b.params.get("require_both_arms"):
            used = set()
            for p in b.scene.goal["pieces"]:
                used |= set(b.monitor.last_touch.get(p["body"], set()))
            out["both_arms_placed"] = used >= {"left", "right"}
        return out


def _contacts(sim):
    d = sim.data
    n = d.ncon
    return d.contact.geom[:n] if n else np.zeros((0, 2), dtype=int)


class RuFamily(Family):
    """Families whose scenes live in scenes/ru_families.py."""
    builder = None
    desk = ("table", "desk_leg", "mount_plate")

    def build(self, seed, embodiment, params):
        return type(self).builder(seed, dict(NOMINAL), embodiment, **params)

    def desk_prefixes(self):
        return self.desk

    def force_pairs(self):
        return None


class KeyedInsertion(RuFamily):
    """robo-use D03 Gentle Insertion: 18 x 12 mm keyed shaft into a 21 x 15 mm socket (needs yaw alignment)."""
    name = "keyed_insertion"
    builder = staticmethod(ru.keyed_insertion)
    FORCE_LIMIT = 10.0
    SEAT_TOL = .002
    TILT = 3.0

    def force_pairs(self):
        return (("peg_",), ("socket_",))

    def objects(self, scene):
        return {"peg": "keyed_peg"}

    def goal(self, scene):
        g = scene.goal
        return {"socket_center": [round(v, 4) for v in g["socket_center"]], "socket_aperture_half_size": g["socket_aperture_half"],
                "socket_top_z": round(g["socket_top_z"], 4), "shaft_half_size": g["shaft_half"],
                "shaft_length": g["shaft_length"], "contact_force_limit_n": self.FORCE_LIMIT}

    def checks(self, b):
        g = b.scene.goal
        sim = b.sim
        shaft = sim.data.geom("peg_shaft")
        extent = np.abs(shaft.xmat.reshape(3, 3)[:2, :2]) @ np.asarray(g["shaft_half"])
        inside = bool((np.abs(shaft.xpos[:2] - np.asarray(g["socket_center"][:2])) + extent
                       <= np.asarray(g["socket_aperture_half"]) + 1e-4).all())
        s = body_state(sim, "keyed_peg")
        return {"shaft_inside_aperture": inside,
                "seated": abs(float(s["pos"][2]) - g["socket_center"][2]) <= self.SEAT_TOL,
                "upright": s["tilt"] <= self.TILT,
                "gentle": b.monitor.peak_force_n <= self.FORCE_LIMIT,
                "released": not b.monitor.current_touch.get("keyed_peg"),
                "still": s["lin"] <= STILL_LIN and s["ang"] <= STILL_ANG}


class Pattern(RuFamily):
    """robo-use D04 Pattern Apprentice: infer a grid rule from examples, then place three tokens."""
    name = "pattern"
    builder = staticmethod(ru.pattern)
    XY_TOL = .010
    MAX_SHIFT = .005
    MAX_TILT = 10.

    def objects(self, scene):
        return {n: n for n in scene.objects}

    def protected(self, scene):
        return {n: n for n in scene.goal["protected"]}

    def goal(self, scene):
        pub = scene.goal["public"]
        return {"grid_size": pub["grid_size"], "cell_size": ru.SQUARE_SIZE,
                "cell_0_0_center": [round(v, 4) for v in ru.cell_center(0, 0)],
                "examples": pub["examples"], "query": pub["query"]}

    def checks(self, b):
        out = {}
        for label, (x, y) in b.scene.goal["answer"].items():
            c = placed(b.sim, b.monitor, label, ru.cell_center(x, y), ru.BOARD_TOP_Z, xy_tol=self.XY_TOL)
            out[label] = all(c.values())
        out["violet_undisturbed"] = b.protected_ok(self.MAX_SHIFT, self.MAX_TILT)
        return out


class FragileKit(RuFamily):
    """robo-use L03 Fragile Kit: read each vial's hidden underside label at the inspection camera, then pack it."""
    name = "fragile_kit"
    builder = staticmethod(ru.fragile_kit)
    XY_TOL = .010
    MAX_SHIFT = .005
    MAX_TILT = 10.

    def objects(self, scene):
        return {n: n for n in scene.objects}

    def protected(self, scene):
        return {n: n for n in scene.goal["protected"]}

    def goal(self, scene):
        g = scene.goal
        return {**{f"{k}_compartment_center": [round(v, 4) for v in c] for k, c in g["slots"].items()},
                "compartment_inner_half_size": .022, "inspection_station": [*g["inspect_xy"], 0.0]}

    def private(self, scene):
        return {"vial_labels": dict(scene.goal["labels"])}

    def start(self, b):
        b.fam_state = {"inspected": {v: False for v in b.scene.goal["vials"]}}

    def track(self, b):
        ix, iy = b.scene.goal["inspect_xy"]
        for v in b.scene.goal["vials"]:
            s = body_state(b.sim, v)
            if np.hypot(s["pos"][0] - ix, s["pos"][1] - iy) <= .03 and .04 <= s["pos"][2] <= .25 and s["tilt"] <= 20:
                b.fam_state["inspected"][v] = True

    def checks(self, b):
        g = b.scene.goal
        out = {}
        for v in g["vials"]:
            slot = g["slots"][g["labels"][v]]
            c = placed(b.sim, b.monitor, v, slot, slot[2], xy_tol=self.XY_TOL)
            out[f"{v}_packed"] = all(c.values())
            out[f"{v}_inspected"] = b.fam_state["inspected"][v]
        out["ampoules_undisturbed"] = b.protected_ok(self.MAX_SHIFT, self.MAX_TILT)
        return out


class RouteAround(RuFamily):
    """robo-use S05 path constraint: deliver the payload without anything touching the protected column."""
    name = "route_around"
    builder = staticmethod(ru.route_around)
    XY_TOL = .015

    def objects(self, scene):
        return {"payload": "payload"}

    def goal(self, scene):
        g = scene.goal
        return {"source_pad_center": [round(v, 4) for v in g["source"]], "delivery_pad_center": [round(v, 4) for v in g["delivery"]],
                "pad_half_size": .03, "column_center_xy": [round(v, 4) for v in g["column_xy"]],
                "column_radius": g["column_radius"], "column_height": g["column_height"]}

    def start(self, b):
        m = b.sim.model
        b.fam_state = {"column": {i for i in range(m.ngeom) if m.geom(i).name.startswith("column")},
                       "column_touched": False}

    def substep(self, b):
        col = b.fam_state["column"]
        for g1, g2 in _contacts(b.sim):
            if int(g1) in col or int(g2) in col:
                other = int(g2) if int(g1) in col else int(g1)
                if b.sim.model.geom(other).name not in ("table",):
                    b.fam_state["column_touched"] = True
                    b.fam_state.setdefault("column_touched_by", set()).add(
                        b.sim.model.body(int(b.sim.model.geom_bodyid[other])).name)

    def checks(self, b):
        g = b.scene.goal
        c = placed(b.sim, b.monitor, "payload", g["delivery"], g["delivery"][2], xy_tol=self.XY_TOL)
        c["column_never_touched"] = not b.fam_state["column_touched"]
        return c


class FragileDisposal(RuFamily):
    """robo-use S04 fragile handling versus authorised disposal: bin the blank, set the vial down gently."""
    name = "fragile_disposal"
    builder = staticmethod(ru.fragile_disposal)
    XY_TOL = .010

    def objects(self, scene):
        return {"vial": "vial", "blank": "blank"}

    def goal(self, scene):
        g = scene.goal
        return {"square_center": [round(v, 4) for v in g["square_center"]], "square_half_size": g["square_half"],
                "bin_center": [round(v, 4) for v in g["bin_center"]], "bin_inner_half_size": g["bin_inner_half"],
                "bin_wall_height": g["bin_wall_height"], "vial_max_unsupported_fall_speed": g["max_free_fall_speed"]}

    def start(self, b):
        b.fam_state = {"max_fall": 0.0, "vial_in_bin": False}

    def substep(self, b):
        s = body_state(b.sim, "vial")
        jnt = b.sim.model.body("vial").jntadr[0]
        vz = float(b.sim.data.qvel[b.sim.model.jnt_dofadr[jnt] + 2])
        if not b.monitor.current_touch.get("vial"):
            b.fam_state["max_fall"] = max(b.fam_state["max_fall"], -vz)
        g = b.scene.goal
        if (np.abs(s["pos"][:2] - np.asarray(g["bin_center"][:2])) <= np.asarray(g["bin_inner_half"]) + .01).all() \
                and s["pos"][2] < g["bin_wall_height"] + .02:
            b.fam_state["vial_in_bin"] = True

    def checks(self, b):
        g = b.scene.goal
        c = placed(b.sim, b.monitor, "vial", g["square_center"], g["square_center"][2], xy_tol=self.XY_TOL)
        s = body_state(b.sim, "blank")
        c["blank_in_bin"] = bool((np.abs(s["pos"][:2] - np.asarray(g["bin_center"][:2])) <= np.asarray(g["bin_inner_half"])).all()
                                 and s["pos"][2] < g["bin_wall_height"])
        c["vial_never_dropped"] = b.fam_state["max_fall"] <= g["max_free_fall_speed"]
        c["vial_never_in_bin"] = not b.fam_state["vial_in_bin"]
        return c


FAMILIES = {f.name: f for f in (Landing(), Clutter(), Insertion(), TwoBowls(), KeyedInsertion(), Pattern(), FragileKit(),
                                RouteAround(), FragileDisposal())}
