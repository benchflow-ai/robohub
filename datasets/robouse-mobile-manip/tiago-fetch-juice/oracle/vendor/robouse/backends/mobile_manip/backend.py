"""Mobile-manip backend: three mobile manipulators from MuJoCo Menagerie in a small apartment.

Robots (all Apache-2.0): Hello Robot Stretch 3 (`hello_robot_stretch_3`), PAL TIAGo (`pal_tiago`) and the Google
Robot (`google_robot`, whose Menagerie model has no mobile base: a planar base is added, see
sim/robots.py). Scene: sim/apartment.py (living room, kitchen with a slide-jointed drawer,
study; 1 m doorways; free objects with realistic masses).

Physics: 2 ms steps with implicitfast integration, elliptic friction cones, impratio 10 and 3 noslip iterations (the
upstream Stretch settings; noslip stops objects creeping out of a friction grasp). One control step (`robo act`) is
50 ms. Grasps are friction grasps only: nothing is welded or teleported.

Action groups (one 50 ms step; zero = hold):
  Stretch: base.twist [V, WZ] (x 0.25 m/s, x 60 deg/s), arm.lift [DLIFT] (target = measured + x 2 cm), arm.extend
      [DEXT] (measured + x 4 cm), wrist [DYAW, DPITCH] (measured + x 15 deg), arm.posture [POSE] (+1 ready, -1 stow),
      gripper [G].
  TIAGo: base.twist [V, WZ] (x 0.45 m/s, x 60 deg/s), arm.ee_delta [DX, DY, DZ] (world frame, x 2 cm per step),
      arm.posture [POSE], gripper [G].
  Google Robot: base.twist [VX, VY, WZ] (robot frame, x 0.5 m/s, x 60 deg/s), arm.ee_delta, arm.posture, gripper.
Skills: drive_to, turn, reach, grasp, open_gripper, ready, stow (closed-loop generators over the same actions).
"""

from __future__ import annotations

import math

import numpy as np

from ..embodied import ActionGroup, Budget, EmbodiedBackend, Embodiment, Oracle, Sensor, Skill, SkillArg, r3, wrap
from .sim import apartment as A
from .sim.robots import ROBOTS, Stretch

PHYS_DT, CONTROL_DT = 0.002, 0.05
N_SUB = round(CONTROL_DT / PHYS_DT)
HIT_N = 400.0  # a contact force above this between the robot's arm and a wall or furniture is a hard collision
BASE_HIT_N = 150.0  # the same for the rest of the robot (base, wheels, mast or torso, head): driving into something
TIP_DEG = 12.0
EE_STEP = 0.02  # arm.ee_delta: metres per unit per step
LIFT_STEP = 0.02  # arm.lift: the lift target is set to (measured + DLIFT x 2 cm)
EXT_STEP = 0.04  # arm.extend: the telescope target is set to (measured + DEXT x 4 cm)
WRIST_STEP = math.radians(15)  # wrist: the target is set to (measured + DYAW x 15 deg), same for DPITCH

# ------------------------------------------------------------------------------------------------------------------
# tasks
# ------------------------------------------------------------------------------------------------------------------


def _o(name, kind, color, x, y, surface_z, yaw=0.0):
    return dict(name=name, kind=kind, color=color, pos=(x, y, surface_z), yaw=yaw)


DESK_Z, TABLE_Z, COUNTER_Z = A.TABLES["desk"]["h"], A.TABLES["dining_table"]["h"], A.COUNTER["h"]

TASKS: dict[str, dict] = {
    # ---- Stretch 3 ----
    "stretch-fetch-cup": dict(
        robot="stretch",
        start=(1.0, -0.6, 180.0),
        max_steps=4000,
        objects=[_o("cup", "cup", "blue", -2.4, -2.02, DESK_Z)],
        place={"cup": "dining_table"},
    ),
    "stretch-open-drawer": dict(robot="stretch", start=(1.2, 0.9, 180.0), max_steps=2400, objects=[], drawer_min=0.25),
    "stretch-shelve-two": dict(
        robot="stretch",
        start=(1.2, 0.2, 90.0),
        max_steps=5000,
        objects=[_o("can", "can", "red", 1.8, 1.12, TABLE_Z), _o("box", "box", "yellow", 2.25, 1.12, TABLE_Z, 90.0)],
        place={"can": "shelf", "box": "shelf"},
    ),
    # ---- TIAGo ----
    "tiago-fetch-juice": dict(
        robot="tiago",
        start=(2.6, -0.6, 180.0),
        max_steps=3600,
        objects=[_o("juice_box", "box", "green", -2.75, 2.0, COUNTER_Z, 90.0)],
        place={"juice_box": "dining_table"},
    ),
    "tiago-drawer-stow": dict(
        robot="tiago",
        start=(1.2, 0.6, 180.0),
        max_steps=3000,
        objects=[_o("spice_box", "spice_box", "orange", -1.4, 2.0, COUNTER_Z, 90.0)],
        place={"spice_box": "drawer"},
    ),
    "tiago-bin-cans": dict(
        robot="tiago",
        start=(1.0, -0.4, 90.0),
        max_steps=4000,
        objects=[
            _o("red_can", "can", "red", 1.75, 1.1, TABLE_Z),
            _o("cup", "cup", "white", 2.0, 1.1, TABLE_Z),
            _o("green_can", "can", "green", 2.25, 1.1, TABLE_Z),
        ],
        place={"red_can": "bin", "green_can": "bin"},
        keep={"cup": "dining_table"},
    ),
    "tiago-study-to-kitchen": dict(
        robot="tiago",
        start=(-1.0, -1.0, 180.0),
        max_steps=3600,
        objects=[_o("mug", "cup", "purple", -2.1, -2.02, DESK_Z)],
        place={"mug": "kitchen_table"},
    ),
    # ---- Google Robot ----
    "google-fetch-can": dict(
        robot="google",
        start=(1.5, -0.3, 180.0),
        max_steps=2600,
        objects=[_o("can", "can", "red", -3.1, 0.6, A.TABLES["kitchen_table"]["h"])],
        place={"can": "dining_table"},
    ),
    "google-open-drawer": dict(robot="google", start=(1.2, 0.3, 180.0), max_steps=1400, objects=[], drawer_min=0.25),
    "google-sort-bottle-can": dict(
        robot="google",
        start=(1.2, -0.5, 90.0),
        max_steps=3600,
        objects=[_o("bottle", "bottle", "blue", 1.8, 1.1, TABLE_Z), _o("can", "can", "green", 2.2, 1.1, TABLE_Z)],
        place={"bottle": "shelf_upper", "can": "bin"},
    ),
}

# ------------------------------------------------------------------------------------------------------------------
# declaration
# ------------------------------------------------------------------------------------------------------------------


def declare(robot_key: str, max_steps: int) -> Embodiment:
    rc = ROBOTS[robot_key]
    posture = ActionGroup(
        "arm.posture",
        "select",
        ["POSE"],
        [-1],
        [1],
        "+1 ready, -1 stow, 0 none",
        "a value above 0.5 starts the move to the ready pose (gripper in front, open), below -0.5 to the "
        "stowed pose; the move runs until done unless another arm command interrupts it",
    )
    grip = ActionGroup(
        "gripper",
        "gripper",
        ["G"],
        [-1],
        [1],
        "rate",
        "> 0 closes, < 0 opens (1 = 10 % of the stroke per step), 0 holds; closing stops squeezing at a bounded "
        "force when the fingers are blocked",
    )
    if robot_key == "google":
        base = ActionGroup(
            "base.twist",
            "base_twist",
            ["VX", "VY", "WZ"],
            [-1] * 3,
            [1] * 3,
            "x 0.5 m/s, x 0.5 m/s, x 60 deg/s",
            "forward, leftward and yaw-rate command for the added planar (holonomic) base, robot frame",
            frame="body",
        )
    else:
        v = rc.v_max
        base = ActionGroup(
            "base.twist",
            "base_twist",
            ["V", "WZ"],
            [-1] * 2,
            [1] * 2,
            f"x {v:g} m/s, x 60 deg/s",
            "forward speed and yaw rate of the differential drive",
            frame="body",
        )
    if robot_key == "stretch":
        arm = [
            ActionGroup(
                "arm.lift",
                "joint_delta",
                ["DLIFT"],
                [-1],
                [1],
                "x 2 cm",
                "lift target = current height + DLIFT x 2 cm (raise +, lower -)",
            ),
            ActionGroup(
                "arm.extend",
                "joint_delta",
                ["DEXT"],
                [-1],
                [1],
                "x 4 cm",
                "telescope target = current extension + DEXT x 4 cm (0 to 0.52 m; the arm points to the robot's right, "
                "-y in the robot frame)",
            ),
            ActionGroup(
                "wrist",
                "joint_delta",
                ["DYAW", "DPITCH"],
                [-1] * 2,
                [1] * 2,
                "x 15 deg",
                "wrist yaw and pitch targets = current angle + value x 15 deg",
            ),
        ]
    else:
        arm = [
            ActionGroup(
                "arm.ee_delta",
                "ee_delta_pos",
                ["DX", "DY", "DZ"],
                [-1] * 3,
                [1] * 3,
                "x 2 cm per step",
                "moves the gripper target (IK; the gripper stays level and points along the robot's heading)",
                frame="world",
            )
        ]
    groups = [base, *arm, posture, grip]
    xyz = [SkillArg("x", unit="m"), SkillArg("y", unit="m"), SkillArg("z", unit="m")]
    skills = [
        Skill(
            "drive_to",
            [
                SkillArg("x", unit="m"),
                SkillArg("y", unit="m"),
                SkillArg(
                    "yaw_deg", "str", unit="deg", default="none", doc="final heading; none keeps the travel heading"
                ),
                SkillArg("tol", unit="m", default=0.05),
            ],
            (
                "turn toward (X, Y), drive there in a straight line and stop, then turn to YAW_DEG; it does not plan around "
                "walls or furniture (give it waypoints through doorways) and stops if the base is blocked"
            )
            if robot_key != "google"
            else (
                "drive in a straight line to (X, Y) (holonomic) while turning to YAW_DEG; it does not plan around walls or "
                "furniture (give it waypoints through doorways) and stops if the base is blocked"
            ),
            700 if robot_key == "google" else 1000,
        ),
        Skill("turn", [SkillArg("yaw_deg", unit="deg")], "rotate in place to heading YAW_DEG (0 = +x, 90 = +y)", 200),
        Skill(
            "reach",
            xyz,
            (
                "move the grasp point (between the fingertips) to (X, Y, Z): lift to the height, drive the base forward or back "
                "along its heading so the point lies on the arm's line, then extend; reports whether it got there"
            )
            if robot_key == "stretch"
            else "move the grasp point (between the fingertips) to (X, Y, Z) in a straight line with the IK arm; reports whether it got there",
            800 if robot_key == "stretch" else 300,
        ),
        Skill("grasp", [], "close the gripper until the fingers stop; reports what is held between the fingers", 40),
        Skill("open_gripper", [], "open the gripper fully", 30),
        Skill("ready", [], "move the arm to the ready pose (gripper in front of the robot, level, open)", 200),
        Skill("stow", [], "fold the arm in for driving", 200),
    ]
    sensors = [
        Sensor(
            "robot",
            "proprio",
            ["robot"],
            "base pose and velocity, gripper point, opening and what it holds, arm state",
            units="m, deg",
            frame="world",
        ),
        Sensor(
            "objects",
            "world",
            ["objects"],
            "task objects: kind, colour, centre position, tilt, what each rests on",
            units="m, deg",
            frame="world",
        ),
        Sensor(
            "apartment",
            "world",
            ["apartment"],
            "rooms, doorways, furniture and the drawer (public positions)",
            units="m",
            frame="world",
        ),
        Sensor("events", "events", ["safety_events"], "hard collisions, dropped or knocked-over objects, tipping"),
    ]
    cams = cameras_for(robot_key)
    mounts = {"chase": "body", "robot/head": "head", "robot/wrist": "wrist"}
    sensors += [Sensor(f"camera:{c}", "camera", [], "", mount=mounts.get(c, "world")) for c in cams]
    return Embodiment(
        robot=rc.name,
        family="mobile_manipulator",
        assets=[rc.asset],
        sensors=sensors,
        action_groups=groups,
        skills=skills,
        budget=Budget(max_steps, CONTROL_DT),
        cameras=cams,
    )


def cameras_for(robot_key: str) -> list[str]:
    own = ["robot/head", "robot/wrist"] if robot_key == "stretch" else ["robot/head"]
    return ["chase", *own, "overview", "living_room_cam", "kitchen_cam", "study_cam"]


# ------------------------------------------------------------------------------------------------------------------
# backend
# ------------------------------------------------------------------------------------------------------------------


class MobileManipBackend(EmbodiedBackend):
    name = "mobile_manip"

    def __init__(self, spec: dict):
        super().__init__(spec)
        env = spec.get("env") or spec.get("id")
        if env not in TASKS:
            raise KeyError(f"unknown mobile_manip task {env!r}")
        self.task_id, self.task = env, TASKS[env]
        self.rkey = self.task["robot"]
        self.camera = spec.get("camera", "chase")
        self.decl = declare(self.rkey, self.task["max_steps"])

    # ---- episode -----------------------------------------------------------------------------------------------
    def reset(self, seed: int) -> None:
        import mujoco

        self._reset_renderers()
        s = mujoco.MjSpec()
        s.compiler.degree = (
            False  # the Menagerie models are in radians; a degree parent would rescale their joint ranges
        )
        s.compiler.autolimits = True
        o = s.option
        o.timestep = PHYS_DT
        o.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
        o.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
        o.impratio = 10
        o.noslip_iterations = 3
        o.enableflags |= mujoco.mjtEnableBit.mjENBL_MULTICCD
        A.build_apartment(s, self.task["objects"])
        self.robot = r = ROBOTS[self.rkey]()
        x, y, yaw = self.task["start"]
        r.attach(s, x, y, math.radians(yaw))
        self.model = m = s.compile()
        self.data = d = mujoco.MjData(m)
        r.bind(m, d)
        r.init_state()
        mujoco.mj_forward(m, d)
        r.make_arm()
        self.posture = None
        self.stowed = True
        self.events, self._event_keys = [], set()
        # bookkeeping for the monitor and the judge
        name = lambda g: mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or ""
        self.gname = [name(g) for g in range(m.ngeom)]
        self.obj = {o["name"]: m.body(f"obj_{o['name']}").id for o in self.task["objects"]}
        self.obj_geom = {k: m.geom(f"obj_{k}").id for k in self.obj}
        self.obj_meta = {o["name"]: o for o in self.task["objects"]}
        self.geom_obj = {g: k for k, g in self.obj_geom.items()}
        drawer = m.body("drawer").id
        self.drawer_geoms = {g for g in range(m.ngeom) if m.geom_bodyid[g] == drawer}
        self.static = {
            g for g in range(m.ngeom) if m.geom_bodyid[g] == 0 and self.gname[g] != "floor" and m.geom_contype[g]
        }
        self.env_geoms = self.static | self.drawer_geoms
        self.floor = m.geom("floor").id
        self.drawer_q = m.jnt_qposadr[m.joint("drawer_slide").id]
        self.handle = m.geom("drawer_handle").id
        self.surf = A.surfaces()
        self.surf_geom = {k: m.geom(v[5]).id for k, v in self.surf.items()}
        self.monitor_armed = False
        mask = lambda gs: np.isin(np.arange(m.ngeom), list(gs))
        self._is_robot, self._is_finger, self._is_base = mask(r.robot_geoms), mask(r.finger_geoms), mask(r.base_geoms)
        self._is_env, self._is_drawer = mask(self.env_geoms), mask(self.drawer_geoms)
        # let everything settle (objects onto their surfaces, the robot onto its wheels) with the controllers holding
        for _ in range(10):
            self.control_step(np.array(self.decl.hold()))
        self.monitor_armed = True
        self.max_hit = 0.0

    # ---- control -----------------------------------------------------------------------------------------------
    def _split(self, a):
        k = self.rkey
        if k == "google":
            return dict(twist=(a[0], a[2], a[1]), arm=a[3:6], pose=a[6], g=a[7])
        if k == "tiago":
            return dict(twist=(a[0], a[1], 0.0), arm=a[2:5], pose=a[5], g=a[6])
        return dict(twist=(a[0], a[1], 0.0), lift=a[2], ext=a[3], wrist=a[4:6], pose=a[6], g=a[7])

    def control_step(self, a: np.ndarray) -> None:
        import mujoco

        r, u = self.robot, self._split(a)
        v, wz, vy = u["twist"]
        r.set_twist(v * r.v_max, wz * r.wz_max, vy * r.v_max)
        # arm
        if u["pose"] > 0.5:
            self._start_posture("ready")
        elif u["pose"] < -0.5:
            self._start_posture("stow")
        if self.rkey == "stretch":
            moving = bool(u["lift"] or u["ext"] or np.any(u["wrist"]))
            if moving:
                self.posture = None
                self.stowed = False
            if self.posture:
                self._stretch_posture_step()
            else:
                r.nudge(
                    u["lift"] * LIFT_STEP, u["ext"] * EXT_STEP, u["wrist"][0] * WRIST_STEP, u["wrist"][1] * WRIST_STEP
                )
        else:
            if np.any(u["arm"]):
                self.stowed = False
                if self.posture:
                    self.posture = None
                    r.arm.release_joints()
                r.arm.nudge(np.asarray(u["arm"]) * EE_STEP)
            elif self.posture and r.arm.joints_done():
                self.posture = None
                r.arm.release_joints()
        r.grip(float(u["g"]))
        r.control_update()
        m, d = self.model, self.data
        for _i in range(N_SUB):
            r.substep(PHYS_DT)
            mujoco.mj_step(m, d)
            if self.monitor_armed:
                self._monitor_contacts()
        if self.monitor_armed:
            self._monitor_state()

    def _start_posture(self, which: str) -> None:
        r = self.robot
        if self.posture and self.posture[0] == which:
            return
        self.stowed = which == "stow"
        if self.rkey == "stretch":
            self.posture = (which, self._stretch_stages(which))
        else:
            self.posture = (which, None)
            r.arm.move_joints(r.READY if which == "ready" else r.STOW)
            if which == "ready":
                r.grip_target = r.FINGER_OPEN

    def _stretch_stages(self, which: str) -> list[dict]:
        r: Stretch = self.robot
        wy, wp = r.wrist()
        straight = abs(wy) < 0.3 and abs(wp) < 0.3
        if which == "ready":
            if straight:
                return [dict(wyaw=0.0, wpitch=0.0, slide=r.SLIDE_OPEN)]
            return [
                dict(lift=max(r.lift(), 0.45)),
                dict(ext=max(r.ext(), 0.1)),
                dict(wyaw=0.0, wpitch=0.0),
                dict(ext=0.0, slide=r.SLIDE_OPEN),
            ]
        return [
            dict(ext=max(r.ext(), 0.1), slide=min(r.t["slide"], 0.0)),
            dict(wyaw=r.STOW["wyaw"], wpitch=r.STOW["wpitch"]),
            dict(ext=0.0),
        ]

    def _stretch_posture_step(self) -> None:
        r: Stretch = self.robot
        which, stages = self.posture
        if not stages:
            self.posture = None
            return
        st = stages[0]
        if "slide" in st:
            r.t["slide"] = st["slide"]
        wy, wp = r.wrist()
        dl = float(np.clip(st["lift"] - r.lift(), -LIFT_STEP, LIFT_STEP)) if "lift" in st else 0.0
        de = float(np.clip(st["ext"] - r.ext(), -EXT_STEP, EXT_STEP)) if "ext" in st else 0.0
        dy = float(np.clip(st["wyaw"] - wy, -WRIST_STEP, WRIST_STEP)) if "wyaw" in st else 0.0
        dp = float(np.clip(st["wpitch"] - wp, -WRIST_STEP, WRIST_STEP)) if "wpitch" in st else 0.0
        r.nudge(dl, de, dy, dp)
        ok = (
            ("lift" not in st or abs(r.lift() - st["lift"]) < 0.01)
            and ("ext" not in st or abs(r.ext() - st["ext"]) < 0.01)
            and ("wyaw" not in st or abs(wy - st["wyaw"]) < 0.05)
            and ("wpitch" not in st or abs(wp - st["wpitch"]) < 0.05)
        )
        if ok:
            stages.pop(0)
            if not stages:
                self.posture = None

    def posture_busy(self) -> bool:
        if self.posture is None:
            return False
        return not (self.rkey != "stretch" and self.robot.arm.joints_done())

    # ---- monitor -----------------------------------------------------------------------------------------------
    def _monitor_contacts(self) -> None:
        """Every physics step: contact forces between the robot and the walls, furniture and the drawer (the fingers are
        exempt only on the drawer, whose handle they are meant to pull)."""
        import mujoco

        m, d = self.model, self.data
        n = d.ncon
        if not n:
            return
        g = d.contact.geom[:n]
        for col, other in ((0, 1), (1, 0)):
            a, b = g[:, col], g[:, other]
            hit = self._is_robot[a] & self._is_env[b] & ~(self._is_finger[a] & self._is_drawer[b])
            for i in np.nonzero(hit)[0]:
                f = np.zeros(6)
                mujoco.mj_contactForce(m, d, int(i), f)
                fn = abs(f[0])
                ga, gb = int(a[i]), int(b[i])
                self.max_hit = max(self.max_hit, fn)
                body = self._is_base[ga]
                if fn > (BASE_HIT_N if body else HIT_N):
                    what = "drawer" if self._is_drawer[gb] else self.gname[gb].replace("_", " ")
                    part = "base or body" if body else "arm"
                    self.event(
                        f"hit:{what}", "crash", f"the robot's {part} hit the {what} hard ({fn:.0f} N contact force)"
                    )

    def _monitor_state(self) -> None:
        r = self.robot
        if r.tilt_deg() > TIP_DEG:
            self.event("tip", "fall", f"the robot base tilted {r.tilt_deg():.0f} degrees")
        for k in self.obj:
            if self._touches(k, {self.floor}) and not A.bin_contains(self._pos(k)):
                self.event(f"drop:{k}", "damage", f"the {k.replace('_', ' ')} fell to the floor")
            elif (
                self._tilt(k) > 60
                and A.OBJECT_KINDS[self.obj_meta[k]["kind"]]["shape"] == "cylinder"
                and self._touches(k, set(self.surf_geom.values()))
            ):
                where = next(n for n, g in self.surf_geom.items() if self._touches(k, {g})).replace("_", " ")
                self.event(f"tip:{k}", "damage", f"the {k.replace('_', ' ')} was knocked over on the {where}")

    # ---- object state helpers ----------------------------------------------------------------------------------
    def _contacts_of(self, k: str) -> set[int]:
        g0 = self.obj_geom[k]
        d = self.data
        out = set()
        for i in range(d.ncon):
            c = d.contact[i]
            if c.geom1 == g0:
                out.add(int(c.geom2))
            elif c.geom2 == g0:
                out.add(int(c.geom1))
        return out

    def _touches(self, k: str, geoms: set[int]) -> bool:
        return bool(self._contacts_of(k) & geoms)

    def _held(self, k: str) -> bool:
        cs = self._contacts_of(k)
        return all(cs & side for side in self.robot.side_geoms)

    def _touch_robot(self, k: str) -> bool:
        return bool(self._contacts_of(k) & self.robot.robot_geoms)

    def _tilt(self, k: str) -> float:
        R = self.data.xmat[self.obj[k]].reshape(3, 3)
        return math.degrees(math.acos(float(np.clip(R[2, 2], -1, 1))))

    def _pos(self, k: str) -> np.ndarray:
        return self.data.xpos[self.obj[k]].copy()

    def drawer_open(self) -> float:
        return float(self.data.qpos[self.drawer_q])

    def _resting_on(self, k: str) -> str | None:
        if self._held(k):
            return "gripper"
        cs = self._contacts_of(k)
        if self.floor in cs:
            return "floor"
        if cs & self.drawer_geoms:
            return "drawer"
        for nm, g in self.surf_geom.items():
            if g in cs:
                return nm
        if any(self.gname[g].startswith("bin_") for g in cs):
            return "bin"
        others = [self.geom_obj[g] for g in cs if g in self.geom_obj]
        if others:
            return others[0]
        if cs & self.robot.robot_geoms:
            return "robot"
        return None

    def _in_place(self, k: str, where: str) -> bool:
        """The judge's placement rule (see the task instructions)."""
        p = self._pos(k)
        if self._touch_robot(k):
            return False
        if where == "bin":
            return A.bin_contains(p)
        if where == "drawer":
            return A.drawer_interior_contains(p, self.drawer_open()) and self._touches(k, self.drawer_geoms)
        names = ["shelf_lower", "shelf_upper"] if where == "shelf" else [where]
        for nm in names:
            x0, x1, y0, y1, _, _ = self.surf[nm]
            if A.in_rect(p, x0, x1, y0, y1) and self._touches(k, {self.surf_geom[nm]}) and self._tilt(k) < 20:
                return True
        return False

    # ---- observation -------------------------------------------------------------------------------------------
    def observe(self) -> dict:
        r = self.robot
        x, y, yaw = r.base_pose()
        v, vy, wz = r.base_twist()
        held = [k for k in self.obj if self._held(k)]
        rob: dict = {
            "base": {"x": round(x, 3), "y": round(y, 3), "yaw_deg": round(math.degrees(yaw), 1)},
            "base_velocity": {
                "forward_m_s": round(v, 3),
                **({"left_m_s": round(vy, 3)} if self.rkey == "google" else {}),
                "yaw_rate_deg_s": round(math.degrees(wz), 1),
            },
            "gripper": {
                "pos": r3(r.ee_pos()),
                "opening": round(r.gripper_opening(), 2),
                "holding": held,
                "touching": sorted({self._geom_label(g) for g in self._finger_contacts()}),
            },
            "arm_posture": (
                (self.posture[0] + " (moving)") if self.posture_busy() else "stowed" if self.stowed else "free"
            ),
        }
        if self.rkey == "stretch":
            wy_, wp_ = r.wrist()
            rob["arm"] = {
                "lift_m": round(r.lift(), 3),
                "extension_m": round(r.ext(), 3),
                "wrist_yaw_deg": round(math.degrees(wy_), 1),
                "wrist_pitch_deg": round(math.degrees(wp_), 1),
                "note": "the arm extends to the robot's right (-y in the robot frame); wrist yaw 0 points the gripper along it",
            }
        else:
            rob["arm"] = {
                "gripper_target": r3(r.arm.target_world()),
                "note": "the gripper points along the robot's heading and stays level; arm.ee_delta moves the target",
            }
        objs = {}
        for k, meta in self.obj_meta.items():
            kd = A.OBJECT_KINDS[meta["kind"]]
            size = (
                {"diameter": 2 * kd["r"], "height": kd["h"]}
                if kd["shape"] == "cylinder"
                else {"box": [2 * h for h in kd["half"]]}
            )
            objs[k] = {
                "kind": meta["kind"],
                "color": meta["color"],
                "pos": r3(self._pos(k)),
                "tilt_deg": round(self._tilt(k), 1),
                "resting_on": self._resting_on(k),
                "size_m": size,
                "mass_kg": kd["mass"],
            }
        handle = self.data.geom_xpos[self.handle]
        st = {
            "robot": rob,
            "objects": objs,
            "apartment": A.layout(self.drawer_open(), list(handle)),
            "time_s": round(float(self.data.time), 2),
            "safety_events": [e["detail"] for e in self.events],
        }
        return st

    def _finger_contacts(self) -> set[int]:
        d, r = self.data, self.robot
        out = set()
        for i in range(d.ncon):
            c = d.contact[i]
            if c.geom1 in r.finger_geoms and c.geom2 not in r.robot_geoms:
                out.add(int(c.geom2))
            elif c.geom2 in r.finger_geoms and c.geom1 not in r.robot_geoms:
                out.add(int(c.geom1))
        return out

    def _geom_label(self, g: int) -> str:
        if g in self.geom_obj:
            return self.geom_obj[g]
        if g in self.drawer_geoms:
            return "drawer handle" if g == self.handle else "drawer"
        n = self.gname[g]
        for pre in ("dining_table", "kitchen_table", "desk", "counter", "shelf", "bin", "wall", "sofa", "cabinet"):
            if n.startswith(pre):
                return pre.replace("_", " ")
        return n or "furniture"

    def _robot_touches_drawer(self) -> bool:
        d = self.data
        g = d.contact.geom[: d.ncon]
        return bool(
            np.any(
                (self._is_robot[g[:, 0]] & self._is_drawer[g[:, 1]])
                | (self._is_robot[g[:, 1]] & self._is_drawer[g[:, 0]])
            )
        )

    # ---- scoring -----------------------------------------------------------------------------------------------
    def success(self) -> bool:
        if self.events:
            return False
        t = self.task
        if "drawer_min" in t and (self.drawer_open() < t["drawer_min"] or self._robot_touches_drawer()):
            return False
        for k, where in t.get("place", {}).items():
            if not self._in_place(k, where):
                return False
        return all(self._in_place(k, where) for k, where in t.get("keep", {}).items())

    # ---- skills (generators over the public action) ------------------------------------------------------------
    def _act(
        self, twist=(0.0, 0.0, 0.0), arm=(0.0, 0.0, 0.0), lift=0.0, ext=0.0, wrist=(0.0, 0.0), pose=0.0, g=0.0
    ) -> list[float]:
        v, wz, vy = (float(np.clip(c, -1, 1)) for c in twist)
        clip = lambda x: float(np.clip(x, -1, 1))
        if self.rkey == "google":
            return [v, vy, wz, *map(clip, arm), clip(pose), clip(g)]
        if self.rkey == "tiago":
            return [v, wz, *map(clip, arm), clip(pose), clip(g)]
        return [v, wz, clip(lift), clip(ext), clip(wrist[0]), clip(wrist[1]), clip(pose), clip(g)]

    def _pose_report(self) -> dict:
        x, y, yaw = self.robot.base_pose()
        return {"base": {"x": round(x, 3), "y": round(y, 3), "yaw_deg": round(math.degrees(yaw), 1)}}

    def _brake(self, n: int = 8):
        for _ in range(n):
            yield self._act()

    def skill_turn(self, yaw_deg: float):
        r = self.robot
        target = math.radians(yaw_deg)
        ok = 0
        for _ in range(190):
            e = wrap(target - r.base_pose()[2])
            if abs(e) < math.radians(1.5):
                ok += 1
                if ok >= 3:
                    break
            else:
                ok = 0
            wz = float(np.clip(2.0 * e / r.wz_max, -1, 1))
            if abs(wz) < 0.08 and abs(e) > math.radians(1.5):
                wz = 0.08 * np.sign(e)
            yield self._act(twist=(0, wz, 0))
        yield from self._brake(4)
        err = math.degrees(wrap(target - r.base_pose()[2]))
        return {"reached": abs(err) < 3, "heading_error_deg": round(err, 1), **self._pose_report()}

    def skill_drive_to(self, x: float, y: float, yaw_deg="none", tol: float = 0.05):
        """Validates the arguments before the generator starts (YAW_DEG is a number or `none`)."""
        if str(yaw_deg).strip().lower() in ("none", ""):
            yaw_goal = None
        else:
            try:
                yv = float(yaw_deg)
            except ValueError:
                raise ValueError("yaw_deg must be a number of degrees or none") from None
            if not math.isfinite(yv):
                raise ValueError("yaw_deg must be a finite number")
            yaw_goal = math.radians(yv)
        return self._drive_to(x, y, yaw_goal, tol)

    def _drive_to(self, x: float, y: float, yaw_goal, tol: float):
        r = self.robot
        tol = max(float(tol), 0.02)
        goal = np.array([x, y])
        blocked = False
        hist: list[np.ndarray] = []
        if self.rkey == "google":
            for i in range(640):
                px, py, yaw = r.base_pose()
                e = goal - np.array([px, py])
                dist = float(np.linalg.norm(e))
                eyaw = 0.0 if yaw_goal is None else wrap(yaw_goal - yaw)
                if dist < tol and abs(eyaw) < math.radians(2):
                    break
                sp = min(r.v_max, 1.2 * dist + 0.01, math.sqrt(1.2 * r.a_max * dist))
                vw = e / max(dist, 1e-6) * sp
                c, s = math.cos(yaw), math.sin(yaw)
                vb = np.array([c * vw[0] + s * vw[1], -s * vw[0] + c * vw[1]]) / r.v_max
                wz = float(np.clip(2.0 * eyaw / r.wz_max, -1, 1))
                hist.append(np.array([px, py]))
                if i > 30 and dist > tol and np.linalg.norm(hist[-1] - hist[-21]) < 0.01:
                    blocked = True
                    break
                yield self._act(twist=(vb[0], wz, vb[1]))
        else:
            # 1) face the goal (if it is not already close), 2) drive with heading correction, 3) final heading
            px, py, yaw = r.base_pose()
            if np.linalg.norm(goal - [px, py]) > tol:
                head = math.degrees(math.atan2(y - py, x - px))
                res = yield from self.skill_turn(head)
                del res
            for i in range(880):
                px, py, yaw = r.base_pose()
                e = goal - np.array([px, py])
                dist = float(np.linalg.norm(e))
                fwd = e @ np.array([math.cos(yaw), math.sin(yaw)])
                if dist < tol or (fwd < 0 and dist < 3 * tol):
                    break
                head_err = wrap(math.atan2(e[1], e[0]) - yaw)
                if abs(head_err) > math.radians(60):  # overshot or pushed off course: turn back towards the goal
                    res = yield from self.skill_turn(math.degrees(math.atan2(e[1], e[0])))
                    continue
                sp = min(r.v_max, 1.2 * dist + 0.02, math.sqrt(1.2 * r.a_max * dist)) * max(0.0, math.cos(head_err))
                wz = float(np.clip(2.5 * head_err / r.wz_max, -1, 1))
                hist.append(np.array([px, py]))
                if i > 30 and np.linalg.norm(hist[-1] - hist[-21]) < 0.01:
                    blocked = True
                    break
                yield self._act(twist=(sp / r.v_max, wz, 0))
        yield from self._brake(6)
        if yaw_goal is not None and self.rkey != "google" and not blocked:
            res = yield from self.skill_turn(math.degrees(yaw_goal))
        px, py, yaw = r.base_pose()
        dist = float(np.linalg.norm(goal - [px, py]))
        out = {"reached": dist < tol * 1.5 and not blocked, "distance_m": round(dist, 3), **self._pose_report()}
        if blocked:
            out["blocked"] = "the base stopped moving (something is in the way)"
        hits = [e["detail"] for e in self.events if e["kind"] == "crash"]
        if hits:
            out["safety_events"] = hits
        return out

    def _arm_ready_first(self):
        """Yield posture steps until an ongoing posture move (or a stowed arm) is ready."""
        r = self.robot
        need = self.posture_busy() or self.stowed
        if self.rkey == "stretch":
            wy, wp = r.wrist()
            need = need or abs(wy) > 0.3 or abs(wp) > 0.3
        if not need:
            return False
        yield self._act(pose=1)
        for _ in range(180):
            if not self.posture_busy():
                break
            yield self._act()
        return True

    def skill_ready(self):
        yield self._act(pose=1)
        for _ in range(195):
            if not self.posture_busy():
                break
            yield self._act()
        return {"done": not self.posture_busy(), "gripper": r3(self.robot.ee_pos())}

    def skill_stow(self):
        yield self._act(pose=-1)
        for _ in range(195):
            if not self.posture_busy():
                break
            yield self._act()
        return {"done": not self.posture_busy(), "gripper": r3(self.robot.ee_pos())}

    def skill_grasp(self):
        r = self.robot
        last, still = r.gripper_opening(), 0
        for _ in range(36):
            yield self._act(g=1)
            o = r.gripper_opening()
            still = still + 1 if abs(o - last) < 0.004 else 0
            last = o
            if still >= 4:
                break
        held = [k for k in self.obj if self._held(k)]
        touching = sorted({self._geom_label(g) for g in self._finger_contacts()})
        return {
            "holding": held,
            "touching": touching,
            "opening": round(r.gripper_opening(), 2),
            "note": "nothing between the fingers" if not held and not touching else "",
        }

    def skill_open_gripper(self):
        for _ in range(14):
            yield self._act(g=-1)
        return {"opening": round(self.robot.gripper_opening(), 2)}

    def skill_reach(self, x: float, y: float, z: float):
        target = np.array([x, y, z], float)
        if self.rkey == "stretch":
            res = yield from self._stretch_reach(target)
            return res
        r = self.robot
        yield from self._arm_ready_first()
        # 1) move the gripper target along a straight line (at most 1 cm per step) until it is at the point
        best_t, stuck = 9.0, 0
        for _ in range(200):
            e_t = target - r.arm.target_world()
            n = float(np.linalg.norm(e_t))
            if n < 1e-4:
                break
            if n < best_t - 1e-3:
                best_t, stuck = n, 0
            else:
                stuck += 1
                if stuck > 15:
                    break  # the target is clamped (outside the arm's work box, or held back because the arm is blocked)
            yield self._act(arm=(e_t * min(1.0, 0.01 / n)) / EE_STEP)
        # 2) hold the target and let the arm converge
        best, stall = 9.0, 0
        for _ in range(60):
            err = float(np.linalg.norm(target - r.ee_pos()))
            if err < 0.005:
                break
            if err < best - 0.0003:
                best, stall = err, 0
            else:
                stall += 1
                if stall > 12:
                    break
            yield self._act()
        err = float(np.linalg.norm(target - r.ee_pos()))
        out = {
            "reached": err < 0.015,
            "error_m": round(err, 3),
            "gripper": r3(r.ee_pos()),
            "touching": sorted({self._geom_label(g) for g in self._finger_contacts()}),
        }
        if err >= 0.015:
            out["note"] = (
                "the gripper did not get there (out of the arm's reach, or blocked); move the base closer or check for contact"
            )
        return out

    def _stretch_reach(self, target):
        r: Stretch = self.robot
        yield from self._arm_ready_first()
        # wrist straight
        for _ in range(60):
            wy, wp = r.wrist()
            if abs(wy) < 0.03 and abs(wp) < 0.03:
                break
            yield self._act(wrist=(-wy / WRIST_STEP, -wp / WRIST_STEP))
        # 1) lift to the height (grasp point offset measured on the robot)
        prev = r.lift()
        for _ in range(160):
            dz = target[2] - r.ee_pos()[2]
            moving = abs(r.lift() - prev) > 0.001
            prev = r.lift()
            if abs(dz) < 0.004 and not moving:
                break
            yield self._act(lift=float(np.clip(dz / LIFT_STEP, -1, 1)))
        # 2) base along its heading so the point lies on the arm's line
        for _k in range(200):
            dx = float(r.to_base(target)[0] - r.to_base(r.ee_pos())[0])
            if abs(dx) < 0.006:
                break
            v = float(np.clip(1.5 * dx, -0.08, 0.08))
            if abs(v) < 0.02:
                v = 0.02 * np.sign(dx)
            yield self._act(twist=(v / r.v_max, 0, 0))
        yield from self._brake(4)
        # 3) extend (the grasp point moves along -y of the robot frame)
        need = float(r.to_base(r.ee_pos())[1] - r.to_base(target)[1])
        e_goal = r.ext() + need
        out_of_range = not (r.EXT[0] - 0.005 <= e_goal <= r.EXT[1] + 0.005)
        e_goal = float(np.clip(e_goal, *r.EXT))
        prev, still = r.ext(), 0
        for _ in range(150):
            de = e_goal - r.ext()
            moving = abs(r.ext() - prev) > 0.001
            prev = r.ext()
            if abs(de) < 0.004 and not moving:
                break
            still = 0 if moving or abs(de) < 0.004 else still + 1
            if still > 20:
                break  # the telescope is blocked
            yield self._act(ext=float(np.clip(de / EXT_STEP, -0.6, 0.6)))
        for _ in range(6):
            yield self._act()
        err = float(np.linalg.norm(target - r.ee_pos()))
        out = {
            "reached": err < 0.015,
            "error_m": round(err, 3),
            "gripper": r3(r.ee_pos()),
            "lift_m": round(r.lift(), 3),
            "extension_m": round(r.ext(), 3),
            "touching": sorted({self._geom_label(g) for g in self._finger_contacts()}),
        }
        if out_of_range:
            out["note"] = (
                "the point is outside the telescope's range (0 to 0.52 m of extension): place the base with its right side "
                "facing the point, 0.40 to 0.91 m from the robot's centre line"
            )
        elif err >= 0.015:
            out["note"] = "the gripper did not get there (blocked?)"
        return out


# ------------------------------------------------------------------------------------------------------------------
# reference solutions (socket only)
# ------------------------------------------------------------------------------------------------------------------


def oracle_main(env: str) -> None:
    from .sim.oracles import solve

    Oracle().run(lambda o: solve(o, env, TASKS[env]))
