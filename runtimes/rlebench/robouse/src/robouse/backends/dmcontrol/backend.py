"""dm_control backend: tasks from DeepMind's dm_control (github.com/google-deepmind/dm_control, Apache-2.0): the
DeepMind Control Suite (cart-pole, pendulum, acrobot, reacher, point mass, finger, ball-in-cup, planar manipulator) and
the dm_control.manipulation tasks with the Kinova Jaco arm.

dm_control 1.0.47 pins its own MuJoCo (3.14.0), newer than the main environment's, so the simulator runs in its own
virtualenv (default ~/.cache/robouse/dmcontrol-venv, override with ROBOUSE_DMCONTROL_PYTHON) as a subprocess
(worker.py) that speaks JSON lines. This module is the trusted side: the task table, the embodiment
declarations, the success rule and the reference solutions. Setup: docs/suites/dmcontrol.md.

Actions. Control Suite tasks take the domain's own actuator commands in [-1, 1] (one step = the task's control timestep);
the Jaco tasks take an end-effector command [DX, DY, DZ, GRIP] (the worker turns it into joint velocities).

Success (every task): the task's own "in target" condition (dm_control's sparse reward equal to 1, e.g. the reacher's
fingertip inside the target sphere, the ball inside the cup, the cart-pole upright and centred) holds for `hold`
consecutive control steps. The episode ends as solved the moment that happens (success_mode: first).
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
DM_CONTROL_VERSION = "1.0.47"
MUJOCO_VERSION = "3.14.0"

# ---------------------------------------------------------------------------------------------------------------------
# domains: actuators (name, unit per command unit, doc) and cameras
# ---------------------------------------------------------------------------------------------------------------------

DOMAINS: dict[str, dict] = {
    "cartpole": dict(
        robot="cart-pole",
        family="classic_control",
        dt=0.01,
        acts=[("F", "force on the cart along x: 10 N per unit (+ = +x)")],
        cameras=["fixed", "lookatcart"],
    ),
    "pendulum": dict(
        robot="torque-limited pendulum",
        family="classic_control",
        dt=0.02,
        acts=[("TAU", "torque at the hinge: 1 N m per unit (+ turns the pole from +z towards +x)")],
        cameras=["fixed", "lookat"],
    ),
    "acrobot": dict(
        robot="acrobot (two-link, elbow-actuated)",
        family="classic_control",
        dt=0.01,
        acts=[
            (
                "TAU",
                "torque at the elbow: 2 N m per unit (+ turns the lower arm from +z towards +x relative to the upper arm)",
            )
        ],
        cameras=["fixed", "lookat"],
    ),
    "reacher": dict(
        robot="planar two-link reacher",
        family="planar_arm",
        dt=0.02,
        acts=[
            ("SHOULDER", "shoulder torque: 0.05 N m per unit (+ = counter-clockwise seen from above)"),
            ("WRIST", "wrist torque: 0.05 N m per unit (+ = counter-clockwise)"),
        ],
        cameras=["fixed"],
    ),
    "point_mass": dict(
        robot="point mass",
        family="point_mass",
        dt=0.02,
        acts=[("FX", "force along x: 0.1 N per unit"), ("FY", "force along y: 0.1 N per unit")],
        cameras=["fixed", "cam0"],
    ),
    "finger": dict(
        robot="planar two-link finger and a free spinner",
        family="planar_arm",
        dt=0.02,
        acts=[
            ("PROXIMAL", "proximal joint torque: 30 N m per unit"),
            ("DISTAL", "distal joint torque: 15 N m per unit"),
        ],
        cameras=["cam0", "cam1"],
    ),
    "ball_in_cup": dict(
        robot="planar cup (ball on a string)",
        family="planar_arm",
        dt=0.02,
        acts=[
            ("FX", "force on the cup along x: 5 N per unit"),
            ("FZ", "force on the cup along z: 5 N per unit (+ = up)"),
        ],
        cameras=["cam0", "cam1"],
    ),
    "manipulator": dict(
        robot="planar manipulator (4-joint arm, two-finger gripper)",
        family="planar_arm",
        dt=0.01,
        acts=[
            ("ROOT", "root joint torque: 12 N m per unit"),
            ("SHOULDER", "shoulder torque: 8 N m per unit"),
            ("ELBOW", "elbow torque: 4 N m per unit"),
            ("WRIST", "wrist torque: 2 N m per unit"),
            ("GRASP", "finger tendon force: 2 N per unit (+ closes the fingers)"),
        ],
        cameras=["fixed", "hand"],
    ),
    "jaco": dict(
        robot="Kinova Jaco arm with the three-finger Jaco hand", family="arm", dt=0.04, acts=[], cameras=["front_close"]
    ),
}

# ---------------------------------------------------------------------------------------------------------------------
# tasks: upstream task, seed (the upstream task's `random` seed: layout, target and initial pose), hold, budget, camera
# ---------------------------------------------------------------------------------------------------------------------

TASKS: dict[str, dict] = {
    # classic control: swing up and balance (dm_control's sparse "in target" conditions, held)
    "dmcontrol-cartpole-swingup": dict(
        kind="suite", domain="cartpole", task="swingup", seed=1, hold=200, max_steps=1500, camera="fixed"
    ),
    "dmcontrol-cartpole-balance": dict(
        kind="suite", domain="cartpole", task="balance", seed=2, hold=800, max_steps=1000, camera="fixed"
    ),
    "dmcontrol-pendulum-swingup": dict(
        kind="suite", domain="pendulum", task="swingup", seed=9, hold=100, max_steps=1000, camera="fixed"
    ),
    "dmcontrol-acrobot-swingup": dict(
        kind="suite", domain="acrobot", task="swingup", seed=9, hold=20, max_steps=3000, camera="fixed"
    ),
    # reaching and point-to-point control
    "dmcontrol-reacher-easy": dict(
        kind="suite", domain="reacher", task="easy", seed=9, hold=25, max_steps=500, camera="fixed"
    ),
    "dmcontrol-reacher-hard": dict(
        kind="suite", domain="reacher", task="hard", seed=0, hold=25, max_steps=500, camera="fixed"
    ),
    "dmcontrol-reacher-hard-b": dict(
        kind="suite", domain="reacher", task="hard", seed=4, hold=25, max_steps=500, camera="fixed"
    ),
    "dmcontrol-point-mass-easy": dict(
        kind="suite", domain="point_mass", task="easy", seed=9, hold=25, max_steps=500, camera="fixed"
    ),
    # contact-rich planar manipulation
    "dmcontrol-finger-turn-easy": dict(
        kind="suite", domain="finger", task="turn_easy", seed=1, hold=25, max_steps=1000, camera="cam1"
    ),
    "dmcontrol-finger-turn-hard": dict(
        kind="suite", domain="finger", task="turn_hard", seed=0, hold=25, max_steps=1000, camera="cam1"
    ),
    "dmcontrol-ball-in-cup-catch": dict(
        kind="suite", domain="ball_in_cup", task="catch", seed=3, hold=50, max_steps=1000, camera="cam1"
    ),
    "dmcontrol-manipulator-bring-ball": dict(
        kind="suite", domain="manipulator", task="bring_ball", seed=2, hold=25, max_steps=2000, camera="fixed"
    ),
    # dm_control.manipulation: Kinova Jaco arm with the three-finger Jaco hand
    "dmcontrol-jaco-reach-site": dict(
        kind="manipulation", task="reach_site_features", seed=4, hold=10, max_steps=300, camera="front_close"
    ),
    "dmcontrol-jaco-lift-brick": dict(
        kind="manipulation", task="lift_brick_features", seed=4, hold=25, max_steps=600, camera="front_close"
    ),
    "dmcontrol-jaco-place-brick": dict(
        kind="manipulation", task="place_brick_features", seed=5, hold=25, max_steps=800, camera="front_close"
    ),
    "dmcontrol-jaco-place-cradle": dict(
        kind="manipulation", task="place_cradle_features", seed=2, hold=25, max_steps=800, camera="front_close"
    ),
    "dmcontrol-jaco-stack-2-bricks": dict(
        kind="manipulation", task="stack_2_bricks_features", seed=5, hold=25, max_steps=1000, camera="front_close"
    ),
}


class DMControlBackend(EmbodiedBackend):
    name = "dmcontrol"
    image_flipped = False

    def __init__(self, spec: dict):
        super().__init__(spec)
        env = spec.get("env") or spec.get("id")
        if env not in TASKS:
            raise KeyError(f"unknown dmcontrol task {env!r}")
        self.task_id, self.task = env, dict(TASKS[env])
        self.domain = "jaco" if self.task["kind"] == "manipulation" else self.task["domain"]
        self.camera = spec.get("camera", self.task["camera"])
        self.render_size = tuple(spec.get("render_size", (480, 480)))
        self.decl = self._declare()
        self.worker = StdioWorker(
            "dmcontrol", config.sim_python("dmcontrol", "dmcontrol"), WORKER, setup=config.SUITES_DOCS
        )
        self._state = None
        self._success = False
        self.last_judge: dict = {}

    def _declare(self, fields: list[str] | None = None) -> Embodiment:
        dom = DOMAINS[self.domain]
        state_doc = (
            "the task's own observation (dm_control's observation dict, same key names) plus the positions named in the instruction"
            if self.domain != "jaco"
            else "the hand, the fingers and the objects (positions in metres, world frame)"
        )
        skip = {"reward", "in_target", "hold_steps", "hold_required"}
        sensors = [
            Sensor("task_state", "world", [f for f in (fields or []) if f not in skip], state_doc, frame="world"),
            Sensor(
                "success",
                "events",
                ["reward", "in_target", "hold_steps", "hold_required"],
                "the task's reward, whether the success condition holds now, and for how many consecutive steps",
            ),
        ]
        sensors += [Sensor(f"camera:{c}", "camera", mount="world") for c in dom["cameras"]]
        if self.domain == "jaco":
            groups = [
                ActionGroup(
                    "arm.ee_delta",
                    "ee_delta_pos",
                    ["DX", "DY", "DZ"],
                    [-2.0] * 3,
                    [2.0] * 3,
                    "cm per unit",
                    "move the commanded pinch point 1 cm per unit along world x, y, z",
                    frame="world",
                ),
                ActionGroup(
                    "arm.yaw_delta",
                    "ee_delta_yaw",
                    ["DYAW"],
                    [-2.0],
                    [2.0],
                    "0.1 rad per unit",
                    "turn the commanded hand yaw about the vertical (+ = counter-clockwise seen from above); the hand always points down",
                ),
                ActionGroup(
                    "gripper", "gripper", ["GRIP"], [-1.0], [1.0], "", "finger velocity: +1 closes, -1 opens, 0 holds"
                ),
            ]
            skills = [
                Skill(
                    "move_to",
                    [
                        SkillArg("x", "float", "m"),
                        SkillArg("y", "float", "m"),
                        SkillArg("z", "float", "m"),
                        SkillArg("yaw", "float", "deg", 999.0, doc="hand yaw to turn to while moving (999 = keep)"),
                        SkillArg("grip", "float", "", 0.0, doc="GRIP held while moving"),
                    ],
                    "move the pinch point in a straight line to (X, Y, Z) in the world frame, optionally turning the hand to YAW",
                    150,
                ),
                Skill(
                    "grip",
                    [SkillArg("value", "float", "", 1.0, doc="+1 close, -1 open"), SkillArg("steps", "int", "", 15)],
                    "close or open the fingers for STEPS steps without moving the arm",
                    50,
                ),
            ]
        else:
            n = len(dom["acts"])
            groups = [
                ActionGroup(
                    "actuators",
                    "motor",
                    [a for a, _ in dom["acts"]],
                    [-1.0] * n,
                    [1.0] * n,
                    "normalised",
                    "; ".join(f"{a}: {d}" for a, d in dom["acts"]),
                )
            ]
            skills = []
        return Embodiment(
            robot=dom["robot"],
            family=dom["family"],
            assets=[f"dm_control {DM_CONTROL_VERSION} ({self.domain})"],
            sensors=sensors,
            action_groups=groups,
            skills=skills,
            budget=Budget(int(self.task["max_steps"]), float(dom["dt"])),
            cameras=list(dom["cameras"]),
        )

    # ---- worker ------------------------------------------------------------------------------------------------
    def _call(self, cmd: str, **kw):
        return self.worker.call(cmd, **kw)

    # ---- episode -----------------------------------------------------------------------------------------------
    def reset(self, seed: int) -> None:
        t = self.task
        kw = {k: v for k, v in t.items() if k not in ("kind", "domain", "task", "seed", "hold", "max_steps", "camera")}
        self.meta = self._call(
            "make",
            kind=t["kind"],
            domain=t.get("domain", "jaco"),
            task=t["task"],
            seed=int(t["seed"]),
            hold=int(t["hold"]),
            **kw,
        )
        self._state = None
        self._success = False
        self.decl = self._declare(list(self.observe()))  # `robo info` lists the observation's field names

    def control_step(self, a: np.ndarray) -> None:
        r = self._call("step", action=[float(x) for x in a])
        self._state = None
        self._success = self._success or bool(r["success"])

    def step(self, action) -> StepInfo:
        a = self.action_spec.clip(action)
        if not np.all(np.isfinite(a)):
            raise ValueError("action values must be finite numbers")
        self.control_step(a)
        return StepInfo(success=self._success)

    def observe(self) -> dict:
        if self._state is None:
            self._state = self._call("observe")
        return json.loads(json.dumps(self._state))

    def success(self) -> bool:
        return self._success

    def judge(self, outcome: str, text: str = "") -> bool:
        try:
            j = self._call("success")
        except RuntimeError as e:
            j = {"success": False, "error": str(e)}
        self.last_judge = {**j, "rule": f"in_target held for {self.task['hold']} consecutive steps"}
        return bool(self._success or j.get("success"))

    def mj_model_data(self):
        return None

    def render(self, width: int | None = None, height: int | None = None) -> np.ndarray:
        w, h = (width, height) if width and height else self.render_size
        return self._call(
            "render", width=int(w), height=int(h), camera=self.camera or DOMAINS[self.domain]["cameras"][0]
        )

    def close(self) -> None:
        self.worker.close()

    # ---- skills (Jaco tasks) -----------------------------------------------------------------------------------
    def skill_move_to(self, x: float, y: float, z: float, yaw: float = 999.0, grip: float = 0.0):
        target = np.array([x, y, z], dtype=float)
        g = float(np.clip(grip, -1, 1))
        last, stuck = None, 0
        for _ in range(400):
            st = self.observe()
            hp = np.asarray(st["hand_pos"], dtype=float)
            d = target - hp
            dyaw = 0.0 if yaw == 999.0 else _wrap(math.radians(yaw) - math.radians(st["hand_target_yaw_deg"]))
            yaw_err = 0.0 if yaw == 999.0 else abs(_wrap(math.radians(yaw) - math.radians(st["hand_yaw_deg"])))
            if np.linalg.norm(d) < 0.006 and yaw_err < 0.05:
                break
            stuck = stuck + 1 if last is not None and np.linalg.norm(hp - last) < 5e-4 else 0
            if stuck > 10:
                break
            last = hp
            cmd = np.asarray(st["hand_target_pos"], dtype=float)
            step = np.clip((target - cmd) / 0.01, -2.0, 2.0)  # move the commanded point straight to the target
            yield list(step) + [float(np.clip(dyaw / 0.1, -2.0, 2.0)), g]
        st = self.observe()
        dist = float(np.linalg.norm(target - np.asarray(st["hand_pos"], dtype=float)))
        return {"reached": dist < 0.015, "distance_m": round(dist, 4), "hand_yaw_deg": st["hand_yaw_deg"]}

    def skill_grip(self, value: float = 1.0, steps: int = 15):
        for _ in range(max(1, int(steps))):
            yield [0.0, 0.0, 0.0, 0.0, float(np.clip(value, -1, 1))]
        return {"finger_joints": self.observe()["finger_joints"]}


# =====================================================================================================================
# reference solutions: scripted controllers that read `robo observe` and send `robo act`, through the socket only
# =====================================================================================================================


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def _v(st: dict, k: str) -> np.ndarray:
    return np.asarray(st[k], dtype=float)


def _loop(o, policy, n: int) -> None:
    """Run a state-feedback policy for up to n steps (one `act` per step)."""
    st = o.state()
    mem: dict = {}
    for _ in range(n):
        a = policy(st, mem)
        st = o.act(a)["state"]


# LQR gains around the upright equilibrium (discrete-time, from finite-difference linearisations of the dm_control
# models; state = [q..., qdot...] relative to upright)
K_CARTPOLE = np.array([-0.9265, -7.4433, -1.135, -1.8162])
K_PENDULUM = np.array([10.2039, 2.2038])
K_ACROBOT = np.array([-202.8203, -51.9946, -74.6147, -22.1877])


def cartpole_policy(st: dict, mem: dict) -> list[float]:
    x, xd, th, thd = st["cart_x"], st["cart_vel"], st["pole_angle"], st["pole_angvel"]
    if math.cos(th) > 0.85:  # near upright: LQR
        u = -float(K_CARTPOLE @ np.array([x, th, xd, thd]))
    else:  # energy pumping (pole: m = 0.1 kg, l = 1 m, com at 0.5 m) with a weak pull to the centre
        E = 0.5 * (1 / 3) * thd**2 + 9.81 * 0.5 * (math.cos(th) - 1)  # per unit mass-ish; 0 at the top, at rest
        u = 3.0 * E * (1 if thd * math.cos(th) < 0 else -1) - 0.3 * x - 0.3 * xd
        u = -u
    return [float(np.clip(u, -1, 1))]


def pendulum_policy(st: dict, mem: dict) -> list[float]:
    th, thd = st["pole_angle"], st["pole_angvel"]
    if math.cos(th) > math.cos(math.radians(25)):
        u = -float(K_PENDULUM @ np.array([th, thd]))
    else:
        E = 0.5 * 0.25 * thd**2 + 9.81 * 0.5 * (math.cos(th) - 1)  # 0 at the top, at rest (m = 1 kg at 0.5 m)
        u = np.sign(thd if abs(thd) > 1e-3 else 1.0) * (1.0 if E < 0 else -1.0)
    return [float(np.clip(u, -1, 1))]


REACHER_L = (0.12, 0.12)  # shoulder -> wrist, wrist -> fingertip centre (m)


def reacher_policy(st: dict, mem: dict) -> list[float]:
    """Analytic two-link inverse kinematics for the target, then joint PD torques."""
    q1, q2 = st["shoulder_angle"], st["wrist_angle"]
    if "q_des" not in mem:
        tx, ty = st["target_pos"]
        l1, l2 = REACHER_L
        r2 = min(tx * tx + ty * ty, (l1 + l2 - 1e-4) ** 2)
        c2 = np.clip((r2 - l1 * l1 - l2 * l2) / (2 * l1 * l2), -1, 1)
        sols = []
        for s2 in (math.acos(c2), -math.acos(c2)):
            a1 = math.atan2(ty, tx) - math.atan2(l2 * math.sin(s2), l1 + l2 * math.cos(s2))
            sols.append((abs(_wrap(a1 - q1)) + abs(s2 - q2), a1, s2))
        _, a1, a2 = min(sols)
        mem["q_des"] = (a1, a2)
    a1, a2 = mem["q_des"]
    e1, e2 = _wrap(a1 - q1), a2 - q2
    return [
        float(np.clip(8.0 * e1 - 0.6 * st["shoulder_vel"], -1, 1)),
        float(np.clip(8.0 * e2 - 0.4 * st["wrist_vel"], -1, 1)),
    ]


def point_mass_policy(st: dict, mem: dict) -> list[float]:
    e = _v(st, "target_pos") - _v(st, "mass_pos")
    u = 30.0 * e - 6.0 * _v(st, "mass_vel")
    return [float(x) for x in np.clip(u, -1, 1)]


FINGER_BASE, FINGER_L = (
    np.array([-0.2, 0.4]),
    (0.18, 0.161),
)  # proximal pivot (x, z); link to distal pivot, to the fingertip end


def _ik2(target, base, l1, l2, q_now, lim=1.9):
    """Planar two-link inverse kinematics (angles counter-clockwise from +x in the x-z plane); the reachable solution
    closest to q_now, or None."""
    d = np.asarray(target, float) - base
    r2 = float(d @ d)
    c2 = (r2 - l1 * l1 - l2 * l2) / (2 * l1 * l2)
    if abs(c2) > 1:
        return None
    best = None
    for q2 in (math.acos(c2), -math.acos(c2)):
        q1 = math.atan2(d[1], d[0]) - math.atan2(l2 * math.sin(q2), l1 + l2 * math.cos(q2))
        q1 = _wrap(q1)
        if abs(q1) > lim or abs(q2) > lim:
            continue
        cost = abs(q1 - q_now[0]) + abs(q2 - q_now[1])
        if best is None or cost < best[0]:
            best = (cost, q1, q2)
    return None if best is None else (best[1], best[2])


def finger_policy(st: dict, mem: dict) -> list[float]:
    """Turn the spinner by pushing the side of one of its arms with the fingertip: pick the reachable point along the
    arm with the most leverage, approach it from behind (clearing the spinner first), push it tangentially towards the
    goal angle, back off once the tip is in the target circle."""
    q = np.array([st["proximal_angle"], st["distal_angle"]])
    qd = np.array([st["proximal_vel"], st["distal_vel"]])
    c = _v(st, "spinner_center")
    tip = _v(st, "spinner_tip_pos") - c
    beta = math.atan2(tip[0], tip[1])  # spinner angle (0 = tip end up, + towards +x)
    tgt = _v(st, "target_pos") - c
    delta = _wrap(math.atan2(tgt[0], tgt[1]) - beta)  # rotation still needed
    ft = _v(st, "fingertip_end_pos")
    tol = 0.5 * st["target_radius"] / 0.13
    u = np.array([math.sin(beta), math.cos(beta)])
    t = np.array([math.cos(beta), -math.sin(beta)])  # motion of the point at +u when beta increases
    sgn = 1.0 if delta > 0 else -1.0
    reach = FINGER_L[0] + FINGER_L[1] - 0.008

    def plan(rho):
        pdir = sgn * (1.0 if rho > 0 else -1.0) * t  # push direction at that point
        surf = c + rho * u - 0.06 * pdir  # arm surface facing the finger's push
        return pdir, surf

    def ok(pt):
        return np.linalg.norm(pt - FINGER_BASE) < reach and pt[0] < c[0] + 0.02

    phase = mem.get("phase", "clear")
    if st["in_target"] or abs(delta) < tol:
        phase = "retract"
    elif phase == "retract":
        phase = "clear"
    cands = []
    for r in (0.085, -0.085, 0.07, -0.07, 0.055, -0.055, 0.04, -0.04, 0.025, -0.025):
        pdir, surf = plan(r)
        if ok(surf - 0.03 * pdir) and ok(surf - 0.07 * pdir):
            # prefer pushing away from the finger's base (the fingertip leads), then the longest lever
            cands.append((pdir[0] > -0.3, abs(r), r))
    rho = max(cands)[2] if cands else 0.025
    if phase == "push" and (np.sign(rho) != np.sign(mem.get("rho", rho)) or mem.get("sgn") != sgn):
        phase = "approach"  # the push point moved to the other end, or the push direction flipped: reposition
    mem["rho"], mem["sgn"] = rho, sgn
    pdir, surf = plan(rho)
    behind = surf - 0.07 * pdir
    if phase == "clear":  # first move out to the left of the spinner, level with the approach point
        want = np.array([c[0] - 0.18, behind[1]])
        if np.linalg.norm(ft - want) < 0.03 or (ft[0] < c[0] - 0.17 and abs(ft[1] - behind[1]) < 0.04):
            phase = "approach"
    if phase == "approach":
        want = behind
        if np.linalg.norm(ft - behind) < 0.015:
            phase = "push"
    if phase == "push":
        want = surf - 0.03 * pdir + min(0.03, 0.08 * abs(delta) + 0.008) * pdir
        if np.dot(ft - (surf - 0.03 * pdir), pdir) < -0.05 or not ok(surf - 0.03 * pdir):
            phase = "clear" if not ok(surf - 0.03 * pdir) else "approach"
            want = behind
    if phase == "retract":
        want = mem.get("retract_at")
        if want is None:
            want = mem["retract_at"] = ft - 0.05 * (pdir if abs(pdir[0]) > 0.3 else np.array([1.0, 0.0]))
    else:
        mem.pop("retract_at", None)
    mem["phase"] = phase
    sol = _ik2(want, FINGER_BASE, *FINGER_L, q_now=q)
    if sol is None:  # out of reach (too far, or too close for the distal joint's range): the nearest reachable point
        d = want - FINGER_BASE
        l1, l2 = FINGER_L
        rmin = math.sqrt(l1 * l1 + l2 * l2 + 2 * l1 * l2 * math.cos(1.85))
        sol = _ik2(
            FINGER_BASE + d / np.linalg.norm(d) * float(np.clip(np.linalg.norm(d), rmin + 0.005, reach - 0.005)),
            FINGER_BASE,
            *FINGER_L,
            q_now=q,
        )
    q_des = np.array(sol) if sol is not None else q
    tau = (
        np.array([40.0, 12.0]) * (q_des - q) - np.array([0.5, 0.1]) * qd
    )  # the finger has no gravity; its joints are damped
    return [float(np.clip(tau[0] / 30.0, -1, 1)), float(np.clip(tau[1] / 15.0, -1, 1))]


MANIP_ROOT, MANIP_L = np.array([0.0, 0.4]), (0.18, 0.15, 0.12, 0.065)  # arm root (x, z); links; wrist -> grasp site
MANIP_LIM = (None, 2.79, 2.79, 2.44)
MANIP_GEAR = np.array([12.0, 8.0, 4.0, 2.0])


def _manip_fk(q) -> tuple[np.ndarray, list[np.ndarray]]:
    """Planar manipulator forward kinematics: joint angles about -y, 0 = link pointing up (+z); returns the grasp site
    and the joint positions."""
    p, Q, pts = MANIP_ROOT.copy(), 0.0, [MANIP_ROOT.copy()]
    for qi, li in zip(q, MANIP_L, strict=False):
        Q += qi
        p = p + li * np.array([-math.sin(Q), math.cos(Q)])
        pts.append(p.copy())
    return p, pts


def _manip_ik(goal, hand_angle: float, q0, w_angle: float = 0.3) -> np.ndarray:
    """Joint angles putting the grasp site at `goal` with the hand pointing along `hand_angle` (0 = up, pi = down),
    within the joint limits: damped least squares on (x, z, hand angle), the angle weighted less than the position."""
    goal = np.asarray(goal, float)
    q = np.array(q0, dtype=float)
    lo = np.array([-1e9, -2.75, -2.75, -2.4])
    hi = -lo
    W = np.diag([1.0, 1.0, w_angle])
    for _ in range(80):
        Q = np.cumsum(q)
        p = MANIP_ROOT + sum(li * np.array([-math.sin(Qi), math.cos(Qi)]) for li, Qi in zip(MANIP_L, Q, strict=False))
        e = np.array([*(goal - p), _wrap(hand_angle - Q[-1])])
        if np.linalg.norm(e[:2]) < 1e-4 and abs(e[2]) < 1e-3:
            break
        J = np.zeros((3, 4))
        for j in range(4):
            for i in range(j, 4):
                J[:2, j] += MANIP_L[i] * np.array([-math.cos(Q[i]), -math.sin(Q[i])])
            J[2, j] = 1.0
        Jw, ew = W @ J, W @ e
        dq = Jw.T @ np.linalg.solve(Jw @ Jw.T + 1e-3 * np.eye(3), ew)
        q = np.clip(q + np.clip(dq, -0.3, 0.3), lo, hi)
    return q


class ManipulatorOracle:
    """Catch the rolling ball with the open hand standing on the floor, grasp it from above (hand pointing down), carry it
    to the target and hold it there. Joint PD torques with an integral term towards inverse-kinematics targets; the
    commanded grasp point moves in straight lines at up to 0.6 m/s."""

    def __init__(self):
        self.phase, self.n, self.t0, self.integ, self.still, self.cmd = "wait", 0, 0, np.zeros(4), 0, None

    def act(self, st: dict) -> list[float]:
        ja, jv = st["joint_angles"], st["joint_vels"]
        q = np.array([ja[k] for k in ("arm_root", "arm_shoulder", "arm_elbow", "arm_wrist")])
        qd = np.array([jv[k] for k in ("arm_root", "arm_shoulder", "arm_elbow", "arm_wrist")])
        grasp = _v(st, "grasp_site_pos")
        ball = _v(st, "ball_site_pos")
        bv = np.asarray(st["object_vel"][:2], float)
        tgt = _v(st, "target_ball_site_pos")
        self.n += 1
        g, ph = -1.0, self.phase
        if ph == "wait":  # the ball rolls freely on the flat floor: stand the open hand on the floor as a barrier
            want = np.array([0.0, 0.06])
            self.still = self.still + 1 if np.linalg.norm(bv) < 0.2 and ball[1] < 0.03 and abs(ball[0]) < 0.3 else 0
            if self.still > 10 and self.n > 60:
                ph = "rise"
        elif ph == "rise":
            want = np.array([0.0, 0.16])
            if grasp[1] > 0.14:
                ph = "above"
        elif ph == "above":
            want = ball + np.array([bv[0] * 0.3, 0.10])
            if np.linalg.norm(grasp - want) < 0.01:
                ph = "descend"
        elif ph == "descend":
            want = ball + np.array([bv[0] * 0.15, 0.004])
            if np.linalg.norm(grasp - want) < 0.006 or self.n > self.t0 + 120:
                ph = "close"
        elif ph == "close":
            want, g = ball + np.array([0.0, 0.004]), 1.0
            if self.n > self.t0 + 40:
                ph = "lift"
        elif ph == "lift":
            want, g = ball + np.array([0.0, 0.12]), 1.0
            if self.n > self.t0 + 60:
                ph = "carry"
        else:  # carry, then hold: keep correcting by the ball's own offset from the target
            want, g = grasp + (tgt - ball), 1.0
        if ph != self.phase:
            self.phase, self.t0 = ph, self.n
        cmd = grasp.copy() if self.cmd is None else self.cmd
        step = want - cmd
        self.cmd = cmd + step * min(1.0, 0.006 / max(float(np.linalg.norm(step)), 1e-9))
        q_des = _manip_ik(self.cmd, math.pi, q)
        e = np.array([_wrap(q_des[0] - q[0]), *(q_des[1:] - q[1:])])
        self.integ = np.clip(self.integ + e * 0.04, -0.3, 0.3)
        tau = (
            np.array([30.0, 25.0, 12.0, 5.0]) * e
            + np.array([20.0, 15.0, 6.0, 2.0]) * self.integ
            - np.array([1.0, 0.8, 0.4, 0.1]) * qd
        )
        return [float(x) for x in np.clip(tau / MANIP_GEAR, -1, 1)] + [g]


# ---- acrobot: sampling-based model-predictive control (cross-entropy method) on the textbook acrobot model with the
# dm_control parameters (two 1 m links, 1 kg each, centre of mass at 0.5 m, joint damping 0.05), then LQR at the top

ACRO_I, ACRO_LC, ACRO_G, ACRO_DAMP = (0.09557, 0.09530), 0.5, 9.81, 0.05
K_ACROBOT_R10 = np.array([-123.388, -30.905, -45.3, -13.436])
P_ACROBOT = np.array(
    [
        [3360682.88, 945632.27, 1242455.47, 384897.34],
        [945632.27, 267527.57, 349734.46, 108502.62],
        [1242455.47, 349734.46, 459393.9, 142327.7],
        [384897.34, 108502.62, 142327.7, 44121.1],
    ]
)


def _acro_f(x, u):
    q1, q2, d1, d2 = x[..., 0], x[..., 1], x[..., 2], x[..., 3]
    I1, I2 = ACRO_I
    lc, l1 = ACRO_LC, 1.0
    c2, s2 = np.cos(q2), np.sin(q2)
    M11 = I1 + I2 + lc**2 + l1**2 + lc**2 + 2 * l1 * lc * c2
    M12 = I2 + lc**2 + l1 * lc * c2
    M22 = I2 + lc**2
    h = l1 * lc * s2
    C1, C2 = -h * (2 * d1 * d2 + d2 * d2), h * d1 * d1
    G1 = -ACRO_G * ((lc + l1) * np.sin(q1) + lc * np.sin(q1 + q2))
    G2 = -ACRO_G * lc * np.sin(q1 + q2)
    r1 = -C1 - G1 - ACRO_DAMP * d1
    r2 = 2 * u - C2 - G2 - ACRO_DAMP * d2
    det = M11 * M22 - M12 * M12
    return np.stack([d1, d2, (M22 * r1 - M12 * r2) / det, (M11 * r2 - M12 * r1) / det], -1)


def _acro_rk4(x, u, dt):
    k1 = _acro_f(x, u)
    k2 = _acro_f(x + dt / 2 * k1, u)
    k3 = _acro_f(x + dt / 2 * k2, u)
    k4 = _acro_f(x + dt * k3, u)
    return x + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)


def _wrapv(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


class AcrobotOracle:
    H, N, IT, KN, REP, DT = 100, 64, 3, 8, 4, 0.02  # 2 s horizon, 8 control knots, replan every 4 steps

    def __init__(self, seed: int = 0):
        self.rng = np.random.default_rng(seed)
        self.mean, self.plan, self.k = np.zeros(self.KN), None, 0
        self.idx = np.arange(self.KN)
        self.tgrid = np.linspace(0, self.KN - 1, self.H)

    def _cost(self, x0, kn):
        U = np.clip(np.stack([np.interp(self.tgrid, self.idx, k) for k in kn]), -1, 1)
        x = np.repeat(x0[None], len(kn), 0)
        c = np.zeros(len(kn))
        for i in range(self.H):
            x = _acro_rk4(x, U[:, i], self.DT)
            q1, q2 = x[:, 0], x[:, 1]
            d = np.hypot(np.sin(q1) + np.sin(q1 + q2), np.cos(q1) + np.cos(q1 + q2) - 2.0)  # tip to the top
            c += d**2 + 0.01 * (x[:, 2] ** 2 + x[:, 3] ** 2) * (d < 0.5)
        z = np.stack([_wrapv(x[:, 0]), _wrapv(x[:, 1]), x[:, 2], x[:, 3]], -1)
        return c + np.minimum(np.einsum("ni,ij,nj->n", z, P_ACROBOT, z) / 2e5, 50)

    def act(self, st: dict) -> list[float]:
        x = np.array([st["shoulder_angle"], st["elbow_angle"], st["shoulder_vel"], st["elbow_vel"]], dtype=float)
        if float(x @ P_ACROBOT @ x) < 2e4:  # inside the LQR's basin at the top
            self.plan = None
            return [float(np.clip(-K_ACROBOT_R10 @ x, -1, 1))]
        if self.plan is None or self.k >= self.REP:
            if self.plan is not None:  # shift the warm start by the time already executed
                sh = (self.REP * 0.01) / (self.H * self.DT / (self.KN - 1))
                self.mean = np.interp(self.idx + sh, self.idx, self.mean)
            std = np.full(self.KN, 0.7)
            for _ in range(self.IT):
                cand = np.clip(self.mean[None] + std * self.rng.standard_normal((self.N, self.KN)), -1, 1)
                cand[0] = self.mean
                elite = cand[np.argsort(self._cost(x, cand))[: max(4, self.N // 10)]]
                self.mean, std = elite.mean(0), elite.std(0) + 0.05
            self.plan = np.clip(np.interp(np.linspace(0, self.KN - 1, self.H), self.idx, self.mean), -1, 1)
            self.k = 0
        u = float(self.plan[int(self.k * 0.01 / self.DT)])
        self.k += 1
        return [u]


# ---- ball in cup: sampling-based model-predictive control on a point-mass model of the cup (spring-mounted, damped),
# the ball and the string (an inextensible, inelastic tether); contacts are not modelled

CUP_M, BALL_M, CUP_K, CUP_DAMP, STRING, CUP_SITE = 0.0652, 0.0654, 20.0, 3.0, 0.3, 0.108
CUP_HOME = np.array([0.0, 0.6])


def _cup_rollout(c, cv, b, bv, U, dt=0.01, sub=2):
    N, H, _ = U.shape
    cost = np.zeros(N)
    h = dt / sub
    grav = np.array([0.0, -9.81])
    for t in range(H):
        F = 5.0 * U[:, t]
        for _ in range(sub):
            acc = (F - CUP_K * (c - CUP_HOME) - CUP_DAMP * cv) / CUP_M + grav
            cv = cv + h * acc
            c = c + h * cv
            bv = bv + h * grav
            b = b + h * bv
            r = b - (c + np.array([0.0, -CUP_SITE]))
            n = np.linalg.norm(r, axis=1)
            taut = n > STRING
            if taut.any():
                u = r[taut] / n[taut, None]
                ex = (n[taut] - STRING)[:, None] * u
                w = BALL_M / (BALL_M + CUP_M)
                b[taut] -= ex * (1 - w)
                c[taut] += ex * w
                vrel = np.sum((bv[taut] - cv[taut]) * u, 1)
                j = np.where(vrel > 0, vrel, 0.0)[:, None] * u
                bv[taut] -= j * (CUP_M / (BALL_M + CUP_M))
                cv[taut] += j * (BALL_M / (BALL_M + CUP_M))
        cost += np.linalg.norm(b - (c + np.array([0.0, -0.05])), axis=1) * (1 + t / H)
    return cost


class BallInCupOracle:
    H, KN, N, SIGMA = 60, 6, 128, 0.5

    def __init__(self, seed: int = 0):
        self.rng = np.random.default_rng(seed)
        self.nom = np.zeros((self.KN, 2))
        t = np.linspace(0, self.KN - 1, self.H)
        self.W = np.stack([np.interp(t, np.arange(self.KN), np.eye(self.KN)[k]) for k in range(self.KN)], 1)  # H x KN

    def act(self, st: dict) -> list[float]:
        c, cv, b, bv = (_v(st, k) for k in ("cup_pos", "cup_vel", "ball_pos", "ball_vel"))
        cand = self.nom[None] + self.SIGMA * self.rng.standard_normal((self.N, self.KN, 2))
        cand[0], cand[1] = self.nom, 0.0
        cand = np.clip(cand, -1, 1)
        U = np.einsum("hk,nkj->nhj", self.W, cand)
        rep = lambda v: np.repeat(v[None], self.N, 0).astype(float)
        cost = _cup_rollout(rep(c), rep(cv), rep(b), rep(bv), U)
        self.nom = cand[int(np.argmin(cost))]
        return [float(x) for x in self.nom[0]]


# ---- Jaco tasks: scripted pick and place with the move_to and grip skills


def _yaw_near(target_deg: float, ref_deg: float, period: float = 180.0) -> float:
    """The angle equal to target_deg modulo `period` that is closest to ref_deg."""
    return ref_deg + ((target_deg - ref_deg + period / 2) % period - period / 2)


def _jaco_pick(o, name: str, tries: int = 4) -> dict:
    """Grasp a brick from above with the fingers across its width; lift it 5 cm and check that it came along,
    otherwise let go and try again from its new pose."""
    for _ in range(tries):
        st = o.state()
        b, yaw = _v(st, f"{name}_pos"), float(st[f"{name}_yaw_deg"])
        grasp_yaw = (
            _wrap(math.radians(_yaw_near(yaw + 90.0, st["hand_yaw_deg"]))) * 180 / math.pi
        )  # fingers across the width
        o.skill("move_to", b[0], b[1], b[2] + 0.12, grasp_yaw, -1)
        o.skill("move_to", b[0], b[1], b[2] + 0.12, grasp_yaw, -1)  # settle the turn before going down
        st = o.state()
        b = _v(st, f"{name}_pos")
        o.skill("move_to", b[0], b[1], b[2] + 0.05, 999, -1)
        o.skill("move_to", b[0], b[1], b[2] + 0.01, 999, -1)
        o.skill("grip", 1, 30)
        o.skill("move_to", b[0], b[1], b[2] + 0.06, 999, 1)
        st = o.state()
        if st[f"{name}_pos"][2] > b[2] + 0.03:
            return st
        o.skill("grip", -1, 15)
        hp = _v(st, "hand_pos")
        o.skill("move_to", hp[0], hp[1], 0.15, 999, -1)
    return o.state()


def jaco_solution(env: str):
    task = TASKS[env]["task"]

    def solve(o):
        if task.startswith("reach_site"):
            st = o.state()
            o.skill("move_to", *st["target_pos"])
            for _ in range(5):
                o.skill("grip", 0, 20)
        elif task.startswith("lift"):
            st = _jaco_pick(o, "brick")
            b = _v(st, "brick_pos")
            o.skill("move_to", b[0], b[1], st["target_height"] + 0.1, 999, 1)
            for _ in range(5):
                o.skill("grip", 1, 20)
        elif task.startswith("place"):
            st = _jaco_pick(o, "brick")
            t = _v(st, "target_pos")
            hp = _v(st, "hand_pos")
            o.skill("move_to", hp[0], hp[1], t[2] + 0.12, 999, 1)
            o.skill("move_to", t[0], t[1], t[2] + 0.12, 999, 1)
            o.skill("move_to", t[0], t[1], t[2] + 0.12, 999, 1)
            st = o.state()
            off = _v(st, "hand_pos") - _v(st, "brick_pos")  # the brick hangs below the pinch point
            o.skill("move_to", t[0] + off[0], t[1] + off[1], t[2] + off[2] + 0.03, 999, 1)
            o.skill("grip", -1, 20)
            o.skill("move_to", t[0], t[1], t[2] + 0.3, 999, -1)
            for _ in range(5):
                o.skill("grip", 0, 20)
        elif task.startswith("stack"):
            st = o.state()
            order = st["desired_order_bottom_to_top"]
            bottom, top = f"brick_{order[0]}", f"brick_{order[1]}"
            st = _jaco_pick(o, top)
            hp = _v(st, "hand_pos")
            o.skill("move_to", hp[0], hp[1], 0.15, 999, 1)
            st = o.state()
            rel = float(st[f"{top}_yaw_deg"]) - float(st["hand_yaw_deg"])  # the brick's yaw in the hand
            want_hand = _yaw_near(float(st[f"{bottom}_yaw_deg"]) - rel, st["hand_yaw_deg"])
            bb = _v(st, f"{bottom}_pos")
            o.skill("move_to", bb[0], bb[1], 0.15, want_hand, 1)
            for _ in range(3):  # correct the offset between the pinch point and the carried brick
                st = o.state()
                off = _v(st, "hand_pos") - _v(st, f"{top}_pos")
                o.skill("move_to", bb[0] + off[0], bb[1] + off[1], 0.08, want_hand, 1)
            o.skill("move_to", bb[0] + off[0], bb[1] + off[1], off[2] + 0.025, 999, 1)
            o.skill("move_to", bb[0] + off[0], bb[1] + off[1], off[2] + 0.0, 999, 1)
            o.skill("grip", -1, 20)
            st = o.state()
            hp = _v(st, "hand_pos")
            o.skill("move_to", hp[0], hp[1], hp[2] + 0.08, 999, -1)
            o.skill("grip", 1, 25)  # close the empty hand, then press the brick's studs into the holes from above
            tb = _v(o.state(), f"{top}_pos")
            o.skill("move_to", tb[0], tb[1], tb[2] + 0.06, 999, 1)
            o.act([0.0, 0.0, -2.0, 0.0, 1.0], 10)
            st = o.state()
            hp = _v(st, "hand_pos")
            o.skill("move_to", hp[0], hp[1], 0.25, 999, -1)
            for _ in range(5):
                o.skill("grip", 0, 20)

    return solve


def _run_policy(o, ctl, n: int) -> None:
    st = o.state()
    for _ in range(n):
        st = o.act(ctl.act(st))["state"]


def solution(env: str):
    t = TASKS[env]
    dom = t.get("domain", "jaco")
    if t["kind"] == "manipulation":
        return jaco_solution(env)
    if dom == "cartpole":
        return lambda o: _loop(o, cartpole_policy, t["max_steps"])
    if dom == "pendulum":
        return lambda o: _loop(o, pendulum_policy, t["max_steps"])
    if dom == "reacher":
        return lambda o: _loop(o, reacher_policy, t["max_steps"])
    if dom == "acrobot":
        return lambda o: _run_policy(o, AcrobotOracle(), t["max_steps"])
    if dom == "ball_in_cup":
        return lambda o: _run_policy(o, BallInCupOracle(), t["max_steps"])
    if dom == "manipulator":

        def solve(o):
            ctl = ManipulatorOracle()
            st = o.state()
            for _ in range(t["max_steps"]):
                st = o.act(ctl.act(st))["state"]

        return solve
    if dom == "finger":
        return lambda o: _loop(o, finger_policy, t["max_steps"])
    if dom == "point_mass":
        return lambda o: _loop(o, point_mass_policy, t["max_steps"])
    raise KeyError(env)


def oracle_main(env: str) -> None:
    Oracle().run(solution(env))
