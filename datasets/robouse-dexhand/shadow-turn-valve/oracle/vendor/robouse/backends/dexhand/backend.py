"""Dexhand backend: a dexterous robot hand hanging from a 6-DoF positioner above a table. Two hands from MuJoCo
Menagerie, both attached unchanged (kinematics, position actuators, gains, contact and friction settings as shipped):
  - Shadow Dexterous Hand E3M5, right (`shadow_hand`, Apache-2.0): 24 joints, 20 position actuators (2 wrist, 5 thumb,
    3 each for index/middle/ring with the two distal joints coupled by a tendon, 4 for the little finger);
  - LEAP Hand, right (`leap_hand`, MIT): 16 joints and 16 position actuators (index/middle/ring: mcp, rot, pip, dip;
    thumb: cmc, axl, mcp, ipl).

Mounting (sim/scene.py): like a lab setup where the hand is carried by an arm or gantry, the hand is bolted to
the flange of a positioner with three prismatic joints (x, y, z) and three revolute joints (yaw, pitch, roll). The
positioner joints are driven by force/torque motors (limits 200-300 N, 40 N m) under a PD law with gravity
feed-forward (gains from the joint-space inertia: 40 rad/s, damping ratio 0.9), updated every 2 ms physics step.

Physics: 2 ms steps, implicitfast integrator, elliptic friction cones, the hand's own upstream impratio (Shadow 10,
LEAP 100). Objects are free bodies with raised friction (1.2, rubber-like); grasps are friction grasps only.

Action (one 40 ms step):
  wrist.pose_delta [DX, DY, DZ, DROLL, DPITCH, DYAW] in [-1, 1]: x 1 cm (world axes) and x 4 deg (positioner joints),
    added to the positioner's target; the target never leads the measured pose by more than 3 cm / 15 deg.
  hand.joints [one value per hand actuator] in [-1, 1]: added to the actuator's position target, x 10 % of its range.
Zero means hold (targets unchanged). Skills: move_to, move_by, rotate, turn_about, grasp, open, pose, set_joint,
wait.
"""

from __future__ import annotations

import math

import numpy as np

from ..embodied import ActionGroup, Budget, EmbodiedBackend, Embodiment, Oracle, Sensor, Skill, SkillArg, r3, wrap
from .sim.scene import CONTAINER_BOTTOM, CUBE_FACES, HANDS, POSITIONER, TABLE_C, TABLE_HALF, TABLE_Z, build

PHYS_DT, CONTROL_DT = 0.002, 0.04
N_SUB = round(CONTROL_DT / PHYS_DT)
STEP_XYZ, STEP_ROT = 0.01, math.radians(4)
HAND_STEP = 0.10  # fraction of an actuator's ctrlrange per step at |a| = 1
LEASH_XYZ, LEASH_ROT = 0.03, math.radians(15)
WN, ZETA = 40.0, 0.9  # positioner PD bandwidth (rad/s) and damping ratio

# ------------------------------------------------------------------------------------------------------------------
# hand postures (actuator targets in rad; Shadow's FFJ0/MFJ0/RFJ0/LFJ0 drive the coupled distal pair, J1 + J2)
# ------------------------------------------------------------------------------------------------------------------

SHADOW_ACTS = [
    "WRJ2",
    "WRJ1",
    "THJ5",
    "THJ4",
    "THJ3",
    "THJ2",
    "THJ1",
    "FFJ4",
    "FFJ3",
    "FFJ0",
    "MFJ4",
    "MFJ3",
    "MFJ0",
    "RFJ4",
    "RFJ3",
    "RFJ0",
    "LFJ5",
    "LFJ4",
    "LFJ3",
    "LFJ0",
]
LEAP_ACTS = [
    "if_mcp",
    "if_rot",
    "if_pip",
    "if_dip",
    "mf_mcp",
    "mf_rot",
    "mf_pip",
    "mf_dip",
    "rf_mcp",
    "rf_rot",
    "rf_pip",
    "rf_dip",
    "th_cmc",
    "th_axl",
    "th_mcp",
    "th_ipl",
]


def _shadow(**kw) -> dict:
    base = {k: 0.0 for k in SHADOW_ACTS}
    base.update(kw)
    return base


def _leap(**kw) -> dict:
    base = {k: 0.0 for k in LEAP_ACTS}
    base.update(kw)
    return base


POSTURES = {
    "shadow": {
        "open": _shadow(THJ5=0.3, THJ4=0.9, THJ2=-0.3),
        "flat": _shadow(),
        "relaxed": _shadow(
            THJ5=0.3, THJ4=0.8, FFJ3=0.3, FFJ0=0.4, MFJ3=0.3, MFJ0=0.4, RFJ3=0.3, RFJ0=0.4, LFJ3=0.3, LFJ0=0.4
        ),
        "point": _shadow(
            THJ5=0.2, THJ4=1.1, THJ2=0.4, THJ1=0.8, MFJ3=1.5, MFJ0=2.6, RFJ3=1.5, RFJ0=2.6, LFJ3=1.5, LFJ0=2.6
        ),
        "fist": _shadow(
            THJ5=0.3,
            THJ4=1.2,
            THJ2=0.3,
            THJ1=0.9,
            FFJ3=1.5,
            FFJ0=2.8,
            MFJ3=1.5,
            MFJ0=2.8,
            RFJ3=1.5,
            RFJ0=2.8,
            LFJ3=1.5,
            LFJ0=2.8,
        ),
        "power_ready": _shadow(
            THJ5=0.5,
            THJ4=1.2,
            THJ3=0.1,
            THJ2=-0.3,
            FFJ3=0.35,
            FFJ0=0.3,
            MFJ3=0.35,
            MFJ0=0.3,
            RFJ3=0.35,
            RFJ0=0.3,
            LFJ3=0.35,
            LFJ0=0.3,
        ),
        "pinch_ready": _shadow(
            THJ5=0.6,
            THJ4=0.95,
            THJ3=0.16,
            THJ2=0.2,
            FFJ4=-0.3,
            FFJ3=0.6,
            FFJ0=0.5,
            MFJ3=1.3,
            MFJ0=2.0,
            RFJ3=1.3,
            RFJ0=2.0,
            LFJ3=1.3,
            LFJ0=2.0,
        ),
        "tripod_ready": _shadow(
            THJ5=0.6,
            THJ4=1.2,
            THJ3=0.2,
            THJ2=0.1,
            FFJ3=0.3,
            FFJ0=0.4,
            MFJ3=0.3,
            MFJ0=0.4,
            RFJ3=1.2,
            RFJ0=2.0,
            LFJ3=1.2,
            LFJ0=2.0,
        ),
    },
    "leap": {
        "open": _leap(th_cmc=0.9, th_axl=0.4),
        "flat": _leap(),
        "relaxed": _leap(
            if_mcp=0.3, if_pip=0.3, mf_mcp=0.3, mf_pip=0.3, rf_mcp=0.3, rf_pip=0.3, th_cmc=0.9, th_axl=0.4, th_mcp=0.2
        ),
        "point": _leap(
            mf_mcp=1.6, mf_pip=1.6, mf_dip=1.2, rf_mcp=1.6, rf_pip=1.6, rf_dip=1.2, th_cmc=0.3, th_mcp=0.6, th_ipl=0.6
        ),
        "fist": _leap(
            if_mcp=1.6,
            if_pip=1.6,
            if_dip=1.2,
            mf_mcp=1.6,
            mf_pip=1.6,
            mf_dip=1.2,
            rf_mcp=1.6,
            rf_pip=1.6,
            rf_dip=1.2,
            th_cmc=1.4,
            th_axl=0.6,
            th_mcp=0.8,
            th_ipl=0.6,
        ),
        "power_ready": _leap(if_mcp=0.3, mf_mcp=0.3, rf_mcp=0.3, th_cmc=1.5, th_axl=0.3),
        "pinch_ready": _leap(
            if_mcp=0.3,
            if_pip=0.3,
            mf_mcp=1.6,
            mf_pip=1.6,
            mf_dip=1.2,
            rf_mcp=1.6,
            rf_pip=1.6,
            rf_dip=1.2,
            th_cmc=1.5,
            th_axl=0.3,
        ),
        "tripod_ready": _leap(
            if_mcp=0.3, if_pip=0.3, mf_mcp=0.3, mf_pip=0.3, rf_mcp=1.6, rf_pip=1.6, rf_dip=1.2, th_cmc=1.5, th_axl=0.3
        ),
    },
}

# grasp types: which fingers close, and the fully closed targets they are driven toward (contact stops them; the
# remaining position error is the squeeze, limited by each actuator's force range)
GRASPS = {
    "shadow": {
        "power": _shadow(
            THJ5=0.6,
            THJ4=1.22,
            THJ3=0.2,
            THJ2=0.5,
            THJ1=1.2,
            FFJ3=1.5,
            FFJ0=2.4,
            MFJ3=1.5,
            MFJ0=2.4,
            RFJ3=1.5,
            RFJ0=2.4,
            LFJ3=1.5,
            LFJ0=2.4,
            LFJ5=0.4,
        ),
        "pinch": _shadow(
            THJ5=0.6,
            THJ4=1.05,
            THJ3=0.16,
            THJ2=0.6,
            THJ1=0.2,
            FFJ4=-0.3,
            FFJ3=1.35,
            FFJ0=1.6,
            MFJ3=1.3,
            MFJ0=2.0,
            RFJ3=1.3,
            RFJ0=2.0,
            LFJ3=1.3,
            LFJ0=2.0,
        ),
        "tripod": _shadow(
            THJ5=0.8,
            THJ4=1.22,
            THJ3=0.2,
            THJ2=0.6,
            THJ1=0.9,
            FFJ3=1.1,
            FFJ0=1.4,
            MFJ3=1.1,
            MFJ0=1.4,
            RFJ3=1.2,
            RFJ0=2.0,
            LFJ3=1.2,
            LFJ0=2.0,
        ),
    },
    "leap": {
        "power": _leap(
            if_mcp=1.6,
            if_pip=1.2,
            if_dip=1.0,
            mf_mcp=1.6,
            mf_pip=1.2,
            mf_dip=1.0,
            rf_mcp=1.6,
            rf_pip=1.2,
            rf_dip=1.0,
            th_cmc=1.6,
            th_axl=0.6,
            th_mcp=1.0,
            th_ipl=0.8,
        ),
        "pinch": _leap(
            if_mcp=1.2,
            if_pip=0.8,
            if_dip=0.6,
            mf_mcp=1.6,
            mf_pip=1.6,
            mf_dip=1.2,
            rf_mcp=1.6,
            rf_pip=1.6,
            rf_dip=1.2,
            th_cmc=1.6,
            th_axl=0.8,
            th_mcp=1.0,
            th_ipl=0.8,
        ),
        "tripod": _leap(
            if_mcp=1.2,
            if_pip=0.8,
            if_dip=0.6,
            mf_mcp=1.2,
            mf_pip=0.8,
            mf_dip=0.6,
            rf_mcp=1.6,
            rf_pip=1.6,
            rf_dip=1.2,
            th_cmc=1.6,
            th_axl=0.8,
            th_mcp=1.0,
            th_ipl=0.8,
        ),
    },
}
GRASP_FINGERS = {
    "power": ["thumb", "index", "middle", "ring", "little"],
    "pinch": ["thumb", "index"],
    "tripod": ["thumb", "index", "middle"],
}

# ------------------------------------------------------------------------------------------------------------------
# tasks
# ------------------------------------------------------------------------------------------------------------------

Z = TABLE_Z
TASKS: dict[str, dict] = {}


def _task(tid: str, **kw) -> None:
    TASKS[tid] = kw


_task(
    "shadow-cube-face-up",
    hand="shadow",
    kind="face_up",
    max_steps=1200,
    face="red",
    tol_deg=20,
    objects=[dict(kind="cube", name="cube", pos=(0.14, 0.0), half=0.028, mass=0.09)],
    start=dict(flange=(-0.14, 0.0, 0.72), rpy=(0, 0, 0), posture="open"),
)
_task(
    "shadow-pour-balls",
    hand="shadow",
    kind="pour",
    max_steps=1200,
    need=4,
    objects=[
        dict(kind="cup", name="cup", pos=(0.14, -0.10), r_in=0.028, height=0.11, wall=0.004, mass=0.08),
        dict(kind="balls", pos=(0.14, -0.10), n=6, r=0.009, ring=0.016, z0=Z + 0.016, mass=0.006),
        dict(
            kind="container",
            name="bowl",
            pos=(0.14, 0.14),
            r_in=0.095,
            height=0.045,
            wall=0.006,
            n=24,
            rgba=[0.9, 0.9, 0.85, 1],
        ),
    ],
    start=dict(flange=(-0.14, 0.0, 0.72), rpy=(0, 0, 0), posture="open"),
)
_task(
    "leap-pen-to-holder",
    hand="leap",
    kind="pen",
    max_steps=1000,
    objects=[
        dict(
            kind="container",
            name="stand",
            pos=(0.14, -0.12),
            r_in=0.012,
            height=0.06,
            wall=0.012,
            n=12,
            rgba=[0.3, 0.5, 0.75, 1],
        ),
        dict(
            kind="container",
            name="holder",
            pos=(0.14, 0.14),
            r_in=0.03,
            height=0.08,
            wall=0.012,
            n=16,
            rgba=[0.35, 0.65, 0.35, 1],
        ),
        dict(
            kind="pen", name="marker", pos=(0.14, -0.12, Z + 0.015 + 0.075 + 0.008), r=0.008, half_len=0.075, mass=0.02
        ),
    ],
    start=dict(flange=(-0.05, 0.0, 0.72), rpy=(0, 0, 0), posture="open"),
)
_task(
    "leap-lift-ball",
    hand="leap",
    kind="lift",
    max_steps=700,
    lift=0.15,
    objects=[
        dict(kind="stand", name="stand", pos=(0.12, 0.04), r=0.022),
        dict(kind="ball", name="ball", pos=(0.12, 0.04), r=0.033, mass=0.06, rgba=[0.8, 0.9, 0.2, 1]),
    ],
    start=dict(flange=(-0.05, 0.0, 0.70), rpy=(0, 0, 0), posture="open"),
)
_task(
    "shadow-turn-valve",
    hand="shadow",
    kind="valve",
    max_steps=1000,
    turn_deg=180,
    objects=[dict(kind="valve", name="valve", pos=(0.16, 0.0), height=0.12, r=0.075, damping=0.05, frictionloss=0.15)],
    start=dict(flange=(-0.14, 0.0, 0.72), rpy=(0, 0, 0), posture="open"),
)
_task(
    "leap-press-buttons",
    hand="leap",
    kind="buttons",
    max_steps=800,
    sequence=["green", "blue", "green"],
    objects=[
        dict(
            kind="buttons",
            pos=(0.16, 0.0),
            panel_h=0.06,
            colors=[("red", [0.85, 0.12, 0.12, 1]), ("green", [0.15, 0.7, 0.2, 1]), ("blue", [0.15, 0.3, 0.9, 1])],
            spacing=0.07,
            r=0.016,
            travel=0.012,
            stiffness=150.0,
        )
    ],
    start=dict(flange=(-0.05, 0.0, 0.72), rpy=(0, 0, 0), posture="open"),
)
_task(
    "leap-cube-in-hand",
    hand="leap",
    kind="inhand",
    max_steps=600,
    face="red",
    tol_deg=25,
    wrist_tol_deg=15,
    objects=[dict(kind="cube", name="cube", pos=(0.0, 0.0, 1.5), half=0.03, mass=0.07)],
    start=dict(flange=(0.05, 0.0, 0.60), rpy=(180, 0, 0), posture="flat", cube_on_palm=(0.12, 0.0)),
)

BUTTON_CLICK = 0.008  # m of travel that counts as a click; released below half of it


def _R(rpy) -> np.ndarray:
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


class DexHandBackend(EmbodiedBackend):
    name = "dexhand"

    def __init__(self, spec: dict):
        super().__init__(spec)
        env = spec.get("env") or spec.get("id")
        if env not in TASKS:
            raise KeyError(f"unknown dexhand task {env!r}")
        self.task_id, self.task = env, TASKS[env]
        self.hand = self.task["hand"]
        self.H = HANDS[self.hand]
        self.act_names = SHADOW_ACTS if self.hand == "shadow" else LEAP_ACTS
        self.camera = spec.get("camera", "front")
        self.decl = self._declare()

    # ---- declaration -------------------------------------------------------------------------------------------
    def _declare(self) -> Embodiment:
        n = len(self.act_names)
        sensors = [
            Sensor(
                "hand_state",
                "proprio",
                ["hand"],
                "wrist pose and target, palm point, fingertips, joint positions and "
                "targets (normalised), per-finger contacts",
                units="m, deg, normalised [-1, 1]",
                frame="world",
            ),
            Sensor(
                "scene",
                "world",
                ["objects", "table", "landmarks"],
                "object poses and public fixture positions",
                units="m",
                frame="world",
            ),
            Sensor("progress", "events", ["progress", "safety_events"], "task counters and drop/knock-off events"),
            *[Sensor(f"camera:{c}", "camera", [], "", mount="world") for c in self._cameras()],
        ]
        groups = [
            ActionGroup(
                "wrist.pose_delta",
                "ee_delta_pose",
                ["DX", "DY", "DZ", "DROLL", "DPITCH", "DYAW"],
                [-1] * 6,
                [1] * 6,
                "x 1 cm (world x, y, z), x 4 deg (positioner roll, pitch, yaw)",
                "added to the positioner target; zero = hold",
                frame="xyz world; rpy positioner joints",
            ),
            ActionGroup(
                "hand.joints",
                "joint_delta",
                list(self.act_names),
                [-1] * n,
                [1] * n,
                "x 10 % of each actuator's range",
                "added to the hand's position targets; zero = hold",
            ),
        ]
        xyz = [SkillArg("x", unit="m"), SkillArg("y", unit="m"), SkillArg("z", unit="m")]
        skills = [
            Skill(
                "move_to",
                xyz + [SkillArg("speed", unit="m/s", default=0.15)],
                "move the palm point (hand.palm_pos) in a straight line to (X, Y, Z) keeping the orientation (SPEED clamped to 0.01-0.25 m/s); "
                "stops when there or blocked, or after 400 steps (then it reports `remaining_m`)",
                420,
            ),
            Skill(
                "move_by",
                [
                    SkillArg("dx", unit="m"),
                    SkillArg("dy", unit="m"),
                    SkillArg("dz", unit="m"),
                    SkillArg("speed", unit="m/s", default=0.15),
                ],
                "move the palm point by (DX, DY, DZ) (SPEED clamped to 0.01-0.25 m/s); stops when there or blocked, or after 400 "
                "steps (then it reports `remaining_m`)",
                420,
            ),
            Skill(
                "rotate",
                [
                    SkillArg("roll", unit="deg"),
                    SkillArg("pitch", unit="deg"),
                    SkillArg("yaw", unit="deg"),
                    SkillArg("speed", unit="deg/s", default=60.0),
                ],
                "turn the hand to absolute positioner angles ROLL PITCH YAW about the palm point (the palm point stays put; SPEED "
                "clamped to 5-100 deg/s)",
                620,
            ),
            Skill(
                "turn_about",
                [
                    SkillArg("x", unit="m"),
                    SkillArg("y", unit="m"),
                    SkillArg("deg", unit="deg"),
                    SkillArg("speed", unit="deg/s", default=45.0),
                ],
                "swing the hand DEG degrees (counter-clockwise seen from above, + ) about the vertical line through (X, Y), yawing with it, "
                "like turning a wheel (SPEED clamped to 5-90 deg/s); whatever the fingers hold or push is carried only by contact; stops "
                "after 580 steps and then reports `remaining_deg`",
                600,
            ),
            Skill(
                "grasp",
                [SkillArg("type", "enum", choices=["power", "pinch", "tripod"])],
                "close the fingers of that grasp until they stall on something or are fully closed; reports which fingers touch what",
                40,
            ),
            Skill("open", [], "open the hand gently (posture `open`)", 40),
            Skill(
                "pose",
                [SkillArg("name", "enum", choices=list(POSTURES[self.hand]))],
                "move all hand joints to a named posture",
                40,
            ),
            Skill(
                "set_joint",
                [SkillArg("name", "enum", choices=list(self.act_names)), SkillArg("value", unit="deg")],
                "set one hand actuator's target (degrees; for Shadow J0 actuators the sum of the two distal joints)",
                30,
            ),
            Skill("wait", [SkillArg("seconds", unit="s", default=0.5)], "hold everything still", 100),
        ]
        return Embodiment(
            robot=self.H["robot"],
            family="dexterous_hand",
            assets=[self.H["folder"]],
            sensors=sensors,
            action_groups=groups,
            skills=skills,
            budget=Budget(self.task["max_steps"], CONTROL_DT),
            cameras=self._cameras(),
        )

    def _cameras(self) -> list[str]:
        return ["front", "side", "top", "back"]

    # ---- episode -----------------------------------------------------------------------------------------------
    def reset(self, seed: int) -> None:
        import mujoco

        self._reset_renderers()
        self.model = m = build(self.hand, self.task)
        self.data = d = mujoco.MjData(m)
        self.pj = [m.joint(j[0]).id for j in POSITIONER]
        self.pq = np.array([m.jnt_qposadr[j] for j in self.pj])
        self.pv = np.array([m.jnt_dofadr[j] for j in self.pj])
        self.pa = np.array([m.actuator(f"{j[0]}_motor").id for j in POSITIONER])
        self.plo = np.array([m.jnt_range[j][0] for j in self.pj])
        self.phi = np.array([m.jnt_range[j][1] for j in self.pj])
        pre, suf = self.H["act_prefix"], self.H.get("act_suffix", "")
        self.ha = np.array([m.actuator(f"h/{pre}{a}{suf}").id for a in self.act_names])
        self.hlo = m.actuator_ctrlrange[self.ha, 0].copy()
        self.hhi = m.actuator_ctrlrange[self.ha, 1].copy()
        st = self.task["start"]
        q0 = np.array([*st["flange"], *[math.radians(v) for v in st["rpy"][::1]]])
        q0 = np.array([q0[0], q0[1], q0[2], q0[5], q0[4], q0[3]])  # joint order: x y z yaw pitch roll
        d.qpos[self.pq] = q0
        self.pt = q0.copy()
        self.ht = self._posture_vec(POSTURES[self.hand][st["posture"]])
        for i, a in enumerate(self.ha):  # start the hand joints at their targets
            jid = m.actuator_trnid[a, 0]
            if m.actuator_trntype[a] == mujoco.mjtTrn.mjTRN_JOINT:
                d.qpos[m.jnt_qposadr[jid]] = self.ht[i]
        d.ctrl[self.ha] = self.ht
        self.site = m.site("h/grasp_site").id
        self.flange = m.body("flange").id
        mujoco.mj_forward(m, d)
        # geometry bookkeeping
        self.hand_geoms: dict[int, str] = {}
        for finger, bodies in self.H["fingers"].items():
            for bn in bodies:
                bid = m.body("h/" + bn).id
                for g in range(m.ngeom):
                    if m.geom_bodyid[g] == bid:
                        self.hand_geoms[g] = finger
        self.tip_geom = {}
        for finger, bn in self.H["tips"].items():
            bid = m.body("h/" + bn).id
            gs = [g for g in range(m.ngeom) if m.geom_bodyid[g] == bid and m.geom_contype[g]]
            self.tip_geom[finger] = gs[-1]
        self.fingers = list(self.H["tips"])
        self.free = {}  # name -> (body id, qpos adr, dof adr)
        for b in range(m.nbody):
            j = m.body_jntadr[b]
            if j >= 0 and m.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE:
                self.free[m.body(b).name] = (b, int(m.jnt_qposadr[j]), int(m.jnt_dofadr[j]))
        self.gowner = []
        for g in range(m.ngeom):
            if g in self.hand_geoms:
                self.gowner.append("hand")
                continue
            bname = m.body(m.geom_bodyid[g]).name
            gname = m.geom(g).name
            if bname in self.free or bname.startswith("button_") or bname == "valve":
                self.gowner.append(bname)
            elif gname.startswith("table"):
                self.gowner.append("table")
            elif gname == "floor":
                self.gowner.append("floor")
            else:
                self.gowner.append(gname.split("_")[0] if "_" in gname else gname)
        if st.get("cube_on_palm"):
            self._place_on_palm("cube", st["cube_on_palm"])
        # PD gains from the joint-space inertia at the start pose
        M = np.zeros((m.nv, m.nv))
        mujoco.mj_fullM(m, M, d.qM)
        self.inertia = np.array([M[v, v] for v in self.pv])
        self.kp = self.inertia * WN**2
        self.kd = 2 * ZETA * WN * self.inertia
        self.events, self._event_keys = [], set()
        self.lifted: set = set()
        self.clicks: list[str] = []
        self._pressed = {}
        self._inhand = False  # enabled after the settle, once the start pose is recorded
        # settle
        for _ in range(15):
            self.control_step(np.zeros(6 + len(self.ha)))
        self.valve0 = self._valve_angle() if "valve" in self.free or self._has_joint("valve") else 0.0
        if self.task["kind"] == "inhand":
            self.wrist0 = self._pq()[3:].copy()
            self._inhand = True
        self.events, self._event_keys = [], set()
        self.clicks = []

    def _has_joint(self, name: str) -> bool:
        try:
            self.model.joint(name)
            return True
        except KeyError:
            return False

    def _place_on_palm(self, name: str, xy) -> None:
        import mujoco

        b, qa, _ = self.free[name]
        h = float(self.model.geom_size[self.model.body_geomadr[b]][0])
        pos = np.array([xy[0], xy[1], self.data.site_xpos[self.site][2] + h - 0.01])
        self.data.qpos[qa : qa + 3] = pos
        self.data.qpos[qa + 3 : qa + 7] = [1, 0, 0, 0]
        mujoco.mj_forward(self.model, self.data)

    def _posture_vec(self, post: dict) -> np.ndarray:
        return np.clip(np.array([post[a] for a in self.act_names]), self.hlo, self.hhi)

    # ---- state helpers -----------------------------------------------------------------------------------------
    def _pq(self) -> np.ndarray:
        return self.data.qpos[self.pq].copy()

    def _rpy_of(self, q) -> tuple[float, float, float]:
        return float(q[5]), float(q[4]), float(q[3])

    def _flange_R(self) -> np.ndarray:
        return self.data.xmat[self.flange].reshape(3, 3).copy()

    def palm_pos(self) -> np.ndarray:
        return self.data.site_xpos[self.site].copy()

    def _palm_normal(self) -> np.ndarray:
        return self._flange_R() @ np.array([0, 0, -1.0])

    def _finger_dir(self) -> np.ndarray:
        return self._flange_R() @ np.array([1.0, 0, 0])

    def _palm_offset(self) -> np.ndarray:
        """Palm point in the flange frame (constant for a rigid hand)."""
        return self._flange_R().T @ (self.palm_pos() - self.data.xpos[self.flange])

    def _obj_pos(self, name) -> np.ndarray:
        return self.data.xpos[self.free[name][0]].copy()

    def _obj_R(self, name) -> np.ndarray:
        return self.data.xmat[self.free[name][0]].reshape(3, 3).copy()

    def _obj_vel(self, name) -> np.ndarray:
        v = self.free[name][2]
        return self.data.qvel[v : v + 3].copy()

    def _wrist_dev(self) -> float:
        """Largest change of the positioner's yaw, pitch or roll since the start (in-hand task)."""
        return float(np.max(np.abs(self._pq()[3:] - self.wrist0)))

    def _valve_angle(self) -> float:
        j = self.model.joint("valve").id
        return float(self.data.qpos[self.model.jnt_qposadr[j]])

    def _contacts(self) -> tuple[dict, set]:
        """Per finger, the set of things it touches; and the set of (thing, thing) pairs among non-hand geoms."""
        d = self.data
        per: dict[str, set] = {}
        pairs: set = set()
        for c in d.contact[: d.ncon]:
            a, b = self.gowner[c.geom1], self.gowner[c.geom2]
            if a == "hand" and b != "hand":
                per.setdefault(self.hand_geoms[c.geom1], set()).add(b)
            elif b == "hand" and a != "hand":
                per.setdefault(self.hand_geoms[c.geom2], set()).add(a)
            elif a != "hand" and b != "hand":
                pairs.add((a, b))
                pairs.add((b, a))
        return per, pairs

    def _touching(self, name: str, other: str) -> bool:
        _, pairs = self._contacts()
        return (name, other) in pairs

    def _hand_touches(self, name: str) -> list[str]:
        per, _ = self._contacts()
        return sorted(f for f, s in per.items() if name in s)

    def _joint_norm(self) -> np.ndarray:
        L = self.data.actuator_length[self.ha]
        return 2 * (L - self.hlo) / (self.hhi - self.hlo) - 1

    # ---- control -----------------------------------------------------------------------------------------------
    def control_step(self, a: np.ndarray) -> None:
        import mujoco

        a = np.asarray(a, float)
        q = self._pq()
        dp = a[:3] * STEP_XYZ
        drot = np.array([a[5], a[4], a[3]]) * STEP_ROT  # joint order yaw, pitch, roll
        t = self.pt + np.concatenate([dp, drot])
        t = np.clip(t, self.plo, self.phi)
        t[:3] = np.clip(t[:3], q[:3] - LEASH_XYZ, q[:3] + LEASH_XYZ)
        t[3:] = np.clip(t[3:], q[3:] - LEASH_ROT, q[3:] + LEASH_ROT)
        self.pt = t
        self.ht = np.clip(self.ht + a[6:] * HAND_STEP * (self.hhi - self.hlo), self.hlo, self.hhi)
        m, d = self.model, self.data
        d.ctrl[self.ha] = self.ht
        lim = m.actuator_ctrlrange[self.pa, 1]
        for _ in range(N_SUB):
            q = d.qpos[self.pq]
            qd = d.qvel[self.pv]
            u = self.kp * (self.pt - q) - self.kd * qd + d.qfrc_bias[self.pv]
            d.ctrl[self.pa] = np.clip(u, -lim, lim)
            mujoco.mj_step(m, d)
            if self._inhand:
                self._inhand_check()
        self._monitor()

    def _inhand_check(self) -> None:
        """In-hand task, every physics step: the cube must never touch the table and the wrist must never rotate more
        than the tolerance away from its start."""
        d = self.data
        for c in d.contact[: d.ncon]:
            if {self.gowner[c.geom1], self.gowner[c.geom2]} == {"cube", "table"}:
                self.event("table:cube", "dropped", "the cube touched the table")
                break
        if self._wrist_dev() > math.radians(self.task["wrist_tol_deg"]):
            self.event(
                "wrist:tilt", "rule", f"the wrist rotated more than {self.task['wrist_tol_deg']} degrees from its start"
            )

    # ---- monitor -----------------------------------------------------------------------------------------------
    def _monitor(self) -> None:
        per, pairs = self._contacts()
        held = {o for s in per.values() for o in s}
        for name, (b, _qa, _v) in self.free.items():
            if name.startswith("ball") and name != "ball":
                continue  # the small balls in the pouring task may spill; they are not tracked
            p = self.data.xpos[b]
            if name in held and (name, "table") not in pairs and p[2] > Z + 0.04:
                self.lifted.add(name)
            if p[2] < Z - 0.06 or (name, "floor") in pairs:
                verb = "was dropped and fell to the floor" if name in self.lifted else "was knocked off the table"
                self.event(f"floor:{name}", "dropped", f"the {name} {verb}")
        k = self.task["kind"]
        if k == "buttons":
            for cname, _ in self.task["objects"][0]["colors"]:
                j = self.model.joint(f"button_{cname}").id
                depth = -float(self.data.qpos[self.model.jnt_qposadr[j]])
                down = self._pressed.get(cname, False)
                if not down and depth >= BUTTON_CLICK:
                    self._pressed[cname] = True
                    self.clicks.append(cname)
                elif down and depth < BUTTON_CLICK / 2:
                    self._pressed[cname] = False

    # ---- observation -------------------------------------------------------------------------------------------
    def observe(self) -> dict:
        m, d = self.model, self.data
        q = self._pq()
        per, pairs = self._contacts()
        R = self._flange_R()
        hand = {
            "model": self.H["robot"],
            "wrist_pos": r3(d.xpos[self.flange]),
            "wrist_rpy_deg": [round(math.degrees(v), 1) for v in self._rpy_of(q)],
            "wrist_target_pos": r3(self.pt[:3]),
            "wrist_target_rpy_deg": [round(math.degrees(v), 1) for v in self._rpy_of(self.pt)],
            "palm_pos": r3(self.palm_pos()),
            "palm_normal": r3(R @ np.array([0, 0, -1.0]), 2),
            "finger_dir": r3(R @ np.array([1.0, 0, 0]), 2),
            "fingertips": {f: r3(d.geom_xpos[g]) for f, g in self.tip_geom.items()},
            "joints": list(self.act_names),
            "joint_pos_norm": [round(float(v), 2) for v in self._joint_norm()],
            "joint_target_norm": [round(float(v), 2) for v in 2 * (self.ht - self.hlo) / (self.hhi - self.hlo) - 1],
            "contacts": {f: sorted(s) for f, s in sorted(per.items())},
        }
        objs = {}
        for name, (b, _qa, _v) in self.free.items():
            p = d.xpos[b]
            o = {
                "pos": r3(p),
                "touching": sorted(
                    {y for (x, y) in pairs if x == name} | ({"hand"} if any(name in s for s in per.values()) else set())
                ),
            }
            if name == "cube":
                Rc = self._obj_R(name)
                o["face_up"] = self._face_up(name)
                o["face_normals"] = {f: r3(Rc @ np.array(n, float), 2) for f, n in CUBE_FACES.items()}
                o["size"] = 2 * float(m.geom_size[m.body_geomadr[b]][0])
            elif name in ("cup", "marker"):
                o["tilt_deg"] = round(math.degrees(math.acos(float(np.clip(d.xmat[b].reshape(3, 3)[2, 2], -1, 1)))), 1)
            if name == "marker":
                ax = d.xmat[b].reshape(3, 3)[:, 2]
                hl = float(m.geom_size[m.body_geomadr[b]][1])
                o["cap_end"] = r3(p + ax * hl)
                o["tip_end"] = r3(p - ax * hl)
            objs[name] = o
        st: dict = {
            "hand": hand,
            "objects": objs,
            "time_s": round(float(d.time), 2),
            "table": {
                "top_z": Z,
                "x": [TABLE_C[0] - TABLE_HALF[0], TABLE_C[0] + TABLE_HALF[0]],
                "y": [TABLE_C[1] - TABLE_HALF[1], TABLE_C[1] + TABLE_HALF[1]],
            },
        }
        st["landmarks"] = self._landmarks()
        st["progress"] = self._progress()
        st["safety_events"] = [e["detail"] for e in self.events]
        return st

    def _face_up(self, name: str) -> str:
        Rc = self._obj_R(name)
        return max(CUBE_FACES, key=lambda f: float((Rc @ np.array(CUBE_FACES[f], float))[2]))

    def _landmarks(self) -> dict:
        out = {}
        for ob in self.task["objects"]:
            k = ob["kind"]
            if k == "container":
                out[ob["name"]] = {
                    "center": [*ob["pos"][:2], Z],
                    "inner_radius": ob["r_in"],
                    "rim_z": round(Z + ob["height"], 3),
                    "inside_bottom_z": round(Z + CONTAINER_BOTTOM, 3),
                    "inside_depth": round(ob["height"] - CONTAINER_BOTTOM, 3),
                }
            elif k == "valve":
                out["valve"] = {
                    "axis": [*ob["pos"][:2]],
                    "wheel_z": round(Z + ob["height"], 3),
                    "wheel_radius": ob["r"],
                    "spokes": 3,
                    "angle_deg": round(math.degrees(self._valve_angle()), 1),
                }
            elif k == "buttons":
                bs = {}
                for cname, _ in ob["colors"]:
                    b = self.model.body(f"button_{cname}").id
                    j = self.model.joint(f"button_{cname}").id
                    bs[cname] = {
                        "top_center": r3(self.data.xpos[b] + np.array([0, 0, 0.006])),
                        "pressed_mm": round(-1000 * float(self.data.qpos[self.model.jnt_qposadr[j]]), 1),
                    }
                out["buttons"] = bs
                out["button_radius"] = ob["r"]
            elif k == "stand":
                out["stand"] = {"center": [*ob["pos"][:2], Z]}
        return out

    def _progress(self) -> dict:
        k = self.task["kind"]
        if k == "pour":
            return {"balls_in_bowl": self._balls_in_bowl()}
        if k == "valve":
            return {"valve_turned_deg": round(math.degrees(self._valve_angle() - self.valve0), 1)}
        if k == "buttons":
            return {"clicks": list(self.clicks)}
        if k == "inhand":
            return {"wrist_rotation_since_start_deg": round(math.degrees(self._wrist_dev()), 1)}
        return {}

    def _balls_in_bowl(self) -> int:
        bowl = next(o for o in self.task["objects"] if o.get("name") == "bowl")
        c = np.array(bowl["pos"][:2])
        n = 0
        for name in self.free:
            if name.startswith("ball") and name != "ball":
                p = self._obj_pos(name)
                if np.linalg.norm(p[:2] - c) < bowl["r_in"] and Z < p[2] < Z + bowl["height"]:
                    n += 1
        return n

    # ---- scoring -----------------------------------------------------------------------------------------------
    def success(self) -> bool:
        if self.events:
            return False
        t, k = self.task, self.task["kind"]
        per, pairs = self._contacts()
        if k == "lift":
            p = self._obj_pos("ball")
            return (
                p[2] >= Z + t["lift"]
                and "ball" in {o for s in per.values() for o in s}
                and ("ball", "table") not in pairs
                and float(np.linalg.norm(self._obj_vel("ball"))) < 0.05
            )
        if k == "face_up":
            Rc = self._obj_R("cube")
            n = Rc @ np.array(CUBE_FACES[t["face"]], float)
            return (
                n[2] >= math.cos(math.radians(t["tol_deg"]))
                and ("cube", "table") in pairs
                and not any("cube" in s for s in per.values())
            )
        if k == "pour":
            cupR = self._obj_R("cup")
            return (
                self._balls_in_bowl() >= t["need"]
                and cupR[2, 2] >= math.cos(math.radians(20))
                and ("cup", "table") in pairs
            )
        if k == "pen":
            hb = next(o for o in t["objects"] if o.get("name") == "holder")
            b = self.free["marker"][0]
            p = self.data.xpos[b]
            ax = self.data.xmat[b].reshape(3, 3)[:, 2]
            hl = float(self.model.geom_size[self.model.body_geomadr[b]][1])
            low = p - ax * hl if ax[2] > 0 else p + ax * hl
            inside = np.linalg.norm(low[:2] - np.array(hb["pos"][:2])) < hb["r_in"] and low[2] < Z + hb["height"]
            return bool(inside) and not any("marker" in s for s in per.values())
        if k == "valve":
            return math.degrees(self._valve_angle() - self.valve0) >= t["turn_deg"]
        if k == "buttons":
            return self.clicks == t["sequence"]
        if k == "inhand":
            n = self._obj_R("cube") @ np.array(CUBE_FACES[t["face"]], float)
            return (
                n[2] >= math.cos(math.radians(t["tol_deg"]))
                and "cube" in {o for s_ in per.values() for o in s_}
                and ("cube", "table") not in pairs
                and self._wrist_dev() <= math.radians(t["wrist_tol_deg"])
            )
        return False

    # ---- skills ------------------------------------------------------------------------------------------------
    def _act(self, dpos=None, drot=None, dhand=None) -> list[float]:
        a = np.zeros(6 + len(self.ha))
        if dpos is not None:
            a[:3] = np.clip(np.asarray(dpos) / STEP_XYZ, -1, 1)
        if drot is not None:  # roll, pitch, yaw
            a[3:6] = np.clip(np.asarray(drot) / STEP_ROT, -1, 1)
        if dhand is not None:
            a[6:] = np.clip(np.asarray(dhand) / (HAND_STEP * (self.hhi - self.hlo)), -1, 1)
        return [float(x) for x in a]

    def _report(self, **kw) -> dict:
        per, _ = self._contacts()
        q = self._pq()
        return {
            **kw,
            "palm_pos": r3(self.palm_pos()),
            "wrist_rpy_deg": [round(math.degrees(v), 1) for v in self._rpy_of(q)],
            "contacts": {f: sorted(s) for f, s in sorted(per.items())},
        }

    def _servo(self, goal_fn, n_max: int, speed: float, rot_speed: float, tol: float = 0.004):
        """Drive the positioner target toward goal_fn(k) -> (flange_xyz, rpy) each step; stop when the measured pose is
        there, or when the target is there but the hand has stopped short (blocked)."""
        hist = []
        for k in range(n_max):
            fxyz, rpy = goal_fn(k)
            cur_rpy = np.array(self._rpy_of(self.pt))
            dpos = np.clip(np.asarray(fxyz) - self.pt[:3], -speed, speed)
            drot = np.array([wrap(g - c) for g, c in zip(rpy, cur_rpy, strict=False)])
            drot = np.clip(drot, -rot_speed, rot_speed)
            q = self._pq()
            err = float(np.linalg.norm(np.asarray(fxyz) - q[:3]))
            rerr = max(abs(wrap(g - c)) for g, c in zip(rpy, self._rpy_of(q), strict=False))
            tgt_there = np.linalg.norm(np.asarray(fxyz) - self.pt[:3]) < 1e-4 and np.max(np.abs(drot)) < 1e-4
            if (
                tgt_there
                and err < tol
                and rerr < math.radians(1.5)
                and np.linalg.norm(self.data.qvel[self.pv[:3]]) < 0.02
            ):
                return "reached"
            hist.append(q.copy())
            if len(hist) > 15:
                old = hist[-16]
                if (
                    np.linalg.norm(q[:3] - old[:3]) < 0.002
                    and np.max(np.abs(q[3:] - old[3:])) < math.radians(0.5)
                    and (err > tol or rerr > math.radians(1.5))
                ):
                    return "blocked"
            yield self._act(dpos, drot)
        return "stopped"

    def skill_move_to(self, x: float, y: float, z: float, speed: float):
        goal = np.array([x, y, z])
        v = float(np.clip(speed, 0.01, 0.25)) * CONTROL_DT
        off = self._flange_R() @ self._palm_offset()
        rpy = self._rpy_of(self.pt)
        status = yield from self._servo(lambda k: (goal - off, rpy), 400, v, STEP_ROT)
        rem = goal - self.palm_pos()
        extra = {"remaining_m": r3(rem)} if status == "stopped" else {}
        return self._report(status=status, goal=r3(goal), error_m=round(float(np.linalg.norm(rem)), 3), **extra)

    def skill_move_by(self, dx: float, dy: float, dz: float, speed: float):
        p = self.palm_pos()
        res = yield from self.skill_move_to(p[0] + dx, p[1] + dy, p[2] + dz, speed)
        return res

    def skill_rotate(self, roll: float, pitch: float, yaw: float, speed: float):
        goal = np.radians([roll, pitch, yaw])
        c = self._palm_offset()
        P0 = self.palm_pos()
        w = math.radians(float(np.clip(speed, 5, 100))) * CONTROL_DT
        cur = {"rpy": np.array(self._rpy_of(self.pt))}

        def goal_fn(k):
            # advance an internal orientation setpoint and keep the palm point where it was
            r = cur["rpy"]
            step = np.clip(np.array([wrap(g - v) for g, v in zip(goal, r, strict=False)]), -w, w)
            cur["rpy"] = r + step
            return P0 - _R(cur["rpy"]) @ c, tuple(cur["rpy"])

        status = yield from self._servo(goal_fn, 600, STEP_XYZ, w)
        return self._report(status=status)

    def skill_turn_about(self, x: float, y: float, deg: float, speed: float):
        C = np.array([x, y, 0.0])
        P0 = self.palm_pos()
        c = self._palm_offset()
        rpy0 = np.array(self._rpy_of(self.pt))
        w = math.radians(float(np.clip(speed, 5, 90))) * CONTROL_DT
        total = math.radians(deg)
        n = max(1, int(abs(total) / w))
        state = {"th": 0.0}

        def goal_fn(k):
            th = total * min(1.0, (k + 1) / n)
            state["th"] = th
            ct, sn = math.cos(th), math.sin(th)
            v = P0 - C
            p = C + np.array([ct * v[0] - sn * v[1], sn * v[0] + ct * v[1], v[2]])
            rpy = rpy0 + np.array([0, 0, th])
            return p - _R(rpy) @ c, tuple(rpy)

        status = yield from self._servo(goal_fn, min(n + 60, 580), STEP_XYZ, w * 1.5)
        turned = math.degrees(self._pq()[3] - rpy0[2])
        extra = {"remaining_deg": round(deg - turned, 1)} if status == "stopped" else {}
        return self._report(status=status, turned_deg=round(turned, 1), **extra)

    def _hand_to(self, target: np.ndarray, n_max: int, settle: int = 4, rate: float = 1.0):
        """Ramp the hand targets to `target` (at `rate` x the full per-step change), then let the fingers settle;
        returns the executed step count."""
        k = 0
        lim = rate * HAND_STEP * (self.hhi - self.hlo)
        while k < n_max:
            diff = target - self.ht
            if np.max(np.abs(diff) / (HAND_STEP * (self.hhi - self.hlo))) < 1e-6:
                break
            yield self._act(dhand=np.clip(diff, -lim, lim))
            k += 1
        prev = self.data.actuator_length[self.ha].copy()
        for _ in range(max(0, min(settle + 10, n_max - k))):
            yield self._act()
            k += 1
            cur = self.data.actuator_length[self.ha].copy()
            if np.max(np.abs(cur - prev)) < 2e-3 and k > settle:
                break
            prev = cur
        return k

    def skill_grasp(self, type: str):
        target = self._posture_vec(GRASPS[self.hand][type])
        yield from self._hand_to(target, 40, settle=6)
        per, _ = self._contacts()
        fingers = [f for f in GRASP_FINGERS[type] if f in self.fingers]
        touching = {f: sorted(per.get(f, set())) for f in fingers}
        objs = {o for f in fingers for o in per.get(f, set())} & set(self.free)
        return self._report(
            type=type,
            fingers_touching=touching,
            holding=sorted(o for o in objs if sum(o in per.get(f, set()) for f in per) >= 2),
        )

    def skill_open(self):
        yield from self._hand_to(self._posture_vec(POSTURES[self.hand]["open"]), 40, rate=0.4)
        return self._report(posture="open")

    def skill_pose(self, name: str):
        yield from self._hand_to(self._posture_vec(POSTURES[self.hand][name]), 40)
        return self._report(posture=name)

    def skill_set_joint(self, name: str, value: float):
        i = self.act_names.index(name)
        t = self.ht.copy()
        t[i] = math.radians(value)
        t = np.clip(t, self.hlo, self.hhi)
        yield from self._hand_to(t, 30)
        return self._report(
            joint=name,
            target_deg=round(math.degrees(t[i]), 1),
            actual_deg=round(math.degrees(float(self.data.actuator_length[self.ha[i]])), 1),
        )

    def skill_wait(self, seconds: float):
        for _ in range(max(1, round(seconds / CONTROL_DT))):
            yield self._act()
        return self._report()


# ------------------------------------------------------------------------------------------------------------------
# reference solutions (socket only)
# ------------------------------------------------------------------------------------------------------------------


def _solve_cube_face_up(o: Oracle, t: dict) -> None:
    # Reorient by regrasp: tripod-grasp the cube from behind (the hand yawed so the target face points to the hand's
    # left), lift, roll the forearm +90 deg (that face turns up), set it down, let go, back off. Repeat if needed.
    for _ in range(3):
        st = o.state()
        cube = st["objects"]["cube"]
        n = np.array(cube["face_normals"][t["face"]])
        if n[2] > 0.9:
            return
        yaw = 0.0 if n[2] < -0.7 else math.degrees(wrap(math.atan2(n[1], n[0]) - math.pi / 2))
        d = np.array([math.cos(math.radians(yaw)), math.sin(math.radians(yaw)), 0])
        g = np.array(cube["pos"]) - 0.02 * d
        o.skill("rotate", 0, 30, yaw)
        o.skill("pose", "tripod_ready")
        o.skill("move_to", g[0], g[1], g[2] + 0.12)
        o.skill("move_to", g[0], g[1], g[2] + 0.025)
        o.skill("grasp", "tripod")
        o.skill("move_by", 0, 0, 0.12)
        o.skill("rotate", 90, 30, yaw)
        cz = o.state()["objects"]["cube"]["pos"][2]
        o.skill("move_by", 0, 0, -(cz - (Z + 0.028)) - 0.01, 0.05)
        o.skill("open")
        o.skill("move_by", 0.06 * math.sin(math.radians(yaw)), -0.06 * math.cos(math.radians(yaw)), 0)
        o.skill("move_by", 0, 0, 0.10)
        o.skill("rotate", 0, 30, yaw)


def _solve_pour(o: Oracle, t: dict) -> None:
    st = o.state()
    c = st["objects"]["cup"]["pos"]
    bowl = st["landmarks"]["bowl"]["center"]
    o.skill("rotate", 90, 15, 0)  # palm facing +y, thumb up, fingers slightly down
    o.skill("pose", "open")
    o.skill("move_to", c[0], c[1] - 0.12, Z + 0.22)
    o.skill("move_to", c[0], c[1] - 0.12, Z + 0.07)
    o.skill("move_to", c[0], c[1] - 0.01, Z + 0.07)
    o.skill("grasp", "power")
    o.skill("move_by", 0, 0, 0.15)
    st = o.state()
    cup, palm = np.array(st["objects"]["cup"]["pos"]), np.array(st["hand"]["palm_pos"])
    goal = palm + (np.array([bowl[0], bowl[1] - 0.03, Z + 0.15]) - cup)  # cup 3 cm short of the bowl centre, 15 cm up
    o.skill("move_to", *goal)
    o.skill("rotate", -45, 15, 0, 30)  # roll the forearm: the cup mouth turns toward +y, over the bowl
    o.skill("wait", 1.5)
    # balls can stay caught behind the thumb (the outcome differs between platforms' MuJoCo builds): roll further and
    # shake until nearly all are in the bowl
    for roll in (-65, -45, -75, -45, -75):
        if o.state()["progress"]["balls_in_bowl"] >= t["need"] + 1:
            break
        o.skill("rotate", roll, 15, 0, 60)
        o.skill("wait", 0.5)
    o.skill("rotate", 90, 15, 0, 40)
    o.skill("move_to", c[0], c[1] - 0.01, Z + 0.22)
    o.skill("move_to", c[0], c[1] - 0.01, Z + 0.075)
    o.skill("open")
    o.skill("move_by", 0, -0.10, 0)
    o.skill("move_by", 0, 0, 0.10)


def _solve_valve(o: Oracle, t: dict) -> None:
    st = o.state()
    v = st["landmarks"]["valve"]
    ax, wz, R = np.array(v["axis"]), v["wheel_z"], v["wheel_radius"]
    o.skill("pose", "flat")
    for _ in range(4):
        st = o.state()
        if st["progress"]["valve_turned_deg"] >= t["turn_deg"] + 5:
            break
        a0 = st["landmarks"]["valve"]["angle_deg"]
        # a free sector (between two spokes) whose radial finger plane keeps the positioner yaw in range for a 110 deg stroke
        mids = [a0 + 60 + 120 * k for k in range(-4, 5)]
        ang = min((m for m in mids if -150 <= m - 90 <= 60), key=lambda m: abs(m - 90 + 30))
        o.skill("rotate", 0, 90, ang - 90)
        h = o.state()["hand"]
        tips = np.mean([h["fingertips"][f] for f in h["fingertips"] if f != "thumb"], 0)
        off = tips - np.array(h["palm_pos"])
        tgt = np.array(
            [ax[0] + 0.6 * R * math.cos(math.radians(ang)), ax[1] + 0.6 * R * math.sin(math.radians(ang)), wz - 0.015]
        )
        goal = tgt - off
        o.skill("move_to", goal[0], goal[1], goal[2] + 0.08)
        o.skill("move_to", *goal)
        o.skill("turn_about", ax[0], ax[1], 110)
        o.skill("move_by", 0, 0, 0.08)


def _solve_lift(o: Oracle, t: dict) -> None:
    b = o.state()["objects"]["ball"]["pos"]
    o.skill("pose", "power_ready")
    o.skill("move_to", b[0], b[1], b[2] + 0.1)
    o.skill("move_to", b[0], b[1], b[2] + 0.011)
    o.skill("grasp", "power")
    o.skill("move_by", 0, 0, 0.20)
    o.skill("wait", 1.0)


def _solve_pen(o: Oracle, t: dict) -> None:
    st = o.state()
    cap = st["objects"]["marker"]["cap_end"]
    hc = st["landmarks"]["holder"]["center"]
    o.skill("rotate", 90, 0, 0)  # palm facing +y, fingers forward
    o.skill("pose", "open")
    o.skill("move_to", cap[0] - 0.02, cap[1], Z + 0.30)
    o.skill("move_to", cap[0] - 0.02, cap[1], Z + 0.12)
    o.skill("grasp", "power")
    o.skill("move_by", 0, 0, 0.14, 0.10)
    st = o.state()
    tip, palm = np.array(st["objects"]["marker"]["tip_end"]), np.array(st["hand"]["palm_pos"])
    o.skill("move_to", *(palm + np.array([hc[0] - tip[0], hc[1] - tip[1], 0])), 0.10)
    tip = o.state()["objects"]["marker"]["tip_end"]
    o.skill("move_by", 0, 0, -(tip[2] - (Z + 0.03)), 0.05)
    o.skill("open")
    o.skill("move_by", 0, 0, 0.14)


def _solve_buttons(o: Oracle, t: dict) -> None:
    o.skill("pose", "point")
    o.skill("rotate", 0, 90, 0)  # fingers down: the index finger points at the panel
    for name in t["sequence"]:
        st = o.state()
        off = np.array(st["hand"]["fingertips"]["index"]) - np.array(st["hand"]["palm_pos"])
        top = np.array(st["landmarks"]["buttons"][name]["top_center"])
        o.skill("move_to", *(top - off + np.array([0, 0, 0.03])))
        o.skill("move_by", 0, 0, -0.045, 0.05)
        o.skill("move_by", 0, 0, 0.04)


def _solve_inhand(o: Oracle, t: dict) -> None:
    # finger roll: curl index, middle and ring together (mcp to ~1.2 rad, pip ~0.3 rad over 10 steps) under the cube's
    # front edge; the cube tips back over its rear edge onto the palm, then the fingers straighten again
    names = LEAP_ACTS
    a = np.zeros(6 + len(names))
    for f in ("if", "mf", "rf"):
        a[6 + names.index(f"{f}_mcp")] = 0.47
        a[6 + names.index(f"{f}_pip")] = 0.125
    for _ in range(3):
        o.act(a, 10)
        o.act(-a, 10)
        o.skill("wait", 1.0)
        if o.state()["objects"]["cube"]["face_normals"][t["face"]][2] > 0.95:
            break


SOLVERS = {
    "face_up": _solve_cube_face_up,
    "pour": _solve_pour,
    "valve": _solve_valve,
    "lift": _solve_lift,
    "pen": _solve_pen,
    "buttons": _solve_buttons,
    "inhand": _solve_inhand,
}


def oracle_main(env: str) -> None:
    t = TASKS[env]
    Oracle().run(lambda o: SOLVERS[t["kind"]](o, t))
