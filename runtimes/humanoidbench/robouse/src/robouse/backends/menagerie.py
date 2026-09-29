"""MuJoCo Menagerie backend: dual Franka Panda and ALOHA 2 desk scenes from the robo-use workspace.

Scenes. Two arms (two Franka Emika Panda, or an ALOHA 2 pair of ViperX 300s) are mounted side by side at
the rear edge of a 1.6 m x 0.94 m desk and reach forward (world +x right, +y toward the robot bases, +z up,
tabletop at z = 0, metres). Objects are free rigid bodies moved only by contact and friction; nothing is
welded, attached or teleported. Physics runs at 2 ms, control at 50 ms (one `robo act` step), joint targets
are slew-limited at 1.2 rad/s and the arm joints get ideal gravity compensation (robo-use `control.py`).
Assets: MuJoCo Menagerie (pinned commit, see assets/menagerie/provenance.json).

Action (Cartesian, through the scene's inverse kinematics). Every arm the task lets you drive has a
commanded gripper target (position, yaw about vertical, finger opening). One step moves the target by
  DX, DY, DZ in [-1, 1]  x 2 cm per step (so 0.25 = 5 mm)
  DYAW in [-1, 1]        x 10 degrees per step (only on tasks with `yaw`)
  GRIP                   0 keeps the current finger setting, any positive value closes the fingers,
                         a negative value -f opens them to fraction f of full width (-1 fully open,
                         -0.4 = 40 % open).
and IK turns the target into joint targets for that 50 ms step. The target stays within 5 cm of the
measured gripper position (so a blocked arm does not wind up) and inside the reachable desk volume; the
gripper always points straight down. Layouts (task `arms`):
  "left" / "right": [DX, DY, DZ, GRIP] (or [DX, DY, DZ, DYAW, GRIP]); skills move_to / grip drive that arm.
  "switch":         [ARM, DX, DY, DZ, GRIP] (+DYAW before GRIP); ARM < 0 selects the left arm, > 0 the right,
                    0 keeps the current one; the other arm holds; skills drive the selected arm.
  "both":           [L_DX, L_DY, L_DZ, L_GRIP, R_DX, R_DY, R_DZ, R_GRIP] (+DYAW per arm); no skills.

Observation (`robo observe`): per arm the measured gripper point (`hand_pos`, the point between the
fingertips), the commanded target (`hand_target`), `gripper_open` (0 closed .. 1 open), `hand_yaw_deg` and
which task objects the arm touches; every task object's position (the centre of its base, m), tilt from
vertical and yaw (degrees); and the task's public goal fields. Success is a physical predicate of the task
family (families.py), judged by the trusted server after `robo done` and a 10-step hold.
"""
from __future__ import annotations

import numpy as np

from .base import ActionSpec, Backend, StepInfo

STEP_M = .02
YAW_STEP = np.deg2rad(10.)
LEAD_M = .05
WORK_LO = np.array([-.75, -.46, .012])
WORK_HI = np.array([.75, .32, .50])
YAW_LIMIT = np.deg2rad(100.)
RESET_SETTLE_STEPS = 20


# id -> task definition. `family` names a class in menagerie_sim.families; `arms` the action layout.
TASKS: dict[str, dict] = {
    "menagerie-panda-precise-landing": dict(family="landing", embodiment="dual_panda", seed=0, arms="left", max_steps=600),
    "menagerie-aloha-pawn-landing": dict(family="landing", embodiment="aloha2", seed=1, arms="right", max_steps=600,
                                         params={"kind": "pawn"}),
    "menagerie-panda-clutter-retrieval": dict(family="clutter", embodiment="dual_panda", seed=0, arms="left", max_steps=800),
    "menagerie-aloha-clutter-retrieval": dict(family="clutter", embodiment="aloha2", seed=1, arms="right", max_steps=800),
    "menagerie-aloha-gentle-insertion": dict(family="insertion", embodiment="aloha2", seed=0, arms="left", max_steps=1200),
    "menagerie-panda-two-bowls-to-board": dict(family="two_bowls", embodiment="dual_panda", seed=0, arms="both", max_steps=3000),
    "menagerie-aloha-opening-four": dict(family="two_bowls", embodiment="aloha2", seed=3, arms="both", max_steps=2000,
                                         params={"kinds": ("pawn", "rook"), "require_both_arms": True}),
    "menagerie-panda-clutter-retrieval-vision": dict(family="clutter", embodiment="dual_panda", seed=2, arms="left", max_steps=800,
                                                     vision={"cameras": ["workspace", "top"], "hide_objects": True}),
    "menagerie-aloha-pawn-landing-vision": dict(family="landing", embodiment="aloha2", seed=4, arms="left", max_steps=600,
                                                params={"kind": "pawn"}, vision={"cameras": ["workspace", "top"], "hide_objects": True}),
    "menagerie-panda-keyed-insertion": dict(family="keyed_insertion", embodiment="dual_panda", seed=1, arms="right", yaw=True,
                                            max_steps=1500),
    "menagerie-panda-pattern-apprentice": dict(family="pattern", embodiment="dual_panda", seed=5, arms="switch", max_steps=2000,
                                               params={"split": "development"}),
    "menagerie-aloha-pattern-heldout": dict(family="pattern", embodiment="aloha2", seed=5, arms="switch", max_steps=2000,
                                            params={"split": "heldout"}),
    "menagerie-panda-fragile-kit": dict(family="fragile_kit", embodiment="dual_panda", seed=2, arms="switch", max_steps=2500,
                                        vision={"cameras": ["workspace", "inspect"], "private": ["vial_labels"]}),
    "menagerie-panda-route-around": dict(family="route_around", embodiment="dual_panda", seed=0, arms="left", max_steps=1000),
    "menagerie-panda-fragile-disposal": dict(family="fragile_disposal", embodiment="dual_panda", seed=0, arms="left",
                                             max_steps=1000),
}

_R = lambda v, n=4: [round(float(x), n) for x in np.ravel(v)]


def _yaw_quat(yaw: float, down) -> np.ndarray:
    import mujoco

    out = np.zeros(4)
    mujoco.mju_mulQuat(out, np.array([np.cos(yaw / 2), 0., 0., np.sin(yaw / 2)]), np.asarray(down, dtype=float))
    return out


class MenagerieBackend(Backend):
    name = "menagerie"

    def __init__(self, spec: dict):
        env = spec.get("env") or spec.get("id")
        if env not in TASKS:
            raise KeyError(f"unknown menagerie task {env!r}")
        self.task_id = env
        self.task = dict(TASKS[env])
        self.params = dict(self.task.get("params", {}))
        self.arms_mode = str(self.task["arms"])
        self.yaw_enabled = bool(self.task.get("yaw", False))
        self.max_steps = int(spec.get("max_steps", self.task.get("max_steps", 1000)))
        self.camera = spec.get("camera", "workspace")
        self._grip = 0.0  # the hold action used by the server keeps the fingers as they are
        self._renderer = None
        self.sim = None
        if self.arms_mode in ("left", "right"):
            self.driven = [self.arms_mode]
        else:
            self.driven = ["left", "right"]
        self.selected = self.driven[0] if self.arms_mode != "switch" else str(self.task.get("start_arm", "left"))
        per = ["DX", "DY", "DZ"] + (["DYAW"] if self.yaw_enabled else []) + ["GRIP"]
        if self.arms_mode == "both":
            names = [f"L_{n}" for n in per] + [f"R_{n}" for n in per]
        elif self.arms_mode == "switch":
            names = ["ARM"] + per
        else:
            names = per
        doc = ("DX/DY/DZ move the gripper target by 2 cm per unit per step (world axes); "
               + ("DYAW turns it by 10 degrees per unit; " if self.yaw_enabled else "")
               + "GRIP: 0 keeps the fingers, > 0 closes, -f opens to fraction f (-1 fully open)"
               + ("; ARM: -1 left arm, +1 right arm, 0 keep the selected arm" if self.arms_mode == "switch" else "")
               + ("; L_* drive the left arm and R_* the right arm in the same step" if self.arms_mode == "both" else "")
               + ". One step = 50 ms.")
        self.action_spec = ActionSpec(names=names, low=[-1.0] * len(names), high=[1.0] * len(names), doc=doc)

    # ---- episode ---------------------------------------------------------------------------------------------
    def reset(self, seed: int) -> None:
        from .menagerie_sim.control import Simulation
        from .menagerie_sim.families import FAMILIES
        from .menagerie_sim.ik import ArmIK
        from .menagerie_sim.monitor import Monitor

        self.family = FAMILIES[self.task["family"]]
        seed = int(self.task.get("seed", seed))
        self.scene = self.family.build(seed, self.task["embodiment"], self.params)
        self.sim = Simulation(self.scene)
        self._objects = self.family.objects(self.scene)
        self._protected = self.family.protected(self.scene)
        mon_task = type("T", (), {"DESK_PREFIXES": self.family.desk_prefixes(), "FORCE_PAIRS": self.family.force_pairs(),
                                  "observation_bodies": staticmethod(lambda s: list(dict.fromkeys(self._objects.values())))})
        self.monitor = Monitor(self.sim, mon_task, self.scene)
        self.ik = {s: ArmIK(self.sim.model, arm) for s, arm in self.scene.arms.items()}
        self.q_cmd = {s: self.sim.arm_joint_positions(s) for s in self.scene.arms}
        self.open_cmd = {s: 1.0 for s in self.scene.arms}
        self.sim.hold(RESET_SETTLE_STEPS)  # reset settle, not part of the episode
        self.monitor.control(self.sim)
        self.target = {s: self.sim.tcp_pose(s)[0].copy() for s in self.scene.arms}
        self.yaw_cmd = {s: 0.0 for s in self.scene.arms}
        self._ref = {n: self.sim.data.body(b).xpos.copy() for n, b in self._protected.items()}
        self._max_shift = {n: 0.0 for n in self._protected}
        self._max_tilt = {n: 0.0 for n in self._protected}
        self.monitor.peak_force_n = 0.0  # count only contact during the episode
        self.family.start(self)

    def _apply(self, side: str, d, dyaw: float, g: float) -> None:
        d = np.clip(np.asarray(d, dtype=float), -1, 1) * STEP_M
        hand = self.sim.tcp_pose(side)[0]
        t = np.clip(self.target[side] + d, WORK_LO, WORK_HI)
        off = t - hand
        n = float(np.linalg.norm(off))
        if n > LEAD_M:
            t = hand + off * (LEAD_M / n)
        self.target[side] = t
        self.yaw_cmd[side] = float(np.clip(self.yaw_cmd[side] + np.clip(dyaw, -1, 1) * YAW_STEP, -YAW_LIMIT, YAW_LIMIT))
        if g > 1e-3:
            self.open_cmd[side] = 0.0
        elif g < -1e-3:
            self.open_cmd[side] = float(min(1.0, -g))

    def _split(self, a: np.ndarray) -> dict:
        """action vector -> {side: (d[3], dyaw, grip)} for the arms that move this step."""
        per = 5 if self.yaw_enabled else 4

        def one(v):
            v = list(v)
            if len(v) == 4:  # skill-length vector [dx, dy, dz, grip] (move_to / grip / settle hold)
                return v[:3], 0.0, v[3]
            return v[:3], v[3], v[4]

        if self.arms_mode == "both":
            if len(a) == 2 * per:
                return {"left": one(a[:per]), "right": one(a[per:])}
            return {"left": one([0.0] * 4), "right": one([0.0] * 4)}  # e.g. the server's settle hold
        if self.arms_mode == "switch" and len(a) == per + 1:
            if a[0] < -1e-3:
                self.selected = "left"
            elif a[0] > 1e-3:
                self.selected = "right"
            return {self.selected: one(a[1:])}
        return {self.selected: one(a[-per:] if len(a) >= per else a)}

    def step(self, action) -> StepInfo:
        a = np.asarray(action, dtype=float)
        for side, (d, dyaw, g) in self._split(a).items():
            self._apply(side, d, dyaw, g)
        targets = {}
        for side, arm in self.scene.arms.items():
            quat = _yaw_quat(self.yaw_cmd[side], arm.down_quat)
            q, _, _ = self.ik[side].solve(self.sim.data, self.q_cmd[side], self.target[side], quat)
            self.q_cmd[side] = q
            targets[side] = (q, self.open_cmd[side])
        self.sim.step_control(targets, substep_hook=self._substep)
        self.monitor.control(self.sim)
        self._track_protected()
        self.family.track(self)
        return StepInfo(success=False)

    def _substep(self, sim) -> None:  # every 5th physics substep (10 ms)
        self.monitor.substep(sim)
        self.family.substep(self)

    def _track_protected(self) -> None:
        from .menagerie_sim.families import body_state

        for n, b in self._protected.items():
            s = body_state(self.sim, b)
            self._max_shift[n] = max(self._max_shift[n], float(np.linalg.norm(s["pos"][:2] - self._ref[n][:2])))
            self._max_tilt[n] = max(self._max_tilt[n], s["tilt"])

    def protected_ok(self, max_shift: float, max_tilt: float) -> bool:
        return all(v <= max_shift for v in self._max_shift.values()) and all(v <= max_tilt for v in self._max_tilt.values())

    # ---- observation -----------------------------------------------------------------------------------------
    def _arm_fields(self, side: str, prefix: str) -> dict:
        arm = self.scene.arms[side]
        pos, _ = self.sim.tcp_pose(side)
        fingers = self.sim.finger_positions(side)
        touching = sorted(n for n, b in self._objects.items() if side in self.monitor.current_touch.get(b, set()))
        out = {f"{prefix}hand_pos": _R(pos), f"{prefix}hand_target": _R(self.target[side]),
               f"{prefix}gripper_open": round(float(np.clip(np.mean(np.abs(fingers)) / arm.finger_open_m, 0, 1)), 3),
               f"{prefix}touching": touching}
        if self.yaw_enabled:
            out[f"{prefix}hand_yaw_deg"] = round(float(np.degrees(self.yaw_cmd[side])), 1)
        return out

    def observe(self) -> dict:
        from .menagerie_sim.families import body_state

        st: dict = {"robot": "panda" if self.scene.embodiment == "dual_panda" else "aloha",
                    "time_s": round(float(self.sim.data.time), 2)}
        if self.arms_mode in ("left", "right"):
            st["arm"] = self.arms_mode
            st.update(self._arm_fields(self.arms_mode, ""))
            other = "right" if self.arms_mode == "left" else "left"
            st["other_arm_hand_pos"] = _R(self.sim.tcp_pose(other)[0])
        else:
            if self.arms_mode == "switch":
                st["selected_arm"] = self.selected
                st.update(self._arm_fields(self.selected, ""))
            for side in ("left", "right"):
                st.update(self._arm_fields(side, f"{side}_"))
        for n, b in self._objects.items():
            s = body_state(self.sim, b)
            st[f"{n}_pos"] = _R(s["pos"])
            st[f"{n}_tilt_deg"] = round(s["tilt"], 1)
            st[f"{n}_yaw_deg"] = round(s["yaw"], 1)
        st.update(self.family.goal(self.scene))
        st.update(self.family.private(self.scene))
        return st

    def hand_pos(self) -> np.ndarray:
        return self.sim.tcp_pose(self.selected)[0].copy()

    def skills(self) -> list[str]:
        return [] if self.arms_mode == "both" else ["move_to", "grip"]

    # ---- scoring ---------------------------------------------------------------------------------------------
    def success_checks(self) -> dict:
        return {k: bool(v) for k, v in self.family.checks(self).items()}

    def success(self) -> bool:
        return all(self.success_checks().values())

    # ---- rendering -------------------------------------------------------------------------------------------
    def mj_model_data(self):
        return self.sim.model, self.sim.data

    def render(self, width: int = 480, height: int = 368) -> np.ndarray:
        import mujoco

        if self._renderer is None or (self._renderer.width, self._renderer.height) != (width, height):
            self._renderer = mujoco.Renderer(self.sim.model, height=height, width=width)
        self._renderer.update_scene(self.sim.data, camera=self.camera)
        return self._renderer.render().copy()

    def close(self) -> None:
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None


def oracle_main(env: str) -> None:
    from .menagerie_sim.oracles import run

    run(env)


def task_spec_extra(env: str) -> dict:
    """Extra `robouse:` fields for a task (observation mode). Vision tasks hide their private fields from agents:
    `visible_fields` lists every observation field except the private ones."""
    t = TASKS[env]
    if not t.get("vision"):
        return {}
    b = MenagerieBackend({"env": env})
    b.reset(0)
    hidden = set(t["vision"].get("private", []))
    if t["vision"].get("hide_objects"):  # object poses must be found in the camera images
        hidden |= {f"{n}_{s}" for n in b._objects for s in ("pos", "tilt_deg", "yaw_deg")}
    keys = [k for k in b.observe() if k not in hidden]
    b.close()
    return {"obs_mode": "vision", "cameras": list(t["vision"]["cameras"]), "visible_fields": keys}
