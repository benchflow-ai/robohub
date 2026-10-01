"""RoboDojo (MMLab@HKU and partners; NVIDIA Isaac Sim 5.1) simulation tasks as a Robo Use backend.

Isaac Sim needs an NVIDIA RTX GPU, so the simulator runs remotely: one `worker.py` process per RoboDojo task
on a GPU machine (see docs/suites/robodojo.md), each holding RoboDojo's own evaluation environment for that task. This
backend is a thin HTTP client for the worker. The episode server, step budget, trace, video, verifier and harnesses all
run locally as for any other backend. The agent never sees the worker's address: it is read from a file under
a private endpoints file (denied to model harnesses by the local runner's sandbox), and the worker rejects requests without the
shared secret.

Worker endpoints come from $ROBOUSE_ROBODOJO_REMOTE (default: <config>/remote/robodojo.json, see robouse.config):
  {"secret": "...", "headers": {...}, "workers": {"<robodojo task>": "https://..."}}

Embodiment: RoboDojo's simulated ARX X5 bimanual platform (two 6-joint ARX X5 arms with parallel grippers, bases 0.6 m
apart at the near edge of the table). One action is one RoboDojo policy step (40 ms: 10 physics steps of 4 ms): absolute
end-effector pose targets for both arms plus both gripper openings, solved by RoboDojo's own cuRobo IK, exactly the
`left_ee_pose` / `right_ee_pose` action a RoboDojo policy sends. The budget is the task's own `step_lim`.
Success is RoboDojo's `run_reward()` reaching 1 (every stage passed, in order, with no forbidden event), checked by the
worker after every action; the episode ends as solved the first time it does (`success_mode: first`), as in RoboDojo.
"""

from __future__ import annotations

import math
import time

import numpy as np

from ... import config
from ...workers.client import WorkerUnavailable, decode_png_b64, load_endpoints, remote_worker
from ..base import StepInfo
from ..embodied import ActionGroup, Budget, EmbodiedBackend, Embodiment, Oracle, Sensor, Skill, SkillArg

RETRY_METHODS = frozenset({"info", "observe", "render", "success", "status"})
SETUP = config.SUITES_DOCS
ARMS = ("left", "right")
STEP_S = 0.04  # one RoboDojo action: 10 physics steps of 4 ms
# A skill call returns after at most this much wall time (the robo client waits 120 s for a reply); the result then
# says `stopped` and the same call continues from where the arm is.
SKILL_WALL_S = float(config.env("ROBOUSE_ROBODOJO_SKILL_WALL_S", "90"))


# ---- small rotation helpers (quaternions are w, x, y, z) ------------------------------------------------------------


def quat_mul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ]
    )


def quat_axis_angle(axis, deg):
    axis = np.asarray(axis, float)
    axis = axis / (np.linalg.norm(axis) + 1e-12)
    h = math.radians(deg) / 2
    return np.array([math.cos(h), *(axis * math.sin(h))])


def quat_norm(q):
    q = np.asarray(q, float)
    q = q / (np.linalg.norm(q) + 1e-12)
    return q if q[0] >= 0 else -q


def slerp(q0, q1, t):
    q0, q1 = quat_norm(q0), quat_norm(q1)
    d = float(np.dot(q0, q1))
    if d < 0:
        q1, d = -q1, -d
    if d > 0.9995:
        return quat_norm(q0 + t * (q1 - q0))
    th = math.acos(min(1.0, d))
    return (math.sin((1 - t) * th) * q0 + math.sin(t * th) * q1) / math.sin(th)


def quat_angle_deg(q0, q1):
    d = abs(float(np.dot(quat_norm(q0), quat_norm(q1))))
    return math.degrees(2 * math.acos(min(1.0, d)))


def mat_quat(R) -> np.ndarray:
    R = np.asarray(R, float)
    t = np.trace(R)
    if t > 0:
        k = 0.5 / math.sqrt(t + 1.0)
        q = [0.25 / k, (R[2, 1] - R[1, 2]) * k, (R[0, 2] - R[2, 0]) * k, (R[1, 0] - R[0, 1]) * k]
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        k = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        q = [(R[2, 1] - R[1, 2]) / k, 0.25 * k, (R[0, 1] + R[1, 0]) / k, (R[0, 2] + R[2, 0]) / k]
    elif R[1, 1] > R[2, 2]:
        k = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        q = [(R[0, 2] - R[2, 0]) / k, (R[0, 1] + R[1, 0]) / k, 0.25 * k, (R[1, 2] + R[2, 1]) / k]
    else:
        k = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        q = [(R[1, 0] - R[0, 1]) / k, (R[0, 2] + R[2, 0]) / k, (R[1, 2] + R[2, 1]) / k, 0.25 * k]
    return quat_norm(q)


def quat_mat(q) -> np.ndarray:
    w, x, y, z = quat_norm(q)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


# The ARX X5 gripper: RoboDojo's end-effector link is `link6`; the fingers point along its +x axis and close along its
# y axis. The grasp point between the fingertips is GRIPPER_BIAS along +x from the link origin (RoboDojo's
# robot_config.yml `gripper_bias`).
GRIPPER_BIAS = 0.145


def hand_quat(pitch_deg: float = 90.0, yaw_deg: float = 90.0) -> np.ndarray:
    """Gripper orientation from two angles. The fingers point along the approach direction
    (cos p cos y, cos p sin y, -sin p): PITCH is how far below horizontal they point (90 = straight down, 0 = level) and
    YAW is the compass heading of that direction (90 = +y, away from the robots). The fingers close along the
    horizontal direction (-sin y, cos y, 0), so with the hand pointing down, YAW = 90 closes the fingers along x."""
    p, y = math.radians(pitch_deg), math.radians(yaw_deg)
    a = np.array([math.cos(p) * math.cos(y), math.cos(p) * math.sin(y), -math.sin(p)])
    c = np.array([-math.sin(y), math.cos(y), 0.0])
    c = c - a * float(np.dot(a, c))
    c /= np.linalg.norm(c)
    return mat_quat(np.column_stack([a, c, np.cross(a, c)]))


def tcp_of(ee_pose) -> np.ndarray:
    """Grasp point of a gripper from its end-effector link pose (x, y, z, qw, qx, qy, qz)."""
    return np.asarray(ee_pose[:3], float) + GRIPPER_BIAS * quat_mat(ee_pose[3:7])[:, 0]


def ee_for_tcp(tcp, quat) -> np.ndarray:
    return np.asarray(tcp, float) - GRIPPER_BIAS * quat_mat(quat)[:, 0]


# ---- the declaration ------------------------------------------------------------------------------------------------


def _decl(max_steps: int) -> Embodiment:
    groups = []
    for arm in ARMS:
        groups.append(
            ActionGroup(
                f"{arm}.ee_pose",
                "ee_pose",
                [f"{arm}_{c}" for c in ("x", "y", "z", "qw", "qx", "qy", "qz")],
                [-1.5, -1.5, 0.0, -1, -1, -1, -1],
                [1.5, 1.5, 2.0, 1, 1, 1, 1],
                units="m, quaternion (w, x, y, z)",
                frame="world",
                doc=f"absolute target pose of the {arm} gripper (RoboDojo's `{arm}_ee_pose`), "
                "reached through RoboDojo's cuRobo IK; an unreachable pose leaves the arm where it is",
            )
        )
        groups.append(
            ActionGroup(
                f"{arm}.gripper",
                "gripper",
                [f"{arm}_grip"],
                [0.0],
                [1.0],
                units="fraction open",
                doc="0 closed, 1 fully open",
            )
        )
    sensors = [
        Sensor(
            "arms",
            "proprio",
            ["robot"],
            doc="per arm: gripper pose (ee_pos, ee_quat, ee_axes), gripper_open, joints, home pose",
        ),
        Sensor(
            "objects",
            "world",
            ["objects"],
            doc="every task object: label, type, category, model_id, description, pos, "
            "quat, up_axis, yaw_deg, bbox_min/bbox_max, size; articulated objects add their joints",
        ),
        Sensor(
            "progress",
            "world",
            ["instruction", "steps", "step_lim", "stages_left", "score", "ended", "failed"],
            doc="RoboDojo's instruction and progress counters",
        ),
        Sensor("camera:cam_head", "camera", mount="head", doc="RoboDojo's head camera"),
        Sensor("camera:cam_left_wrist", "camera", mount="wrist"),
        Sensor("camera:cam_right_wrist", "camera", mount="wrist"),
    ]
    arm = SkillArg("arm", "enum", choices=list(ARMS), doc="which arm")
    skills = [
        Skill(
            "move",
            [
                arm,
                SkillArg("x", unit="m"),
                SkillArg("y", unit="m"),
                SkillArg("z", unit="m"),
                SkillArg(
                    "pitch",
                    unit="deg",
                    default=90.0,
                    doc="how far below horizontal the fingers point (90 = straight down)",
                ),
                SkillArg(
                    "yaw",
                    unit="deg",
                    default=90.0,
                    doc="compass heading of the finger direction; with the hand pointing "
                    "down the fingers close along (-sin YAW, cos YAW, 0)",
                ),
                SkillArg("speed", unit="m/step", default=0.012, doc="at most this far per 40 ms step (0.002-0.03)"),
            ],
            "move one gripper's grasp point (between the fingertips) in a straight line to (X, Y, Z) with the given "
            "orientation; the other arm holds still",
            max_steps=250,
        ),
        Skill(
            "grip",
            [arm, SkillArg("open", doc="0 closed .. 1 fully open"), SkillArg("steps", "int", default=12)],
            "set one gripper's opening and hold still for STEPS steps",
            max_steps=50,
        ),
        Skill(
            "home",
            [SkillArg("arm", "enum", default="both", choices=["left", "right", "both"])],
            "move the arm(s) back to the starting pose (many tasks require this at the end)",
            max_steps=250,
        ),
        Skill("wait", [SkillArg("steps", "int", default=10)], "hold both arms still", max_steps=50),
    ]
    return Embodiment(
        robot="ARX X5 bimanual (RoboDojo)",
        family="bimanual_arm",
        assets=["RoboDojo Assets/Robots/x5"],
        sensors=sensors,
        action_groups=groups,
        skills=skills,
        budget=Budget(max_steps, STEP_S),
        cameras=["cam_head", "cam_left_wrist", "cam_right_wrist"],
    )


class RoboDojoBackend(EmbodiedBackend):
    name = "robodojo"

    def __init__(self, spec: dict):
        super().__init__(spec)
        self.task = spec["env"]  # RoboDojo task name, e.g. stack_bowls
        self.worker = remote_worker(
            load_endpoints("robodojo", SETUP),
            self.task,
            "RoboDojo",
            float(spec.get("rpc_timeout_s", 300)),
            RETRY_METHODS,
        )
        self.decl = _decl(int(spec.get("max_steps", 600)))
        self.camera = str(spec.get("camera", "cam_head"))
        self.frame_every = int(spec.get("frame_every", 2))
        self._cmd: dict[str, list[float]] = {}  # last commanded pose + grip per arm (what "hold still" repeats)
        self._home: dict[str, list[float]] = {}
        self._ee: dict[str, list[float]] = {}
        self._last_frame: np.ndarray | None = None
        self._n = 0
        self.last: dict = {}
        self.last_judge = None

    def _call(self, method: str, _timeout: float | None = None, **args):
        return self.worker.call(method, _timeout, **args)

    # ---- Backend interface -----------------------------------------------------------------------------------------
    def _wait_ready(self, timeout: float = 900) -> None:
        t = time.time()
        while time.time() - t < timeout:
            time.sleep(5)
            try:
                if self._call("info", _timeout=30):
                    return
            except WorkerUnavailable:
                continue
        raise RuntimeError("RoboDojo worker did not come back after a restart")

    def reset(self, seed: int) -> None:
        try:
            r = self._call("reset", layout=int(seed), _timeout=600)
        except WorkerUnavailable:
            # the worker is still starting (or restarting after the previous episode): wait for it once
            self.worker.close()
            self._wait_ready(float(self.spec.get("ready_timeout_s", 900)))
            r = self._call("reset", layout=int(seed), _timeout=600)
        for _ in range(3):
            if not r.get("restarting"):
                break
            # the worker starts a fresh RoboDojo process for every episode (worker.py, Worker.reset)
            self.worker.close()
            time.sleep(10)
            self._wait_ready(float(self.spec.get("ready_timeout_s", 900)))
            r = self._call("reset", layout=int(seed), _timeout=600)
        if r.get("restarting"):
            raise RuntimeError("RoboDojo worker kept restarting")
        self.instruction = r.get("instruction", "")
        self._ee = {k: list(v) for k, v in r["ee"].items()}
        self._home = {k: list(v[:7]) for k, v in self._ee.items()}
        self._cmd = {k: list(v[:7]) + [1.0] for k, v in self._ee.items()}  # RoboDojo starts with open grippers
        self._n = 0
        self._last_frame = None
        self.last = {}

    def _vec(self) -> list[float]:
        return [float(x) for arm in ARMS for x in self._cmd[arm]]

    def control_step(self, a: np.ndarray) -> None:
        a = [float(x) for x in a]
        for i, arm in enumerate(ARMS):
            seg = a[i * 8 : (i + 1) * 8]
            q = np.asarray(seg[3:7])
            if np.linalg.norm(q) < 1e-6:
                raise ValueError(f"{arm} quaternion must be non-zero")
            self._cmd[arm] = seg[:3] + list(quat_norm(q)) + [min(1.0, max(0.0, seg[7]))]
        self._n += 1
        frame = self._n % self.frame_every == 0
        r = self._call("step", action=self._vec(), frame=frame, camera=self.camera)
        self._last_frame = decode_png_b64(r.pop("frame")) if r.get("frame") else None
        self._ee = {k: list(v) for k, v in r.get("ee", {}).items()} or self._ee
        self.last = r

    def step(self, action) -> StepInfo:
        a = np.asarray(action, dtype=float).ravel()
        if a.shape[0] != 16 or not np.all(np.isfinite(a)):
            raise ValueError("action must be 16 finite numbers")
        self.control_step(a)
        return StepInfo(success=bool(self.last.get("success")))

    def hold_action(self) -> list[float]:
        return self._vec()

    def observe(self) -> dict:
        o = self._call("observe")
        o["commanded"] = {
            arm: {
                "pos": [round(x, 4) for x in self._cmd[arm][:3]],
                "quat": [round(x, 4) for x in self._cmd[arm][3:7]],
                "grip": round(self._cmd[arm][7], 3),
            }
            for arm in ARMS
        }
        return o

    def render(self, width: int = 640, height: int = 480) -> np.ndarray:
        if self.camera == self.spec.get("camera", "cam_head") and self._last_frame is not None:
            return self._last_frame
        img = decode_png_b64(self._call("render", camera=self.camera)["jpeg"])
        if self.camera == self.spec.get("camera", "cam_head"):
            self._last_frame = img
        return img

    def success(self) -> bool:
        st = self._call("success")
        self.last_judge = {k: st.get(k) for k in ("success", "failed", "steps", "step_lim", "stages_left", "score")}
        return bool(st.get("success"))

    def judge(self, outcome: str, text: str = "") -> bool:
        # RoboDojo's protocol: solved the first time run_reward() reaches 1 within step_lim (the worker latches it)
        return self.success()

    def embodiment(self) -> dict:
        spec = super().embodiment()
        for g in spec["action_groups"]:
            if g["mode"] == "ee_pose":
                g["hold"] = "last"
                g.pop("hold_value", None)
        spec["reward"]["success_mode"] = "first"
        return spec

    # ---- skills (generators: one action per 40 ms step) --------------------------------------------------------
    def _reach_status(self, arm, pos, quat) -> dict:
        cur = self._ee.get(arm)
        if not cur:
            return {}
        err = float(np.linalg.norm(np.asarray(cur[:3]) - np.asarray(pos)))
        return {
            "reached": err < 0.01 and quat_angle_deg(cur[3:7], quat) < 8,
            "pos_error_m": round(err, 4),
            "angle_error_deg": round(quat_angle_deg(cur[3:7], quat), 1),
            "ee_pos": [round(x, 4) for x in cur[:3]],
            "ik": self.last.get("ik", {}).get(arm, ""),
        }

    def _line(self, arm, pos, quat, speed: float):
        start_p = np.asarray(self._cmd[arm][:3], float)
        start_q = np.asarray(self._cmd[arm][3:7], float)
        pos = np.asarray(pos, float)
        dist = float(np.linalg.norm(pos - start_p))
        ang = quat_angle_deg(start_q, quat)
        n = max(1, int(math.ceil(max(dist / speed, ang / 4.0))))
        t0 = time.time()
        for i in range(1, n + 1):
            if time.time() - t0 > SKILL_WALL_S:
                return False
            t = i / n
            p = start_p + t * (pos - start_p)
            q = slerp(start_q, quat, t)
            self._cmd[arm] = [*p, *q, self._cmd[arm][7]]
            yield self._vec()
        for _ in range(12):  # let the arm catch up with the last target
            cur = self._ee.get(arm)
            if cur and np.linalg.norm(np.asarray(cur[:3]) - pos) < 0.004:
                break
            yield self._vec()
        return True

    def skill_move(
        self, arm: str, x: float, y: float, z: float, pitch: float = 90.0, yaw: float = 90.0, speed: float = 0.012
    ):
        speed = min(0.03, max(0.002, speed))
        quat = hand_quat(pitch, yaw)
        ee = ee_for_tcp([x, y, z], quat)
        finished = yield from self._line(arm, ee, quat, speed)
        st = self._reach_status(arm, ee, quat)
        if not finished:
            st["stopped"] = f"wall-time limit of one call ({SKILL_WALL_S:.0f} s) reached; call move again to continue"
        if self._ee.get(arm):
            st["grasp_point"] = [round(float(v), 4) for v in tcp_of(self._ee[arm][:7])]
        return st

    def skill_grip(self, arm: str, open: float, steps: int = 12):
        self._cmd[arm][7] = min(1.0, max(0.0, float(open)))
        for _ in range(max(1, min(50, steps))):
            yield self._vec()
        return {"gripper_open": self._ee.get(arm, [0] * 8)[7] if len(self._ee.get(arm, [])) > 7 else None}

    def skill_home(self, arm: str = "both"):
        arms = ARMS if arm == "both" else (arm,)
        starts = {a: (np.asarray(self._cmd[a][:3], float), np.asarray(self._cmd[a][3:7], float)) for a in arms}
        n = 1
        for a in arms:
            p0, q0 = starts[a]
            h = self._home[a]
            n = max(
                n, int(math.ceil(max(np.linalg.norm(np.asarray(h[:3]) - p0) / 0.012, quat_angle_deg(q0, h[3:7]) / 4.0)))
            )
        t0 = time.time()
        for i in range(1, n + 1):
            if time.time() - t0 > SKILL_WALL_S:
                return {
                    "stopped": f"wall-time limit of one call ({SKILL_WALL_S:.0f} s) reached; call home again to continue"
                }
            t = i / n
            for a in arms:
                p0, q0 = starts[a]
                h = self._home[a]
                self._cmd[a] = [*(p0 + t * (np.asarray(h[:3]) - p0)), *slerp(q0, h[3:7], t), self._cmd[a][7]]
            yield self._vec()
        for _ in range(15):
            if all(np.linalg.norm(np.asarray(self._ee[a][:3]) - np.asarray(self._home[a][:3])) < 0.01 for a in arms):
                break
            yield self._vec()
        return {a: self._reach_status(a, self._home[a][:3], self._home[a][3:7]) for a in arms}

    def skill_wait(self, steps: int = 10):
        for _ in range(max(1, min(50, steps))):
            yield self._vec()
        return {}

    def close(self) -> None:
        pass


def oracle_main(env: str) -> None:
    """Reference solutions: scripted per-task plans (oracles.py) that read `robo observe` and send
    `robo skill move|grip|home` through the episode socket, like an agent."""
    from .oracles import PLANS

    task = config.env("ROBOUSE_ROBODOJO_TASK") or env
    o = Oracle()
    plain = o.skill

    def skill(name, *args):  # a long skill on a slow worker returns early (SKILL_WALL_S): repeat it until it finishes
        for _ in range(10):
            r = plain(name, *args)
            if "wall-time limit" not in str(r.get("stopped", "")):
                return r
        return r

    o.skill = skill
    o.run(PLANS.get(task) or PLANS[task.removesuffix("_random")])  # a _random variant has its base task's objective
