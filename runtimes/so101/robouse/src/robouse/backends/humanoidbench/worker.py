"""HumanoidBench simulation worker for the Robo Use `humanoidbench` suite. Runs inside its own virtualenv (Python 3.11,
HumanoidBench at a pinned commit with its own pins: mujoco 3.1.6, gymnasium 0.29.1, dm_control 1.0.20, numpy 1.26),
because those pins must not enter the main Robo Use environment.

The Robo Use backend (humanoidbench.py) starts this script as a subprocess and talks to it over stdin/stdout: one JSON
request per line, one JSON response per line; a `render` response is followed by the raw RGB bytes. The robouse package is
never imported here.

Simulation: HumanoidBench's own environment (`h1hand-<task>-v0`: Unitree H1 with two Shadow hands, position-controlled
joints, 50 Hz control = 10 MuJoCo steps of 2 ms), its own reward, termination and success signals, unchanged.

Low-level controller (the layer under `robo act`, all on the trusted side):
- legs: Unitree's pretrained H1 locomotion policy from unitree_rl_gym (BSD-3-Clause; an LSTM actor, 41 observations ->
  10 leg joint targets, run at 50 Hz). It takes a velocity command (forward, left, turn rate) and keeps the robot
  balanced; zero command = step in place. The weights are read from the TorchScript file with a small zip/pickle reader
  (no torch needed) and evaluated in numpy.
- arms: damped least-squares inverse kinematics of the 4 arm joints (shoulder pitch, roll, yaw, elbow) to a goal point for
  each hand, given in the robot's heading frame (x forward, y left, origin on the floor under the pelvis, z = height
  above the floor), with gravity feed-forward on the position targets. Wrists, fingers and the torso joint hold their
  start positions.
"""

from __future__ import annotations

import math
import pickle
import sys
import types
import zipfile

import numpy as np


def _stub_modules() -> None:
    """HumanoidBench imports jax.numpy and torch at module level only for its optional hierarchical-policy wrappers
    (flax_to_torch.py), which Robo Use never uses. Stand-ins keep those imports from failing when the packages are
    absent (the runtime image installs neither)."""
    try:
        import jax.numpy  # noqa: F401
    except Exception:
        j = types.ModuleType("jax")
        j.numpy = types.ModuleType("jax.numpy")
        sys.modules["jax"], sys.modules["jax.numpy"] = j, j.numpy
    try:
        import torch  # noqa: F401
    except Exception:
        t = types.ModuleType("torch")
        t.nn = types.ModuleType("torch.nn")
        t.nn.Module = object
        sys.modules["torch"], sys.modules["torch.nn"] = t, t.nn


# ------------------------------------------------------------------------------------------------------------------
# Unitree H1 locomotion policy (unitree_rl_gym deploy/pre_train/h1/motion.pt), numpy inference
# ------------------------------------------------------------------------------------------------------------------


def load_torchscript_weights(path: str) -> dict[str, np.ndarray]:
    """Tensors of a TorchScript archive by state-dict name, without torch: data.pkl is unpickled with stand-ins for the
    torch classes and each tensor is rebuilt from its raw storage file (little-endian)."""
    z = zipfile.ZipFile(path)
    root = z.namelist()[0].split("/")[0]

    class Obj:
        def __init__(self, *a):
            self.args = a

        def __setstate__(self, st):
            self.state = st

    def rebuild(storage, offset, size, stride, *_):
        key, dtype = storage
        buf = np.frombuffer(z.read(f"{root}/data/{key}"), dtype=dtype)
        return np.lib.stride_tricks.as_strided(
            buf[offset:], shape=size, strides=[s * buf.itemsize for s in stride]
        ).copy()

    class U(pickle.Unpickler):
        def find_class(self, mod, name):
            if name == "_rebuild_tensor_v2":
                return rebuild
            if mod == "collections" and name == "OrderedDict":
                import collections

                return collections.OrderedDict
            return type(name, (Obj,), {})

        def persistent_load(self, pid):
            kind = getattr(pid[1], "__name__", str(pid[1]))
            return pid[2], {"FloatStorage": "<f4", "DoubleStorage": "<f8", "LongStorage": "<i8"}[kind]

    top = U(z.open(f"{root}/data.pkl")).load()
    out: dict[str, np.ndarray] = {}

    def walk(o, pre):
        st = getattr(o, "state", None)
        if isinstance(st, dict):
            for k, v in st.items():
                if isinstance(v, np.ndarray):
                    out[pre + k] = v
                elif isinstance(v, Obj):
                    walk(v, pre + k + ".")

    walk(top, "")
    return out


class H1WalkPolicy:
    """unitree_rl_gym's H1 policy (LSTM 41 -> 64, MLP 64 -> 32 -> 10 with ELU), as in its deploy_mujoco.py."""

    LEGS = [
        "left_hip_yaw",
        "left_hip_roll",
        "left_hip_pitch",
        "left_knee",
        "left_ankle",
        "right_hip_yaw",
        "right_hip_roll",
        "right_hip_pitch",
        "right_knee",
        "right_ankle",
    ]
    DEFAULT = np.array([0, 0, -0.1, 0.3, -0.2] * 2)
    CMD_SCALE = np.array([2.0, 2.0, 0.25])
    ACTION_SCALE = 0.25
    PERIOD = 0.8

    def __init__(self, path: str):
        w = load_torchscript_weights(path)
        self.Wih, self.Whh = w["memory.weight_ih_l0"].astype(np.float64), w["memory.weight_hh_l0"].astype(np.float64)
        self.b = (w["memory.bias_ih_l0"] + w["memory.bias_hh_l0"]).astype(np.float64)
        self.W0, self.b0 = w["actor.0.weight"].astype(np.float64), w["actor.0.bias"].astype(np.float64)
        self.W2, self.b2 = w["actor.2.weight"].astype(np.float64), w["actor.2.bias"].astype(np.float64)
        self.reset()

    def reset(self):
        self.h = np.zeros(64)
        self.c = np.zeros(64)
        self.action = np.zeros(10)
        self.k = 0

    def __call__(self, quat, omega_body, q, dq, cmd) -> np.ndarray:
        qw, qx, qy, qz = quat
        grav = np.array([2 * (-qz * qx + qw * qy), -2 * (qz * qy + qw * qx), 1 - 2 * (qw * qw + qz * qz)])
        ph = (self.k * 0.02) % self.PERIOD / self.PERIOD
        obs = (
            np.concatenate(
                [
                    omega_body * 0.25,
                    grav,
                    np.asarray(cmd) * self.CMD_SCALE,
                    q - self.DEFAULT,
                    dq * 0.05,
                    self.action,
                    [math.sin(2 * math.pi * ph), math.cos(2 * math.pi * ph)],
                ]
            )
            .astype(np.float32)
            .astype(np.float64)
        )
        g = self.Wih @ obs + self.Whh @ self.h + self.b
        i, f, gg, o = np.split(g, 4)
        sig = lambda x: 1 / (1 + np.exp(-x))
        self.c = sig(f) * self.c + sig(i) * np.tanh(gg)
        self.h = sig(o) * np.tanh(self.c)
        x = self.W0 @ self.h + self.b0
        x = np.where(x > 0, x, np.expm1(np.minimum(x, 0)))
        self.action = self.W2 @ x + self.b2
        self.k += 1
        return self.action * self.ACTION_SCALE + self.DEFAULT


# ------------------------------------------------------------------------------------------------------------------
# tasks: HumanoidBench env id, success rule, public scene facts
# ------------------------------------------------------------------------------------------------------------------

# success: "return" = the episode return reaches HumanoidBench's success_bar for the task (its published success
# threshold); "flag" = HumanoidBench's own `success` signal in the step info fires.
TASKS = {
    "stand": dict(env="h1hand-stand-v0", success="return"),
    "walk": dict(env="h1hand-walk-v0", success="return"),
    "maze": dict(env="h1hand-maze-v0", success="return"),
    "pole": dict(env="h1hand-pole-v0", success="return"),
    "sit_simple": dict(env="h1hand-sit_simple-v0", success="return"),
    "reach": dict(env="h1hand-reach-v0", success="return"),
    "push": dict(env="h1hand-push-v0", success="flag"),
}

ARM = {
    "left": ["left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw", "left_elbow"],
    "right": ["right_shoulder_pitch", "right_shoulder_roll", "right_shoulder_yaw", "right_elbow"],
}
SIDES = ("left", "right")
SIT_POSE = np.array([0, 0, -1.3, 1.6, 0.15] * 2)  # sit-down target: hip yaw, roll, pitch, knee, ankle (both legs)
SIT_HOLD, SIT_STEPS = 2, 30  # steps in the standing stance, then steps to the seated pose
MAX_CMD = np.array([1.0, 0.5, 1.0])  # forward m/s, left m/s, turn rad/s (inside the policy's training ranges)
HAND_STEP = 0.02  # m per step per unit of the hand action
ARM_KP = {"left_shoulder_pitch": 100, "left_shoulder_roll": 100, "left_shoulder_yaw": 100, "left_elbow": 100}


def _r(x, n=3):
    return round(float(x), n)


def _r3(v, n=3):
    return [round(float(x), n) for x in v]


def _yaw(quat) -> float:
    w, x, y, z = quat
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


class Worker:
    def __init__(self):
        self.env = None
        self.renderers = {}

    # ---- episode -------------------------------------------------------------------------------------------------
    def make(self, task: str, seed: int, policy: str, noise_seed: int | None = None):
        _stub_modules()
        import gymnasium as gym
        import humanoid_bench  # noqa: F401  (registers the h1hand-*-v0 environments)
        import mujoco

        self.mj = mujoco
        if self.env is not None:
            self.env.close()
        self.task_id, self.cfg = task, TASKS[task]
        # HumanoidBench samples some task layouts from numpy's global generator (push goal, reach target, package spots)
        np.random.seed(seed)
        self.env = gym.make(self.cfg["env"], render_mode=None)
        self.u = self.env.unwrapped
        # noise_seed (tests only): reseed HumanoidBench's start-state noise without changing the layout drawn above
        self.env.reset(seed=seed if noise_seed is None else int(noise_seed))
        m, d = self.model, self.data = self.u.model, self.u.data
        self.hb_task = self.u.task
        self.max_episode_steps = int(self.env.spec.max_episode_steps)
        self.bar = float(getattr(self.hb_task, "success_bar", 0.0))
        self.policy = H1WalkPolicy(policy)
        jq = lambda n: int(m.jnt_qposadr[m.joint(n).id])
        jv = lambda n: int(m.jnt_dofadr[m.joint(n).id])
        self.leg_q = [jq(n) for n in H1WalkPolicy.LEGS]
        self.leg_v = [jv(n) for n in H1WalkPolicy.LEGS]
        self.act_of = {m.actuator(i).name: i for i in range(m.nu)}
        self.leg_a = [self.act_of[n] for n in H1WalkPolicy.LEGS]
        self.arm_q = {s: [jq(n) for n in ARM[s]] for s in SIDES}
        self.arm_v = {s: [jv(n) for n in ARM[s]] for s in SIDES}
        self.arm_a = {s: [self.act_of[n] for n in ARM[s]] for s in SIDES}
        self.arm_kp = {s: np.array([m.actuator_gainprm[a, 0] for a in self.arm_a[s]]) for s in SIDES}
        self.arm_lo = {s: np.array([m.actuator_ctrlrange[a, 0] for a in self.arm_a[s]]) for s in SIDES}
        self.arm_hi = {s: np.array([m.actuator_ctrlrange[a, 1] for a in self.arm_a[s]]) for s in SIDES}
        self.hand_site = {s: m.site(f"{s}_hand").id for s in SIDES}
        # every actuator holds its start position unless the controller moves it
        self.ctrl = np.array([d.qpos[m.jnt_qposadr[m.actuator_trnid[i, 0]]] for i in range(m.nu)])
        # HumanoidBench's keyframes start with bent knees (-0.4, 0.8, -0.4); the policy starts from its own default
        # stance, so the legs are set to it before the first step (a static pose change, no motion)
        d.qpos[self.leg_q] = H1WalkPolicy.DEFAULT
        # arms start neutral (upper arm down, forearm forward, wrist straight); some HumanoidBench keyframes raise an arm
        # backwards (push: right shoulder pitch 1.57), from where the arm IK cannot bring the hand forward past the body
        for sd in SIDES:
            d.qpos[self.arm_q[sd]] = np.clip(0.0, self.arm_lo[sd], self.arm_hi[sd])
            wj = f"{sd}_wrist_yaw"
            if wj in self.act_of:
                d.qpos[jq(wj)] = 0.0
                self.ctrl[self.act_of[wj]] = 0.0
        d.qvel[:] = 0
        mujoco.mj_forward(m, d)
        self.scratch = mujoco.MjData(m)
        self.q_arm = {s: d.qpos[self.arm_q[s]].copy() for s in SIDES}
        self.q_arm_home = {s: self.q_arm[s].copy() for s in SIDES}
        self.hand_goal = {s: self.hand_body(s) for s in SIDES}
        self.hand_home = {s: self.hand_goal[s].copy() for s in SIDES}
        self.ibias = {s: np.zeros(3) for s in SIDES}
        self.anchor = {s: None for s in SIDES}  # a world point the hand keeps tracking (set by the reach skill)
        self.cmd = np.zeros(3)
        self.hold = None  # (pelvis xy, heading) held while the velocity command is zero
        self.sit_k = None  # steps since sit_down started (None: walking controller active)
        self.hold_moving = False
        self.steps = 0
        self.ret = 0.0
        self.last_reward = 0.0
        self.info: dict = {}
        self.success = False
        self.success_step = None
        self.terminated = False
        self.best = {}
        return {
            "max_episode_steps": self.max_episode_steps,
            "success_bar": self.bar,
            "success": self.cfg["success"],
            "env": self.cfg["env"],
            "camera": self.hb_task.camera_name,
            "hand_home_body": {sd: _r3(self.hand_home[sd]) for sd in SIDES},
            "cameras": [m.camera(i).name for i in range(m.ncam)],
        }

    # ---- frames --------------------------------------------------------------------------------------------------
    def base(self):
        """Pelvis position and heading (yaw) in the world."""
        d = self.data
        return d.qpos[:3].copy(), _yaw(d.qpos[3:7])

    def to_body(self, p):
        """World point -> heading frame (x forward, y left, origin on the floor under the pelvis, z = height)."""
        b, yaw = self.base()
        c, s = math.cos(yaw), math.sin(yaw)
        dx, dy = p[0] - b[0], p[1] - b[1]
        return np.array([c * dx + s * dy, -s * dx + c * dy, p[2]])

    def to_world(self, p):
        b, yaw = self.base()
        c, s = math.cos(yaw), math.sin(yaw)
        return np.array([b[0] + c * p[0] - s * p[1], b[1] + s * p[0] + c * p[1], p[2]])

    def hand_world(self, side):
        return self.data.site_xpos[self.hand_site[side]].copy()

    def hand_body(self, side):
        return self.to_body(self.hand_world(side))

    # ---- arm inverse kinematics ------------------------------------------------------------------------------------
    def arm_ik(self, side, goal_world, q0, iters=12):
        m, s = self.model, self.scratch
        s.qpos[:] = self.data.qpos
        qa, va = self.arm_q[side], self.arm_v[side]
        lo, hi = self.arm_lo[side], self.arm_hi[side]
        q = q0.copy()
        home = np.clip(np.zeros(4), lo, hi)  # posture pulled toward in the null space: upper arm down, forearm forward
        jac = np.zeros((3, m.nv))
        err = np.zeros(3)
        for _ in range(iters):
            s.qpos[qa] = q
            self.mj.mj_kinematics(m, s)
            self.mj.mj_comPos(m, s)
            x = s.site_xpos[self.hand_site[side]]
            err = goal_world - x
            if float(np.linalg.norm(err)) < 1e-3:
                break
            self.mj.mj_jacSite(m, s, jac, None, self.hand_site[side])
            J = jac[:, va]
            lam = 0.02
            JJt = J @ J.T + lam * np.eye(3)
            dq = J.T @ np.linalg.solve(JJt, err)
            # null-space pull toward the start posture keeps the elbow natural
            N = np.eye(4) - J.T @ np.linalg.solve(JJt, J)
            dq += N @ (0.05 * (home - q))
            q = np.clip(q + np.clip(dq, -0.2, 0.2), lo, hi)
        return q, float(np.linalg.norm(err))

    def sit_down(self):
        """Switch the legs from the walking policy to a scripted sit-down: both legs to the policy's standing stance (the
        swing foot lands), then hips and knees bend to about 90 degrees over 0.6 s, lowering the pelvis about 0.3 m and
        moving it about 0.2 m back; the robot stays seated (no way back to walking)."""
        if self.sit_k is None:
            self.sit_k = 0
            self.sit_from = H1WalkPolicy.DEFAULT.copy()
        return {"sitting": True}

    def set_anchor(self, side, world=None):
        self.anchor[side] = None if world is None else np.asarray(world, float)
        return {"anchor": None if world is None else _r3(world)}

    # ---- stepping --------------------------------------------------------------------------------------------------
    def step(self, action):
        a = np.asarray(action, float)
        if self.terminated or self.steps >= self.max_episode_steps:
            return self._summary()
        _m, d = self.model, self.data
        self.cmd = np.clip(a[:3], -1, 1) * MAX_CMD
        cmd = self.cmd.copy()
        if not np.any(self.cmd):
            # zero command = hold this spot: the policy steps in place and drifts a few cm/s, so a slow proportional
            # correction pulls the pelvis back to where the robot stopped and keeps its heading
            b, yaw = self.base()
            if self.hold is None:
                self.hold, self.hold_moving = (b[:2].copy(), yaw), False
            e = self.to_body([self.hold[0][0], self.hold[0][1], 0.0])[:2]
            eh = (self.hold[1] - yaw + math.pi) % (2 * math.pi) - math.pi
            # the policy treats planar commands below 0.2 m/s as "stand" (as in its training), so a correction is a
            # short walk at 0.25-0.35 m/s toward the spot whenever the pelvis is more than 4 cm off (hysteresis to 2 cm)
            n = float(np.linalg.norm(e))
            self.hold_moving = n > (0.02 if self.hold_moving else 0.04)
            v = e / max(n, 1e-9) * float(np.clip(1.5 * n, 0.25, 0.35)) if self.hold_moving else np.zeros(2)
            turn = float(np.clip(1.5 * eh, -0.5, 0.5)) if abs(eh) > math.radians(3) else 0.0
            cmd = np.array([v[0], np.clip(v[1], -0.3, 0.3), turn])
        else:
            self.hold = None
        for i, side in enumerate(SIDES):
            delta = np.clip(a[3 + 3 * i : 6 + 3 * i], -1, 1) * HAND_STEP
            if np.any(delta):
                self.anchor[side] = None
            if self.anchor[side] is not None:  # hold a world point while the body sways
                self.hand_goal[side] = self.to_body(self.anchor[side])
            g = self.hand_goal[side] + delta
            # the goal never runs more than 10 cm ahead of the measured hand (no wind-up against contacts)
            cur = self.hand_body(side)
            off = g - cur
            n = float(np.linalg.norm(off))
            if n > 0.10:
                g = cur + off * 0.10 / n
            self.hand_goal[side] = g
            err = g - cur
            if not np.any(delta) and float(np.linalg.norm(err)) < 0.04:  # slow integral term against sag under load
                self.ibias[side] = np.clip(self.ibias[side] + 0.1 * err, -0.04, 0.04)
            gw = self.to_world(g + self.ibias[side])
            q, res = self.arm_ik(side, gw, self.q_arm[side])
            if (
                res > 0.03
            ):  # stuck (joint limit or local minimum): solve again from the neutral arm and keep the better one
                q2, res2 = self.arm_ik(side, gw, np.clip(np.zeros(4), self.arm_lo[side], self.arm_hi[side]), iters=30)
                if res2 < res - 0.01:
                    q = q2
            self.q_arm[side] = self.q_arm[side] + np.clip(q - self.q_arm[side], -0.08, 0.08)
        # legs: the locomotion policy, or the scripted sit-down motion once it has started
        if self.sit_k is None:
            legs = self.policy(d.qpos[3:7], d.qvel[3:6], d.qpos[self.leg_q], d.qvel[self.leg_v], cmd)
        else:
            a_sit = min(1.0, max(0.0, (self.sit_k - SIT_HOLD) / SIT_STEPS))
            legs = self.sit_from + (SIT_POSE - self.sit_from) * a_sit
            self.sit_k += 1
        self.ctrl[self.leg_a] = legs
        for side in SIDES:
            # gravity (and velocity-product) feed-forward so the position servos hold the arm where IK put it
            ff = d.qfrc_bias[self.arm_v[side]] / self.arm_kp[side]
            self.ctrl[self.arm_a[side]] = np.clip(self.q_arm[side] + ff, self.arm_lo[side], self.arm_hi[side])
        act = self.hb_task.normalize_action(self.ctrl)
        _, rew, term, trunc, info = self.env.step(np.clip(act, -1, 1))
        self.steps += 1
        self.last_reward = float(rew)
        self.ret += float(rew)
        self.info = {k: (float(v) if np.isscalar(v) else v) for k, v in info.items() if np.isscalar(v)}
        if self.cfg["success"] == "return":
            hit = self.ret >= self.bar
        else:
            hit = bool(info.get("success", False))
        if hit and not self.success:
            self.success, self.success_step = True, self.steps
        if term:
            self.terminated = True
        return self._summary()

    def _summary(self):
        return {"steps": self.steps, "return": _r(self.ret, 2), "success": self.success, "terminated": self.terminated}

    # ---- observation -----------------------------------------------------------------------------------------------
    def observe(self):
        _m, d = self.model, self.data
        b, yaw = self.base()
        rob = self.u.robot
        v = rob.center_of_mass_velocity()
        c, s = math.cos(yaw), math.sin(yaw)
        vb = [c * v[0] + s * v[1], -s * v[0] + c * v[1]]
        hands = {}
        for side in SIDES:
            hands[side] = {
                "pos": _r3(self.hand_world(side)),
                "pos_body": _r3(self.hand_body(side)),
                "goal_body": _r3(self.hand_goal[side]),
                "anchored_at": None if self.anchor[side] is None else _r3(self.anchor[side]),
            }
        robot = {
            "pelvis": _r3(b),
            "heading_deg": _r(math.degrees(yaw), 1),
            "head_height": _r(rob.head_height()),
            "torso_upright": _r(rob.torso_upright()),
            "com_velocity": _r3(v[:2]),
            "com_velocity_body": _r3(vb),
            "command": {"forward": _r(self.cmd[0], 2), "left": _r(self.cmd[1], 2), "turn": _r(self.cmd[2], 2)},
            "holding_spot": None if self.hold is None else _r3(self.hold[0], 2),
            "hands": hands,
            "fallen": self.terminated,
            "seated_mode": self.sit_k is not None,
        }
        st = {"robot": robot, "scene": self.scene(), "time_s": _r(d.time, 2)}
        st["progress"] = {
            "steps": self.steps,
            "episode_steps": self.max_episode_steps,
            "return": _r(self.ret, 2),
            "last_reward": _r(self.last_reward, 4),
            **({"success_bar": self.bar} if self.cfg["success"] == "return" else {}),
            "reward_terms": {
                k: _r(v, 3) for k, v in self.info.items() if k not in ("per_timestep_reward",) and isinstance(v, float)
            },
            "solved": self.success,
        }
        return st

    def scene(self) -> dict:
        _m, d, t = self.model, self.data, self.task_id
        if t == "push":
            box = d.qpos[-7:-4]
            return {
                "table": {"top_height": 0.95, "x_range": [0.4, 1.4], "y_range": [-1.0, 1.0]},
                "box": {"pos": _r3(box), "size_m": 0.2},
                "goal": _r3(self.hb_task.goal),
                "box_to_goal_m": _r(np.linalg.norm(box - self.hb_task.goal)),
            }
        if t == "reach":
            g = self.hb_task.goal
            return {"target": _r3(g), "left_hand_to_target_m": _r(np.linalg.norm(self.hand_world("left") - g))}
        if t == "maze":
            cps = [c[:2].tolist() for c in self.hb_task.checkpoints[1:4]]
            return {
                "checkpoints": cps,
                "checkpoints_reached": int(self.hb_task.maze_stage),
                "next_checkpoint": (cps[self.hb_task.maze_stage] if self.hb_task.maze_stage < 3 else None),
                "walls": self._walls(),
            }
        if t == "pole":
            return {"poles": self._poles(), "corridor_y": [-3.0, 3.0]}
        if t == "sit_simple":
            ch = d.body("chair").xpos
            return {"chair": {"pos": _r3(ch), "seat_height": 0.475, "seat_size_m": [0.42, 0.42], "backrest": "at -x"}}
        return {}

    def _walls(self):
        m, d = self.model, self.data
        out = []
        for g in range(m.ngeom):
            n = m.geom(g).name
            if n.startswith("block_collision") or (
                m.geom_bodyid[g] and m.body(m.geom_bodyid[g]).name.startswith("wall")
            ):
                p, sz = d.geom_xpos[g], m.geom_size[g]
                out.append(
                    {"x": [_r(p[0] - sz[0], 2), _r(p[0] + sz[0], 2)], "y": [_r(p[1] - sz[1], 2), _r(p[1] + sz[1], 2)]}
                )
        return out

    def _poles(self):
        m, d = self.model, self.data
        return [_r3(d.geom_xpos[g][:2], 2) for g in range(m.ngeom) if m.geom(g).name.startswith("pole_r")]

    def judge(self):
        return {
            "success": self.success,
            "success_step": self.success_step,
            "return": _r(self.ret, 2),
            "success_rule": self.cfg["success"],
            "success_bar": self.bar,
            "steps": self.steps,
            "terminated": self.terminated,
            "last_info": {k: _r(v, 4) for k, v in self.info.items() if isinstance(v, float)},
        }

    # ---- rendering -------------------------------------------------------------------------------------------------
    def render(self, width, height, camera):
        key = (width, height)
        r = self.renderers.get(key)
        if r is None:
            m = self.model
            m.vis.global_.offwidth, m.vis.global_.offheight = (
                max(m.vis.global_.offwidth, width),
                max(m.vis.global_.offheight, height),
            )
            r = self.renderers[key] = self.mj.Renderer(self.model, height=height, width=width)
        cam = self.model.camera(camera).id if camera else -1
        r.update_scene(self.data, camera=cam)
        # goal markers HumanoidBench draws in its own viewer (visual only): the push goal, the reach target
        marks = []
        if self.task_id == "push":
            marks.append((self.hb_task.goal, 0.05, (0.8, 0.28, 0.28, 0.6)))
        if self.task_id == "reach":
            marks.append((self.hb_task.goal, 0.05, (0.8, 0.28, 0.28, 0.8)))
        for pos, size, rgba in marks:
            if r.scene.ngeom < r.scene.maxgeom:
                self.mj.mjv_initGeom(
                    r.scene.geoms[r.scene.ngeom],
                    self.mj.mjtGeom.mjGEOM_SPHERE,
                    np.array([size, size, size]),
                    np.asarray(pos, float),
                    np.eye(3).ravel(),
                    np.asarray(rgba, np.float32),
                )
                r.scene.ngeom += 1
        return r.render().copy()

    def close(self):
        for r in self.renderers.values():
            r.close()
        if self.env is not None:
            self.env.close()
