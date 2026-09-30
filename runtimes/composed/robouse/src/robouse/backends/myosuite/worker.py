"""MyoSuite simulation worker. Runs inside the separate MyoSuite virtualenv (Python 3.11, myosuite 2.12.2 with its pinned
MuJoCo 3.6.0 and gymnasium 1.2.3, scipy), because those pins conflict with the main robouse environment.

The robouse `myosuite` backend (backend.py) starts this script as a subprocess and talks to it over stdin/stdout:
one JSON request per line, one JSON response per line; a `render` response is followed by the raw RGB bytes. Only numpy,
scipy, mujoco and myosuite are imported here (never the robouse package).

Each episode is one registered MyoSuite environment (gymnasium id, e.g. myoHandPose4Fixed-v0), reset with the task's
seed. An action is one muscle excitation in [0, 1] per muscle, in the model's actuator order; the worker converts it to
MyoSuite's normalised action (the env maps an action a to the excitation 1 / (1 + exp(-5 (a - 0.5)))), so the muscle
controls MuJoCo receives are exactly the excitations sent. One step = one env step (MyoSuite's own frame skip).

Success is MyoSuite's own `solved` term of the env's reward dictionary, computed by the env from the current state.

The muscle-space controller behind the `set_joint_targets` / `reach_tips` skills (`control`): each step it asks for the
joint acceleration of a PD law towards the (ramped) joint targets, turns it into the joint torques that produce it with
MuJoCo's inverse dynamics, and solves a bounded least-squares problem for the muscle activations in [0, 1] whose
forces (MuJoCo's muscle model: force = gain(length, velocity) * activation + bias) best produce those torques, then
picks the excitations that bring the muscles' activation dynamics to those activations within the step. This is the
inverse-dynamics approach of MyoSuite's own tutorial (tutorials/6_Inverse_Dynamics.ipynb) run as a feedback loop.
"""

from __future__ import annotations

import os
import sys

os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault(
    "GIT_PYTHON_REFRESH", "quiet"
)  # myosuite imports gitpython; the runtime image has no git executable
os.environ.setdefault("MUJOCO_GL", "egl" if sys.platform.startswith("linux") else "glfw")

import numpy as np

KP, KD, REG = 1000.0, 60.0, 1e-5  # PD gains (1/s^2, 1/s) and activation regulariser of the muscle-space controller
EXC_EPS = 1e-4


def exc_to_action(e: np.ndarray) -> np.ndarray:
    """Excitation in [0, 1] -> MyoSuite's normalised muscle action (inverse of its sigmoid)."""
    e = np.clip(np.asarray(e, dtype=np.float64), EXC_EPS, 1 - EXC_EPS)
    return 0.5 + np.log(e / (1 - e)) / 5.0


def r(v, n=4):
    return [round(float(x), n) for x in np.ravel(v)]


class Worker:
    def __init__(self):
        self.env = None

    # ---- setup ---------------------------------------------------------------------------------------------------
    def make(self, env_id: str, seed: int = 0, camera: dict | None = None) -> dict:
        import mujoco
        import myosuite  # noqa: F401  (registers the envs)
        from myosuite.utils import gym

        self.env_id = env_id
        self.env = gym.make(env_id, seed=int(seed))  # MyoSuite seeds its generator in the constructor, not in reset()
        self.u = self.env.unwrapped
        self.env.reset()
        self.m, self.d = self.u.mj_model, self.u.mj_data
        m = self.m
        cls = type(self.u).__name__
        self.kind = {
            "PoseEnvV0": "pose",
            "ReachEnvV0": "reach",
            "KeyTurnEnvV0": "keyturn",
            "ObjHoldFixedEnvV0": "objhold",
            "ObjHoldRandomEnvV0": "objhold",
        }.get(cls, "other")
        self.muscles = [m.actuator(i).name for i in range(m.nu)]
        # joints moved by muscles (the hand / finger / elbow joints; not the key or a free object)
        mujoco.mj_forward(m, self.d)
        R = self._moment(self.d)
        self.jids = [j for j in range(m.njnt) if m.jnt_type[j] in (2, 3) and np.abs(R[:, m.jnt_dofadr[j]]).max() > 1e-6]
        self.jnames = [m.joint(j).name for j in self.jids]
        self.qadr = np.array([m.jnt_qposadr[j] for j in self.jids])
        self.dadr = np.array([m.jnt_dofadr[j] for j in self.jids])
        self.jlo = m.jnt_range[self.jids, 0].copy()
        self.jhi = m.jnt_range[self.jids, 1].copy()
        self.targets = self.d.qpos[self.qadr].copy()
        self.ramp_from = self.targets.copy()
        self.ramp_left = 0
        self.ramp_len = 1
        self.last_exc = np.zeros(m.nu)
        self.scratch = mujoco.MjData(m)
        self._renderers = {}
        self.cam = camera or {}
        self.u.forward()
        tips = []
        if self.kind == "reach":
            tips = [m.site(int(s)).name for s in self.u.tip_sids]
        return {
            "kind": self.kind,
            "muscles": self.muscles,
            "joints": self.jnames,
            "joint_ranges": {
                n: [round(float(lo), 4), round(float(hi), 4)]
                for n, lo, hi in zip(self.jnames, self.jlo, self.jhi, strict=False)
            },
            "dt": float(self.u.dt),
            "tips": tips,
            "upstream_max_steps": int(self.env.spec.max_episode_steps or 0),
            "obs_keys": list(self.u.obs_keys),
            "obs_dim": int(np.size(self.u.get_obs())),
        }

    def _moment(self, d) -> np.ndarray:
        import mujoco

        R = np.zeros((self.m.nu, self.m.nv))
        mujoco.mju_sparse2dense(R, d.actuator_moment, d.moment_rownnz, d.moment_rowadr, d.moment_colind)
        return R

    # ---- stepping ------------------------------------------------------------------------------------------------
    def step(self, action) -> dict:
        e = np.clip(np.asarray(action, dtype=np.float64), 0.0, 1.0)
        if e.shape != (self.m.nu,):
            raise ValueError(f"action must have {self.m.nu} muscle excitations")
        self.last_exc = e
        self.env.step(exc_to_action(e))
        return {"solved": bool(self.u.rwd_dict["solved"])}

    def success(self) -> dict:
        self.u.forward()  # recompute MyoSuite's obs and reward dictionaries from the current state (no stepping)
        rd = self.u.rwd_dict
        return {"success": bool(rd["solved"]), "detail": self._metric()}

    # ---- muscle-space controller ---------------------------------------------------------------------------------
    def set_targets(self, targets: dict, ramp: int = 20, from_current: bool = False) -> dict:
        """Update the joint targets (the named joints; `from_current` first sets every target to the current angle)."""
        if from_current:
            self.targets = self.d.qpos[self.qadr].copy()
        unknown = [k for k in targets if k not in self.jnames]
        if unknown:
            raise ValueError(f"unknown joint(s) {unknown}; joints: {self.jnames}")
        new = self.targets.copy()
        clipped = {}
        for k, v in targets.items():
            i = self.jnames.index(k)
            c = float(np.clip(float(v), self.jlo[i], self.jhi[i]))
            if abs(c - float(v)) > 1e-9:
                clipped[k] = round(c, 4)
            new[i] = c
        self.ramp_from = self.d.qpos[self.qadr].copy()
        self.targets = new
        self.ramp_len = max(1, int(ramp))
        self.ramp_left = self.ramp_len
        return {
            "targets": {n: round(float(t), 4) for n, t in zip(self.jnames, self.targets, strict=False)},
            "clipped_to_range": clipped,
        }

    def control(self) -> dict:
        """Excitations for the next step that drive the joints towards the current (ramped) targets."""
        import mujoco
        from scipy.optimize import lsq_linear

        m, d, s = self.m, self.d, self.scratch
        if self.ramp_left > 0:
            f = 1.0 - (self.ramp_left - 1) / self.ramp_len
            self.ramp_left -= 1
        else:
            f = 1.0
        qref = self.ramp_from + f * (self.targets - self.ramp_from)
        mujoco.mj_copyData(s, m, d)
        mujoco.mj_forward(m, s)
        qacc = np.zeros(m.nv)
        qacc[self.dadr] = KP * (qref - d.qpos[self.qadr]) - KD * d.qvel[self.dadr]
        s.qacc[:] = qacc
        dis = int(mujoco.mjtDisableBit.mjDSBL_CONSTRAINT)
        old = m.opt.disableflags
        m.opt.disableflags = old | dis
        try:
            mujoco.mj_inverse(m, s)
        finally:
            m.opt.disableflags = old
        tau = s.qfrc_inverse.copy()
        s.act[:] = 0.0
        mujoco.mj_forward(m, s)
        f0 = s.actuator_force.copy()
        R = self._moment(s)
        s.act[:] = 1.0
        mujoco.mj_forward(m, s)
        g = s.actuator_force - f0
        A = (R.T * g)[self.dadr]
        b = (tau - R.T @ f0)[self.dadr]
        n = m.nu
        res = lsq_linear(
            np.vstack([A, np.sqrt(REG) * np.eye(n)]), np.concatenate([b, np.zeros(n)]), bounds=(0.0, 1.0), method="bvls"
        )
        a = np.clip(res.x, 0.0, 1.0)
        # excitation that brings MuJoCo's first-order activation dynamics to `a` about half-way through the step
        act = d.act.copy()
        t_act = m.actuator_dynprm[:, 0] * (0.5 + 1.5 * act)
        t_deact = m.actuator_dynprm[:, 1] / (0.5 + 1.5 * act)
        tc = np.where(a > act, t_act, t_deact)
        ee = np.exp(-(0.5 * self.u.dt) / tc)
        exc = np.clip((a - act * ee) / (1 - ee), 0.0, 1.0)
        err = self.targets - d.qpos[self.qadr]
        return {
            "excitations": r(exc, 5),
            "target_error_norm": round(float(np.linalg.norm(err)), 4),
            "max_speed": round(float(np.abs(d.qvel[self.dadr]).max()), 3),
            "ramping": self.ramp_left > 0,
        }

    def ik(self, tips: dict, iters: int = 300) -> dict:
        """Joint targets that put the named fingertip sites at the given world positions (damped least squares
        within the joint ranges, starting from the current pose; the other joints stay near their current values)."""
        import mujoco

        m, s = self.m, self.scratch
        names = list(tips)
        for t in names:
            if mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, t) < 0:
                raise ValueError(f"unknown fingertip {t!r}")
        sids = [m.site(t).id for t in names]
        goal = np.array([tips[t] for t in names], dtype=np.float64)
        mujoco.mj_copyData(s, m, self.d)
        q = s.qpos.copy()
        e = np.zeros(1)
        for _ in range(iters):
            s.qpos[:] = q
            mujoco.mj_kinematics(m, s)
            mujoco.mj_comPos(m, s)
            errs, Js = [], []
            for sid, gpos in zip(sids, goal, strict=False):
                errs.append(gpos - s.site_xpos[sid])
                J = np.zeros((3, m.nv))
                mujoco.mj_jacSite(m, s, J, None, sid)
                Js.append(J[:, self.dadr])
            e = np.concatenate(errs)
            if np.linalg.norm(e) < 1e-5:
                break
            J = np.vstack(Js)
            dq = J.T @ np.linalg.solve(J @ J.T + 1e-4 * np.eye(len(e)), e)
            q[self.qadr] = np.clip(q[self.qadr] + dq, self.jlo, self.jhi)
        return {
            "joint_targets": {n: round(float(v), 4) for n, v in zip(self.jnames, q[self.qadr], strict=False)},
            "residual_m": {t: round(float(np.linalg.norm(e[3 * i : 3 * i + 3])), 4) for i, t in enumerate(names)},
        }

    # ---- observation ---------------------------------------------------------------------------------------------
    def _metric(self) -> dict:
        u, d = self.u, self.d
        od = u.get_obs_dict(self.m, d)
        if self.kind == "pose":
            return {"pose_error_norm": round(float(np.linalg.norm(od["pose_err"])), 4), "threshold": float(u.pose_thd)}
        if self.kind == "reach":
            return {
                "reach_error_norm": round(float(np.linalg.norm(od["reach_err"])), 4),
                "threshold": round(0.0125 * len(u.tip_sids), 4),
            }
        if self.kind == "keyturn":
            return {"key_angle_rad": round(float(d.qpos[-1]), 4), "goal_angle_rad": round(float(u.goal_th), 4)}
        if self.kind == "objhold":
            return {"object_error_m": round(float(np.linalg.norm(od["obj_err"])), 4), "threshold_m": 0.01}
        return {}

    def observe(self) -> dict:
        import mujoco

        m, d, u = self.m, self.d, self.u
        out = {
            "joint_angles_rad": {n: round(float(v), 4) for n, v in zip(self.jnames, d.qpos[self.qadr], strict=False)},
            "joint_velocities_rad_s": {
                n: round(float(v), 3) for n, v in zip(self.jnames, d.qvel[self.dadr], strict=False)
            },
            "muscle_activations": r(d.act, 3),
            "joint_targets_rad": {n: round(float(v), 4) for n, v in zip(self.jnames, self.targets, strict=False)},
        }
        u.get_obs_dict(m, d)
        if self.kind == "pose":
            tgt = np.asarray(u.target_jnt_value, dtype=np.float64)
            out["target_pose_rad"] = {n: round(float(tgt[a]), 4) for n, a in zip(self.jnames, self.qadr, strict=False)}
        elif self.kind == "reach":
            for i, sid in enumerate(u.tip_sids):
                nm = m.site(int(sid)).name
                out.setdefault("fingertips", {})[nm] = r(d.site_xpos[sid])
                out.setdefault("fingertip_targets", {})[nm] = r(d.site_xpos[u.target_sids[i]])
        elif self.kind == "keyturn":
            out["key_angle_rad"] = round(float(d.qpos[-1]), 4)
            out["key_goal_angle_rad"] = round(float(u.goal_th), 4)
            out["key_head"] = r(d.site_xpos[u.keyhead_sid])
            out["index_tip"] = r(d.site_xpos[u.IF_sid])
            out["thumb_tip"] = r(d.site_xpos[u.TH_sid])
        elif self.kind == "objhold":
            out["object_pos"] = r(d.site_xpos[u.object_sid])
            out["goal_pos"] = r(d.site_xpos[u.goal_sid])
        out.update(self._metric())
        out["contacts"] = sorted(
            {
                "-".join(sorted((m.body(int(m.geom_bodyid[c.geom1])).name, m.body(int(m.geom_bodyid[c.geom2])).name)))
                for c in d.contact[: d.ncon]
            }
        )
        out["time_s"] = round(float(d.time), 3)
        out["obs_vector"] = [
            float(x) for x in np.ravel(u.get_obs())
        ]  # MyoSuite's own observation vector (its obs_keys, in order), full precision
        _ = mujoco
        return out

    # ---- rendering -----------------------------------------------------------------------------------------------
    def render(self, width: int, height: int) -> np.ndarray:
        import mujoco

        key = (width, height)
        if key not in self._renderers:
            self.m.vis.global_.offwidth = max(self.m.vis.global_.offwidth, width)
            self.m.vis.global_.offheight = max(self.m.vis.global_.offheight, height)
            self._renderers[key] = mujoco.Renderer(self.m, height=height, width=width)
        rr = self._renderers[key]
        cam = mujoco.MjvCamera()
        cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        c = self.cam
        if "lookat_site" in c:
            cam.lookat[:] = self.d.site_xpos[self.m.site(c["lookat_site"]).id]
        elif "lookat_body" in c:
            cam.lookat[:] = self.d.xpos[self.m.body(c["lookat_body"]).id]
        else:
            cam.lookat[:] = c.get("lookat", self.m.stat.center)
        cam.lookat[:] = cam.lookat + np.asarray(c.get("offset", [0, 0, 0]), dtype=float)
        cam.distance = float(c.get("distance", 0.5))
        cam.azimuth = float(c.get("azimuth", 90.0))
        cam.elevation = float(c.get("elevation", -20.0))
        opt = mujoco.MjvOption()
        opt.flags[mujoco.mjtVisFlag.mjVIS_TENDON] = (
            0  # hide the muscle-tendon paths: the bones and targets stay visible
        )
        rr.update_scene(self.d, camera=cam, scene_option=opt)
        return np.ascontiguousarray(rr.render(), dtype=np.uint8)
