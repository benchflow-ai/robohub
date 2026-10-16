"""Inverse kinematics of the so101 suite's 5-joint arms: damped least squares on the MJCF (scratch MjData), for the
tool point's position with a fixed top-down pitch (the gripper's approach axis straight down). The wrist roll is not
solved: it keeps its value, so on these 5-joint arms the gripper's yaw about the vertical follows the base angle (and
the roll). Position comes first: the pitch term has a small weight, and where top-down is out of reach the solution
tilts and says by how much."""

from __future__ import annotations

import math

import numpy as np

DOWN = np.array([0.0, 0.0, -1.0])
W_PITCH = 0.05  # m of position error per unit of approach-axis error (about 1 mm per degree)


class ArmIK:
    def __init__(self, model, prefix: str, robot: dict):
        import mujoco  # here, not at module level: the agent container imports this package without MuJoCo

        self.mj = mujoco
        self.m = model
        self.d = mujoco.MjData(model)
        self.robot = robot
        self.jid = [model.joint(f"{prefix}/{j}").id for j in robot["joints"]]
        self.qadr = np.array([model.jnt_qposadr[j] for j in self.jid])
        self.dadr = np.array([model.jnt_dofadr[j] for j in self.jid])
        self.lo = np.radians(robot["low"])
        self.hi = np.radians(robot["high"])
        site = robot["tool_site"] or "tool"
        self.site = model.site(f"{prefix}/{site}").id
        self.body = model.body(f"{prefix}/{robot['tool_body']}").id
        self.approach_local = np.asarray(robot["approach"], float)
        self.jaw_local = np.asarray(robot["jaw"], float)

    def fk(self, qpos_full: np.ndarray, q5: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Tool point, approach axis and jaw axis (world) with the arm's five joints at q5 (rad)."""
        d = self.d
        d.qpos[:] = qpos_full
        d.qpos[self.qadr] = q5
        self.mj.mj_kinematics(self.m, d)
        R = d.xmat[self.body].reshape(3, 3)
        return d.site_xpos[self.site].copy(), R @ self.approach_local, R @ self.jaw_local

    def _solve_from(self, qpos_full, q0: np.ndarray, target: np.ndarray, iters: int = 120):
        q = q0.copy()
        lam = 0.02
        jacp = np.zeros((3, self.m.nv))
        jacr = np.zeros((3, self.m.nv))
        best = (q.copy(), 1e9, 1e9)
        for _ in range(iters):
            d = self.d
            d.qpos[:] = qpos_full
            d.qpos[self.qadr] = q
            self.mj.mj_kinematics(self.m, d)
            self.mj.mj_comPos(self.m, d)
            R = d.xmat[self.body].reshape(3, 3)
            p = d.site_xpos[self.site]
            a = R @ self.approach_local
            ep = p - target
            ea = a - DOWN
            pe, ae = float(np.linalg.norm(ep)), float(np.linalg.norm(ea))
            cost = pe + W_PITCH * ae
            if cost < best[1] + W_PITCH * best[2]:
                best = (q.copy(), pe, ae)
            if pe < 2e-4 and ae < 2e-3:
                break
            self.mj.mj_jacSite(self.m, d, jacp, jacr, self.site)
            Jp = jacp[:, self.dadr[:4]]
            Jr = jacr[:, self.dadr[:4]]
            Ja = -np.cross(a, Jr.T).T  # d(approach)/dq = omega x a
            J = np.vstack([Jp, W_PITCH * Ja])
            e = np.concatenate([ep, W_PITCH * ea])
            dq = -J.T @ np.linalg.solve(J @ J.T + lam**2 * np.eye(6), e)
            n = float(np.max(np.abs(dq)))
            if n > 0.25:
                dq *= 0.25 / n
            q[:4] = np.clip(q[:4] + dq, self.lo[:4], self.hi[:4])
        return best

    def solve(self, qpos_full: np.ndarray, q_now: np.ndarray, target, seeds_deg=None):
        """Joint angles (rad, five joints; the roll kept at q_now's) that put the tool point at `target` (world) with the
        approach axis as close to straight down as the arm allows. Returns (q, position error m, pitch error deg)."""
        target = np.asarray(target, float)
        starts = [np.asarray(q_now, float).copy()]
        for sd in seeds_deg if seeds_deg is not None else self.robot["ik_seeds"]:
            q = np.asarray(q_now, float).copy()
            q[:4] = np.radians(sd)
            # aim the base at the target from the start (the seeds are in the arm's plane)
            starts.append(q)
        best = None
        for q0 in starts:
            q, pe, ae = self._solve_from(qpos_full, q0, target)
            score = pe + W_PITCH * ae + 1e-4 * float(np.linalg.norm(q[:4] - q_now[:4]))
            if best is None or score < best[3]:
                best = (q, pe, ae, score)
            if pe < 5e-4 and ae < 0.01 and q0 is starts[0]:
                break  # the warm start converged: keep the nearest solution
        q, pe, ae, _ = best
        pitch = math.degrees(2 * math.asin(min(1.0, ae / 2)))
        return q, pe, pitch
