"""Warm-started damped least-squares IK for the Cartesian action interface.

Same solver as robo-use `control.solve_ik` (position + 0.3-weighted rotation error, 1e-4 damping, 0.12 rad
iteration cap), but it starts from the previous joint command, stops after a bounded number of iterations
and returns the best iterate instead of raising, so an unreachable target makes the arm stop at the
closest pose it can reach instead of failing the episode. Only a scratch MjData is written.
"""
from __future__ import annotations

import mujoco
import numpy as np


class ArmIK:
    def __init__(self, model, arm):
        self.model = model
        self.arm = arm
        joints = [model.joint(n).id for n in arm.joints]
        self.qids = model.jnt_qposadr[joints]
        self.vids = model.jnt_dofadr[joints]
        self.site = model.site(arm.tcp).id
        self.lo = model.jnt_range[joints, 0] + .001
        self.hi = model.jnt_range[joints, 1] - .001
        self.scratch = mujoco.MjData(model)
        self._jacp = np.zeros((3, model.nv))
        self._jacr = np.zeros((3, model.nv))

    def solve(self, data, q_start, target, quat, iterations: int = 150, tol: float = 2e-4, rtol: float = 4e-3):
        s = self.scratch
        s.qpos[:] = data.qpos
        q = np.clip(np.asarray(q_start, dtype=float), self.lo, self.hi)
        cur, conj, diff, rot = np.zeros(4), np.zeros(4), np.zeros(4), np.zeros(3)
        best = None
        for _ in range(iterations):
            s.qpos[self.qids] = q
            mujoco.mj_kinematics(self.model, s)
            mujoco.mj_comPos(self.model, s)
            perr = target - s.site_xpos[self.site]
            mujoco.mju_mat2Quat(cur, s.site_xmat[self.site])
            mujoco.mju_negQuat(conj, cur)
            mujoco.mju_mulQuat(diff, quat, conj)
            mujoco.mju_quat2Vel(rot, diff, 1.)
            pe, re = float(np.linalg.norm(perr)), float(np.linalg.norm(rot))
            if best is None or pe + .3 * re < best[0]:
                best = (pe + .3 * re, q.copy(), pe, re)
            if pe < tol and re < rtol:
                break
            mujoco.mj_jacSite(self.model, s, self._jacp, self._jacr, self.site)
            jac = np.vstack([self._jacp[:, self.vids], .3 * self._jacr[:, self.vids]])
            err = np.r_[perr, .3 * rot]
            dq = jac.T @ np.linalg.solve(jac @ jac.T + np.eye(6) * 1e-4, err)
            dq *= min(1., .12 / (np.max(np.abs(dq)) + 1e-9))
            q = np.clip(q + dq, self.lo, self.hi)
        _, q, pe, re = best
        return q, pe, re
