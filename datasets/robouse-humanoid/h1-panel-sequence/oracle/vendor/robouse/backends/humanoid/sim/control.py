"""Whole-robot servo and per-arm operational-space (IK) controller for the fixed-base humanoids.

Firmware model (runs under `robo act`):
  * Every actuated joint has a setpoint q_des. Legs, waist and (unless looking) the neck keep the standing pose.
  * Position-servo robots (G1, Apollo, T1): the Menagerie position actuators track q_des; the controller adds a gravity
    feed-forward term tau_g / kp to the setpoint, which is what a joint servo with gravity compensation does, and the
    joints' own torque limits (actuatorfrcrange) still apply.
  * Torque-motor robots (H1): a joint PD servo with gravity compensation, tau = kp (q_des - q) - kd qdot + tau_g,
    runs every physics step and is clipped to the motor torque limits.
  * Arms: each side has an end-effector goal x_goal (world frame). An `ee_delta` action moves the goal; zero holds
    it. Each control step, damped least-squares IK on the arm chain (warm-started from the previous q_des, a fixed
    hand orientation on 7-DoF arms, a posture term in the null space) moves the arm's q_des toward the goal, at most
    MAX_DQ rad per joint per step. The goal is kept within GOAL_LEAD m of the measured end effector and within the
    arm's reach, so a blocked or out-of-reach arm stops where it is instead of winding up.
"""

from __future__ import annotations

import math

import mujoco
import numpy as np

MAX_DQ = 0.12  # rad per control step per arm joint
GOAL_LEAD = 0.06  # m: goal kept this close to the measured end effector
REACH_SLACK = 0.015  # m: goal kept this close to what the IK can reach
DAMP = 3e-3
ROT_W = 0.35  # weight of the orientation error (per rad) vs position (per m)
NULL_K = 0.08  # posture pull in the null space, per step


class Servo:
    """Holds q_des for every actuator and writes ctrl (position servos) or joint torques (motors)."""

    def __init__(self, model, data, motor: dict | None):
        self.m, self.d = model, data
        m = model
        self.act_joint = m.actuator_trnid[:, 0].copy()
        self.qadr = m.jnt_qposadr[self.act_joint]
        self.dadr = m.jnt_dofadr[self.act_joint]
        self.names = [m.joint(j).name for j in self.act_joint]
        self.q_des = data.qpos[self.qadr].copy()
        self.motor = motor
        self.kp = np.zeros(m.nu)
        self.kd = np.zeros(m.nu)
        if motor:
            for i, n in enumerate(self.names):
                part = next(k for k in ("hip", "knee", "ankle", "torso", "shoulder", "elbow") if k in n)
                self.kp[i], self.kd[i] = motor["kp"][part], motor["kd"][part]
            self.lim = m.actuator_ctrlrange.copy()
        else:
            self.kp = m.actuator_gainprm[:, 0].copy()
            self.lo = m.actuator_ctrlrange[:, 0].copy()
            self.hi = m.actuator_ctrlrange[:, 1].copy()
        self.index = {n: i for i, n in enumerate(self.names)}

    def idx(self, joints: list[str]) -> np.ndarray:
        return np.array([self.index[j] for j in joints])

    def apply(self) -> None:
        """Position servos: once per control step. Motors: every physics step."""
        d = self.d
        g = d.qfrc_bias[self.dadr]
        if self.motor:
            tau = self.kp * (self.q_des - d.qpos[self.qadr]) - self.kd * d.qvel[self.dadr] + g
            d.ctrl[:] = np.clip(tau, self.lim[:, 0], self.lim[:, 1])
        else:
            d.ctrl[:] = np.clip(self.q_des + g / self.kp, self.lo, self.hi)


class ArmIK:
    """Damped least-squares IK for one arm on a scratch MjData (the real state is never written)."""

    def __init__(self, model, scratch, joints: list[str], site: str, orient: bool, q_nom: np.ndarray):
        m = model
        jids = [m.joint(n).id for n in joints]
        self.m, self.s = m, scratch
        self.qadr = m.jnt_qposadr[jids]
        self.dadr = m.jnt_dofadr[jids]
        self.lo = m.jnt_range[jids, 0] + 0.02
        self.hi = m.jnt_range[jids, 1] - 0.02
        self.site = m.site(site).id
        self.orient = orient
        self.q_nom = np.clip(q_nom, self.lo, self.hi)
        self.R_goal: np.ndarray | None = None
        self.jp = np.zeros((3, m.nv))
        self.jr = np.zeros((3, m.nv))

    def fk(self, q: np.ndarray, base_qpos: np.ndarray):
        s = self.s
        s.qpos[:] = base_qpos
        s.qpos[self.qadr] = q
        mujoco.mj_kinematics(self.m, s)
        return s.site_xpos[self.site].copy(), s.site_xmat[self.site].reshape(3, 3).copy()

    def solve(self, q0: np.ndarray, goal: np.ndarray, base_qpos: np.ndarray, iters: int = 6, max_dq: float = MAX_DQ):
        """A few DLS iterations from q0 toward goal; total change capped at max_dq per joint. Returns (q, residual)."""
        q = q0.copy()
        n = len(q)
        for _ in range(iters):
            x, R = self.fk(q, base_qpos)
            mujoco.mj_comPos(self.m, self.s)
            mujoco.mj_jacSite(self.m, self.s, self.jp, self.jr, self.site)
            Jp = self.jp[:, self.dadr]
            ep = goal - x
            if self.orient and self.R_goal is not None:
                Rerr = self.R_goal @ R.T
                er = 0.5 * np.array([Rerr[2, 1] - Rerr[1, 2], Rerr[0, 2] - Rerr[2, 0], Rerr[1, 0] - Rerr[0, 1]])
                J = np.vstack([Jp, ROT_W * self.jr[:, self.dadr]])
                e = np.r_[ep, ROT_W * er]
            else:
                J, e = Jp, ep
            JJ = J @ J.T + DAMP * np.eye(J.shape[0])
            Jpinv = J.T @ np.linalg.inv(JJ)
            dq = Jpinv @ e
            dq += (np.eye(n) - Jpinv @ J) @ (NULL_K * (self.q_nom - q))
            q = np.clip(q + np.clip(dq, -0.08, 0.08), self.lo, self.hi)
            q = q0 + np.clip(q - q0, -max_dq, max_dq)
        x, _ = self.fk(q, base_qpos)
        return q, float(np.linalg.norm(goal - x))


def rot_about(axis: str, ang: float) -> np.ndarray:
    c, s = math.cos(ang), math.sin(ang)
    if axis == "x":
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    if axis == "y":
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
