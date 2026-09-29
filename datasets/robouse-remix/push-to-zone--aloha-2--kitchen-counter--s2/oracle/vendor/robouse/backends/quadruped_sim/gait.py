"""Model-based trot controller for the Menagerie quadrupeds (no learned policy).

Per robot, a `RobotCfg` names the Menagerie files, the leg joints and feet, the standing pose and the gait and PD
parameters. `Gait` runs under the `base.twist` action like a robot's locomotion firmware:

  clock (trot: diagonal pairs FL+RR and FR+RL alternate, duty 0.5)
  -> foot targets in the heading frame (origin at the base, yaw-aligned axes, z up):
       stance: the foot moves backwards under the body at the commanded velocity (vx, vy, and wz about the base);
       swing: from lift-off to a Raibert landing point (nominal foot + v T_stance / 2 + k (v - v_cmd)), lifted by a
       sine arc
  -> rotated into the body frame for the desired roll/pitch (level, or the commanded body pitch) plus an attitude
     feedback term, so stance legs push the base back towards the desired attitude
  -> joint targets from inverse kinematics (damped Newton on a kinematic copy of the robot, warm-started)
  -> joint PD: torque motors (Go2) get tau = kp (q* - q) - kd dq + a gravity feed-forward on stance legs; position
     actuators (Go1, Spot, ANYmal) get q* as ctrl.

Nothing pushes the base: the only forces on it come from the legs through the simulated foot contacts.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np


@dataclass
class RobotCfg:
    key: str                  # "go2", "go1", "spot", "anymal"
    name: str                 # display name
    folder: str               # Menagerie folder
    xml: str                  # model file inside it
    base: str                 # base body name
    legs: list[str]           # labels in controller order: front-left, front-right, rear-left, rear-right
    joints: list[list[str]]   # per leg: abduction, hip, knee joint names
    feet: list[str]           # per leg: foot geom name ("" -> the sphere geom on the leg's last body)
    home: list[float]         # 12 joint angles of the standing pose (controller order)
    torque: bool = False      # True: the actuators are torque motors and the controller closes the PD loop
    kp: float = 0.0           # torque PD gain, or the overridden position-actuator stiffness (0: keep upstream)
    kd: float = 0.0           # torque PD damping, or the overridden position-actuator damping (0: keep upstream)
    period: float = .4        # gait cycle (s)
    swing_h: float = .08      # foot lift (m)
    height: float = 0.0       # standing base height above the ground (m); 0: from the home pose
    v_max: tuple = (.6, .4, 1.0)  # command limits |vx|, |vy| (m/s), |wz| (rad/s)
    k_raibert: float = .08    # landing-point velocity feedback (s)
    k_att: float = .5         # attitude feedback (extra body-frame rotation per rad of error)
    k_vel: float = 1.0        # stance-speed feedback on the base velocity error
    k_int: float = 4.0        # integral gain on the base velocity error (1/s)
    stance_width: float = 0.0  # extra lateral foot offset (m)
    stance_fwd: float = 0.0   # front feet further forward, rear feet further back (m)
    extra_arm: list[str] = field(default_factory=list)


CFGS: dict[str, RobotCfg] = {
    "go2": RobotCfg("go2", "Unitree Go2", "unitree_go2", "go2.xml", "base", ["FL", "FR", "RL", "RR"],
                    [[f"{l}_{j}_joint" for j in ("hip", "thigh", "calf")] for l in ("FL", "FR", "RL", "RR")],
                    ["FL", "FR", "RL", "RR"], [0, .9, -1.8] * 4, torque=True, kp=150.0, kd=4.0, period=.36, swing_h=.08,
                    height=.30, v_max=(.6, .4, 1.2), stance_width=.03),
    "go1": RobotCfg("go1", "Unitree Go1", "unitree_go1", "go1.xml", "trunk", ["FL", "FR", "RL", "RR"],
                    [[f"{l}_{j}_joint" for j in ("hip", "thigh", "calf")] for l in ("FL", "FR", "RL", "RR")],
                    ["FL", "FR", "RL", "RR"], [0, .9, -1.8] * 4, period=.36, swing_h=.08, height=.29, v_max=(.6, .4, 1.2),
                    stance_width=.03),
    "spot": RobotCfg("spot", "Boston Dynamics Spot", "boston_dynamics_spot", "spot.xml", "body", ["FL", "FR", "HL", "HR"],
                     [[f"{l}_{j}" for j in ("hx", "hy", "kn")] for l in ("fl", "fr", "hl", "hr")],
                     ["FL", "FR", "HL", "HR"], [0, 1.04, -1.8] * 4, period=.4, swing_h=.1, height=.5, v_max=(.5, .4, .8),
                     k_raibert=.15, stance_width=.05),
    "anymal": RobotCfg("anymal", "ANYbotics ANYmal C", "anybotics_anymal_c", "anymal_c.xml", "base", ["LF", "RF", "LH", "RH"],
                       [[f"{l}_{j}" for j in ("HAA", "HFE", "KFE")] for l in ("LF", "RF", "LH", "RH")],
                       ["", "", "", ""], [0, .4, -.8, 0, .4, -.8, 0, -.4, .8, 0, -.4, .8], kp=300.0, kd=8.0, period=.5,
                       swing_h=.1, height=.5, v_max=(.6, .4, 1.0)),
}
CFGS["spot_arm"] = RobotCfg(**{**CFGS["spot"].__dict__, "key": "spot_arm", "name": "Boston Dynamics Spot with arm", "xml": "spot_arm.xml",
                               "extra_arm": ["arm_sh0", "arm_sh1", "arm_el0", "arm_el1", "arm_wr0", "arm_wr1", "arm_f1x"]})

TROT_OFFSET = [0.0, .5, .5, 0.0]  # FL and RR swing together, then FR and RL
DUTY = .5
ACCEL = [1.2, 0.8, 2.5]  # command filter: m/s^2, m/s^2, rad/s^2
FOOT_V_MAX = 1.0  # x the robot's top speed: limit on a foot's speed relative to the body (translation + turning)


def _rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def _rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


class LegKinematics:
    """Forward and inverse kinematics of the four legs in the base frame, on a kinematic copy of the robot."""

    def __init__(self, cfg: RobotCfg, robot_spec_xml: str):
        import mujoco

        self.mj = mujoco
        self.m = mujoco.MjModel.from_xml_path(robot_spec_xml)
        self.d = mujoco.MjData(self.m)
        m = self.m
        self.qadr = np.array([[m.jnt_qposadr[m.joint(j).id] for j in leg] for leg in cfg.joints])
        self.dadr = np.array([[m.jnt_dofadr[m.joint(j).id] for j in leg] for leg in cfg.joints])
        self.lo = np.array([[m.jnt_range[m.joint(j).id][0] for j in leg] for leg in cfg.joints])
        self.hi = np.array([[m.jnt_range[m.joint(j).id][1] for j in leg] for leg in cfg.joints])
        lim = np.array([[m.jnt_limited[m.joint(j).id] for j in leg] for leg in cfg.joints]).astype(bool)
        self.lo[~lim], self.hi[~lim] = -2 * math.pi, 2 * math.pi
        self.foot_body, self.foot_off, self.foot_r = [], [], []
        for i, leg in enumerate(cfg.joints):
            b = m.jnt_bodyid[m.joint(leg[2]).id]
            if cfg.feet[i]:
                g = m.geom(cfg.feet[i]).id
            else:
                g = next(g for g in range(m.ngeom) if m.geom_bodyid[g] == b and m.geom_type[g] == mujoco.mjtGeom.mjGEOM_SPHERE
                         and (m.geom_contype[g] or m.geom_conaffinity[g]))
            assert m.geom_bodyid[g] == b, (cfg.key, i)
            self.foot_body.append(b)
            self.foot_off.append(m.geom_pos[g].copy())
            self.foot_r.append(float(m.geom_size[g][0]))
        self.jac = np.zeros((3, m.nv))
        self.q = np.array(cfg.home, float).reshape(4, 3)
        free = m.jnt_qposadr[0]
        self.d.qpos[free:free + 7] = [0, 0, 0, 1, 0, 0, 0]

    def fk(self, q: np.ndarray) -> np.ndarray:
        d = self.d
        d.qpos[self.qadr.ravel()] = q.ravel()
        self.mj.mj_kinematics(self.m, d)
        return np.array([d.xpos[b] + d.xmat[b].reshape(3, 3) @ o for b, o in zip(self.foot_body, self.foot_off)])

    def ik(self, targets: np.ndarray, iters: int = 3) -> np.ndarray:
        """Joint angles (4 x 3) putting each foot centre at its target (base frame); warm-started from the last call."""
        m, d, mj = self.m, self.d, self.mj
        q = self.q
        for _ in range(iters):
            d.qpos[self.qadr.ravel()] = q.ravel()
            mj.mj_kinematics(m, d)
            mj.mj_comPos(m, d)
            worst = 0.0
            for i in range(4):
                b = self.foot_body[i]
                p = d.xpos[b] + d.xmat[b].reshape(3, 3) @ self.foot_off[i]
                e = targets[i] - p
                worst = max(worst, float(np.abs(e).max()))
                mj.mj_jac(m, d, self.jac, None, p, b)
                J = self.jac[:, self.dadr[i]]
                dq = J.T @ np.linalg.solve(J @ J.T + 1e-4 * np.eye(3), e)
                q[i] = np.clip(q[i] + np.clip(dq, -.5, .5), self.lo[i], self.hi[i])
            if worst < 5e-4:
                break
        self.q = q
        return q.copy()


class Gait:
    """Trot controller for one robot inside a scene model (joint and actuator names may carry a prefix)."""

    def __init__(self, cfg: RobotCfg, model, data, prefix: str, kin_xml: str):
        import mujoco

        self.cfg, self.m, self.d = cfg, model, data
        m = model
        self.base = m.body(prefix + cfg.base).id
        fj = m.body_jntadr[self.base]
        self.q0 = m.jnt_qposadr[fj]
        self.v0 = m.jnt_dofadr[fj]
        jid = [[m.joint(prefix + j).id for j in leg] for leg in cfg.joints]
        self.qadr = np.array([[m.jnt_qposadr[j] for j in leg] for leg in jid])
        self.dadr = np.array([[m.jnt_dofadr[j] for j in leg] for leg in jid])
        act_of = {int(m.actuator_trnid[a, 0]): a for a in range(m.nu)}
        self.act = np.array([[act_of[j] for j in leg] for leg in jid])
        self.kin = LegKinematics(cfg, kin_xml)
        self.foot_geoms = []
        for i, leg in enumerate(jid):
            b = m.jnt_bodyid[leg[2]]
            self.foot_geoms.append(next(g for g in range(m.ngeom) if m.geom_bodyid[g] == b and m.geom_type[g] == mujoco.mjtGeom.mjGEOM_SPHERE
                                        and (m.geom_contype[g] or m.geom_conaffinity[g])))
        # position actuators: optional stiffness/damping override (ANYmal's upstream kp = 100 cannot carry 45 kg in a trot)
        if not cfg.torque and cfg.kp > 0:
            for a in self.act.ravel():
                m.actuator_gainprm[a, 0] = cfg.kp
                m.actuator_biasprm[a, 1] = -cfg.kp
                m.actuator_biasprm[a, 2] = -cfg.kd
        # nominal feet from the home pose
        home = np.array(cfg.home, float).reshape(4, 3)
        feet = self.kin.fk(home)
        self.kin.q = home.copy()
        self.foot_r = self.kin.foot_r[0]
        self.h_nom = cfg.height or float(-feet[:, 2].mean() + self.foot_r)
        self.nominal = feet[:, :2].copy()
        for i in range(4):
            self.nominal[i, 1] += math.copysign(cfg.stance_width, self.nominal[i, 1])
            self.nominal[i, 0] += math.copysign(cfg.stance_fwd, self.nominal[i, 0])
        self.mass = float(m.body_subtreemass[self.base])
        self.reset()

    def reset(self) -> None:
        self.phase = 0.0
        self.active = False
        self.cmd = np.zeros(3)
        self.v_f = np.zeros(3)                  # filtered command (acceleration-limited)
        self.e_int = np.zeros(3)                # integral of the velocity error (heading frame; vx, vy, wz)
        self.height = self.h_nom
        self.h_target = self.h_nom
        self.pitch_des = 0.0
        self.pitch_target = 0.0
        self.feet = np.column_stack([self.nominal, np.full(4, -self.h_nom + self.foot_r)])  # heading-frame targets
        self.lift = self.feet.copy()
        self.swinging = np.zeros(4, bool)
        self.swings_since_zero = np.zeros(4, int)
        self.q_des = np.array(self.cfg.home, float).reshape(4, 3)
        self.kin.q = self.q_des.copy()

    # ---- state -------------------------------------------------------------------------------------------------
    def R(self) -> np.ndarray:
        return self.d.xmat[self.base].reshape(3, 3)

    def yaw(self) -> float:
        R = self.R()
        return math.atan2(R[1, 0], R[0, 0])

    def rpy(self) -> tuple[float, float, float]:
        R = self.R()
        return (math.atan2(R[2, 1], R[2, 2]), -math.asin(float(np.clip(R[2, 0], -1, 1))), math.atan2(R[1, 0], R[0, 0]))

    def vel_heading(self) -> np.ndarray:
        """Base linear velocity in the heading frame (x forward, y left) and yaw rate."""
        v = self.d.qvel[self.v0:self.v0 + 3]
        yaw = self.yaw()
        c, s = math.cos(yaw), math.sin(yaw)
        w = self.R() @ self.d.qvel[self.v0 + 3:self.v0 + 6]
        return np.array([c * v[0] + s * v[1], -s * v[0] + c * v[1], w[2]])

    def feet_in_contact(self) -> np.ndarray:
        d = self.d
        out = np.zeros(4, bool)
        fg = {g: i for i, g in enumerate(self.foot_geoms)}
        for c in d.contact[:d.ncon]:
            for g in (c.geom1, c.geom2):
                if g in fg:
                    out[fg[g]] = True
        return out

    # ---- command -----------------------------------------------------------------------------------------------
    def set_command(self, vx: float, vy: float, wz: float) -> None:
        cmd = np.array([vx, vy, wz], float)
        # combined translation and turning: keep every foot's speed relative to the body within FOOT_V_MAX
        r = np.column_stack([self.nominal, np.zeros(4)])
        foot_v = max(float(np.hypot(cmd[0] - cmd[2] * y, cmd[1] + cmd[2] * x)) for x, y, _ in r)
        vmax = FOOT_V_MAX * max(self.cfg.v_max[0], self.cfg.v_max[1])
        if foot_v > vmax:
            cmd *= vmax / foot_v
        self.cmd = cmd
        if np.any(np.abs(self.cmd) > 1e-6):
            self.active = True
            self.swings_since_zero[:] = 0

    # ---- one physics step ----------------------------------------------------------------------------------------
    def step(self, dt: float) -> None:
        cfg = self.cfg
        # command filter: acceleration limits ACCEL (m/s^2, m/s^2, rad/s^2)
        dv = self.cmd - self.v_f
        lim = np.array(ACCEL) * dt
        self.v_f += np.clip(dv, -lim, lim)
        self.height += float(np.clip(self.h_target - self.height, -.25 * dt, .25 * dt))
        self.pitch_des += float(np.clip(self.pitch_target - self.pitch_des, -.5 * dt, .5 * dt))
        T = cfg.period
        t_st = T * DUTY
        v = self.v_f
        moving = bool(np.any(np.abs(self.cmd) > 1e-6))
        if self.active:
            self.phase = (self.phase + dt / T) % 1.0
            touchdown = False
            for i in range(4):
                p = (self.phase + TROT_OFFSET[i]) % 1.0
                sw = p >= DUTY
                if sw and not self.swinging[i]:
                    self.lift[i] = self.feet[i].copy()
                if not sw and self.swinging[i]:
                    touchdown = True
                    if not moving:
                        self.swings_since_zero[i] += 1
                self.swinging[i] = sw
            # stop at a touchdown once every leg has stepped with a zero command
            if touchdown and not moving and np.all(self.swings_since_zero >= 1) and np.all(np.abs(v) < 1e-3):
                self.active = False
                self.swinging[:] = False
        vm = self.vel_heading()
        if self.active:
            err = v - vm
            self.e_int = np.clip(self.e_int + err * dt, -.15, .15)
            corr = np.clip(cfg.k_vel * err + cfg.k_int * self.e_int, -.6, .6)
        else:
            self.e_int[:] = 0
            corr = np.zeros(3)
        for i in range(4):
            nom = np.array([*self.nominal[i], 0.0])
            if self.active and self.swinging[i]:
                p = (self.phase + TROT_OFFSET[i]) % 1.0
                s = (p - DUTY) / (1 - DUTY)
                # Raibert landing point (heading frame)
                rot = np.array([-nom[1], nom[0]]) * v[2]  # velocity of the nominal foot point due to yaw rate
                land = nom[:2] + (v[:2] + rot) * t_st / 2 + cfg.k_raibert * (vm[:2] - v[:2])
                dl = np.clip(land - nom[:2], -.25, .25)
                if dl[1] * nom[1] < 0:  # stepping inwards: never past 40 % of the way to the midline (legs would cross)
                    dl[1] = math.copysign(min(abs(dl[1]), .4 * abs(nom[1])), dl[1])
                land = nom[:2] + dl
                k = .5 - .5 * math.cos(math.pi * min(1.0, s * 1.15))
                xy = self.lift[i][:2] + (land - self.lift[i][:2]) * k
                z = -self.height + self.foot_r + cfg.swing_h * math.sin(math.pi * s)
                self.feet[i] = [xy[0], xy[1], z]
            else:
                # stance: the foot moves backwards under the body
                if self.active:
                    xy = self.feet[i][:2]
                    vc = v + corr
                    vel = vc[:2] + vc[2] * np.array([-xy[1], xy[0]])
                    self.feet[i][:2] = xy - vel * dt
                self.feet[i][2] = -self.height + self.foot_r
        # heading frame -> body frame, with attitude feedback
        roll, pitch, _ = self.rpy()
        e_r = 0.0 - roll
        e_p = self.pitch_des - pitch
        Rd = _rot_y(self.pitch_des + cfg.k_att * e_p) @ _rot_x(cfg.k_att * e_r)
        targets = (Rd.T @ self.feet.T).T
        self.q_des = self.kin.ik(targets)
        self.apply()

    def apply(self) -> None:
        m, d, cfg = self.m, self.d, self.cfg
        if cfg.torque:
            q = d.qpos[self.qadr]
            dq = d.qvel[self.dadr]
            tau = cfg.kp * (self.q_des - q) - cfg.kd * dq
            tau += self._gravity_ff()
            lo = m.actuator_ctrlrange[self.act, 0]
            hi = m.actuator_ctrlrange[self.act, 1]
            d.ctrl[self.act] = np.clip(tau, lo, hi)
        else:
            d.ctrl[self.act] = self.q_des

    def _gravity_ff(self) -> np.ndarray:
        """Stance legs share the body weight: tau = -J^T f with f the vertical support force (world frame)."""
        import mujoco

        m, d = self.m, self.d
        stance = ~self.swinging if self.active else np.ones(4, bool)
        n = max(1, int(stance.sum()))
        f = np.array([0, 0, -self.mass * 9.81 / n])
        out = np.zeros((4, 3))
        jac = np.zeros((3, m.nv))
        for i in range(4):
            if not stance[i]:
                continue
            g = self.foot_geoms[i]
            mujoco.mj_jacGeom(m, d, jac, None, g)
            out[i] = jac[:, self.dadr[i]].T @ f  # the foot pushes down on the ground
        return out
