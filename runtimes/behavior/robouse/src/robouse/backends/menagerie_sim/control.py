"""Model-only IK and a shared-clock, slew-limited joint-position gateway.

Both arms advance on one physics clock. Objects move only through `mj_step`;
nothing here assigns object state. Position targets are slew limited at
1.2 rad/s and ideal model-based bias compensation is applied to arm degrees of
freedom only. This is a declared simulation controller, not a calibrated driver.
"""
import mujoco
import numpy as np

from .layout import BOWL_X, BOWL_Y, READY_Z

CONTROL_DT = .05
PHYSICS_DT = .002
MAX_JOINT_SPEED = 1.2


class IKError(RuntimeError):
    pass


def solve_ik(model, data, arm, target, quat=None, *, iterations=2000,
             position_tolerance=.0005, rotation_tolerance=.008):
    """Damped least-squares IK on a scratch copy of the current state."""
    scratch = mujoco.MjData(model)
    scratch.qpos[:] = data.qpos
    joints = [model.joint(name).id for name in arm.joints]
    qids = model.jnt_qposadr[joints]
    vids = model.jnt_dofadr[joints]
    site = model.site(arm.tcp).id
    target = np.asarray(target, dtype=float)
    quat = np.asarray(quat if quat is not None else arm.down_quat, dtype=float)
    jacp, jacr = np.zeros((3, model.nv)), np.zeros((3, model.nv))
    current_q, conjugate_q, difference, rot_err = np.zeros(4), np.zeros(4), np.zeros(4), np.zeros(3)
    position_error = np.zeros(3)
    for _ in range(iterations):
        mujoco.mj_kinematics(model, scratch)
        mujoco.mj_comPos(model, scratch)
        position_error = target - scratch.site_xpos[site]
        mujoco.mju_mat2Quat(current_q, scratch.site_xmat[site])
        mujoco.mju_negQuat(conjugate_q, current_q)
        mujoco.mju_mulQuat(difference, quat, conjugate_q)
        mujoco.mju_quat2Vel(rot_err, difference, 1.)
        if (np.linalg.norm(position_error) < position_tolerance
                and np.linalg.norm(rot_err) < rotation_tolerance):
            return scratch.qpos[qids].copy()
        mujoco.mj_jacSite(model, scratch, jacp, jacr, site)
        jac = np.vstack([jacp[:, vids], .3 * jacr[:, vids]])
        error = np.r_[position_error, .3 * rot_err]
        dq = jac.T @ np.linalg.solve(jac @ jac.T + np.eye(6) * 1e-4, error)
        dq *= min(1., .12 / (np.max(np.abs(dq)) + 1e-9))
        scratch.qpos[qids] = np.clip(scratch.qpos[qids] + dq,
                                     model.jnt_range[joints, 0] + .001,
                                     model.jnt_range[joints, 1] - .001)
    raise IKError(f"{arm.name}: IK did not converge for {target.tolist()}: "
                  f"position error {np.linalg.norm(position_error):.4f} m")


class Simulation:
    """One physics clock for both arms behind a paired joint-position gateway."""

    control_dt = CONTROL_DT
    max_joint_speed = MAX_JOINT_SPEED

    def __init__(self, scene):
        self.scene = scene
        self.model = scene.spec.compile()
        self.data = mujoco.MjData(self.model)
        self.physics_step = 0
        self.control_step = 0
        self._dofs = []
        self.joint_ids = {}
        self.actuator_ids = {}
        for side, arm in scene.arms.items():
            self.joint_ids[side] = np.array([self.model.joint(j).id for j in arm.joints])
            self.actuator_ids[side] = np.array([self.model.actuator(a).id for a in arm.actuators])
            for joint, q in zip(arm.joints, arm.home):
                self.data.joint(joint).qpos[0] = q
                self._dofs.append(self.model.joint(joint).dofadr[0])
            for joint in arm.finger_joints:
                self.data.joint(joint).qpos[0] = arm.finger_open_m
        mujoco.mj_forward(self.model, self.data)
        # The ready pose is part of the reset, not an evaluated movement.
        for side, arm in scene.arms.items():
            q = solve_ik(self.model, self.data, arm, [BOWL_X[side], BOWL_Y, READY_Z])
            for joint, value in zip(arm.joints, q):
                self.data.joint(joint).qpos[0] = value
            for actuator, value in zip(arm.actuators, q):
                self.data.actuator(actuator).ctrl[0] = value
            self.data.actuator(arm.gripper_actuator).ctrl[0] = arm.open_ctrl
            mujoco.mj_forward(self.model, self.data)
        self.joint_limits = {s: self.model.jnt_range[self.joint_ids[s]].copy() for s in scene.arms}

    def joint_target_command(self, side, target, openness, quat=None):
        q = solve_ik(self.model, self.data, self.scene.arms[side], target, quat)
        return np.asarray(q, dtype=float), float(openness)

    def step_control(self, targets, *, substep_hook=None, hook_stride=5):
        """Advance one 50 ms control step toward `targets` = {side: (q, openness)}."""
        substeps = round(self.control_dt / self.model.opt.timestep)
        ctrl_target = self.data.ctrl.copy()
        for side, (q, openness) in targets.items():
            arm = self.scene.arms[side]
            ctrl_target[self.actuator_ids[side]] = np.asarray(q, dtype=float)
            gid = self.model.actuator(arm.gripper_actuator).id
            ctrl_target[gid] = arm.closed_ctrl + openness * (arm.open_ctrl - arm.closed_ctrl)
        limit = self.max_joint_speed * self.model.opt.timestep
        for index in range(substeps):
            for side, arm in self.scene.arms.items():
                ids = self.actuator_ids[side]
                self.data.ctrl[ids] += np.clip(ctrl_target[ids] - self.data.ctrl[ids], -limit, limit)
                gid = self.model.actuator(arm.gripper_actuator).id
                self.data.ctrl[gid] = ctrl_target[gid]
            self.data.qfrc_applied[:] = 0
            self.data.qfrc_applied[self._dofs] = self.data.qfrc_bias[self._dofs]
            mujoco.mj_step(self.model, self.data)
            mujoco.mj_forward(self.model, self.data)
            self.physics_step += 1
            if substep_hook is not None and index % hook_stride == 0:
                substep_hook(self)
        self.control_step += 1

    def hold(self, steps, **kwargs):
        for _ in range(steps):
            self.step_control({}, **kwargs)

    def arm_joint_positions(self, side):
        return self.data.qpos[self.model.jnt_qposadr[self.joint_ids[side]]].copy()

    def arm_joint_velocities(self, side):
        return self.data.qvel[self.model.jnt_dofadr[self.joint_ids[side]]].copy()

    def tcp_pose(self, side):
        site = self.data.site(self.scene.arms[side].tcp)
        quat = np.zeros(4)
        mujoco.mju_mat2Quat(quat, site.xmat)
        return site.xpos.copy(), quat

    def tcp_jacobian(self, side):
        arm = self.scene.arms[side]
        jacp, jacr = np.zeros((3, self.model.nv)), np.zeros((3, self.model.nv))
        mujoco.mj_jacSite(self.model, self.data, jacp, jacr, self.model.site(arm.tcp).id)
        vids = self.model.jnt_dofadr[self.joint_ids[side]]
        return jacp[:, vids].copy(), jacr[:, vids].copy()

    def finger_positions(self, side):
        arm = self.scene.arms[side]
        return np.array([float(self.data.joint(j).qpos[0]) for j in arm.finger_joints])
