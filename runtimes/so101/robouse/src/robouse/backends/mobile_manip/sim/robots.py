"""The three mobile manipulators and their controllers ("firmware" under `robo act`).

Models (MuJoCo Menagerie, pinned commit, Apache-2.0), attached unchanged except where noted:
  Hello Robot Stretch 3 (hello_robot_stretch_3/stretch.xml): differential drive, lift, telescoping arm, wrist yaw /
      pitch / roll, SG3 gripper, head pan / tilt. Change: the wheel velocity ctrlrange is widened from +-6 to +-15
      (actuator units; gear 3), i.e. from 2 to 5 rad/s of wheel speed (0.1 to 0.25 m/s), close to the real robot's
      navigation speed. Visual meshes stay visual (upstream collision classes are kept).
  PAL TIAGo (pal_tiago/tiago_position.xml): differential drive, torso lift, 7-DoF arm, parallel gripper, head.
      Change: the finger meshes are visual only (their convex hulls wedge objects at the finger base); each finger
      collides through a flat rubber pad box on its inner face.
  Google Robot (google_robot/robot.xml): upstream the base is fixed to the world (nq = 9: torso yaw, 6 more arm
      joints, 2 fingers). We add a planar base: slide x, slide y and hinge yaw joints on the root body, driven by
      velocity actuators with force limits; the wheels are not simulated (they are 3 mm above the floor), so this is a
      holonomic abstraction. A head camera is added where the head meshes are.

Controllers (every 2 ms physics step unless noted):
  base.twist (diff drive): [V, WZ] -> acceleration-limited (V, WZ) -> wheel speeds from the model's wheel radius and
      track -> the upstream wheel velocity actuators; the base moves only through wheel-floor friction.
  base.twist (planar): [VX, VY, WZ] in the robot frame -> acceleration-limited world-frame joint velocities -> the added
      velocity actuators (force-limited, so walls and furniture stop the base).
  arm.ee_delta (TIAGo, Google Robot): the gripper target (held in the robot frame, so it moves with the base) moves by
      [DX, DY, DZ] in the world frame each 50 ms step; once per step, weighted damped-least-squares inverse kinematics
      (position + fixed orientation: the gripper points along the robot's heading, level, fingers closing horizontally;
      joint-limit locking; a posture term) runs on a scratch copy with the base level (plus a low-pass of its static
      tilt), and the joint setpoints of the upstream position actuators are slewed at 1.2 rad/s with a joint-space
      integral term near rest (the Google Robot's upstream servos are soft).
  arm.lift / arm.extend / wrist (Stretch): targets = measured joint position + delta for the upstream lift, telescope
      and wrist actuators, with an integral term on the lift and telescope.
  gripper: [G] > 0 closes, < 0 opens, 0 holds the finger target; the target never runs more than a small margin past
      the measured finger position, so a grasp squeezes with a bounded force (a stalled gripper, like the real ones).
"""

from __future__ import annotations

import math

import numpy as np

from ...embodied import wrap, yaw_of


def _rotz(yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


# gripper orientation used by the IK arms (columns: gripper x, y, z in the robot frame): approach (z) along the robot's
# heading, fingers closing along the robot's y axis (horizontal), gripper x pointing down
R_FORWARD = np.array([[0, 0, 1.0], [0, 1.0, 0], [-1.0, 0, 0]])


class Robot:
    key = ""
    name = ""
    asset = ""
    prefix = "robot/"
    cameras = {}  # upstream camera name -> public name (prefixed with "robot/")
    root_body = "base_link"
    # physical / controller limits
    v_max = 0.4
    wz_max = math.radians(60)
    a_max = 0.6  # m/s^2
    alpha_max = 2.0  # rad/s^2

    def __init__(self):
        self.m = self.d = None

    # ---- building -------------------------------------------------------------------------------------------------
    def load_spec(self):
        raise NotImplementedError

    def attach(self, s, x: float, y: float, yaw: float) -> None:
        import mujoco

        r = self.load_spec()
        for k in list(r.keys):
            k.delete()
        self.edit_spec(r)
        for c in r.cameras:
            if c.name in self.cameras:
                c.name = self.cameras[c.name]
        # chase camera: follows the robot, fixed world orientation, looking north and down from behind
        from ...embodied import lookat_xyaxes

        pos, tgt = [0, -1.25, 3.2], [0, 0.35, 0.45]
        r.body(self.root_body).add_camera(
            name="chase",
            pos=pos,
            xyaxes=[float(v) for v in lookat_xyaxes(pos, tgt).split()],
            fovy=62,
            mode=mujoco.mjtCamLight.mjCAMLIGHT_TRACKCOM,
        )
        frame = s.worldbody.add_frame(pos=[0, 0, 0])
        s.attach(r, prefix=self.prefix, frame=frame)
        for c in s.cameras:
            if c.name == self.prefix + "chase":
                c.name = "chase"
        self.start = (x, y, yaw)

    def edit_spec(self, r) -> None:
        pass

    def j(self, name):
        return self.m.joint(self.prefix + name)

    def body_id(self, name) -> int:
        return self.m.body(self.prefix + name).id

    def act_id(self, name) -> int:
        return self.m.actuator(self.prefix + name).id

    def bind(self, m, d) -> None:
        self.m, self.d = m, d
        self.root = self.body_id(self.root_body)
        pre = self.prefix
        import mujoco

        self.robot_bodies = {
            b for b in range(m.nbody) if (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b) or "").startswith(pre)
        }
        self.robot_geoms = {g for g in range(m.ngeom) if m.geom_bodyid[g] in self.robot_bodies}
        self.finger_geoms = {
            g for g in self.robot_geoms if m.geom_bodyid[g] in {self.body_id(b) for b in self.finger_bodies}
        }
        # "body" geoms: everything except the arm (the base, wheels, mast, torso, head): driving into things
        arm_root = self.body_id(self.arm_root)

        def in_arm(b):
            while b > 0:
                if b == arm_root:
                    return True
                b = m.body_parentid[b]
            return False

        self.base_geoms = {g for g in self.robot_geoms if not in_arm(m.geom_bodyid[g])}
        self.side_geoms = [
            {g for g in self.robot_geoms if m.geom_bodyid[g] in {self.body_id(b) for b in side}}
            for side in self.finger_sides
        ]
        self.v_cur, self.wz_cur, self.vy_cur = 0.0, 0.0, 0.0
        self.v_cmd, self.wz_cmd, self.vy_cmd = 0.0, 0.0, 0.0

    # ---- state ----------------------------------------------------------------------------------------------------
    def base_pose(self) -> tuple[float, float, float]:
        p = self.d.xpos[self.root]
        return float(p[0]), float(p[1]), yaw_of(self.d.xmat[self.root].reshape(3, 3))

    def base_R(self) -> np.ndarray:
        return self.d.xmat[self.root].reshape(3, 3)

    def tilt_deg(self) -> float:
        R = self.base_R()
        return math.degrees(math.acos(float(np.clip(R[2, 2], -1, 1))))

    def base_twist(self) -> tuple[float, float, float]:
        """Measured (forward, left, yaw rate) velocity in the robot frame."""
        import mujoco

        v = np.zeros(6)
        mujoco.mj_objectVelocity(self.m, self.d, mujoco.mjtObj.mjOBJ_BODY, self.root, v, 1)  # local frame: [rot, lin]
        return float(v[3]), float(v[4]), float(v[2])

    def level_base(self, qpos, tilt=None) -> None:
        """Put the base at the origin with no yaw and the given small tilt (rotation vector; default none), in a
        scratch qpos."""
        adr = self.m.jnt_qposadr[self.m.body_jntadr[self.root]]
        q = [1.0, 0, 0, 0]
        if tilt is not None and np.any(tilt):
            a = float(np.linalg.norm(tilt))
            q = [math.cos(a / 2), *(np.asarray(tilt) / a * math.sin(a / 2))]
        qpos[adr : adr + 7] = [0, 0, 0, *q]

    def base_tilt(self) -> np.ndarray:
        """Tilt of the base (rotation vector without the yaw), small-angle."""
        R = _rotz(-self.base_pose()[2]) @ self.base_R()
        return 0.5 * np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])

    def to_world(self, p_b) -> np.ndarray:
        x, y, yaw = self.base_pose()
        return np.array([x, y, 0.0]) + _rotz(yaw) @ np.asarray(p_b, float)

    def to_base(self, p_w) -> np.ndarray:
        x, y, yaw = self.base_pose()
        return _rotz(yaw).T @ (np.asarray(p_w, float) - np.array([x, y, 0.0]))

    # ---- commands -------------------------------------------------------------------------------------------------
    def set_twist(self, v: float, wz: float, vy: float = 0.0) -> None:
        self.v_cmd, self.wz_cmd, self.vy_cmd = v, wz, vy

    def _ramp(self, dt: float) -> None:
        self.v_cur += float(np.clip(self.v_cmd - self.v_cur, -self.a_max * dt, self.a_max * dt))
        self.vy_cur += float(np.clip(self.vy_cmd - self.vy_cur, -self.a_max * dt, self.a_max * dt))
        self.wz_cur += float(np.clip(self.wz_cmd - self.wz_cur, -self.alpha_max * dt, self.alpha_max * dt))

    def ee_pos(self) -> np.ndarray:
        raise NotImplementedError

    def gripper_opening(self) -> float:
        raise NotImplementedError

    def substep(self, dt: float) -> None:
        raise NotImplementedError

    def control_update(self) -> None:
        pass


# ------------------------------------------------------------------------------------------------------------------
# bases
# ------------------------------------------------------------------------------------------------------------------


class DiffDriveMixin:
    wheel_acts: tuple[str, str]  # (left, right)
    wheel_gear = 1.0
    wheel_radius = 0.1
    track = 0.4
    wheel_max = 5.0  # rad/s

    def _drive_substep(self, dt: float) -> None:
        self._ramp(dt)
        v, wz = self.v_cur, self.wz_cur
        wl = (v - wz * self.track / 2) / self.wheel_radius
        wr = (v + wz * self.track / 2) / self.wheel_radius
        k = max(1.0, abs(wl) / self.wheel_max, abs(wr) / self.wheel_max)
        self.d.ctrl[self._wl] = self.wheel_gear * wl / k
        self.d.ctrl[self._wr] = self.wheel_gear * wr / k

    def _bind_wheels(self) -> None:
        self._wl, self._wr = self.act_id(self.wheel_acts[0]), self.act_id(self.wheel_acts[1])


# ------------------------------------------------------------------------------------------------------------------
# damped-least-squares IK arm (TIAGo, Google Robot)
# ------------------------------------------------------------------------------------------------------------------


class IKArm:
    STEP = 0.02  # m per unit action per step
    LEAD = 0.08  # the target stays within this distance of the measured gripper point
    ROT_W = 0.25  # weight of the orientation error (m per rad)
    LAM = 0.03  # damping
    KI = 3.0  # joint-space integral gain (1/s) on the actuator setpoints
    QI_MAX = 0.2

    def __init__(
        self,
        robot: Robot,
        joints: list[str],
        acts: list[str],
        site: str,
        q_rest: list[float],
        box_lo,
        box_hi,
        posture_gain: float = 0.05,
    ):
        m = robot.m
        self.r = robot
        self.jid = [robot.j(n).id for n in joints]
        self.qadr = np.array([m.jnt_qposadr[j] for j in self.jid])
        self.dadr = np.array([m.jnt_dofadr[j] for j in self.jid])
        self.aid = np.array([robot.act_id(a) for a in acts])
        self.lo = np.array([m.jnt_range[j][0] for j in self.jid])
        self.hi = np.array([m.jnt_range[j][1] for j in self.jid])
        self.site = m.site(robot.prefix + site).id
        self.q_rest = np.array(q_rest, float)
        self.box_lo, self.box_hi = np.array(box_lo, float), np.array(box_hi, float)
        self.posture_gain = posture_gain
        # slide joints (a torso lift) move metres, not radians: give them a lower weight so the arm joints do most of it
        self.weights = np.array([0.3 if m.jnt_type[j] == 2 else 1.0 for j in self.jid])
        # joint speed limits for the actuator targets: 1.2 rad/s, 0.07 m/s for a torso lift (TIAGo's)
        self.rate = np.array([0.07 if m.jnt_type[j] == 2 else 1.2 for j in self.jid])
        import mujoco

        self.scratch = mujoco.MjData(m)
        self.reset()

    def reset(self) -> None:
        d = self.r.d
        self.q_cmd = d.qpos[self.qadr].copy()
        self.q_sp = self.q_cmd.copy()
        self.qi = np.zeros(len(self.q_cmd))
        self.tilt_f = np.zeros(3)
        self.joint_goal = None
        self.target_b = self.r.to_base(self.site_pos())
        self.moved = False

    def site_pos(self) -> np.ndarray:
        return self.r.d.site_xpos[self.site].copy()

    def site_R(self) -> np.ndarray:
        return self.r.d.site_xmat[self.site].reshape(3, 3).copy()

    def target_world(self) -> np.ndarray:
        return self.r.to_world(self.target_b)

    def set_target_world(self, p_w) -> None:
        self.target_b = np.clip(self.r.to_base(p_w), self.box_lo, self.box_hi)
        self.moved = True

    def nudge(self, dw) -> None:
        """Move the target by dw (world frame, metres); it is kept near the measured gripper point and in the work box."""
        x, y, yaw = self.r.base_pose()
        tb = self.target_b + _rotz(yaw).T @ np.asarray(dw, float)
        meas_b = self.r.to_base(self.site_pos())
        off = tb - meas_b
        n = float(np.linalg.norm(off))
        if n > self.LEAD and np.linalg.norm(dw) > 0:
            # only limit growth: allow motion that reduces the offset
            prev = float(np.linalg.norm(self.target_b - meas_b))
            if n > prev:
                tb = meas_b + off * max(prev, self.LEAD) / n
        self.target_b = np.clip(tb, self.box_lo, self.box_hi)
        self.moved = self.moved or bool(np.any(dw))

    def solve(self, target_b, R_b, q0, iters: int = 6) -> tuple[np.ndarray, float]:
        """IK in the robot frame: the scratch copy has the base at the origin, level, so a rocking base does not feed
        back into the arm (the arm holds its pose relative to the base, like a real arm controller)."""
        import mujoco

        m, s = self.r.m, self.scratch
        s.qpos[:] = self.r.d.qpos
        self.r.level_base(s.qpos, self.tilt_f)
        target_w, R_w = np.asarray(target_b, float), R_b
        q = q0.copy()
        err = 9.0
        n = len(q)
        for _ in range(iters):
            s.qpos[self.qadr] = q
            mujoco.mj_kinematics(m, s)
            mujoco.mj_comPos(m, s)
            p = s.site_xpos[self.site]
            R = s.site_xmat[self.site].reshape(3, 3)
            ep = target_w - p
            er = 0.5 * (np.cross(R[:, 0], R_w[:, 0]) + np.cross(R[:, 1], R_w[:, 1]) + np.cross(R[:, 2], R_w[:, 2]))
            err = float(np.linalg.norm(ep))
            jp = np.zeros((3, m.nv))
            jr = np.zeros((3, m.nv))
            mujoco.mj_jacSite(m, s, jp, jr, self.site)
            J = np.vstack([jp[:, self.dadr], self.ROT_W * jr[:, self.dadr]])
            e = np.concatenate([ep, self.ROT_W * er])
            free = np.ones(n, bool)
            for _ in range(
                4
            ):  # weighted damped least squares; joints that would pass a limit are locked and it is re-solved
                W = np.diag(self.weights * free)
                JW = J @ W
                A = JW @ J.T + self.LAM**2 * np.eye(6)
                Jpinv = W @ J.T @ np.linalg.inv(A)
                dq = Jpinv @ e
                dq += (np.eye(n) - Jpinv @ J) @ (W @ (self.posture_gain * (self.q_rest - q)))
                mx = float(np.max(np.abs(dq)))
                if mx > 0.25:
                    dq *= 0.25 / mx
                qn = q + dq
                bad = free & (((qn < self.lo) & (dq < 0)) | ((qn > self.hi) & (dq > 0)))
                if not bad.any():
                    break
                free &= ~bad
            q = np.clip(qn, self.lo, self.hi)
        return q, err

    def update(self) -> None:
        """Once per control step: IK for the target (fixed gripper orientation); the joint targets are slewed and
        integral-corrected in substep()."""
        d = self.r.d
        if self.joint_goal is not None:  # a joint-space move (stow / ready); the Cartesian target follows the gripper
            self.q_cmd = self.joint_goal.copy()
            self.target_b = self.r.to_base(self.site_pos())
            self.moved = False
            return
        # the base leans a little on its casters when the arm reaches out: compensate the slow (static) part of its tilt
        self.tilt_f += 0.1 * (self.r.base_tilt() - self.tilt_f)
        q, _ = self.solve(self.target_b, R_FORWARD, self.q_cmd)
        # keep the commanded joints close to the measured ones (no wind-up when the arm is blocked)
        qm = d.qpos[self.qadr]
        self.q_cmd = np.clip(q, qm - 0.3, qm + 0.3)
        self.moved = False

    def substep(self, dt: float) -> None:
        """Slew the actuator targets towards the IK solution at the joint speed limits (smooth, no jerks)."""
        d = self.r.d
        dq = self.q_cmd - self.q_sp
        k = float(np.max(np.abs(dq) / (self.rate * dt)))
        self.q_sp = self.q_sp + (dq / k if k > 1 else dq)  # all joints scaled together: a straight joint-space path
        # joint-space integral on the setpoint: removes the static error of soft servos (joint friction, payload)
        if k <= 1 and float(np.max(np.abs(d.qvel[self.dadr]))) < 0.08:  # only near rest (no wind-up during motion)
            self.qi = np.clip(self.qi + self.KI * (self.q_sp - d.qpos[self.qadr]) * dt, -self.QI_MAX, self.QI_MAX)
        d.ctrl[self.aid] = np.clip(self.q_sp + self.qi, self.lo, self.hi)

    def move_joints(self, q) -> None:
        self.joint_goal = np.clip(np.asarray(q, float), self.lo, self.hi)

    def joints_done(self, tol: float = 0.03) -> bool:
        return self.joint_goal is not None and float(np.max(np.abs(self.r.d.qpos[self.qadr] - self.joint_goal))) < tol

    def release_joints(self) -> None:
        """End a joint-space move: hold the gripper where it is and go back to Cartesian control."""
        self.joint_goal = None
        self.q_cmd = self.q_sp.copy()
        self.target_b = self.r.to_base(self.site_pos())

    def reach_error(self) -> float:
        return float(np.linalg.norm(self.target_world() - self.site_pos()))


# ------------------------------------------------------------------------------------------------------------------
# PAL TIAGo
# ------------------------------------------------------------------------------------------------------------------


class Tiago(DiffDriveMixin, Robot):
    key = "tiago"
    name = "PAL TIAGo"
    asset = "pal_tiago"
    cameras = {"head_camera": "head"}
    wheel_acts = ("wheel_left_joint_vel", "wheel_right_joint_vel")
    wheel_radius = 0.0985
    track = 0.4044
    wheel_max = 5.0  # upstream ctrlrange +-5 rad/s
    v_max = 0.45
    wz_max = math.radians(60)
    a_max = 0.5
    alpha_max = 1.2
    finger_bodies = ("gripper_left_finger_link", "gripper_right_finger_link")
    finger_sides = (("gripper_left_finger_link",), ("gripper_right_finger_link",))
    arm_root = "arm_1_link"
    ARM = [
        "torso_lift_joint",
        "arm_1_joint",
        "arm_2_joint",
        "arm_3_joint",
        "arm_4_joint",
        "arm_5_joint",
        "arm_6_joint",
        "arm_7_joint",
    ]
    ARM_ACTS = ["torso_lift_joint_position"] + [f"arm_position_{i}_position" for i in range(1, 8)]
    # stowed: the upstream `home` keyframe (arm folded at the side), torso at 0.15
    STOW = [0.15, 0.20, -1.34, -0.20, 1.94, -1.57, 1.37, 0.0]
    # ready: gripper ~0.55 m ahead of the base centre at 0.85 m, pointing forward (found with the IK below)
    READY = [0.317, 0.572, -0.558, -1.212, 2.134, 0.93, 1.228, 1.538]
    FINGER_OPEN = 0.045
    GRIP_MARGIN = 0.004

    def load_spec(self):
        import mujoco

        from ....assets import robot_dir

        return mujoco.MjSpec.from_file(str(robot_dir(self.asset) / "tiago_position.xml"))

    def edit_spec(self, r) -> None:
        # Finger collision: the upstream finger meshes collide as convex hulls, which fill the hook-shaped finger into a
        # wedge that touches objects only at its inner edge. They are kept as visuals, and each finger gets a flat rubber
        # pad box on its inner face (4 cm tall, 14.5 cm long, out to the fingertip).
        import mujoco

        for nm, sgn in (("gripper_left_finger_link", -1), ("gripper_right_finger_link", 1)):
            b = r.body(nm)
            for g in b.geoms:
                g.contype, g.conaffinity = 0, 0
            b.add_geom(
                name=f"{nm}_pad",
                type=mujoco.mjtGeom.mjGEOM_BOX,
                pos=[sgn * 0.004, 0, -0.1459],
                size=[0.004, 0.02, 0.0725],
                rgba=[0.1, 0.1, 0.1, 0],
                friction=[2.0, 0.01, 0.001],
                contype=1,
                conaffinity=1,
                group=3,
            )
        b = r.body("arm_7_link")
        b.add_site(name="grasp", pos=[0, 0, 0.262], size=[0.008, 0, 0], rgba=[1, 0, 0, 0], group=5)

    def bind(self, m, d) -> None:
        super().bind(m, d)
        self._bind_wheels()
        self.arm_q = np.array([m.jnt_qposadr[self.j(n).id] for n in self.ARM])
        self.fing = [self.j("gripper_left_finger_joint"), self.j("gripper_right_finger_joint")]
        self.fing_adr = [m.jnt_qposadr[f.id] for f in self.fing]
        self.fing_act = [self.act_id("gripper_left_finger_position"), self.act_id("gripper_right_finger_position")]
        self.head_act = [self.act_id("head_1_joint_position"), self.act_id("head_2_joint_position")]
        self.head_adr = [m.jnt_qposadr[self.j(n).id] for n in ("head_1_joint", "head_2_joint")]

    def init_state(self) -> None:
        m, d = self.m, self.d
        x, y, yaw = self.start
        adr = m.jnt_qposadr[self.j("reference").id]
        d.qpos[adr : adr + 7] = [x, y, 0.0, math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]
        d.qpos[self.arm_q] = self.STOW
        for a in self.fing_adr:
            d.qpos[a] = self.FINGER_OPEN
        self.grip_target = self.FINGER_OPEN
        d.qpos[self.head_adr[1]] = -0.3
        d.ctrl[self.head_act[1]] = -0.3

    def make_arm(self) -> None:
        self.arm = IKArm(
            self,
            self.ARM,
            self.ARM_ACTS,
            "grasp",
            q_rest=self.READY,
            box_lo=[0.15, -0.6, 0.25],
            box_hi=[1.0, 0.6, 1.45],
            posture_gain=0.08,
        )
        self.d.ctrl[self.arm.aid] = self.d.qpos[self.arm.qadr]

    def ee_pos(self) -> np.ndarray:
        return self.arm.site_pos()

    def gripper_opening(self) -> float:
        return float(np.clip(np.mean([self.d.qpos[a] for a in self.fing_adr]) / self.FINGER_OPEN, 0, 1))

    def grip(self, g: float) -> None:
        if g == 0:
            return
        meas = float(np.mean([self.d.qpos[a] for a in self.fing_adr]))
        t = self.grip_target - g * 0.1 * self.FINGER_OPEN
        t = float(np.clip(t, 0.0, self.FINGER_OPEN))
        if g > 0:
            t = max(t, meas - self.GRIP_MARGIN * 4)
        self.grip_target = t

    def substep(self, dt: float) -> None:
        self._drive_substep(dt)
        self.arm.substep(dt)
        for a in self.fing_act:
            self.d.ctrl[a] = self.grip_target

    def control_update(self) -> None:
        self.arm.update()

    def set_head(self, pan: float, tilt: float) -> None:
        self.d.ctrl[self.head_act[0]] = float(np.clip(pan, -1.309, 1.309))
        self.d.ctrl[self.head_act[1]] = float(np.clip(tilt, -1.0472, 0.785))

    def head_angles(self) -> tuple[float, float]:
        return float(self.d.qpos[self.head_adr[0]]), float(self.d.qpos[self.head_adr[1]])


# ------------------------------------------------------------------------------------------------------------------
# Google Robot (planar base added)
# ------------------------------------------------------------------------------------------------------------------


class GoogleRobot(Robot):
    key = "google"
    name = "Google Robot"
    asset = "google_robot"
    cameras = {"head_camera": "head"}
    v_max = 0.5
    wz_max = math.radians(60)
    a_max = 0.6
    finger_bodies = ("link_finger_right", "link_finger_left", "link_finger_tip_right", "link_finger_tip_left")
    finger_sides = (("link_finger_left", "link_finger_tip_left"), ("link_finger_right", "link_finger_tip_right"))
    arm_root = "link_torso"
    ARM = [
        "joint_torso",
        "joint_shoulder",
        "joint_bicep",
        "joint_elbow",
        "joint_forearm",
        "joint_wrist",
        "joint_gripper",
    ]
    STOW = [0.0, 0.0, 0.0, 2.6, 0.0, 1.1, 0.0]
    READY = [-0.784, 1.338, 1.849, 1.657, 0.442, -0.961, -1.875]
    FINGER_OPEN, FINGER_CLOSED = 0.01, 1.3

    def load_spec(self):
        import mujoco

        from ....assets import robot_dir

        return mujoco.MjSpec.from_file(str(robot_dir(self.asset) / "robot.xml"))

    def edit_spec(self, r) -> None:
        import mujoco

        b = r.body("base_link")
        b.pos = [0, 0, 0.06205 + 0.003]  # wheels 3 mm above the floor: the planar joints carry the base
        for nm, typ, ax in (
            ("base_x", mujoco.mjtJoint.mjJNT_SLIDE, [1, 0, 0]),
            ("base_y", mujoco.mjtJoint.mjJNT_SLIDE, [0, 1, 0]),
            ("base_yaw", mujoco.mjtJoint.mjJNT_HINGE, [0, 0, 1]),
        ):
            b.add_joint(name=nm, type=typ, axis=ax, damping=0.0, armature=0.0)
        b.add_camera(name="head_camera", pos=[0.13, 0, 1.30], xyaxes=[0, -1, 0, 0.45, 0, 0.89], fovy=60)
        g = r.body("link_gripper")
        g.add_site(name="grasp", pos=[0, 0, 0.115], size=[0.008, 0, 0], rgba=[1, 0, 0, 0], group=5)
        for i, a in enumerate(r.actuators):  # upstream actuators are unnamed
            a.name = a.name or f"a_{self.ARM[i] if i < 7 else ['joint_finger_right', 'joint_finger_left'][i - 7]}"
        for nm, gain, frc in (("base_x", 3000.0, 500.0), ("base_y", 3000.0, 500.0), ("base_yaw", 600.0, 150.0)):
            r.add_actuator(
                name=f"{nm}_vel",
                target=nm,
                trntype=mujoco.mjtTrn.mjTRN_JOINT,
                gaintype=mujoco.mjtGain.mjGAIN_FIXED,
                biastype=mujoco.mjtBias.mjBIAS_AFFINE,
                gainprm=[gain] + [0] * 9,
                biasprm=[0, 0, -gain] + [0] * 7,
                ctrlrange=[-1.5, 1.5],
                ctrllimited=True,
                forcerange=[-frc, frc],
                forcelimited=True,
            )

    def bind(self, m, d) -> None:
        super().bind(m, d)
        self.base_adr = [m.jnt_qposadr[self.j(n).id] for n in ("base_x", "base_y", "base_yaw")]
        self.base_act = [self.act_id(f"{n}_vel") for n in ("base_x", "base_y", "base_yaw")]
        self.fing_adr = [m.jnt_qposadr[self.j(n).id] for n in ("joint_finger_right", "joint_finger_left")]
        self.fing_act = [self.act_id(f"a_{n}") for n in ("joint_finger_right", "joint_finger_left")]

    def base_pose(self) -> tuple[float, float, float]:
        q = self.d.qpos
        return float(q[self.base_adr[0]]), float(q[self.base_adr[1]]), float(wrap(q[self.base_adr[2]]))

    def level_base(self, qpos, tilt=None) -> None:
        for a in self.base_adr:
            qpos[a] = 0.0

    def base_tilt(self) -> np.ndarray:
        return np.zeros(3)

    def init_state(self) -> None:
        d = self.d
        x, y, yaw = self.start
        for a, v in zip(self.base_adr, (x, y, yaw), strict=False):
            d.qpos[a] = v
        adr = [self.m.jnt_qposadr[self.j(n).id] for n in self.ARM]
        d.qpos[adr] = self.STOW
        for a in self.fing_adr:
            d.qpos[a] = self.FINGER_OPEN
        self.grip_target = self.FINGER_OPEN

    def make_arm(self) -> None:
        self.arm = IKArm(
            self,
            self.ARM,
            [f"a_{n}" for n in self.ARM],
            "grasp",
            q_rest=self.READY,
            box_lo=[0.2, -0.7, 0.15],
            box_hi=[1.0, 0.5, 1.5],
            posture_gain=0.08,
        )
        self.d.ctrl[self.arm.aid] = self.d.qpos[self.arm.qadr]

    def ee_pos(self) -> np.ndarray:
        return self.arm.site_pos()

    def gripper_opening(self) -> float:
        q = float(np.mean([self.d.qpos[a] for a in self.fing_adr]))
        return float(np.clip((self.FINGER_CLOSED - q) / (self.FINGER_CLOSED - self.FINGER_OPEN), 0, 1))

    def grip(self, g: float) -> None:
        if g == 0:
            return
        meas = float(np.mean([self.d.qpos[a] for a in self.fing_adr]))
        span = self.FINGER_CLOSED - self.FINGER_OPEN
        t = float(np.clip(self.grip_target + g * 0.1 * span, self.FINGER_OPEN, self.FINGER_CLOSED))
        if g > 0:
            t = min(t, meas + 0.5)
        self.grip_target = t

    def substep(self, dt: float) -> None:
        self._ramp(dt)
        yaw = self.d.qpos[self.base_adr[2]]
        c, s = math.cos(yaw), math.sin(yaw)
        vx = c * self.v_cur - s * self.vy_cur
        vy = s * self.v_cur + c * self.vy_cur
        d = self.d
        d.ctrl[self.base_act[0]] = vx
        d.ctrl[self.base_act[1]] = vy
        d.ctrl[self.base_act[2]] = self.wz_cur
        self.arm.substep(dt)
        for a in self.fing_act:
            d.ctrl[a] = self.grip_target

    def control_update(self) -> None:
        self.arm.update()


# ------------------------------------------------------------------------------------------------------------------
# Hello Robot Stretch 3
# ------------------------------------------------------------------------------------------------------------------


class Stretch(DiffDriveMixin, Robot):
    key = "stretch"
    name = "Hello Robot Stretch 3"
    asset = "hello_robot_stretch_3"
    cameras = {"d435i_camera_rgb": "head", "d405_rgb": "wrist"}
    wheel_acts = ("left_wheel_vel", "right_wheel_vel")
    wheel_gear = 3.0
    wheel_radius = 0.0508
    track = 0.3407
    wheel_max = 5.0
    v_max = 0.25
    wz_max = math.radians(60)
    a_max = 0.4
    alpha_max = 1.5
    finger_bodies = ("link_gripper_finger_left", "link_gripper_finger_right", "rubber_tip_left", "rubber_tip_right")
    finger_sides = (("link_gripper_finger_left", "rubber_tip_left"), ("link_gripper_finger_right", "rubber_tip_right"))
    arm_root = "link_lift"
    LIFT = (0.0, 1.1)
    EXT = (0.0, 0.52)
    WYAW = (-1.39, 4.42)
    WPITCH = (-1.57, 0.56)
    SLIDE_OPEN, SLIDE_CLOSED = 0.04, -0.02
    STOW = dict(lift=0.25, ext=0.0, wyaw=3.14, wpitch=-0.4, wroll=0.0)

    def load_spec(self):
        import mujoco

        from ....assets import robot_dir

        return mujoco.MjSpec.from_file(str(robot_dir(self.asset) / "stretch.xml"))

    def edit_spec(self, r) -> None:
        for a in r.actuators:
            if a.name in ("left_wheel_vel", "right_wheel_vel"):
                a.ctrlrange = [-15, 15]
        # grasp point: where the rubber fingertips meet when closing on a small object, 0.21 m out from the gripper body
        # (the upstream link_grasp_center, 0.23 m out, is where the open fingertips are)
        r.body("link_SG3_gripper_body").add_site(
            name="grasp", pos=[0, 0, 0.21], size=[0.008, 0, 0], rgba=[1, 0, 0, 0], group=5
        )

    def bind(self, m, d) -> None:
        super().bind(m, d)
        self._bind_wheels()
        self.q = {
            n: m.jnt_qposadr[self.j(n).id]
            for n in (
                "joint_lift",
                "joint_arm_l0",
                "joint_arm_l1",
                "joint_arm_l2",
                "joint_arm_l3",
                "joint_wrist_yaw",
                "joint_wrist_pitch",
                "joint_wrist_roll",
                "joint_gripper_slide",
                "joint_head_pan",
                "joint_head_tilt",
            )
        }
        self.a = {
            n: self.act_id(n)
            for n in ("lift", "arm", "wrist_yaw", "wrist_pitch", "wrist_roll", "gripper", "head_pan", "head_tilt")
        }
        self.site = m.site(self.prefix + "grasp").id

    def init_state(self) -> None:
        m, d = self.m, self.d
        x, y, yaw = self.start
        adr = m.jnt_qposadr[m.body_jntadr[self.root]]
        d.qpos[adr : adr + 7] = [x, y, 0.0, math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]
        s = self.STOW
        self.t = dict(lift=s["lift"], ext=s["ext"], wyaw=s["wyaw"], wpitch=s["wpitch"], wroll=s["wroll"], slide=0.0)
        d.qpos[self.q["joint_lift"]] = s["lift"]
        d.qpos[self.q["joint_wrist_yaw"]] = s["wyaw"]
        d.qpos[self.q["joint_wrist_pitch"]] = s["wpitch"]
        d.qpos[self.q["joint_gripper_slide"]] = 0.0
        d.qpos[self.q["joint_head_tilt"]] = -0.5
        self.head = [0.0, -0.5]
        self.qi = [0.0, 0.0]
        self._write()

    def make_arm(self) -> None:
        self.arm = None

    def lift(self) -> float:
        return float(self.d.qpos[self.q["joint_lift"]])

    def ext(self) -> float:
        return float(sum(self.d.qpos[self.q[f"joint_arm_l{k}"]] for k in range(4)))

    def wrist(self) -> tuple[float, float]:
        return float(self.d.qpos[self.q["joint_wrist_yaw"]]), float(self.d.qpos[self.q["joint_wrist_pitch"]])

    def ee_pos(self) -> np.ndarray:
        return self.d.site_xpos[self.site].copy()

    def gripper_opening(self) -> float:
        s = float(self.d.qpos[self.q["joint_gripper_slide"]])
        return float(np.clip((s - self.SLIDE_CLOSED) / (self.SLIDE_OPEN - self.SLIDE_CLOSED), 0, 1))

    def nudge(self, dlift: float, dext: float, dyaw: float = 0.0, dpitch: float = 0.0) -> None:
        """A non-zero delta sets that joint's target to (measured position + delta), so the joint moves at a speed set
        by the delta and stops within one step's delta of where it is told to stop; a zero delta holds the target."""
        t = self.t
        wy, wp = self.wrist()
        if dlift:
            t["lift"] = float(np.clip(self.lift() + dlift, *self.LIFT))
        if dext:
            t["ext"] = float(np.clip(self.ext() + dext, *self.EXT))
        if dyaw:
            t["wyaw"] = float(np.clip(wy + dyaw, *self.WYAW))
        if dpitch:
            t["wpitch"] = float(np.clip(wp + dpitch, *self.WPITCH))

    def grip(self, g: float) -> None:
        if g == 0:
            return
        meas = float(self.d.qpos[self.q["joint_gripper_slide"]])
        span = self.SLIDE_OPEN - self.SLIDE_CLOSED
        t = float(np.clip(self.t["slide"] - g * 0.1 * span, self.SLIDE_CLOSED, self.SLIDE_OPEN))
        if g > 0:
            t = max(t, meas - 0.012)
        self.t["slide"] = t

    def _write(self, dt: float = 0.0) -> None:
        d, a, t = self.d, self.a, self.t
        # integral on the lift / telescope setpoints (joint friction and payload), like the real motor controllers
        self.qi[0] = float(np.clip(self.qi[0] + 4.0 * (t["lift"] - self.lift()) * dt, -0.05, 0.05))
        self.qi[1] = float(np.clip(self.qi[1] + 6.0 * (t["ext"] - self.ext()) * dt, -0.1, 0.1))
        d.ctrl[a["lift"]] = float(np.clip(t["lift"] + self.qi[0], *self.LIFT))
        d.ctrl[a["arm"]] = float(np.clip(t["ext"] + self.qi[1], *self.EXT))
        d.ctrl[a["wrist_yaw"]] = t["wyaw"]
        d.ctrl[a["wrist_pitch"]] = t["wpitch"]
        d.ctrl[a["wrist_roll"]] = t["wroll"]
        d.ctrl[a["gripper"]] = t["slide"]
        d.ctrl[a["head_pan"]] = self.head[0]
        d.ctrl[a["head_tilt"]] = self.head[1]

    def substep(self, dt: float) -> None:
        self._drive_substep(dt)
        self._write(dt)

    def set_head(self, pan: float, tilt: float) -> None:
        self.head = [float(np.clip(pan, -4.04, 1.73)), float(np.clip(tilt, -1.53, 0.79))]

    def head_angles(self) -> tuple[float, float]:
        return float(self.d.qpos[self.q["joint_head_pan"]]), float(self.d.qpos[self.q["joint_head_tilt"]])


ROBOTS = {"stretch": Stretch, "tiago": Tiago, "google": GoogleRobot}
