"""ManiSkill3 simulation worker. Runs inside the separate ManiSkill virtualenv (mani-skill 3.0.1, SAPIEN 3.0.3, CPU torch),
because SAPIEN, torch and ManiSkill's own pins do not belong in the main robouse environment.

The robouse `maniskill` backend (maniskill.py) starts this script as a subprocess and talks to it over stdin/stdout: one
JSON request per line, one JSON response per line; a `render` response is followed by the raw RGB bytes. Only numpy,
torch, SAPIEN, pytorch_kinematics and mani_skill are imported here (never the robouse package).

Physics runs on ManiSkill's CPU backend (sim_backend="cpu", one environment, PhysX on the CPU); rendering uses SAPIEN's
Vulkan renderer on whatever Vulkan device exists (MoltenVK on macOS, Mesa lavapipe in the Linux image, no GPU needed).

Control. The upstream env runs in its `pd_joint_pos` control mode (absolute arm joint targets + the normalised gripper
target, 20 Hz). This worker adds a world-frame end-effector interface on top: it keeps a commanded TCP pose, moves it by
each action's deltas and solves damped-least-squares inverse kinematics (pytorch_kinematics on the robot's URDF, the
solver ManiSkill uses on GPU) for the joint targets it passes to the upstream `env.step`.
"""

from __future__ import annotations

import os
import sys

import numpy as np

STEP_M = 0.02  # metres per action unit per step (world frame)
STEP_RAD = 0.1  # radians per action unit per step (about the world axes, centred on the TCP)
LEAD_M = 0.05  # the commanded position stays within this distance of the measured TCP
LEAD_RAD = 0.35  # and the commanded orientation within this angle of the measured one
KI = 0.3  # integral gain of the tracking-error correction (per step)
I_MAX_M = 0.015  # bound on the position correction
I_MAX_RAD = 0.06  # bound on the orientation correction


def _clip_norm(v, m):
    n = float(np.linalg.norm(v))
    return v * (m / n) if n > m else v


def _skew(v):
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


def _rot(axis_angle) -> np.ndarray:
    v = np.asarray(axis_angle, dtype=float)
    a = float(np.linalg.norm(v))
    if a < 1e-12:
        return np.eye(3)
    K = _skew(v / a)
    return np.eye(3) + np.sin(a) * K + (1 - np.cos(a)) * K @ K


def _rotvec(R) -> np.ndarray:
    c = np.clip((np.trace(R) - 1) / 2, -1, 1)
    a = float(np.arccos(c))
    if a < 1e-9:
        return np.zeros(3)
    if a > np.pi - 1e-4:  # near 180 degrees: take the axis from the symmetric part
        w, V = np.linalg.eigh((R + R.T) / 2)
        return V[:, np.argmax(w)] * a
    return np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / (2 * np.sin(a)) * a


def _quat_to_mat(q_wxyz) -> np.ndarray:
    w, x, y, z = [float(v) for v in q_wxyz]
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def _mat_to_quat(R) -> np.ndarray:
    """Rotation matrix -> unit quaternion (w, x, y, z), ManiSkill's convention."""
    w = np.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = np.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2
    y = np.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2
    z = np.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2
    x = np.copysign(x, R[2, 1] - R[1, 2])
    y = np.copysign(y, R[0, 2] - R[2, 0])
    z = np.copysign(z, R[1, 0] - R[0, 1])
    q = np.array([w, x, y, z])
    return q / np.linalg.norm(q)


def _np(t) -> np.ndarray:
    return np.asarray(t.detach().cpu().numpy() if hasattr(t, "detach") else t, dtype=float)


def _pose(p) -> tuple[np.ndarray, np.ndarray]:
    """(position, rotation matrix) of a ManiSkill Pose (batched, first env)."""
    return _np(p.p)[0], _quat_to_mat(_np(p.q)[0])


def _r(v, n=4):
    return [round(float(x), n) for x in np.ravel(v)]


def _yaw_deg(R) -> float:
    return round(float(np.degrees(np.arctan2(R[1, 0], R[0, 0]))), 1)


class Worker:
    def __init__(self):
        self.env = None

    # ---- setup ---------------------------------------------------------------------------------------------------
    def make(
        self, env_id: str, seed: int, sensor_size: int = 256, render_size: int = 384, kwargs: dict | None = None
    ) -> dict:
        import gymnasium as gym
        import mani_skill.envs  # noqa: F401  (registers the environments)
        import torch

        torch.set_num_threads(1)

        if self.env is not None:
            self.env.close()
        self.env_id = env_id
        self.env = gym.make(
            env_id,
            obs_mode="state_dict",
            control_mode="pd_joint_pos",
            sim_backend="cpu",
            render_mode="rgb_array",
            reward_mode="none",
            num_envs=1,
            sensor_configs=dict(width=int(sensor_size), height=int(sensor_size)),
            human_render_camera_configs=dict(width=int(render_size), height=int(render_size)),
            **(kwargs or {}),
        )
        self.u = self.env.unwrapped
        self.env.reset(seed=int(seed))
        self._setup_ik()
        self.q_cmd = self._arm_q().copy()
        self.p_cmd, self.R_cmd = _pose(self.u.agent.tcp.pose)
        self.grip_target = 1.0  # ManiSkill's normalised gripper target: +1 open ... -1 closed
        self.I_p, self.I_r = np.zeros(3), np.zeros(3)
        return {
            "cameras": list(self.u._human_render_cameras.keys()) + list(self.u._sensors.keys()),
            "robot": self.u.agent.uid,
            "control_freq": self.u.control_freq,
        }

    def _setup_ik(self) -> None:
        import pytorch_kinematics as pk
        import torch

        agent = self.u.agent
        with open(agent.urdf_path, "rb") as f:
            urdf = f.read()
        import contextlib
        import io

        with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            self.chain = pk.build_serial_chain_from_urdf(urdf, end_link_name=agent.ee_link_name).to(dtype=torch.float64)
        names = self.chain.get_joint_parameter_names()
        active = [j.name for j in agent.robot.active_joints]
        self.arm_idx = [active.index(n) for n in names]
        lim = np.asarray(self.chain.get_joint_limits(), dtype=float)
        self.q_lo, self.q_hi = lim[0] + 1e-3, lim[1] - 1e-3
        self.q_rest = self._arm_q().copy()

    def _arm_q(self) -> np.ndarray:
        return _np(self.u.agent.robot.get_qpos())[0][self.arm_idx]

    def _base(self):
        return _pose(self.u.agent.robot.pose)

    def _ik(self, p_w: np.ndarray, R_w: np.ndarray, q0: np.ndarray, iters: int = 30) -> np.ndarray:
        """Damped least squares toward the world-frame TCP pose (p_w, R_w), starting at q0, with a small null-space
        pull toward the starting arm posture."""
        import torch

        bp, bR = self._base()
        p_t = bR.T @ (p_w - bp)
        R_t = bR.T @ R_w
        q = q0.copy()
        lam = 0.03
        for _ in range(iters):
            qt = torch.tensor(q[None], dtype=torch.float64)
            M = self.chain.forward_kinematics(qt).get_matrix()[0].numpy()
            e = np.r_[p_t - M[:3, 3], _rotvec(R_t @ M[:3, :3].T)]
            if np.linalg.norm(e[:3]) < 2e-4 and np.linalg.norm(e[3:]) < 2e-3:
                break
            J = self.chain.jacobian(qt)[0].numpy()
            JJt = J @ J.T + lam**2 * np.eye(6)
            dq = J.T @ np.linalg.solve(JJt, e)
            N = np.eye(len(q)) - J.T @ np.linalg.solve(JJt, J)
            dq += N @ (0.05 * (self.q_rest - q))
            n = np.linalg.norm(dq)
            if n > 0.3:
                dq *= 0.3 / n
            q = np.clip(q + dq, self.q_lo, self.q_hi)
        if os.environ.get("ROBOUSE_MANISKILL_DEBUG"):
            print(
                "ik",
                np.round(e, 4).tolist(),
                np.round(q, 2).tolist(),
                np.round(self._arm_q(), 2).tolist(),
                file=sys.stderr,
                flush=True,
            )
        return q

    # ---- stepping ------------------------------------------------------------------------------------------------
    def step(self, action) -> dict:
        a = np.clip(np.asarray(action, dtype=float), -1, 1)
        p_now, R_now = _pose(self.u.agent.tcp.pose)
        p = self.p_cmd + a[:3] * STEP_M
        off = p - p_now
        n = float(np.linalg.norm(off))
        if n > LEAD_M:  # a blocked arm does not wind up
            p = p_now + off * (LEAD_M / n)
        R = _rot(a[3:6] * STEP_RAD) @ self.R_cmd
        dv = _rotvec(R @ R_now.T)
        if np.linalg.norm(dv) > LEAD_RAD:
            R = _rot(dv * (LEAD_RAD / np.linalg.norm(dv))) @ R_now
        self.p_cmd, self.R_cmd = p, R
        g = float(a[6])
        if g > 1e-3:
            self.grip_target = -1.0
        elif g < -1e-3:  # -f opens to fraction f
            self.grip_target = float(-1.0 + 2.0 * min(1.0, -g))
        # integral correction: the joint PD controller sags under load (a held object, contact), so the IK target is the
        # commanded pose plus a bounded integral of the tracking error; the hand then settles on the commanded pose
        self.I_p = _clip_norm(self.I_p + KI * (self.p_cmd - p_now), I_MAX_M)
        self.I_r = _clip_norm(self.I_r + KI * _rotvec(self.R_cmd @ R_now.T), I_MAX_RAD)
        self.q_cmd = self._ik(self.p_cmd + self.I_p, _rot(self.I_r) @ self.R_cmd, self.q_cmd)
        self.env.step(np.r_[self.q_cmd, self.grip_target].astype(np.float32)[None])
        return {"success": self._success()}

    def _success(self) -> bool:
        return bool(_np(self.u.evaluate()["success"]).ravel()[0])

    def success(self) -> dict:
        info = self.u.evaluate()
        out = {}
        for k, v in info.items():
            try:
                arr = _np(v).ravel()
                out[k] = bool(arr[0]) if arr.dtype == bool or k.startswith(("is_", "success")) else _r(arr)
            except Exception:
                pass
        out["success"] = self._success()
        return out

    # ---- observation ---------------------------------------------------------------------------------------------
    def _obj(self, st: dict, name: str, actor, yaw: bool = True) -> None:
        p, R = _pose(actor.pose)
        st[f"{name}_pos"] = _r(p)
        st[f"{name}_quat"] = _r(_np(actor.pose.q)[0])
        if yaw:
            st[f"{name}_yaw_deg"] = _yaw_deg(R)

    def observe(self) -> dict:
        u = self.u
        agent = u.agent
        p, R = _pose(agent.tcp.pose)
        qf = _np(agent.robot.get_qpos())[0][-2:]
        st = {
            "hand_pos": _r(p),
            "hand_quat": _r(_np(agent.tcp.pose.q)[0]),
            "hand_down_axis": _r(R[:, 2], 3),
            "finger_axis": _r(R[:, 1], 3),
            "hand_target_pos": _r(self.p_cmd),
            "hand_target_quat": _r(_mat_to_quat(self.R_cmd)),
            "gripper_open": round(float(np.clip(np.mean(qf) / 0.04, 0, 1)), 3),
            "gripper_width": round(float(np.sum(qf)), 4),
        }
        e = self.env_id
        grasped = None
        if e in ("PickCube-v1", "PushCube-v1", "PullCube-v1"):
            self._obj(st, "cube", u.cube if e == "PickCube-v1" else u.obj)
            st["cube_half_size"] = 0.02
            grasped = u.cube if e == "PickCube-v1" else None
            if e == "PickCube-v1":
                st["goal_pos"] = _r(_np(u.goal_site.pose.p)[0])
                st["goal_radius"] = float(u.goal_thresh)
            else:
                st["goal_pos"] = _r(_np(u.goal_region.pose.p)[0])
                st["goal_radius"] = float(u.goal_radius)
        elif e == "StackCube-v1":
            self._obj(st, "red_cube", u.cubeA)
            self._obj(st, "green_cube", u.cubeB)
            st["cube_half_size"] = 0.02
            grasped = u.cubeA
        elif e == "StackPyramid-v1":
            self._obj(st, "red_cube", u.cubeA)
            self._obj(st, "green_cube", u.cubeB)
            self._obj(st, "blue_cube", u.cubeC)
            st["cube_half_size"] = 0.02
        elif e == "PokeCube-v1":
            self._obj(st, "peg", u.peg)
            st["peg_head_pos"] = _r(
                _np(u.peg_head_pose.p)[0]
            )  # the head end (upstream peg_head_pos ignores the rotation)
            st["peg_half_length"], st["peg_half_width"] = float(u.peg_half_length), float(u.peg_half_width)
            self._obj(st, "cube", u.cube)
            st["cube_half_size"] = float(u.cube_half_size)
            st["goal_pos"] = _r(_np(u.goal_region.pose.p)[0])
            st["goal_radius"] = float(u.goal_radius)
            grasped = u.peg
        elif e == "PlaceSphere-v1":
            self._obj(st, "sphere", u.obj, yaw=False)
            st["sphere_radius"] = float(u.radius)
            st["bin_pos"] = _r(_np(u.bin.pose.p)[0])
            st["bin_inner_half_size"] = _r(
                np.asarray(u.inner_side_half_len, dtype=float) if hasattr(u, "inner_side_half_len") else [0.0]
            )
            st["bin_floor_top_z"] = round(float(_np(u.bin.pose.p)[0][2] + float(u.block_half_size[0])), 4)
            grasped = u.obj
        elif e == "RollBall-v1":
            self._obj(st, "ball", u.ball, yaw=False)
            st["ball_radius"] = float(u.ball_radius)
            st["ball_vel"] = _r(_np(u.ball.linear_velocity)[0], 3)
            st["goal_pos"] = _r(_np(u.goal_region.pose.p)[0])
            st["goal_radius"] = float(u.goal_radius)
        elif e == "LiftPegUpright-v1":
            self._obj(st, "peg", u.peg, yaw=False)
            st["peg_axis"] = _r(_pose(u.peg.pose)[1][:, 0], 3)
            st["peg_half_length"], st["peg_half_width"] = float(u.peg_half_length), float(u.peg_half_width)
            st["robot_base_pos"] = _r(self._base()[0])
            grasped = u.peg
        elif e == "PegInsertionSide-v1":
            self._obj(st, "peg", u.peg)
            pR = _pose(u.peg.pose)[1]
            st["peg_axis"] = _r(pR[:, 0], 3)
            hs = _np(u.peg_half_sizes)[0]
            st["peg_half_length"], st["peg_radius"] = round(float(hs[0]), 4), round(float(hs[1]), 4)
            st["peg_head_pos"] = _r(
                _np(u.peg_head_pose.p)[0]
            )  # the head end (upstream peg_head_pos ignores the rotation)
            hp, hR = _pose(u.box_hole_pose)
            st["hole_pos"] = _r(hp)
            st["hole_axis"] = _r(hR[:, 0], 3)
            st["hole_radius"] = round(float(_np(u.box_hole_radii).ravel()[0]), 4)
            grasped = u.peg
        elif e == "PlugCharger-v1":
            self._obj(st, "charger", u.charger)
            cR = _pose(u.charger.pose)[1]
            st["charger_plug_axis"] = _r(cR[:, 0], 3)
            st["charger_base_pos"] = _r(_np(u.charger_base_pose.p)[0])
            st["charger_base_half_size"] = [float(x) for x in u._base_size]
            gp, gR = _pose(u.goal_pose)
            st["goal_pos"] = _r(gp)
            st["goal_quat"] = _r(_np(u.goal_pose.q)[0])
            st["goal_plug_axis"] = _r(gR[:, 0], 3)
            self._obj(st, "receptacle", u.receptacle)
            grasped = u.charger
        elif e == "PullCubeTool-v1":
            self._obj(st, "cube", u.cube)
            st["cube_half_size"] = float(u.cube_half_size)
            self._obj(st, "tool", u.l_shape_tool)
            tR = _pose(u.l_shape_tool.pose)[1]
            st["tool_handle_axis"] = _r(tR[:, 0], 3)
            st["tool_hook_end_pos"] = _r(_np(u.l_shape_tool.pose.p)[0] + tR @ np.array([float(u.handle_length), 0, 0]))
            st["tool_handle_length"], st["tool_hook_length"] = float(u.handle_length), float(u.hook_length)
            st["tool_width"], st["tool_height"] = float(u.width), float(u.height)
            st["robot_base_pos"] = _r(self._base()[0])
            grasped = u.l_shape_tool
        if grasped is not None:
            st["grasped"] = bool(_np(agent.is_grasping(grasped)).ravel()[0])
        return st

    def hand(self) -> dict:
        p, _ = _pose(self.u.agent.tcp.pose)
        return {"hand_pos": [float(x) for x in p]}

    # ---- rendering -----------------------------------------------------------------------------------------------
    def render(self, camera: str, width: int, height: int) -> np.ndarray:
        u = self.u
        if camera in u._sensors:
            imgs = u.get_sensor_images()
            img = imgs[camera]["rgb"] if isinstance(imgs[camera], dict) else imgs[camera]
        else:
            img = u.render_rgb_array(camera or None)
        img = _np(img)[0].astype(np.uint8) if img.ndim == 4 else _np(img).astype(np.uint8)
        if img.shape[0] != height or img.shape[1] != width:
            from PIL import Image

            img = np.asarray(Image.fromarray(img).resize((width, height)))
        return np.ascontiguousarray(img[..., :3])

    def camera(self, name: str) -> dict:
        u = self.u
        cam = u._sensors.get(name) or u._human_render_cameras.get(name)
        prm = cam.get_params()
        return {
            "extrinsic_cv": _np(prm["extrinsic_cv"])[0].tolist(),
            "intrinsic_cv": _np(prm["intrinsic_cv"])[0].tolist(),
            "width": int(cam.config.width),
            "height": int(cam.config.height),
        }
