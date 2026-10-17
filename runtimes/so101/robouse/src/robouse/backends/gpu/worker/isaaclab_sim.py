"""Isaac Lab Factory tasks for the GPU track (runs on the GPU worker; see server.py).

NVIDIA Isaac Lab 2.3 on Isaac Sim 5.1: the Factory contact-rich assembly environments (Narang et al., "Factory: Fast
Contact for Robotic Assembly", RSS 2022; Isaac Lab port `isaaclab_tasks.direct.factory`) with PhysX GPU dynamics and
RTX rendering. One episode = one process = one Isaac Sim app with one environment (num_envs=1).

The robot (Franka Panda) starts holding the part (peg, gear or nut) a few centimetres from the fixed part, as in
Factory. Actions are Factory's own: a 6-D end-effector command relative to the current fingertip pose, smoothed by an
exponential moving average and executed by Factory's task-space impedance controller at 15 Hz:
  robo act DX DY DZ RX RY RZ      one 1/15 s control step: DX..DZ in [-1, 1] x 2 cm target offset, RX..RZ in
                                  [-1, 1] x 0.097 rad; the target position is clamped to 5 cm around the fixed part.
                                  Roll and pitch are held level (Factory keeps the gripper pointing down).
                                  Nut threading: RZ is mapped to [-1, 0] x 0.097 rad (the nut only turns clockwise).
  robo skill hold [N]             zero command for N control steps (default 15)
"""

from __future__ import annotations

import math
import os

import numpy as np
from common import jpeg, pose_dict, r3

IMG_W, IMG_H = 640, 480
FRAME_EVERY = 2
# Isaac Sim 5.1's RTX renderer crashes at start-up on NVIDIA driver 595 (it runs on 580). Without RTX the physics
# still runs on the GPU, and images come from a MuJoCo (OpenGL, EGL) scene of the Factory parts' shapes whose poses
# are copied from Isaac Lab every frame. ROBOUSE_IL_RTX=1 uses Isaac Sim's own RTX viewport instead.
RTX = os.environ.get("ROBOUSE_IL_RTX", "0") == "1"

TASKS: dict[str, dict] = {}


def task(key, **kw):
    TASKS[key] = kw


task("factory-peg-insert", gym_id="Isaac-Factory-PegInsert-Direct-v0", max_steps=300)
task("factory-gear-mesh", gym_id="Isaac-Factory-GearMesh-Direct-v0", max_steps=400)
task("factory-nut-thread", gym_id="Isaac-Factory-NutThread-Direct-v0", max_steps=600)
task("factory-peg-insert-noisy", gym_id="Isaac-Factory-PegInsert-Direct-v0", max_steps=300, obs_noise=0.004)
task("factory-nut-thread-noisy", gym_id="Isaac-Factory-NutThread-Direct-v0", max_steps=600, obs_noise=0.003)


def make(key: str, seed: int, opts: dict):
    if key not in TASKS:
        raise KeyError(f"unknown Isaac Lab task {key!r}; known: {sorted(TASKS)}")
    return ILTask(key, seed, opts)


_APP = None


def _app():
    global _APP
    if _APP is None:
        from isaaclab.app import AppLauncher

        _APP = AppLauncher(headless=True, enable_cameras=RTX).app
    return _APP


class ILTask:
    def __init__(self, key: str, seed: int, opts: dict):
        self.key, self.cfg, self.seed = key, {**TASKS[key], **(opts or {})}, seed
        _app()
        import gymnasium as gym
        import isaaclab_tasks  # noqa: F401  (registers the environments)
        import torch
        from isaaclab_tasks.utils import parse_env_cfg

        self.torch = torch
        cfg = parse_env_cfg(self.cfg["gym_id"], device="cuda:0", num_envs=1)
        cfg.episode_length_s = 36000.0  # the episode server enforces the budget; never auto-reset
        cfg.viewer.resolution = (IMG_W, IMG_H)
        self.env = gym.make(self.cfg["gym_id"], cfg=cfg, render_mode="rgb_array" if RTX else None)
        self.u = self.env.unwrapped
        self.name = self.u.cfg_task.name
        self.frames: list[str] = []
        self._recording = False
        self.reset(seed)

    # ---- helpers -----------------------------------------------------------------------------------------------------
    def _np(self, t):
        return t[0].detach().cpu().numpy().astype(float)

    def _poses(self):
        from isaaclab_tasks.direct.factory import factory_utils as fu

        u = self.u
        hb_pos, hb_quat = fu.get_held_base_pose(
            u.held_pos, u.held_quat, self.name, u.cfg_task.fixed_asset_cfg, 1, u.device
        )
        tb_pos, tb_quat = fu.get_target_held_base_pose(
            u.fixed_pos, u.fixed_quat, self.name, u.cfg_task.fixed_asset_cfg, 1, u.device
        )
        return self._np(hb_pos), self._np(hb_quat), self._np(tb_pos), self._np(tb_quat)

    def _set_cam(self, cam: str) -> None:
        f = self._np(self.u.fixed_pos) + self._np(self.u.scene.env_origins)
        views = {
            "scene": ([0.45, 0.45, 0.3], [0.0, 0.0, 0.02]),
            "close": ([0.14, 0.14, 0.08], [0.0, 0.0, 0.02]),
            "side": ([0.0, 0.25, 0.05], [0.0, 0.0, 0.02]),
        }
        eye, tgt = views[cam]
        self.u.sim.set_camera_view(eye=list(f + np.array(eye)), target=list(f + np.array(tgt)))

    def _render(self, cam: str = "scene") -> np.ndarray:
        if not RTX:
            return self._mj_render(cam)
        self._set_cam(cam)
        img = self.env.render()
        if img is None:
            img = np.zeros((IMG_H, IMG_W, 3), np.uint8)
        return np.asarray(img)[..., :3]

    def _env_step(self, a) -> None:
        t = self.torch.tensor(np.asarray(a, dtype=np.float32)[None], device=self.u.device)
        self.env.step(t)
        self.n += 1
        if self._recording and self.n % FRAME_EVERY == 0 and len(self.frames) < 60:
            self.frames.append(jpeg(self._render("scene")))

    # ---- non-RTX images: a MuJoCo scene of the parts' shapes ---------------------------------------------------
    def _mj_setup(self) -> None:
        import mujoco

        fc = self.u.cfg_task.fixed_asset_cfg
        hc = self.u.cfg_task.held_asset_cfg
        n = self.name
        if n == "peg_insert":
            r_in, R_out = fc.diameter / 2, fc.diameter / 2 + 0.012
            fixed = "".join(
                f'<geom type="box" size="{(R_out - r_in) / 2} {R_out} {fc.height / 2}" pos="{x * (r_in + (R_out - r_in) / 2)} 0 {fc.height / 2}" rgba="0.55 0.57 0.6 1"/>'
                for x in (-1, 1)
            )
            fixed += "".join(
                f'<geom type="box" size="{r_in} {(R_out - r_in) / 2} {fc.height / 2}" pos="0 {y * (r_in + (R_out - r_in) / 2)} {fc.height / 2}" rgba="0.55 0.57 0.6 1"/>'
                for y in (-1, 1)
            )
            fixed += (
                f'<geom type="box" size="{R_out + 0.01} {R_out + 0.01} 0.002" pos="0 0 0.002" rgba="0.4 0.42 0.45 1"/>'
            )
            held = f'<geom type="cylinder" size="{hc.diameter / 2} {hc.height / 2}" pos="0 0 {hc.height / 2}" rgba="0.95 0.8 0.1 1"/>'
        elif n == "gear_mesh":
            off = fc.medium_gear_base_offset
            fixed = f'<geom type="box" size="0.08 0.03 {fc.base_height / 2}" pos="0 0 {fc.base_height / 2}" rgba="0.4 0.42 0.45 1"/>'
            for dx, rad in ((-0.05, 0.012), (0.05, 0.028)):
                fixed += f'<geom type="cylinder" size="{rad} 0.01" pos="{off[0] + dx} 0 {fc.base_height + 0.01}" rgba="0.7 0.5 0.2 1"/>'
            fixed += f'<geom type="cylinder" size="0.005 {fc.height / 2}" pos="{off[0]} 0 {fc.height / 2}" rgba="0.6 0.6 0.65 1"/>'
            held = f'<geom type="cylinder" size="{hc.diameter / 2} {hc.height / 4}" pos="0 0 {hc.height / 4}" rgba="0.2 0.6 0.9 1"/>'
        else:
            fixed = f'<geom type="cylinder" size="0.016 {fc.base_height / 2}" pos="0 0 {fc.base_height / 2}" rgba="0.4 0.42 0.45 1"/>'
            fixed += f'<geom type="cylinder" size="{fc.diameter / 2} {fc.height / 2}" pos="0 0 {fc.base_height + fc.height / 2}" rgba="0.6 0.6 0.65 1"/>'
            held = f'<geom type="cylinder" size="{hc.diameter / 2 + 0.005} {hc.height / 2}" pos="0 0 {hc.height / 2}" rgba="0.85 0.75 0.3 1"/>'
        xml = f"""<mujoco><visual><global offwidth="{IMG_W}" offheight="{IMG_H}"/><quality shadowsize="2048"/></visual>
<asset><texture name="g" type="2d" builtin="checker" rgb1="0.82 0.82 0.8" rgb2="0.75 0.75 0.73" width="256" height="256"/>
<material name="g" texture="g" texrepeat="20 20"/></asset>
<worldbody><light pos="0.3 0.3 1.2" dir="-0.3 -0.3 -1" diffuse="0.9 0.9 0.9"/><light pos="-0.5 0.2 1" dir="0.5 -0.2 -1" diffuse="0.4 0.4 0.4"/>
<geom type="plane" size="1 1 0.1" material="g"/>
<body name="fixed" mocap="true">{fixed}</body>
<body name="held" mocap="true">{held}</body>
<body name="hand" mocap="true"><geom type="box" size="0.03 0.1 0.03" pos="0 0 -0.08" rgba="0.92 0.92 0.92 1"/>
<geom type="box" size="0.01 0.01 0.025" pos="0 0.02 -0.025" rgba="0.2 0.2 0.2 1"/><geom type="box" size="0.01 0.01 0.025" pos="0 -0.02 -0.025" rgba="0.2 0.2 0.2 1"/>
<geom type="cylinder" size="0.045 0.15" pos="0 0 -0.26" rgba="0.95 0.95 0.95 1"/></body></worldbody></mujoco>"""
        self.mjm = mujoco.MjModel.from_xml_string(xml)
        self.mjd = mujoco.MjData(self.mjm)
        self.mjr = mujoco.Renderer(self.mjm, IMG_H, IMG_W)
        self.mujoco = mujoco

    def _mj_render(self, cam: str) -> np.ndarray:
        if not hasattr(self, "mjm"):
            self._mj_setup()
        mj, u = self.mujoco, self.u
        f = self._np(u.fixed_pos)
        poses = [
            (self._np(u.fixed_pos) - f, self._np(u.fixed_quat)),
            (self._np(u.held_pos) - f, self._np(u.held_quat)),
            (self._np(u.fingertip_midpoint_pos) - f, self._np(u.fingertip_midpoint_quat)),
        ]
        for i, (p, q) in enumerate(poses):
            self.mjd.mocap_pos[i] = p
            self.mjd.mocap_quat[i] = q
        mj.mj_forward(self.mjm, self.mjd)
        c = mj.MjvCamera()
        c.type = mj.mjtCamera.mjCAMERA_FREE
        c.lookat[:] = [0, 0, 0.02]
        c.distance, c.azimuth, c.elevation = {
            "scene": (0.45, 135, -25),
            "close": (0.16, 135, -15),
            "side": (0.2, 90, -3),
        }[cam]
        self.mjr.update_scene(self.mjd, camera=c)
        return self.mjr.render()

    # ---- server methods ----------------------------------------------------------------------------------------------
    def info(self) -> dict:
        rz = (
            "RZ in [-1, 1] maps to 0 ... -0.097 rad (clockwise only)" if self.name == "nut_thread" else "RZ x 0.097 rad"
        )
        return {
            "action": {
                "names": ["dx", "dy", "dz", "rx", "ry", "rz"],
                "low": [-1.0] * 6,
                "high": [1.0] * 6,
                "doc": f"Factory end-effector command per 1/15 s step: position target offset x 2 cm (clamped to 5 cm around the fixed part), rotation x 0.097 rad, roll and pitch held level; {rz}; smoothed by an exponential moving average",
            },
            "skills": ["hold [N] - zero command for N control steps (default 15)"],
            "camera": "scene",
            "cameras": ["scene", "close", "side"],
            "hold_last": 0.0,
            "max_steps": self.cfg["max_steps"],
            "embodiment": self.embodiment(),
        }

    def embodiment(self) -> dict:
        return {
            "spec_version": "1",
            "name": "franka-panda-factory",
            "kind": "arm",
            "step_s": 1 / 15,
            "action_groups": [
                {
                    "name": "arm.ee_delta",
                    "components": ["dx", "dy", "dz"],
                    "low": [-1] * 3,
                    "high": [1] * 3,
                    "units": "x 2 cm target offset",
                    "mode": "ee_delta_pos",
                    "frame": "world",
                    "hold": "zero",
                },
                {
                    "name": "arm.ee_rot_delta",
                    "components": ["rx", "ry", "rz"],
                    "low": [-1] * 3,
                    "high": [1] * 3,
                    "units": "x 0.097 rad (roll/pitch held level)",
                    "mode": "ee_delta_rot",
                    "frame": "world",
                    "hold": "zero",
                },
            ],
            "sensors": {
                "cameras": [
                    {"name": c, "mount": "world", "width": IMG_W, "height": IMG_H, "calibrated": False}
                    for c in ("scene", "close", "side")
                ],
                "proprioception": [{"name": "fingertip", "shape": [7]}],
                "state": [{"name": "held_part", "shape": [7]}, {"name": "fixed_part_estimate", "shape": [3]}],
            },
            "skills": [{"name": "hold", "impl": "backend"}],
            "budgets": {"max_steps": self.cfg["max_steps"]},
            "reward": {"dense": "sparse", "success_mode": "final"},
        }

    def reset(self, seed: int | None = None) -> dict:
        self.seed = self.seed if seed is None else int(seed)
        self.torch.manual_seed(self.seed)
        np.random.seed(self.seed)
        self.env.reset(seed=self.seed)
        self.n = 0
        self.plan = None
        rng = np.random.default_rng(self.seed + 3)
        noise = float(self.cfg.get("obs_noise", 0.0))
        self.fixed_noise = rng.normal(0, noise, 3) if noise else np.zeros(3)
        self.fixed_noise[2] = 0.0
        return {"frame": jpeg(self._render("scene")), "hold_last": 0.0}

    def step(self, action) -> dict:
        self._env_step(np.clip(np.asarray(action, dtype=float), -1, 1))
        return {"success": False, "frame": jpeg(self._render("scene")), "hold_last": 0.0}

    def skill(self, name: str, args: list) -> dict:
        self.frames = []
        self._recording = True
        try:
            if name != "hold":
                raise ValueError(f"unknown skill {name!r}; skills: hold")
            n = max(1, min(60, int(args[0]) if args else 15))
            for _ in range(n):
                self._env_step(np.zeros(6))
            ok, msg = True, f"held {n} steps"
        except ValueError as e:
            ok, msg = False, str(e)
        finally:
            self._recording = False
        out = {"ok": ok, "message": msg, "frames": self.frames, "frame": jpeg(self._render("scene")), "hold_last": 0.0}
        self.frames = []
        return out

    def observe(self, privileged: bool = False) -> dict:
        u = self.u
        hb, hq, tb, tq = self._poses()
        f = self._np(u.fixed_pos)
        fc = u.cfg_task.fixed_asset_cfg
        st = {
            "fingertip": pose_dict(self._np(u.fingertip_midpoint_pos), self._np(u.fingertip_midpoint_quat)),
            "held_part_base": pose_dict(hb, hq),
            "fixed_part_position_estimate": r3(f + self.fixed_noise, 4),
            "fixed_part_yaw_deg": round(
                math.degrees(2 * math.atan2(float(u.fixed_quat[0, 3]), float(u.fixed_quat[0, 0]))), 1
            ),
            "fixed_part": {
                "height_m": float(fc.height),
                "diameter_m": float(fc.diameter),
                "base_height_m": float(fc.base_height),
            },
            "steps": self.n,
        }
        if self.name == "gear_mesh":
            off = u.cfg_task.fixed_asset_cfg.medium_gear_base_offset
            st["gear_peg_offset_in_base_frame_m"] = [float(x) for x in off]
        if self.name == "nut_thread":
            st["thread_pitch_m"] = float(fc.thread_pitch)
        if privileged:
            if self.plan is None:
                self.plan = self._oracle()
            try:
                st["_oracle"] = {"next": next(self.plan)}
            except StopIteration:
                st["_oracle"] = {"next": ["done"]}
            st["_target_base"] = r3(tb, 5)
        return st

    def _success(self) -> bool:
        u = self.u
        check_rot = self.name == "nut_thread"
        return bool(u._get_curr_successes(success_threshold=u.cfg_task.success_threshold, check_rot=check_rot)[0])

    def success(self) -> dict:
        return {"success": self._success()}

    def render(self, camera: str | None = None) -> dict:
        return {"jpeg": jpeg(self._render(camera or "scene"))}

    def camera_info(self, cameras: list[str]) -> list[dict]:
        return [{"name": c, "width": IMG_W, "height": IMG_H, "projection": None} for c in cameras]

    def close(self) -> None:
        try:
            self.env.close()
        except Exception:
            pass

    # ---- reference policy (privileged: the true fixed-part pose) ---------------------------------------------------
    def _oracle(self):
        """Align the held part's base over its target, then descend (nut: descend while turning)."""
        for phase, z_above in (("align", 0.012), ("descend", -0.003)):
            for k in range(250):
                hb, _, tb, _ = self._poses()
                e = tb + np.array([0, 0, z_above]) - hb
                if phase == "align" and np.linalg.norm(e[:2]) < 0.0008 and abs(e[2]) < 0.004:
                    break
                if phase == "descend" and self._success():
                    break
                a = np.zeros(6)
                if phase == "descend":
                    a[:2] = np.clip(e[:2] / 0.02 * 1.0, -1, 1)
                    a[2] = -0.35 if self.name != "nut_thread" else -0.15
                    if self.name == "nut_thread":
                        a[5] = 1.0  # maps to the full clockwise turn increment
                    if self.name == "gear_mesh":  # wiggle the yaw so the teeth find the mesh with the other gears
                        a[5] = 0.5 * math.sin(k * 0.6)
                        a[2] = -0.25
                else:
                    a[:3] = np.clip(e / 0.02 * 1.0, -1, 1)
                    if self.name == "nut_thread":
                        a[5] = -1.0  # no turn
                yield ["act", [round(float(x), 4) for x in a]]
        for _ in range(3):
            yield ["skill", "hold", [10]]
