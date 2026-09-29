"""ManiSkill3 tasks for the GPU track (runs on the GPU worker; see server.py).

Physics: SAPIEN/PhysX. Rendering: SAPIEN's ray tracer ("rt" shader pack) on the GPU for every camera the agent or the
video sees. One episode = one process = one ManiSkill env with num_envs=1.

Robot interface (Franka Panda with a wrist camera, `panda_wristcam`; TwoRobot tasks: two Pandas):
  robo act DX DY DZ DRX DRY DRZ GRIP     one control step (1/20 s) of ManiSkill's pd_ee_delta_pose controller:
                                         translation of the TCP by up to 0.1 m per unit and rotation by up to 0.1 rad
                                         per unit (axis-angle, world-aligned axes); GRIP is the finger target,
                                         +1 open (4 cm per finger) ... -1 closed
  robo skill move X Y Z [R P Y | QW QX QY QZ]   motion-planned move of the TCP to a pose (screw motion, then RRT);
                                         orientation as roll/pitch/yaw in degrees or a quaternion; omitted = keep
  robo skill gripper open|close          open or close the fingers (holds the arm)
  robo skill wait [N]                    hold still for N control steps (default 10)
  robo skill depth CAMERA U V            RGB-D: the 3D world point seen at pixel (U, V) of CAMERA (camera-only tasks)
Every `act` step and every skill call is one step of the budget.

The TCP frame: +z points out between the fingers (the approach direction), +y is the axis along which the fingers
close. Pointing straight down with the fingers closing along world y is roll=180, pitch=0, yaw=0 (quat [0, 1, 0, 0]).
"""
from __future__ import annotations

import math
import os

import numpy as np

from common import jpeg, pose_dict, projection, quat_to_rpy_deg, r3, rpy_deg_to_quat

IMG_W, IMG_H = 640, 480
FRAME_EVERY = 3
MAX_FRAMES = 48
REFINE = 15  # extra control steps holding the final joint target, so the arm settles on the goal


# ---- task table -------------------------------------------------------------------------------------------------------
# key -> dict(env=ManiSkill env id, sub=subclass tweaks, fields=state fields, cams=cameras, oracle=plan method, ...)
TASKS: dict[str, dict] = {}


def task(key: str, **kw):
    TASKS[key] = kw


task("peg-insertion-side", env="PegInsertionSide-v1", oracle="plan_peg", max_steps=100)
task("peg-insertion-side-1mm", env="PegInsertionSide-v1", clearance=0.001, oracle="plan_peg", max_steps=100, fine_step=0.002)
task("peg-insertion-moving-box", env="PegInsertionSide-v1", oracle="plan_peg", max_steps=120, perturb="move_box")
task("peg-insertion-side-vision", env="PegInsertionSide-v1", oracle="plan_peg", max_steps=150, vision=True)
task("plug-charger", env="PlugCharger-v1", oracle="plan_charger", max_steps=40)
task("plug-charger-vision", env="PlugCharger-v1", oracle="plan_charger", max_steps=60, vision=True)
task("stack-pyramid", env="StackPyramid-v1", oracle="plan_pyramid", max_steps=40)
task("pull-cube-tool", env="PullCubeTool-v1", oracle="plan_pull_tool", max_steps=40)
task("pick-moving-cube", env="PickCube-v1", oracle="plan_moving_cube", max_steps=150, perturb="conveyor")
task("stack-pyramid-vision", env="StackPyramid-v1", oracle="plan_pyramid", max_steps=60, vision=True)
task("pick-moving-cube-vision", env="PickCube-v1", oracle="plan_moving_cube", max_steps=150, perturb="conveyor", vision=True)
task("lift-peg-upright", env="LiftPegUpright-v1", oracle="plan_peg_upright", max_steps=40)


def make(key: str, seed: int, opts: dict):
    if key not in TASKS:
        raise KeyError(f"unknown ManiSkill task {key!r}; known: {sorted(TASKS)}")
    return MSTask(key, seed, opts)


def _env_class(cfg: dict):
    import mani_skill.envs  # noqa: F401  (registers the envs)
    import sapien
    from mani_skill.sensors.camera import CameraConfig
    from mani_skill.utils import sapien_utils
    from mani_skill.utils.registration import REGISTERED_ENVS

    base = REGISTERED_ENVS[cfg["env"]].cls
    attrs = {}
    if "clearance" in cfg:
        attrs["_clearance"] = cfg["clearance"]

    def cams(self):
        eye, tgt = cfg.get("scene_cam", ([0.55, -0.55, 0.65], [0.0, 0.05, 0.1]))
        front = cfg.get("front_cam", ([0.9, 0.0, 0.45], [0.0, 0.0, 0.05]))
        top = cfg.get("top_cam", ([-0.3, 0.0, 1.0], [0.05, 0.0, 0.0]))
        out = [CameraConfig("scene", sapien_utils.look_at(eye, tgt), IMG_W, IMG_H, 1.0, 0.01, 100, shader_pack="rt"),
               CameraConfig("front", sapien_utils.look_at(*front), IMG_W, IMG_H, 1.0, 0.01, 100, shader_pack="rt"),
               CameraConfig("top", sapien_utils.look_at(*top), IMG_W, IMG_H, 1.0, 0.01, 100, shader_pack="rt")]
        links = getattr(getattr(self, "agent", None), "robot", None)
        if links is not None and "camera_link" in self.agent.robot.links_map:
            out.append(CameraConfig("wrist", sapien.Pose(), IMG_W, IMG_H, math.pi / 2, 0.01, 100, shader_pack="rt",
                                    mount=self.agent.robot.links_map["camera_link"]))
        return out

    attrs["_default_human_render_camera_configs"] = property(cams)
    return type(base.__name__ + "RoboUse", (base,), attrs)


class _Rec:
    """What the motion planner steps: the env plus Robo Use's per-step hooks (frames, perturbations)."""

    def __init__(self, env, owner):
        self.env, self.owner = env, owner
        self.unwrapped = env

    def step(self, action):
        return self.owner._env_step(action)

    def __getattr__(self, k):
        return getattr(self.env, k)


class MSTask:
    def __init__(self, key: str, seed: int, opts: dict):
        import torch

        torch.set_num_threads(4)
        self.key, self.cfg, self.seed = key, TASKS[key], seed
        cls = _env_class(self.cfg)
        self.env = cls(obs_mode="state_dict", control_mode="pd_joint_pos", render_mode="rgb_array",
                       sim_backend=self.cfg.get("sim_backend", os.environ.get("ROBOUSE_MS_SIM", "physx_cpu")),
                       reward_mode="sparse", num_envs=1,
                       sensor_configs=dict(width=128, height=128))
        self.rec = _Rec(self.env, self)
        self.vision = bool(self.cfg.get("vision"))
        self.cameras = [c for c in ("scene", "front", "top", "wrist") if c in self.env.scene.human_render_cameras]
        self.frames: list[str] = []
        self.reset(seed)

    # ---- low level ---------------------------------------------------------------------------------------------------
    @property
    def u(self):
        return self.env

    def _set_mode(self, mode: str) -> None:
        if self.u.agent.control_mode != mode:
            self.u.agent.set_control_mode(mode)
            self.u.agent.controller.reset()

    def _env_step(self, action):
        out = self.u.step(action)
        self.n_sim += 1
        self._hooks()
        if self._recording and self.n_sim % FRAME_EVERY == 0 and len(self.frames) < MAX_FRAMES:
            self.frames.append(jpeg(self._render("scene")))
        return out

    def _hooks(self) -> None:
        p = self.cfg.get("perturb")
        if p == "conveyor":
            self._conveyor()
        if p == "move_box" and not self._perturbed and self._peg_grasped_steps() >= 10 and self._peg_lifted():
            self._perturb_move_box()

    def _render(self, cam: str) -> np.ndarray:
        img = self.u.render_rgb_array(cam)
        return img[0].cpu().numpy() if hasattr(img, "cpu") else np.asarray(img)[0]

    def _pose(self, actor_or_pose):
        p = actor_or_pose.pose if hasattr(actor_or_pose, "pose") else actor_or_pose
        return p.p[0].cpu().numpy(), p.q[0].cpu().numpy()

    # ---- server methods ----------------------------------------------------------------------------------------------
    def info(self) -> dict:
        names = ["dx", "dy", "dz", "drx", "dry", "drz", "grip"]
        skills = ["move X Y Z [ROLL PITCH YAW | QW QX QY QZ] [speed=S] - motion-planned TCP move (degrees / quaternion; no orientation = keep; S in 0.05-1 slows it down)",
                  "gripper open|close|VALUE - open or close the fingers (VALUE in [-1, 1]: finger target; a partly closed grip lets a held object pivot)",
                  "wait [N] - hold still for N control steps (default 10)"]
        if self.vision:
            skills.append("depth CAMERA U V - 3D world point seen at pixel (U, V) of CAMERA (RGB-D camera)")
        return {
            "action": {"names": names, "low": [-1.0] * 7, "high": [1.0] * 7,
                       "doc": "ManiSkill pd_ee_delta_pose: TCP translation 0.1 m and rotation 0.1 rad per unit per step "
                              "(world-aligned axes); grip +1 open ... -1 closed (absolute finger target)"},
            "skills": skills, "camera": "scene", "cameras": self.cameras, "hold_last": 1.0,
            "max_steps": self.cfg.get("max_steps", 40), "embodiment": self.embodiment(),
        }

    def embodiment(self) -> dict:
        cams = [{"name": "scene", "mount": "world", "width": IMG_W, "height": IMG_H, "calibrated": True},
                {"name": "front", "mount": "world", "width": IMG_W, "height": IMG_H, "calibrated": True},
                {"name": "top", "mount": "world", "width": IMG_W, "height": IMG_H, "calibrated": True},
                {"name": "wrist", "mount": "wrist", "width": IMG_W, "height": IMG_H, "calibrated": True}]
        return {
            "spec_version": "1", "name": "panda-wristcam", "kind": "arm", "step_s": 0.05,
            "action_groups": [
                {"name": "arm.ee_delta", "components": ["dx", "dy", "dz"], "low": [-1] * 3, "high": [1] * 3,
                 "units": "x 0.1 m per step", "mode": "ee_delta_pos", "frame": "world", "hold": "zero"},
                {"name": "arm.ee_rot_delta", "components": ["drx", "dry", "drz"], "low": [-1] * 3, "high": [1] * 3,
                 "units": "x 0.1 rad per step (axis-angle)", "mode": "ee_delta_rot", "frame": "world", "hold": "zero"},
                {"name": "gripper", "components": ["grip"], "low": [-1], "high": [1], "units": "normalized",
                 "mode": "gripper", "hold": "last", "initial": [1], "doc": "+1 open, -1 closed"}],
            "sensors": {"cameras": cams,
                        "proprioception": [{"name": "tcp", "shape": [7], "units": "m, quaternion wxyz", "frame": "world"},
                                           {"name": "qpos", "shape": [7], "units": "rad"},
                                           {"name": "gripper", "shape": [], "units": "open/closed + width m"}],
                        "state": [{"name": k, "shape": [], "privileged": True} for k in self._object_fields()]},
            "skills": [
                {"name": "move", "impl": "backend", "args": [{"name": "x", "type": "float", "units": "m"},
                                                              {"name": "y", "type": "float", "units": "m"},
                                                              {"name": "z", "type": "float", "units": "m"},
                                                              {"name": "orientation", "type": "float", "optional": True,
                                                               "doc": "roll pitch yaw (deg) or qw qx qy qz"}],
                 "doc": "motion-planned TCP move (mplib screw motion, RRT-Connect fallback)"},
                {"name": "gripper", "impl": "backend", "args": [{"name": "state", "type": "enum", "choices": ["open", "close"]}]},
                {"name": "wait", "impl": "backend", "args": [{"name": "n", "type": "int", "optional": True, "default": 10}]}]
            + ([{"name": "depth", "impl": "backend", "args": [{"name": "camera", "type": "str"}, {"name": "u", "type": "int"},
                                                              {"name": "v", "type": "int"}],
                 "doc": "RGB-D deprojection of one pixel to a world point"}] if self.vision else []),
            "budgets": {"max_steps": self.cfg.get("max_steps", 40)},
            "reward": {"dense": "sparse", "success_mode": "final"},
        }

    def reset(self, seed: int | None = None) -> dict:
        from mani_skill.examples.motionplanning.panda.motionplanner import PandaArmMotionPlanningSolver

        self.seed = self.seed if seed is None else int(seed)
        self.u.reset(seed=self.seed, options=dict(reconfigure=True))
        self._set_mode("pd_joint_pos")
        self.planner = PandaArmMotionPlanningSolver(self.rec, debug=False, vis=False, base_pose=self.u.agent.robot.pose,
                                                    visualize_target_grasp_pose=False, print_env_info=False,
                                                    joint_vel_limits=0.75, joint_acc_limits=0.75)
        self.grip = 1.0
        self.n_sim = 0
        self._recording = False
        self._perturbed = False
        self._grasp_count = 0
        self.plan = None
        if self.cfg.get("perturb") == "conveyor":
            self._conveyor_init()
        return {"frame": jpeg(self._render("scene")), "hold_last": self.grip}

    def step(self, action) -> dict:
        a = np.clip(np.asarray(action, dtype=np.float32), -1, 1)
        self._set_mode("pd_ee_delta_pose")
        self._env_step(a[None])
        self.grip = float(a[6])
        self.planner.gripper_state = self.grip
        self._set_mode("pd_joint_pos")
        return {"success": self._success(), "frame": jpeg(self._render("scene")), "hold_last": self.grip}

    def skill(self, name: str, args: list) -> dict:
        self.frames = []
        self._recording = True
        try:
            msg = self._skill(name, [str(a) for a in args])
            ok = True
        except (ValueError, RuntimeError) as e:
            ok, msg = False, str(e)
        finally:
            self._recording = False
        out = {"ok": ok, "message": msg, "tcp": pose_dict(*self._pose(self.u.agent.tcp)), "frames": self.frames,
               "frame": jpeg(self._render("scene")), "hold_last": self.grip}
        self.frames = []
        return out

    def _skill(self, name: str, args: list[str]) -> str:
        import sapien

        if name == "move":
            speed = 1.0
            kw = [a for a in args if a.startswith("speed=")]
            if kw:
                speed = min(1.0, max(0.05, float(kw[-1].split("=", 1)[1])))
                args = [a for a in args if not a.startswith("speed=")]
            v = [float(a) for a in args]
            if len(v) == 3:
                q = self._pose(self.u.agent.tcp)[1]
            elif len(v) == 6:
                q = np.array(rpy_deg_to_quat(*v[3:]))
            elif len(v) == 7:
                q = np.array(v[3:]) / max(1e-9, np.linalg.norm(v[3:]))
            else:
                raise ValueError("move takes X Y Z, X Y Z ROLL PITCH YAW (degrees) or X Y Z QW QX QY QZ")
            target = sapien.Pose(p=v[:3], q=q)
            self._set_mode("pd_joint_pos")
            self.planner.gripper_state = self.grip
            pl = self.planner.planner
            vel0, acc0 = pl.joint_vel_limits.copy(), pl.joint_acc_limits.copy()
            pl.joint_vel_limits, pl.joint_acc_limits = vel0 * speed, acc0 * speed
            try:
                res = self.planner.move_to_pose_with_screw(target, refine_steps=REFINE)
                how = "screw"
                if res == -1:
                    res = self.planner.move_to_pose_with_RRTConnect(target, refine_steps=REFINE)
                    how = "sampling planner"
            finally:
                pl.joint_vel_limits, pl.joint_acc_limits = vel0, acc0
            if res == -1:
                raise RuntimeError("no motion plan to that pose (out of reach, or the arm would collide)")
            self._ik_finish(target)
            p, q2 = self._pose(self.u.agent.tcp)
            err = float(np.linalg.norm(p - np.asarray(v[:3])))
            ang = math.degrees(2 * math.acos(min(1.0, abs(float(np.dot(q2, q))))))
            return f"moved ({how}); TCP now {r3(p)}; position error {err * 1000:.1f} mm, orientation error {ang:.1f} deg"
        if name == "gripper":
            if not args:
                raise ValueError("gripper open|close|VALUE")
            if args[0] in ("open", "close"):
                val = 1.0 if args[0] == "open" else -1.0
            else:
                try:
                    val = max(-1.0, min(1.0, float(args[0])))
                except ValueError:
                    raise ValueError("gripper open|close|VALUE (VALUE in [-1, 1]: finger target, -1 closed, +1 open)")
            self._set_mode("pd_joint_pos")
            if val > 0:
                self.planner.open_gripper(t=10, gripper_state=val)
            else:
                self.planner.close_gripper(t=10, gripper_state=val)
            self.grip = val
            return f"gripper {args[0]}; finger gap {self._finger_gap() * 1000:.1f} mm"
        if name == "wait":
            n = int(args[0]) if args else 10
            n = max(1, min(100, n))
            self._set_mode("pd_joint_pos")
            qpos = self.u.agent.robot.get_qpos()[0, :7].cpu().numpy()
            for _ in range(n):
                self._env_step(np.hstack([qpos, self.grip]))
            return f"waited {n} steps"
        if name == "depth":
            if not self.vision:
                raise ValueError("no depth camera in this task")
            cam, uu, vv = args[0], int(float(args[1])), int(float(args[2]))
            pt = self._deproject(cam, uu, vv)
            return f"camera {cam} pixel ({uu}, {vv}) sees world point {pt}"
        raise ValueError(f"unknown skill {name!r}; skills: move, gripper, wait" + (", depth" if self.vision else ""))

    def _ik_finish(self, target) -> None:
        """The screw planner stops a few mm short of its goal; finish with an exact IK solution reached by a short
        joint-space interpolation (only when the remaining error exceeds 0.5 mm or 0.2 degrees)."""
        p, q = self._pose(self.u.agent.tcp)
        ang = math.degrees(2 * math.acos(min(1.0, abs(float(np.dot(q, target.q))))))
        if np.linalg.norm(p - target.p) < 0.0005 and ang < 0.2:
            return
        pl = self.planner.planner
        cur = self.u.agent.robot.get_qpos()[0].cpu().numpy()
        goal = pl.transform_goal_to_wrt_base(np.concatenate([target.p, target.q]))
        status, sols = pl.IK(goal, cur, n_init_qpos=4, threshold=1e-4)
        if status != "Success" or not sols:
            return
        n = len(pl.move_group_joint_indices)
        sol = min(sols, key=lambda x: np.linalg.norm(x[:n] - cur[:n]))[:n]
        if np.abs(sol - cur[:n]).max() > 0.3:  # a different arm configuration: not a small correction
            return
        steps = 8
        path = np.array([cur[:n] + (sol - cur[:n]) * (i + 1) / steps for i in range(steps)])
        self.planner.follow_path({"position": path}, refine_steps=REFINE)

    def _finger_gap(self) -> float:
        q = self.u.agent.robot.get_qpos()[0].cpu().numpy()
        return float(q[-1] + q[-2])

    def observe(self, privileged: bool = False) -> dict:
        tp, tq = self._pose(self.u.agent.tcp)
        st = {"tcp": pose_dict(tp, tq), "qpos": r3(self.u.agent.robot.get_qpos()[0, :7].cpu().numpy()),
              "gripper": {"command": "open" if self.grip > 0 else "closed", "finger_gap_m": round(self._finger_gap(), 4)}}
        st.update(self._object_state())
        if privileged:
            if self.plan is None:
                self.plan = getattr(self, self.cfg["oracle"])()
            try:
                nxt = next(self.plan)
            except StopIteration:
                nxt = ["done"]
            st["_oracle"] = {"next": nxt}
        return st

    def _object_fields(self) -> list[str]:
        return list(self._object_state().keys())

    def _object_state(self) -> dict:
        info = self.u.get_info()
        extra = self.u._get_obs_extra(info)
        out = {}
        for k, v in extra.items():
            if k == "tcp_pose":
                continue
            a = v[0].cpu().numpy() if hasattr(v, "cpu") else np.asarray(v)
            if k.endswith("_pose") and a.shape[-1] == 7:
                out[k] = pose_dict(a[:3], a[3:])
            else:
                out[k] = r3(a) if a.ndim else round(float(a), 5)
        return out

    def render(self, camera: str | None = None) -> dict:
        cam = camera or "scene"
        if cam not in self.cameras:
            raise ValueError(f"unknown camera {cam!r}; cameras: {self.cameras}")
        return {"jpeg": jpeg(self._render(cam))}

    def camera_info(self, cameras: list[str]) -> list[dict]:
        out = []
        for name in cameras:
            cam = self.u.scene.human_render_cameras[name]
            prm = cam.get_params()
            K = prm["intrinsic_cv"][0].cpu().numpy()
            E = prm["extrinsic_cv"][0].cpu().numpy()
            c2w = prm["cam2world_gl"][0].cpu().numpy()
            out.append({"name": name, "width": IMG_W, "height": IMG_H, "fovy_deg": round(math.degrees(2 * math.atan(IMG_H / 2 / K[1, 1])), 2),
                        "position": r3(c2w[:3, 3]), "projection": projection(K, E)})
        return out

    def _deproject(self, cam: str, u: int, v: int) -> list[float]:
        c = self.u.scene.human_render_cameras[cam]
        self.u.scene.update_render(update_sensors=False, update_human_render_cameras=True)
        c.capture()
        pos = c.get_obs(rgb=False, depth=False, position=True, segmentation=False)["position"]
        pos = pos[0].detach().cpu().numpy().astype(np.float64) / 1000.0  # mm (int16) -> m, OpenGL camera frame
        if not (0 <= u < pos.shape[1] and 0 <= v < pos.shape[0]):
            raise ValueError(f"pixel out of range ({pos.shape[1]}x{pos.shape[0]})")
        p = pos[v, u, :3]
        if not np.isfinite(p).all() or np.linalg.norm(p) < 1e-6:
            raise ValueError("no depth at that pixel (background)")
        c2w = c.get_params()["cam2world_gl"][0].cpu().numpy()
        w = c2w[:3, :3] @ p + c2w[:3, 3]
        w = w + np.random.default_rng().normal(0, 0.002, 3)  # depth-sensor noise, 2 mm
        return r3(w)

    def _success(self) -> bool:
        return bool(self.u.evaluate()["success"][0])

    def success(self) -> dict:
        return {"success": self._success()}

    def close(self) -> None:
        self.env.close()

    # ---- perturbations -----------------------------------------------------------------------------------------------
    def _peg_lifted(self) -> bool:
        """The held peg is at least 6 cm above the table and at least 15 cm from the hole entrance."""
        rel = self.u.box_hole_pose.sp.inv() * self.u.peg_head_pose.sp
        return float(self.u.peg.pose.sp.p[2]) > 0.06 and float(np.linalg.norm(rel.p)) > 0.15

    def _conveyor_init(self) -> None:
        """The cube starts near one side of the table and slides across it at a constant 5 cm/s (as if on a
        conveyor belt: it is displaced by 2.5 mm every control step) until it is grasped or lifted more than 5 cm;
        past y = +-0.3 m the belt carries it off the end, out of reach (moved to y = +-1.5 m)."""
        import torch
        from mani_skill.utils.structs import Pose

        rng = np.random.default_rng(self.seed + 11)
        self.cdir = float(rng.choice([-1.0, 1.0]))
        p, q = self._pose(self.u.cube)
        p2 = np.array([rng.uniform(-0.05, 0.08), -0.2 * self.cdir, p[2]])
        self.u.cube.set_pose(Pose.create_from_pq(torch.tensor(p2[None], dtype=torch.float32), torch.tensor(q[None], dtype=torch.float32)))
        self.cvel = np.array([0.0, 0.05 * self.cdir, 0.0])
        self._gone = False

    def _conveyor(self) -> None:
        import torch

        from mani_skill.utils.structs import Pose

        p, _ = self._pose(self.u.cube)
        if p[2] > 0.05 or bool(self.u.agent.is_grasping(self.u.cube)[0]) or self._gone:
            return
        if abs(p[1]) > 0.3:  # the belt carries the cube off the end: it drops out of reach
            pose = self.u.cube.pose
            p2 = pose.p.clone()
            p2[0, 1] = float(np.sign(p[1])) * 1.5
            p2[0, 2] = 0.02
            self.u.cube.set_pose(Pose.create_from_pq(p2, pose.q))
            self._gone = True
            return

        pose = self.u.cube.pose
        p2 = pose.p.clone()
        p2[0, :] += torch.tensor(self.cvel * self.u.control_timestep, dtype=p2.dtype, device=p2.device)
        self.u.cube.set_pose(Pose.create_from_pq(p2, pose.q))

    def _near_hole(self) -> bool:
        """The peg's head is within 5 cm of the hole entrance (the agent is lining it up)."""
        rel = self.u.box_hole_pose.sp.inv() * self.u.peg_head_pose.sp
        L = float(self.u.peg_half_sizes[0, 0])
        return float(np.linalg.norm(rel.p - np.array([-L, 0.0, 0.0]))) < 0.05

    def _peg_grasped_steps(self) -> int:
        if self.u.agent.is_grasping(self.u.peg)[0]:
            self._grasp_count += 1
        return self._grasp_count

    def _perturb_move_box(self) -> None:
        """Once the peg has been held for 10 control steps and lifted clear of the table (still at least 15 cm from the
        hole), the box slides 6-10 cm and turns 12-20 degrees (as if bumped), staying inside the region the task
        samples boxes from (within reach)."""
        import torch
        from mani_skill.utils.structs import Pose

        from common import quat_mul

        rng = np.random.default_rng(self.seed + 7)
        p, q = self._pose(self.u.box)
        yaw0 = 2 * math.atan2(q[3], q[0])
        for _ in range(50):
            d = rng.uniform(0.06, 0.10)
            th = rng.uniform(0, 2 * math.pi)
            p2 = p + np.array([d * math.cos(th), d * math.sin(th), 0.0])
            dyaw = rng.choice([-1, 1]) * rng.uniform(math.radians(12), math.radians(20))
            if -0.08 <= p2[0] <= 0.08 and 0.2 <= p2[1] <= 0.4 and abs(yaw0 + dyaw - math.pi / 2) <= math.radians(30):
                break
        yaw_q = np.array([math.cos(dyaw / 2), 0, 0, math.sin(dyaw / 2)])
        q2 = quat_mul(yaw_q, q)
        self.u.box.set_pose(Pose.create_from_pq(torch.tensor(p2[None], dtype=torch.float32), torch.tensor(q2[None], dtype=torch.float32)))
        self._perturbed = True

    # ---- reference policies (privileged; yield one action at a time) -------------------------------------------------
    @staticmethod
    def _mv(pose, speed: float | None = None) -> list:
        args = [round(float(x), 6) for x in list(pose.p) + list(pose.q)]
        return ["skill", "move", args + ([f"speed={speed}"] if speed else [])]

    def _grasp_pose(self, actor, depth=0.025, center=None):
        from mani_skill.examples.motionplanning.base_motionplanner.utils import compute_grasp_info_by_obb, get_actor_obb

        obb = get_actor_obb(actor)
        approaching = np.array([0, 0, -1])
        closing0 = self.u.agent.tcp.pose.to_transformation_matrix()[0, :3, 1].cpu().numpy()
        gi = compute_grasp_info_by_obb(obb, approaching=approaching, target_closing=closing0, depth=depth)
        return self.u.agent.build_grasp_pose(approaching, gi["closing"], gi["center"] if center is None else center)

    def plan_peg(self):
        import sapien

        env = self.u
        L = env.peg_half_sizes[0, 0].item()
        grasp = self._grasp_pose(env.peg) * sapien.Pose([-max(0.05, L / 2 + 0.01), 0, 0])
        yield self._mv(grasp * sapien.Pose([0, 0, -0.05]))
        yield self._mv(grasp)
        yield ["skill", "gripper", ["close"]]
        yield self._mv(sapien.Pose([0, 0, 0.10]) * grasp)
        # where the TCP must be for the peg to sit 1 cm in front of the hole, from the peg's pose in the hand now
        off = sapien.Pose([-0.01 - L, 0, 0])
        peg_in_tcp = env.agent.tcp.pose.sp.inv() * env.peg.pose.sp
        pre = env.goal_pose.sp * off * peg_in_tcp.inv()
        yield self._mv(pre * sapien.Pose([0, 0, 0]) if False else sapien.Pose(pre.p - self._axis(env.goal_pose.sp) * 0.05, pre.q))
        yield self._mv(pre)
        for _ in range(10):  # close the loop on the peg itself: shift the commanded pose by the peg's remaining error
            err = (env.goal_pose.sp * off).inv() * env.peg.pose.sp
            ang = 2 * math.degrees(math.acos(min(1.0, abs(float(err.q[0])))))
            if np.linalg.norm(err.p) < 0.0006 and ang < 0.25:
                break
            delta = env.goal_pose.sp * off * env.peg.pose.sp.inv()
            pre = delta * pre
            yield self._mv(pre, speed=0.3)
        d = self._axis(env.goal_pose.sp)
        depth = 0.0
        while depth < 0.01 + L + 0.015:  # push in slowly; the commanded pose integrates the peg's remaining error
            depth += self.cfg.get("fine_step", 0.004) if depth < 0.018 else 0.025  # small steps across the entrance
            tgt = env.goal_pose.sp * sapien.Pose([-0.01 - L + depth, 0, 0])
            pre = (tgt * env.peg.pose.sp.inv()) * pre
            yield self._mv(pre, speed=0.2)

    def _servo_insert(self, head_fn, hole_fn, goal_x: float, adv: float = 0.008, max_steps: int = 80):
        for _ in range(max_steps):
            head, hole = head_fn(), hole_fn()
            R = hole.to_transformation_matrix()[:3, :3]
            rel = hole.inv() * head
            if rel.p[0] >= goal_x:
                return
            Rh = head.to_transformation_matrix()[:3, :3]
            ax_h, ax_t = Rh[:, 0], R[:, 0]
            rotvec = np.cross(ax_h, ax_t)
            dpos = ax_t * adv + R @ np.array([0.0, -rel.p[1], -rel.p[2]]) * 0.8
            drot = rotvec * 0.0
            a = np.r_[np.clip(dpos / 0.1, -1, 1), np.clip(drot / 0.1, -1, 1), -1.0]
            yield ["act", [round(float(x), 5) for x in a]]

    @staticmethod
    def _axis(pose) -> np.ndarray:
        return pose.to_transformation_matrix()[:3, 0]

    def plan_charger(self):
        import sapien
        import trimesh
        from transforms3d.euler import euler2quat
        from mani_skill.examples.motionplanning.base_motionplanner.utils import compute_grasp_info_by_obb

        env = self.u
        obb = trimesh.primitives.Box(extents=np.array(env._base_size) * 2, transform=env.charger_base_pose.sp.to_transformation_matrix())
        closing0 = env.agent.tcp.pose.sp.to_transformation_matrix()[:3, 1]
        gi = compute_grasp_info_by_obb(obb, approaching=np.array([0, 0, -1]), target_closing=closing0, depth=0.025)
        grasp = env.agent.build_grasp_pose(np.array([0, 0, -1]), gi["closing"], gi["center"]) * sapien.Pose(q=euler2quat(0, np.deg2rad(15), 0))
        yield self._mv(grasp * sapien.Pose([0, 0, -0.05]))
        yield self._mv(grasp)
        yield ["skill", "gripper", ["close"]]
        yield self._mv(sapien.Pose([0, 0, 0.05]) * env.agent.tcp.pose.sp)
        off = sapien.Pose([-0.05, 0, 0])
        cmd = env.goal_pose.sp * off * env.charger.pose.sp.inv() * env.agent.tcp.pose.sp
        yield self._mv(cmd)
        yield from self._align_insert(lambda: env.charger.pose.sp, lambda: env.goal_pose.sp, cmd, 0.05, 0.05,
                                      fine=0.002, fine_until=0.012, coarse=0.01)

    def _align_insert(self, obj_fn, goal_fn, cmd, standoff: float, travel: float, fine: float, fine_until: float,
                      coarse: float):
        """Closed-loop alignment and insertion along the goal frame's +x axis. `cmd` is the TCP pose last commanded;
        every correction is applied to it (an integrator), so the arm's steady-state sag is cancelled."""
        import sapien

        off = sapien.Pose([-standoff, 0, 0])
        for _ in range(10):
            err = (goal_fn() * off).inv() * obj_fn()
            ang = 2 * math.degrees(math.acos(min(1.0, abs(float(err.q[0])))))
            if np.linalg.norm(err.p) < 0.0005 and ang < 0.25:
                break
            cmd = (goal_fn() * off * obj_fn().inv()) * cmd
            yield self._mv(cmd, speed=0.3)
        depth = 0.0
        while depth < travel - 1e-9:
            depth = min(travel, depth + (fine if depth < fine_until else coarse))
            tgt = goal_fn() * sapien.Pose([-standoff + depth, 0, 0])
            cmd = (tgt * obj_fn().inv()) * cmd
            yield self._mv(cmd, speed=0.2)

    def plan_pyramid(self):
        import sapien
        from transforms3d.euler import euler2quat

        env = self.u
        h = float(env.cube_half_size[2]) if not hasattr(env.cube_half_size, "cpu") else float(env.cube_half_size.flatten()[-1])
        # A next to B, then C on top of both (the pyramid): pick A, place beside B; pick C, place on the pair
        grasp = self._grasp_pose(env.cubeA, center=env.cubeA.pose.sp.p)
        b = env.cubeB.pose.sp.p
        a = env.cubeA.pose.sp.p
        d = a[:2] - b[:2]
        d = d / max(1e-6, np.linalg.norm(d))
        tgt = np.array([b[0] + d[0] * 2 * h * 1.05, b[1] + d[1] * 2 * h * 1.05, b[2]])
        yield self._mv(grasp * sapien.Pose([0, 0, -0.05]))
        yield self._mv(grasp)
        yield ["skill", "gripper", ["close"]]
        yield self._mv(sapien.Pose([0, 0, 0.08]) * grasp)
        yield self._mv(sapien.Pose(tgt + [0, 0, 0.08], grasp.q))
        yield self._mv(sapien.Pose(tgt + [0, 0, 0.004], grasp.q))
        yield ["skill", "gripper", ["open"]]
        yield self._mv(sapien.Pose(tgt + [0, 0, 0.08], grasp.q))
        grasp = self._grasp_pose(env.cubeC, center=env.cubeC.pose.sp.p)
        yield self._mv(grasp * sapien.Pose([0, 0, -0.05]))
        yield self._mv(grasp)
        yield ["skill", "gripper", ["close"]]
        lift = sapien.Pose([0, 0, 0.1]) * grasp
        yield self._mv(lift)
        top = (env.cubeA.pose.sp.p + env.cubeB.pose.sp.p) / 2 + np.array([0, 0, 2 * h])
        off = top - env.cubeC.pose.sp.p
        yield self._mv(sapien.Pose(lift.p + off + [0, 0, 0.02], lift.q))
        off = top - env.cubeC.pose.sp.p
        yield self._mv(sapien.Pose(env.agent.tcp.pose.sp.p + off + [0, 0, 0.003], lift.q))
        yield ["skill", "gripper", ["open"]]
        yield self._mv(sapien.Pose(env.agent.tcp.pose.sp.p + [0, 0, 0.08], lift.q))

    def plan_moving_cube(self):
        """Align the gripper's yaw with the sliding cube, then track it with end-effector delta steps (proportional
        control plus the cube's velocity as feed-forward), descend onto it, close, and carry it to the goal."""
        import sapien

        env = self.u
        dt = 0.05

        def cube():
            return env.cube.pose.sp

        c = cube()
        grasp = self._grasp_pose(env.cube, center=c.p)
        lead = self.cvel * 2.0
        yield self._mv(sapien.Pose(c.p + lead + [0, 0, 0.08], grasp.q))

        def servo(z_off, grip, n_max, tol_xy, tol_z=None):
            for _ in range(n_max):
                c = cube().p
                t = env.agent.tcp.pose.sp.p
                tgt = c + self.cvel * dt + np.array([0, 0, z_off])
                e = tgt - t
                if np.linalg.norm(e[:2]) < tol_xy and (tol_z is None or abs(e[2]) < tol_z):
                    return
                a = np.clip(e * 1.5 / 0.1 + self.cvel * dt / 0.1, -1, 1)
                yield ["act", [round(float(x), 5) for x in a] + [0.0, 0.0, 0.0, grip]]

        yield from servo(0.08, 1.0, 40, 0.003)
        yield from servo(0.0, 1.0, 40, 0.003, 0.004)
        for _ in range(8):
            c = cube().p
            e = c + self.cvel * dt - env.agent.tcp.pose.sp.p
            a = np.clip(e * 1.5 / 0.1, -1, 1)
            yield ["act", [round(float(x), 5) for x in a] + [0.0, 0.0, 0.0, -1.0]]
        g = env.goal_site.pose.sp.p
        yield self._mv(sapien.Pose(env.agent.tcp.pose.sp.p + [0, 0, 0.08], env.agent.tcp.pose.sp.q))
        off = g - cube().p
        yield self._mv(sapien.Pose(env.agent.tcp.pose.sp.p + off, env.agent.tcp.pose.sp.q))
        off = g - cube().p
        yield self._mv(sapien.Pose(env.agent.tcp.pose.sp.p + off, env.agent.tcp.pose.sp.q))
        yield ["skill", "wait", [10]]

    def plan_pull_tool(self):
        import sapien

        env = self.u
        grasp = self._grasp_pose(env.l_shape_tool, depth=0.03, center=env.l_shape_tool.pose.sp.p) * sapien.Pose([0.02, 0, 0])
        yield self._mv(grasp * sapien.Pose([0, 0, -0.05]))
        yield self._mv(grasp)
        yield ["skill", "gripper", ["close"]]
        lift = sapien.Pose(grasp.p + np.array([0, 0, 0.35]), grasp.q)
        yield self._mv(lift)
        cube = env.cube.pose.sp.p
        hl, ch = float(env.hook_length), float(env.cube_half_size)
        ap = sapien.Pose(cube) * sapien.Pose([-(hl + ch + 0.08), 0, 0.3])
        yield self._mv(sapien.Pose(ap.p, grasp.q))
        hook = sapien.Pose(cube) * sapien.Pose([-(hl + ch), -0.067, 0])
        hook = sapien.Pose(hook.p, grasp.q)
        yield self._mv(hook)
        yield self._mv(hook * sapien.Pose([-0.35, 0, 0]))

    def plan_peg_upright(self):
        import sapien

        env = self.u
        grasp = self._grasp_pose(env.peg) * sapien.Pose([0.10, 0, 0])
        yield self._mv(grasp * sapien.Pose([0, 0, -0.05]))
        yield self._mv(grasp)
        yield ["skill", "gripper", ["-0.6"]]
        lift = sapien.Pose([0, 0, 0.30]) * grasp
        yield self._mv(lift)
        th = np.pi / 10
        fin = lift * sapien.Pose(q=[np.cos(th), 0, np.sin(th), 0])
        yield self._mv(fin)
        yield self._mv(sapien.Pose([0, 0, -0.10]) * fin)
        yield ["skill", "gripper", ["open"]]
