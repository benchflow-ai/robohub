"""Genesis tasks for the GPU track: liquids (SPH), cloth (PBD) and soft solids (MPM) with a Franka Panda.

Runs on the GPU worker (see server.py); one episode = one process = one Genesis scene on the CUDA backend.

Robot interface (Franka Panda from Genesis's bundled MJCF, base at the world origin, objects on the floor, z = 0):
  robo act DX DY DZ DRX DRY DRZ GRIP     one control step (0.05 s): move the tool point (TCP) by up to 1 cm per unit
                                         and rotate it by up to 3 degrees per unit about world axes; GRIP +1 open,
                                         -1 close (absolute)
  robo skill move X Y Z [R P Y]          Cartesian straight-line move of the TCP to a pose (IK along the line,
                                         0.25 m/s, 45 deg/s); orientation in degrees, omitted = keep
  robo skill gripper open|close
  robo skill wait [N]                    hold still for N control steps (default 10)
"""
from __future__ import annotations

import math
import os
import tempfile

import numpy as np

from common import jpeg, pose_dict, quat_mul, quat_to_mat, r3, rpy_deg_to_quat

IMG_W, IMG_H = 640, 480
CTRL_DT = 0.05
TCP_OFFSET = 0.1034  # hand link origin -> point between the fingertips, along the hand's +z
FRAME_EVERY = 2
MAX_FRAMES = 60

TASKS: dict[str, dict] = {}


def task(key, **kw):
    TASKS[key] = kw


task("pour-water", kind="pour", max_steps=60, target_frac=0.8, spill_max=0.1)
task("pour-water-narrow", kind="pour", max_steps=60, target_frac=0.7, spill_max=0.15, narrow=True)
task("pour-water-vision", kind="pour", max_steps=80, target_frac=0.8, spill_max=0.1, vision=True)


def make(key: str, seed: int, opts: dict):
    if key not in TASKS:
        raise KeyError(f"unknown Genesis task {key!r}; known: {sorted(TASKS)}")
    return GSTask(key, seed, opts)


def _container_mjcf(name: str, inner: float, height: float, wall: float, mass: float, free: bool, rgba: str) -> str:
    """An open-top square container (a floor and four walls) as an MJCF file; returns its path."""
    h, w, t = height / 2, inner / 2, wall / 2
    o = w + t
    geoms = [f'<geom type="box" size="{w + wall} {w + wall} {t}" pos="0 0 {t}" rgba="{rgba}" mass="{mass / 5}"/>']
    for x, y, sx, sy in ((o, 0, t, w + wall), (-o, 0, t, w + wall), (0, o, w, t), (0, -o, w, t)):
        geoms.append(f'<geom type="box" size="{sx} {sy} {h}" pos="{x} {y} {h}" rgba="{rgba}" mass="{mass / 5}"/>')
    joint = '<freejoint/>' if free else ''
    xml = f'<mujoco model="{name}"><worldbody><body name="{name}">{joint}{"".join(geoms)}</body></worldbody></mujoco>'
    f = tempfile.NamedTemporaryFile("w", suffix=".xml", delete=False, prefix=name)
    f.write(xml)
    f.close()
    return f.name


class GSTask:
    def __init__(self, key: str, seed: int, opts: dict):
        import genesis as gs

        self.key, self.cfg, self.seed = key, {**TASKS[key], **(opts or {})}, seed
        self.rng = np.random.default_rng(seed)
        gs.init(backend=gs.gpu, logging_level="warning", seed=seed)
        self.gs = gs
        kind = self.cfg["kind"]
        opt = dict(sim_options=gs.options.SimOptions(dt=0.01, substeps=25 if kind == "pour" else 10),
                   vis_options=gs.options.VisOptions(show_world_frame=False, visualize_sph_boundary=False,
                                                     ambient_light=(0.4, 0.4, 0.4)),
                   renderer=gs.renderers.Rasterizer(), show_viewer=False)
        if kind == "pour":
            opt["sph_options"] = gs.options.SPHOptions(lower_bound=(-0.2, -0.6, -0.02), upper_bound=(1.0, 0.6, 0.8),
                                                       particle_size=0.007, pressure_solver="DFSPH")
        if kind == "cloth":
            opt["pbd_options"] = gs.options.PBDOptions(lower_bound=(-0.2, -0.6, -0.02), upper_bound=(1.0, 0.6, 0.8),
                                                       particle_size=0.01)
        self.scene = gs.Scene(**opt)
        self.scene.add_entity(gs.morphs.Plane(), surface=gs.surfaces.Default(color=(0.82, 0.78, 0.72)))
        self.robot = self.scene.add_entity(gs.morphs.MJCF(file="xml/franka_emika_panda/panda.xml"))
        getattr(self, f"_build_{kind}")()
        self.cam_cfg = {"scene": ((1.35, -0.9, 0.95), (0.5, 0.0, 0.1), (0.0, 0.0, 1.0), 40),
                        "front": ((1.3, 0.0, 0.45), (0.5, 0.0, 0.1), (0.0, 0.0, 1.0), 45),
                        "top": ((0.55, 0.0, 1.3), (0.55, 0.0, 0.0), (1.0, 0.0, 0.0), 50)}
        self.cams = {n: self.scene.add_camera(res=(IMG_W, IMG_H), pos=c[0], lookat=c[1], up=c[2], fov=c[3])
                     for n, c in self.cam_cfg.items()}
        self.scene.build()
        self.hand = self.robot.get_link("hand")
        self.arm = np.arange(7)
        self.fingers = np.arange(7, 9)
        self.robot.set_dofs_kp(np.array([4500, 4500, 3500, 3500, 2000, 2000, 2000, 300, 300]))
        self.robot.set_dofs_kv(np.array([450, 450, 350, 350, 200, 200, 200, 30, 30]))
        self.robot.set_dofs_force_range(np.array([-87, -87, -87, -87, -12, -12, -12, -40, -40]),
                                        np.array([87, 87, 87, 87, 12, 12, 12, 40, 40]))
        self.init_state = None
        self.frames: list[str] = []
        self._recording = False
        self.reset(seed)

    # ---- scene builders ------------------------------------------------------------------------------------------
    def _build_pour(self) -> None:
        gs = self.gs
        narrow = bool(self.cfg.get("narrow"))
        r = self.rng
        self.glass_xy = np.array([0.52 + r.uniform(-0.04, 0.04), -0.18 + r.uniform(-0.04, 0.04)])
        self.bowl_xy = np.array([0.55 + r.uniform(-0.04, 0.04), 0.14 + r.uniform(-0.04, 0.04)])
        self.glass_inner, self.glass_h, self.glass_wall = 0.05, 0.10, 0.006
        self.bowl_inner = 0.085 if narrow else 0.13
        self.bowl_h, self.bowl_wall = (0.10 if narrow else 0.06), 0.01
        g = _container_mjcf("glass", self.glass_inner, self.glass_h, self.glass_wall, 0.12, True, "0.75 0.85 0.95 0.35")
        b = _container_mjcf("bowl", self.bowl_inner, self.bowl_h, self.bowl_wall, 1.0, False, "0.85 0.45 0.25 1")
        self.glass = self.scene.add_entity(gs.morphs.MJCF(file=g, pos=(*self.glass_xy, 0.0)))
        self.bowl = self.scene.add_entity(gs.morphs.MJCF(file=b, pos=(*self.bowl_xy, 0.0)))
        fill = self.glass_inner - 0.008
        self.water = self.scene.add_entity(
            material=gs.materials.SPH.Liquid(),
            morph=gs.morphs.Box(pos=(*self.glass_xy, self.glass_wall + 0.004 + 0.04), size=(fill, fill, 0.08)),
            surface=gs.surfaces.Default(color=(0.25, 0.55, 0.95, 1.0), vis_mode="particle"))

    def _build_cloth(self) -> None:
        gs = self.gs
        r = self.rng
        self.cloth_xy = np.array([0.55 + r.uniform(-0.03, 0.03), r.uniform(-0.05, 0.05)])
        self.cloth_yaw = float(r.uniform(-20, 20))
        self.cloth_size = 0.30
        self.cloth = self.scene.add_entity(
            material=gs.materials.PBD.Cloth(rho=float(self.cfg.get("rho", 1.0)), static_friction=float(self.cfg.get("mu_s", 0.6)),
                                            kinetic_friction=float(self.cfg.get("mu_k", 0.5)),
                                            bending_compliance=float(self.cfg.get("bend", 1e-3))),
            morph=gs.morphs.Mesh(file="meshes/cloth.obj", scale=self.cloth_size / 0.5, pos=(*self.cloth_xy, 0.005),
                                 euler=(0.0, 0.0, self.cloth_yaw)),
            surface=gs.surfaces.Default(color=(0.85, 0.2, 0.25, 1.0), vis_mode="visual"))

    # ---- low-level -------------------------------------------------------------------------------------------------
    def _q(self) -> np.ndarray:
        return self.robot.get_dofs_position().cpu().numpy()

    def _tcp(self):
        p = self.hand.get_pos().cpu().numpy()
        q = self.hand.get_quat().cpu().numpy()
        return p + quat_to_mat(q) @ np.array([0, 0, TCP_OFFSET]), q

    def _ik(self, pos, quat):
        q = self.robot.inverse_kinematics(link=self.hand, pos=np.asarray(pos), quat=np.asarray(quat),
                                          local_point=np.array([0, 0, TCP_OFFSET]))
        return q.cpu().numpy() if hasattr(q, "cpu") else np.asarray(q)

    def _sim(self, n: int = 5) -> None:
        for _ in range(n):
            self._carry()
            self.scene.step()
        self.n_sim += 1
        if self._recording and self.n_sim % FRAME_EVERY == 0 and len(self.frames) < MAX_FRAMES:
            self.frames.append(jpeg(self._render("scene")))

    def _set_targets(self, q_arm) -> None:
        self.robot.control_dofs_position(np.asarray(q_arm)[:7], self.arm)
        f = 0.04 if self.grip > 0 else 0.0
        self.robot.control_dofs_position(np.array([f, f]), self.fingers)

    def _render(self, cam: str) -> np.ndarray:
        rgb = self.cams[cam].render(rgb=True)[0]
        return np.asarray(rgb)

    # ---- server methods --------------------------------------------------------------------------------------------
    def info(self) -> dict:
        return {"action": {"names": ["dx", "dy", "dz", "drx", "dry", "drz", "grip"], "low": [-1.0] * 7, "high": [1.0] * 7,
                           "doc": "TCP translation 1 cm and rotation 3 degrees per unit per 0.05 s step (world axes); grip +1 open, -1 closed"},
                "skills": ["move X Y Z [ROLL PITCH YAW] [speed=S] - straight-line TCP move (degrees; no orientation = keep; S in 0.1-2 scales 0.25 m/s and 45 deg/s)",
                           "gripper open|close", "wait [N] - hold still N control steps (default 10)"]
                + (["depth CAMERA U V - 3D world point seen at pixel (U, V) of CAMERA (RGB-D camera)"] if self.cfg.get("vision") else []),
                "camera": "scene", "cameras": list(self.cams), "hold_last": 1.0, "max_steps": self.cfg["max_steps"],
                "embodiment": self.embodiment()}

    def embodiment(self) -> dict:
        return {"spec_version": "1", "name": "franka-panda-genesis", "kind": "arm", "step_s": CTRL_DT,
                "action_groups": [
                    {"name": "arm.ee_delta", "components": ["dx", "dy", "dz"], "low": [-1] * 3, "high": [1] * 3,
                     "units": "x 1 cm per step", "mode": "ee_delta_pos", "frame": "world", "hold": "zero"},
                    {"name": "arm.ee_rot_delta", "components": ["drx", "dry", "drz"], "low": [-1] * 3, "high": [1] * 3,
                     "units": "x 3 deg per step", "mode": "ee_delta_rot", "frame": "world", "hold": "zero"},
                    {"name": "gripper", "components": ["grip"], "low": [-1], "high": [1], "units": "normalized",
                     "mode": "gripper", "hold": "last", "initial": [1], "doc": "+1 open, -1 closed"}],
                "sensors": {"cameras": [{"name": c, "mount": "world", "width": IMG_W, "height": IMG_H, "calibrated": False}
                                        for c in self.cams],
                            "proprioception": [{"name": "tcp", "shape": [7]}, {"name": "gripper", "shape": []}]},
                "skills": [{"name": "move", "impl": "backend"}, {"name": "gripper", "impl": "backend"},
                           {"name": "wait", "impl": "backend"}],
                "budgets": {"max_steps": self.cfg["max_steps"]}, "reward": {"dense": "sparse", "success_mode": "final"}}

    def reset(self, seed: int | None = None) -> dict:
        if self.init_state is None:
            q0 = np.array([0.0, -0.3, 0.0, -2.2, 0.0, 1.95, 0.785, 0.04, 0.04])
            self.robot.set_dofs_position(q0)
            self.grip = 1.0
            self.n_sim = 0
            self._set_targets(q0)
            for _ in range(20):
                self.scene.step()
        self.n_sim = 0
        self.grip = 1.0
        self.attached = None
        self.welded = False
        self.plan = None
        return {"frame": jpeg(self._render("scene")), "hold_last": self.grip}

    def step(self, action) -> dict:
        a = np.clip(np.asarray(action, dtype=float), -1, 1)
        p, q = self._tcp()
        p2 = p + a[:3] * 0.01
        rv = a[3:6] * math.radians(3)
        ang = float(np.linalg.norm(rv))
        dq = np.array([1.0, 0, 0, 0]) if ang < 1e-9 else np.r_[math.cos(ang / 2), np.sin(ang / 2) * rv / ang]
        q2 = quat_mul(dq, q)
        self._grip_to(float(a[6]))
        self._set_targets(self._ik(p2, q2))
        self._sim(5)
        return {"success": False, "frame": jpeg(self._render("scene")), "hold_last": self.grip}

    def _grip_to(self, g: float) -> None:
        was = self.grip
        self.grip = 1.0 if g > 0 else -1.0
        if self.cfg["kind"] == "pour":
            if was > 0 and self.grip < 0:
                self._weld_glass()
            elif was < 0 and self.grip > 0 and self.welded:
                self.scene.sim.rigid_solver.delete_weld_constraint(self.glass.base_link.idx, self.hand.idx)
                self.welded = False
        if self.cfg["kind"] == "cloth":
            if was > 0 and self.grip < 0:
                self._pinch()
            elif was < 0 and self.grip > 0:
                self._unpinch()

    def _weld_glass(self) -> None:
        """Rigid grasp model: fingers closing around the glass (its wall between the fingertips) hold it firmly, as a
        weld between hand and glass, until they open. Friction alone let the full glass slide out of the fingers."""
        p, _ = self._tcp()
        gp = self.glass.get_pos().cpu().numpy()
        half = self.glass_inner / 2 + self.glass_wall
        if abs(p[0] - gp[0]) < half and abs(p[1] - gp[1]) < half + 0.01 and gp[2] - 0.01 < p[2] < gp[2] + self.glass_h + 0.01:
            self.scene.sim.rigid_solver.add_weld_constraint(self.glass.base_link.idx, self.hand.idx)
            self.welded = True

    def _pinch(self) -> None:
        """Cloth grasp model: closing the fingers pins the cloth particles within 2.5 cm of the TCP to the hand (they
        are held fixed and carried with the TCP, keeping their offsets, until the fingers open)."""
        p, _ = self._tcp()
        pts = self.cloth.get_particles_pos().cpu().numpy()
        dist = np.linalg.norm(pts - p, axis=1)
        idx = np.where(dist < 0.025)[0]
        self._pinch_dbg = f"min particle distance {dist.min() * 1000:.1f} mm, {len(idx)} within 25 mm"
        if len(idx):
            self.attached = idx
            self._pin_off = pts[idx] - p
            if self.cfg.get("pin", "link") == "link":
                self.cloth.fix_particles_to_link(self.hand.idx, particles_idx_local=idx)
            else:
                self.cloth.fix_particles(particles_idx_local=idx)

    def _carry(self) -> None:
        if self.attached is None or self.cfg["kind"] != "cloth" or self.cfg.get("pin", "link") == "link":
            return
        p, _ = self._tcp()
        tgt = p + self._pin_off
        tgt[:, 2] = np.maximum(tgt[:, 2], 0.004)
        self.cloth.set_particles_pos(tgt, particles_idx_local=self.attached)

    def _unpinch(self) -> None:
        if self.attached is not None and len(self.attached):
            self.cloth.release_particle(self.attached)
        self.attached = None

    def skill(self, name: str, args: list) -> dict:
        self.frames = []
        self._recording = True
        try:
            msg, ok = self._skill(name, [str(a) for a in args]), True
        except (ValueError, RuntimeError) as e:
            ok, msg = False, str(e)
        finally:
            self._recording = False
        out = {"ok": ok, "message": msg, "tcp": pose_dict(*self._tcp()), "frames": self.frames,
               "frame": jpeg(self._render("scene")), "hold_last": self.grip}
        self.frames = []
        return out

    def _skill(self, name: str, args: list[str]) -> str:
        if name == "move":
            speed = 1.0
            kw = [a for a in args if a.startswith("speed=")]
            if kw:
                speed = min(2.0, max(0.1, float(kw[-1].split("=", 1)[1])))
                args = [a for a in args if not a.startswith("speed=")]
            v = [float(a) for a in args]
            p0, q0 = self._tcp()
            if len(v) == 3:
                q1 = q0
            elif len(v) == 6:
                q1 = np.array(rpy_deg_to_quat(*v[3:]))
            else:
                raise ValueError("move takes X Y Z or X Y Z ROLL PITCH YAW (degrees)")
            p1 = np.array(v[:3])
            if p1[2] < -0.01 or np.linalg.norm(p1[:2]) > 0.9:
                raise ValueError("target out of the arm's workspace")
            dot = abs(float(np.dot(q0, q1)))
            ang = 2 * math.degrees(math.acos(min(1.0, dot)))
            n = max(2, int(max(np.linalg.norm(p1 - p0) / (0.25 * speed * CTRL_DT), ang / (45 * speed * CTRL_DT))))
            qa = self._q()
            for i in range(1, n + 1):
                s = i / n
                q = _slerp(q0, q1, s)
                self._set_targets(self._ik(p0 + (p1 - p0) * s, q))
                self._sim(5)
            for _ in range(4):
                self._sim(5)
            p, q = self._tcp()
            err = float(np.linalg.norm(p - p1))
            return f"moved; TCP now {r3(p)}; position error {err * 1000:.1f} mm"
        if name == "gripper":
            if not args or args[0] not in ("open", "close"):
                raise ValueError("gripper open|close")
            self._grip_to(1.0 if args[0] == "open" else -1.0)
            qa = self._q()
            for _ in range(8):
                self._set_targets(qa)
                self._sim(5)
            held = " holding the glass" if self.welded else ""
            return f"gripper {args[0]}" + held + (f"; holding {len(self.attached)} cloth particles" if self.attached is not None else "")
        if name == "wait":
            n = max(1, min(100, int(args[0]) if args else 10))
            qa = self._q()
            for _ in range(n):
                self._set_targets(qa)
                self._sim(5)
            return f"waited {n} steps"
        if name == "depth":
            if not self.cfg.get("vision"):
                raise ValueError("no depth camera in this task")
            cam, uu, vv = args[0], int(float(args[1])), int(float(args[2]))
            return f"camera {cam} pixel ({uu}, {vv}) sees world point {self._deproject(cam, uu, vv)}"
        raise ValueError(f"unknown skill {name!r}; skills: move, gripper, wait" + (", depth" if self.cfg.get("vision") else ""))

    # ---- observation and success ---------------------------------------------------------------------------------
    def observe(self, privileged: bool = False) -> dict:
        p, q = self._tcp()
        st = {"tcp": pose_dict(p, q), "gripper": "open" if self.grip > 0 else "closed",
              "finger_gap_m": round(float(self._q()[7:9].sum()), 4)}
        st.update(getattr(self, f"_obs_{self.cfg['kind']}")())
        if privileged:
            if self.plan is None:
                self.plan = getattr(self, f"_plan_{self.cfg['kind']}")()
            try:
                st["_oracle"] = {"next": next(self.plan)}
            except StopIteration:
                st["_oracle"] = {"next": ["done"]}
            st["_measure"] = self._measure()
        return st

    def _obs_pour(self) -> dict:
        gp = self.glass.get_pos().cpu().numpy()
        gq = self.glass.get_quat().cpu().numpy()
        return {"glass": {**pose_dict(gp, gq), "inner_width_m": self.glass_inner, "height_m": self.glass_h,
                          "wall_m": self.glass_wall},
                "bowl": {"pos": r3(self.bowl.get_pos().cpu().numpy()), "inner_width_m": self.bowl_inner,
                         "height_m": self.bowl_h, "wall_m": self.bowl_wall}}

    def _obs_cloth(self) -> dict:
        pts = self.cloth.get_particles_pos().cpu().numpy()
        c = self._corner_idx()
        return {"cloth_corners": {k: r3(pts[i]) for k, i in c.items()}, "cloth_center": r3(pts.mean(0)),
                "cloth_size_m": self.cloth_size}

    def _corner_idx(self) -> dict:
        if not hasattr(self, "_corners"):
            pts = self.cloth.get_particles_pos().cpu().numpy()
            c = pts.mean(0)
            yaw = math.radians(self.cloth_yaw)
            R = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
            loc = (pts[:, :2] - c[:2]) @ R
            self._corners = {name: int(np.argmin(np.linalg.norm(loc - np.array(s) * self.cloth_size / 2, axis=1)))
                             for name, s in (("c1", (1, 1)), ("c2", (-1, 1)), ("c3", (-1, -1)), ("c4", (1, -1)))}
        return self._corners

    def _measure(self) -> dict:
        if self.cfg["kind"] == "pour":
            pts = self.water.get_particles_pos().cpu().numpy()
            inb = _inside_box(pts, self.bowl.get_pos().cpu().numpy(), np.array([1, 0, 0, 0]), self.bowl_inner, self.bowl_h)
            ing = _inside_box(pts, self.glass.get_pos().cpu().numpy(), self.glass.get_quat().cpu().numpy(),
                              self.glass_inner, self.glass_h + 0.02)
            n = len(pts)
            return {"in_bowl": round(float(inb.sum()) / n, 3), "in_glass": round(float((ing & ~inb).sum()) / n, 3),
                    "spilled": round(float((~inb & ~ing).sum()) / n, 3)}
        pts = self.cloth.get_particles_pos().cpu().numpy()
        c = self._corner_idx()
        return {"corner_dist": {f"{a}-{b}": round(float(np.linalg.norm(pts[c[a]] - pts[c[b]])), 4)
                                for a, b in (("c1", "c2"), ("c1", "c4"), ("c2", "c3"), ("c3", "c4"), ("c1", "c3"), ("c2", "c4"))},
                "max_height": round(float(pts[:, 2].max()), 4)}

    def _success(self) -> bool:
        m = self._measure()
        if self.cfg["kind"] == "pour":
            if m["spilled"] > self.cfg["spill_max"]:
                return False
            if "band" in self.cfg:
                lo, hi = self.cfg["band"]
                return lo <= m["in_bowl"] <= hi
            return m["in_bowl"] >= self.cfg["target_frac"]
        d = m["corner_dist"]
        if self.attached is not None or m["max_height"] > 0.08:
            return False
        if self.cfg["folds"] == 1:  # two adjacent corners on top of the other two
            return (d["c1-c2"] < 0.03 and d["c3-c4"] < 0.03) or (d["c1-c4"] < 0.03 and d["c2-c3"] < 0.03)
        return max(d.values()) < 0.04  # quarter fold: all four corners together

    def success(self) -> dict:
        return {"success": self._success()}

    def render(self, camera: str | None = None) -> dict:
        cam = camera or "scene"
        if cam not in self.cams:
            raise ValueError(f"unknown camera {cam!r}; cameras: {list(self.cams)}")
        return {"jpeg": jpeg(self._render(cam))}

    def _intrinsics(self, name):
        pos, look, up, fov = (np.array(x, float) if i < 3 else x for i, x in enumerate(self.cam_cfg[name]))
        f = look - pos
        f /= np.linalg.norm(f)
        r = np.cross(f, up)
        r /= np.linalg.norm(r)
        u = np.cross(r, f)
        R = np.stack([r, -u, f])  # world -> OpenCV camera (x right, y down, z forward)
        fy = (IMG_H / 2) / math.tan(math.radians(fov) / 2)
        K = np.array([[fy, 0, IMG_W / 2], [0, fy, IMG_H / 2], [0, 0, 1.0]])
        E = np.hstack([R, (-R @ pos)[:, None]])
        return K, E, pos, R

    def camera_info(self, cameras: list[str]) -> list[dict]:
        from common import projection

        out = []
        for c in cameras:
            K, E, pos, _ = self._intrinsics(c)
            out.append({"name": c, "width": IMG_W, "height": IMG_H, "fovy_deg": float(self.cam_cfg[c][3]),
                        "position": r3(pos), "projection": projection(K, E)})
        return out

    def _deproject(self, cam: str, u: int, v: int) -> list[float]:
        if cam not in self.cams:
            raise ValueError(f"unknown camera {cam!r}")
        if not (0 <= u < IMG_W and 0 <= v < IMG_H):
            raise ValueError(f"pixel out of range ({IMG_W}x{IMG_H})")
        depth = np.asarray(self.cams[cam].render(rgb=False, depth=True)[1])
        z = float(depth[v, u])
        if not np.isfinite(z) or z <= 0 or z > 20:
            raise ValueError("no depth at that pixel")
        K, _, pos, R = self._intrinsics(cam)
        ray = np.array([(u + 0.5 - K[0, 2]) / K[0, 0], (v + 0.5 - K[1, 2]) / K[1, 1], 1.0])
        w = pos + R.T @ (ray * z)
        return r3(w + np.random.default_rng().normal(0, 0.002, 3))

    def close(self) -> None:
        pass

    # ---- reference policies -------------------------------------------------------------------------------------
    def _plan_pour(self):
        """Top-down grasp of the glass near its rim, carry it beside the bowl, tilt it about world x, set it back."""
        gp = self.glass.get_pos().cpu().numpy()
        down = (180.0, 0.0, 0.0)  # TCP pointing down, fingers closing along world y
        zg = self.glass_h - 0.025
        yield ["skill", "gripper", ["open"]]
        yield ["skill", "move", [gp[0], gp[1], self.glass_h + 0.08, *down]]
        yield ["skill", "move", [gp[0], gp[1], zg, *down]]
        yield ["skill", "gripper", ["close"]]
        yield ["skill", "move", [gp[0], gp[1], zg + 0.15, *down]]
        bp = self.bowl.get_pos().cpu().numpy()
        dy = self.cfg.get("pour_dy", -(self.bowl_inner / 2 - 0.005))
        dz = self.cfg.get("pour_dz", 0.03)
        z = self.bowl_h + dz + (self.glass_h - 0.025)
        yield ["skill", "move", [bp[0], bp[1] + dy, z, *down]]
        band = "band" in self.cfg
        if band:  # tilt in 3-degree steps, letting the flow settle each time, until 45 % has left the glass
            tilt = 20
            while tilt < 110 and 1.0 - self._measure()["in_glass"] < 0.45:
                tilt += 3
                yield ["skill", "move", [bp[0], bp[1] + dy, z, 180.0 - tilt, 0.0, 0.0]]
                yield ["skill", "wait", [6]]
            yield ["skill", "move", [bp[0], bp[1] + dy, z, 180.0 - max(0, tilt - 25), 0.0, 0.0, "speed=2"]]
            yield ["skill", "wait", [20]]
        else:
            for tilt in (40, 70, 90, 105):
                yield ["skill", "move", [bp[0], bp[1] + dy, z, 180.0 - tilt, 0.0, 0.0]]
                yield ["skill", "wait", [8]]
            yield ["skill", "move", [bp[0], bp[1] + dy, z, 180.0 - 130.0, 0.0, 0.0]]
            yield ["skill", "wait", [25]]
        yield ["skill", "move", [bp[0], bp[1] + dy, z, *down]]
        yield ["skill", "wait", [10]]

    def _plan_cloth(self):
        pts = lambda: self.cloth.get_particles_pos().cpu().numpy()  # noqa: E731
        c = self._corner_idx()
        down = (180.0, 0.0, 0.0)

        def fold(a, b):  # carry corner a onto corner b, in an arc high enough that the cloth is not dragged
            pa, pb = pts()[c[a]], pts()[c[b]]
            yield ["skill", "gripper", ["open"]]
            yield ["skill", "move", [pa[0], pa[1], 0.06, *down]]
            yield ["skill", "move", [pa[0], pa[1], 0.02, *down]]
            yield ["skill", "gripper", ["close"]]
            span = float(np.linalg.norm(pb[:2] - pa[:2]))
            sp = f"speed={self.cfg.get('fold_speed', 0.3)}"
            fold_mid = (pa + pb) / 2
            for th in np.linspace(0.0, math.pi, 7)[1:-1]:  # semicircle about the fold line
                q = fold_mid + (pa - fold_mid) * math.cos(th)
                yield ["skill", "move", [q[0], q[1], 0.02 + span / 2 * math.sin(th), *down, sp]]
            yield ["skill", "move", [pb[0], pb[1], 0.025, *down]]
            yield ["skill", "gripper", ["open"]]
            yield ["skill", "move", [pb[0], pb[1], 0.08, *down]]

        yield from fold("c1", "c2")
        yield from fold("c4", "c3")
        if self.cfg["folds"] == 2:
            yield from fold("c2", "c3")


def _slerp(q0, q1, s):
    q0, q1 = np.asarray(q0, float), np.asarray(q1, float)
    d = float(np.dot(q0, q1))
    if d < 0:
        q1, d = -q1, -d
    if d > 0.9995:
        q = q0 + s * (q1 - q0)
        return q / np.linalg.norm(q)
    th = math.acos(d)
    return (math.sin((1 - s) * th) * q0 + math.sin(s * th) * q1) / math.sin(th)


def _inside_box(pts, pos, quat, inner, height):
    R = quat_to_mat(quat)
    loc = (pts - pos) @ R
    h = inner / 2
    return (np.abs(loc[:, 0]) <= h) & (np.abs(loc[:, 1]) <= h) & (loc[:, 2] >= -0.005) & (loc[:, 2] <= height)
