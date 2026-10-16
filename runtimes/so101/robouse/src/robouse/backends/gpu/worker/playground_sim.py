"""MuJoCo Playground locomotion tasks for the GPU track (runs on the GPU worker; see server.py).

The robots walk with joystick policies trained on the GPU worker with Brax PPO on MuJoCo Playground's own
environments (train_playground.py; Go1JoystickFlatTerrain, G1JoystickFlatTerrain). The agent is the robot's
high-level policy: it sends body-frame velocity commands, and the learned policy turns them into joint targets at
50 Hz. Physics is MuJoCo Warp (Playground's `warp` implementation) on the GPU; images are MuJoCo's OpenGL renderer.

Robot interface:
  robo act VX VY WZ            hold a body-frame velocity command for 0.5 s (25 policy steps): VX forward m/s,
                               VY left m/s, WZ yaw rate rad/s (clipped to the robot's trained command range)
  robo skill walk VX VY WZ S   hold a command for S seconds (at most 5 s); stops early if the robot touches an
                               obstacle or falls
  robo skill stand [S]         zero command for S seconds (default 1)
Every `act` and every skill call is one step of the budget.

Obstacles are visual boxes checked geometrically: the robot's footprint (a circle around its base) must not
overlap any obstacle, wall or moving block; touching one ends in failure (it is recorded, the episode continues).
"""

from __future__ import annotations

import json
import math
import os
import pickle
import re
import tempfile

import numpy as np
from common import jpeg, r3

IMG_W, IMG_H = 640, 480
POLICY_DIR = os.environ.get("ROBOUSE_PG_POLICIES", "/root/policies")
ACT_S = 0.5
FRAME_EVERY = 5  # control steps between recorded video frames during an act/skill

ROBOTS = {
    "go1": dict(
        env="Go1JoystickFlatTerrain",
        policy="go1_flat",
        radius=0.32,
        vmax=(1.5, 0.8, 1.2),
        height=0.27,
        root="trunk",
        pert="pert_config",
        xml_dir="go1",
        base_xml="scene_mjx_feetonly_flat_terrain.xml",
    ),
    "g1": dict(
        env="G1JoystickFlatTerrain",
        policy="g1_flat",
        radius=0.28,
        vmax=(1.0, 0.8, 1.0),
        height=0.75,
        root="torso_link",
        pert="push_config",
        xml_dir="g1",
        base_xml="scene_mjx_feetonly_flat_terrain.xml",
    ),
}

TASKS: dict[str, dict] = {}


def task(key, **kw):
    TASKS[key] = kw


task("go1-obstacle-field", robot="go1", layout="field", max_steps=80, goal_tol=0.4)
task("go1-corridor-push", robot="go1", layout="corridor", max_steps=100, goal_tol=0.4, push=True)
task("go1-moving-gates", robot="go1", layout="gates", max_steps=200, goal_tol=0.4)
task("go1-waypoint-tour", robot="go1", layout="tour", max_steps=250, goal_tol=0.35)
task("go1-find-ball", robot="go1", layout="ball", max_steps=100, goal_tol=0.6, vision=True)


def make(key: str, seed: int, opts: dict):
    if key not in TASKS:
        raise KeyError(f"unknown Playground task {key!r}; known: {sorted(TASKS)}")
    return PGTask(key, seed, opts)


# ---- scene layouts: obstacles are axis-aligned boxes (cx, cy, hx, hy, height) ------------------------------------
def _layout(kind: str, rng: np.random.Generator) -> dict:
    out = {"start": (0.0, 0.0, 0.0), "obstacles": [], "movers": [], "waypoints": [], "ball": None}
    if kind == "field":
        goal = (6.0, rng.uniform(-1.0, 1.0))
        obs = []
        for _ in range(5000):
            if len(obs) >= 10:
                break
            cx, cy = rng.uniform(1.2, 5.0), rng.uniform(-2.0, 2.0)
            hx, hy = rng.uniform(0.2, 0.45), rng.uniform(0.2, 0.6)
            if (
                any(abs(cx - o[0]) < o[2] + hx + 0.75 and abs(cy - o[1]) < o[3] + hy + 0.75 for o in obs)
                or abs(cx - 3.0) < hx + 0.9
            ):
                continue
            if math.hypot(cx - goal[0], cy - goal[1]) < 1.0:
                continue
            obs.append((cx, cy, hx, hy, 0.4))
        # a wall of boxes across the straight line, so the direct path is blocked
        obs.append((3.0, 0.0, 0.15, 1.2, 0.4))
        out.update(goal=goal, obstacles=obs, bounds=(-1.0, 7.0, -3.0, 3.0))
    elif kind == "corridor":
        # a zig-zag corridor 0.9 m wide: three legs
        w = 0.45
        pts = [(0.0, 0.0), (3.0, 0.0), (3.0, 2.5 * (1 if rng.random() < 0.5 else -1)), (6.0, 0.0)]
        pts[3] = (6.0, pts[2][1])
        walls = []
        for (x0, y0), (x1, y1) in zip(pts[:-1], pts[1:], strict=False):
            if abs(y1 - y0) < 1e-6:  # along x
                cx, hx = (x0 + x1) / 2, abs(x1 - x0) / 2 + w
                walls += [(cx, y0 + w + 0.05, hx, 0.05, 0.6), (cx, y0 - w - 0.05, hx, 0.05, 0.6)]
            else:  # along y
                cy, hy = (y0 + y1) / 2, abs(y1 - y0) / 2 + w
                walls += [(x0 + w + 0.05, cy, 0.05, hy, 0.6), (x0 - w - 0.05, cy, 0.05, hy, 0.6)]
        walls = _carve(walls, pts, w)
        out.update(goal=pts[3], obstacles=walls, bounds=(-1.0, 7.0, -3.5, 3.5), path=pts)
    elif kind == "gates":
        # three lanes crossing the path; in each, a block slides back and forth across the path
        goal = (9.0, 0.0)
        movers = []
        for _i, x in enumerate((2.0, 4.5, 7.0)):
            movers.append(
                dict(cx=x, hx=0.35, hy=0.5, amp=1.8, period=rng.uniform(8.0, 12.0), phase=rng.uniform(0, 2 * math.pi))
            )
        side = [(5.0, 2.6, 5.0, 0.05, 0.6), (5.0, -2.6, 5.0, 0.05, 0.6)]
        out.update(goal=goal, obstacles=side, movers=movers, bounds=(-1.0, 10.0, -2.7, 2.7))
    elif kind == "tour":
        wps = []
        for _ in range(5000):
            if len(wps) >= 6:
                break
            p = (rng.uniform(-2.5, 2.5), rng.uniform(-2.5, 2.5))
            if all(math.hypot(p[0] - q[0], p[1] - q[1]) > 1.5 for q in wps) and math.hypot(*p) > 1.0:
                wps.append(p)
        obs = []
        for _ in range(5000):
            if len(obs) >= 5:
                break
            cx, cy = rng.uniform(-2.5, 2.5), rng.uniform(-2.5, 2.5)
            if (
                all(math.hypot(cx - q[0], cy - q[1]) > 1.0 for q in wps)
                and math.hypot(cx, cy) > 1.0
                and all(math.hypot(cx - o[0], cy - o[1]) > 1.2 for o in obs)
            ):
                obs.append((cx, cy, 0.3, 0.3, 0.4))
        out.update(goal=wps[-1], waypoints=wps, obstacles=obs, bounds=(-3.5, 3.5, -3.5, 3.5))
    elif kind == "ball":
        # a room with low interior walls; the ball is behind one of them
        room = [
            (0.0, 4.0, 4.0, 0.05, 0.6),
            (0.0, -4.0, 4.0, 0.05, 0.6),
            (4.0, 0.0, 0.05, 4.0, 0.6),
            (-4.0, 0.0, 0.05, 4.0, 0.6),
        ]
        inner = [
            (1.5, 1.5, 0.8, 0.08, 0.6),
            (-1.5, 1.8, 0.08, 1.0, 0.6),
            (1.8, -1.8, 0.08, 1.0, 0.6),
            (-1.5, -1.5, 1.0, 0.08, 0.6),
        ]
        spots = [(1.5, 2.6), (-2.6, 1.8), (2.8, -1.8), (-1.5, -2.6)]
        ball = spots[int(rng.integers(len(spots)))]
        out.update(goal=ball, ball=ball, obstacles=room + inner, bounds=(-4.0, 4.0, -4.0, 4.0))
    elif kind == "doorway":
        gy = rng.uniform(-0.6, 0.6)
        wall_x = 2.5
        door_w = 0.9
        walls = [(wall_x, gy + door_w / 2 + 1.5, 0.08, 1.5, 1.2), (wall_x, gy - door_w / 2 - 1.5, 0.08, 1.5, 1.2)]
        extra = [(1.2, rng.uniform(-0.5, 0.5), 0.25, 0.25, 0.5)]
        out.update(goal=(4.5, rng.uniform(-1.0, 1.0)), obstacles=walls + extra, bounds=(-1.0, 5.5, -3.0, 3.0))
    else:
        raise KeyError(kind)
    return out


def _carve(walls, pts, w):
    """Remove the wall pieces that block the corridor's own turns (keep only wall parts outside every leg)."""

    def inside_leg(x, y):
        for (x0, y0), (x1, y1) in zip(pts[:-1], pts[1:], strict=False):
            if (
                min(x0, x1) - w - 1e-6 <= x <= max(x0, x1) + w + 1e-6
                and min(y0, y1) - w - 1e-6 <= y <= max(y0, y1) + w + 1e-6
            ):
                return True
        return False

    out = []
    for cx, cy, hx, hy, h in walls:
        # split each wall into 0.1 m pieces and drop those inside a leg's free space
        n = max(1, int(round(max(hx, hy) * 2 / 0.1)))
        for i in range(n):
            if hx > hy:
                x = cx - hx + (i + 0.5) * (2 * hx / n)
                piece = (x, cy, hx / n, hy, h)
            else:
                y = cy - hy + (i + 0.5) * (2 * hy / n)
                piece = (cx, y, hx, hy / n, h)
            if not inside_leg(piece[0], piece[1]):
                out.append(piece)
    return _merge(out)


def _merge(pieces):
    """Merge collinear adjacent wall pieces back into longer boxes (fewer geoms to draw)."""
    pieces = sorted(pieces, key=lambda p: (round(p[1], 3), round(p[0], 3)))
    out = []
    for p in pieces:
        if out:
            q = out[-1]
            if q[3] == p[3] and abs(q[1] - p[1]) < 1e-6 and abs((q[0] + q[2]) - (p[0] - p[2])) < 1e-6:
                cx0, cx1 = q[0] - q[2], p[0] + p[2]
                out[-1] = ((cx0 + cx1) / 2, q[1], (cx1 - cx0) / 2, q[3], q[4])
                continue
        out.append(p)
    return out


def _box_circle(o, x, y, r) -> bool:
    cx, cy, hx, hy = o[:4]
    dx = max(abs(x - cx) - hx, 0.0)
    dy = max(abs(y - cy) - hy, 0.0)
    return dx * dx + dy * dy < r * r


class PGTask:
    def __init__(self, key: str, seed: int, opts: dict):
        os.environ.setdefault("XLA_PYTHON_CLIENT_MEM_FRACTION", "0.06")
        import jax
        import jax.numpy as jp
        import mujoco
        from brax.training.acme import running_statistics
        from brax.training.agents.ppo import networks as ppo_networks
        from mujoco_playground import registry

        self.jax, self.jp, self.mujoco = jax, jp, mujoco
        self.key, self.cfg, self.seed = key, {**TASKS[key], **(opts or {})}, seed
        self.rb = ROBOTS[self.cfg["robot"]]
        self.rng = np.random.default_rng(seed)
        self.lay = _layout(self.cfg["layout"], self.rng)
        env_name = self.rb["env"]
        ecfg = registry.get_default_config(env_name)
        getattr(ecfg, self.rb["pert"]).enable = bool(self.cfg.get("push"))
        if self.cfg.get("push") and self.rb["pert"] == "pert_config":
            ecfg.pert_config.velocity_kick = [0.5, 1.5]
            ecfg.pert_config.kick_wait_times = [2.0, 4.0]
        # scene: Playground's own scene XML plus our visual-only obstacles, goal markers and cameras
        self._write_scene()
        self.env = registry.load(env_name, config=ecfg)
        self.dt = float(self.env.dt)
        pdir = os.path.join(POLICY_DIR, self.rb["policy"])
        pcfg = json.load(open(os.path.join(pdir, "config.json")))
        params = pickle.load(open(os.path.join(pdir, "params.pkl"), "rb"))
        net = pcfg.get("network") or {}
        obs_size = self.env.observation_size
        nets = ppo_networks.make_ppo_networks(
            obs_size, self.env.action_size, preprocess_observations_fn=running_statistics.normalize, **net
        )
        self.policy = jax.jit(ppo_networks.make_inference_fn(nets)(params, deterministic=True))
        self.jstep = jax.jit(self.env.step)
        self.jreset = jax.jit(self.env.reset)
        self.renderer = mujoco.Renderer(self.env.mj_model, height=IMG_H, width=IMG_W)
        self.mjd = mujoco.MjData(self.env.mj_model)
        self.frames: list[str] = []
        self._recording = False
        self.cameras = ["chase", "overview", "head"]
        self.grip = 0.0
        self.reset(seed)

    # ---- scene -----------------------------------------------------------------------------------------------------
    def _write_scene(self) -> None:
        import mujoco_playground._src.locomotion as loc

        base = os.path.join(os.path.dirname(loc.__file__), self.rb["xml_dir"], "xmls")
        src = open(os.path.join(base, self.rb["base_xml"])).read()
        geoms = []
        for i, (cx, cy, hx, hy, h) in enumerate(self.lay["obstacles"]):
            geoms.append(
                f'<geom name="obst{i}" type="box" size="{hx} {hy} {h / 2}" pos="{cx} {cy} {h / 2}" rgba="0.55 0.5 0.45 1" contype="0" conaffinity="0"/>'
            )
        bodies = []
        for i, m in enumerate(self.lay["movers"]):
            bodies.append(
                f'<body name="mover{i}" mocap="true" pos="{m["cx"]} 0 0.3"><geom type="box" size="{m["hx"]} {m["hy"]} 0.3" rgba="0.9 0.3 0.1 1" contype="0" conaffinity="0"/></body>'
            )
        g = self.lay["goal"]
        if self.lay["ball"] is not None:
            geoms.append(
                f'<geom name="ball" type="sphere" size="0.15" pos="{g[0]} {g[1]} 0.15" rgba="0.95 0.1 0.1 1" contype="0" conaffinity="0"/>'
            )
        else:
            geoms.append(
                f'<geom name="goal" type="cylinder" size="{self.cfg["goal_tol"]} 0.005" pos="{g[0]} {g[1]} 0.005" rgba="0.1 0.8 0.2 0.6" contype="0" conaffinity="0"/>'
            )
        for i, (x, y) in enumerate(self.lay["waypoints"]):
            geoms.append(
                f'<geom name="wp{i}" type="cylinder" size="0.2 0.006" pos="{x} {y} 0.006" rgba="0.2 0.4 0.95 0.7" contype="0" conaffinity="0"/>'
            )
        extra = "\n".join(geoms + bodies)
        m = re.search(r"<worldbody>", src)
        if m:
            out = src[: m.end()] + "\n" + extra + "\n" + src[m.end() :]
        else:
            out = src.replace("</mujoco>", f"<worldbody>{extra}</worldbody></mujoco>")
        f = tempfile.NamedTemporaryFile("w", suffix=".xml", delete=False, dir=base, prefix="robouse_")
        f.write(out)
        f.close()
        self.scene_xml = f.name
        mod = __import__(
            f"mujoco_playground._src.locomotion.{self.rb['xml_dir']}.{self.rb['xml_dir']}_constants",
            fromlist=["task_to_xml"],
        )
        from etils import epath

        mod.task_to_xml = lambda task_name, _p=f.name: epath.Path(_p)

    # ---- state helpers ---------------------------------------------------------------------------------------------
    def _base(self):
        q = np.asarray(self.state.data.qpos)
        w, x, y, z = q[3:7]
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        up_z = 1 - 2 * (x * x + y * y)
        return q[0], q[1], q[2], yaw, up_z

    def _mover_pos(self, i: int, t: float):
        m = self.lay["movers"][i]
        return m["cx"], m["amp"] * math.sin(2 * math.pi * t / m["period"] + m["phase"])

    def _set_movers(self) -> None:
        if not self.lay["movers"]:
            return
        mp = np.asarray(self.state.data.mocap_pos).copy()
        for i in range(len(self.lay["movers"])):
            x, y = self._mover_pos(i, self.t)
            mp[i] = [x, y, 0.3]
        self.state = self.state.replace(data=self.state.data.replace(mocap_pos=self.jp.asarray(mp)))

    def _obstacles_now(self):
        obs = list(self.lay["obstacles"])
        for i, m in enumerate(self.lay["movers"]):
            x, y = self._mover_pos(i, self.t)
            obs.append((x, y, m["hx"], m["hy"], 0.6))
        return obs

    def _check(self) -> None:
        x, y, z, yaw, up = self._base()
        if up < 0.3 or z < self.rb["height"] * 0.45:
            self.fallen = True
        r = self.rb["radius"]
        if any(_box_circle(o, x, y, r) for o in self._obstacles_now()):
            self.touched += 1
        for i, (wx, wy) in enumerate(self.lay["waypoints"]):
            if i == self.next_wp and math.hypot(x - wx, y - wy) < self.cfg["goal_tol"]:
                self.next_wp += 1

    # ---- stepping ----------------------------------------------------------------------------------------------------
    def _run(self, cmd, seconds: float, stop_on_event: bool = False) -> int:
        vmax = self.rb["vmax"]
        cmd = np.array([np.clip(cmd[i], -vmax[i], vmax[i]) for i in range(3)], dtype=np.float32)
        n = max(1, int(round(seconds / self.dt)))
        touched0 = self.touched
        for k in range(n):
            info = dict(self.state.info)
            info["command"] = self.jp.asarray(cmd)
            if "steps_until_next_cmd" in info:
                info["steps_until_next_cmd"] = self.jp.asarray(10**6, dtype=self.jp.int32)
            if "step" in info:
                info["step"] = self.jp.asarray(0, dtype=self.jp.int32)
            self.state = self.state.replace(info=info)
            self.rngkey, sub = self.jax.random.split(self.rngkey)
            act, _ = self.policy(self.state.obs, sub)
            if self.fallen:
                act = act * 0
            self.state = self.jstep(self.state, act)
            self.t += self.dt
            self._set_movers()
            self._check()
            if self._recording and k % FRAME_EVERY == 0 and len(self.frames) < 80:
                self.frames.append(jpeg(self._render("chase")))
            if stop_on_event and (self.fallen or self.touched > touched0):
                return k + 1
        return n

    # ---- server methods ----------------------------------------------------------------------------------------------
    def info(self) -> dict:
        v = self.rb["vmax"]
        cams = ["chase", "overview"] + (["head"] if self.cfg.get("vision") else ["head"])
        return {
            "action": {
                "names": ["vx", "vy", "wz"],
                "low": [-v[0], -v[1], -v[2]],
                "high": list(v),
                "doc": f"body-frame velocity command held for {ACT_S} s: forward m/s, left m/s, yaw rate rad/s",
            },
            "skills": [
                "walk VX VY WZ SECONDS - hold a velocity command for up to 5 s (stops if the robot touches something or falls)",
                "stand [SECONDS] - zero command (default 1 s)",
            ],
            "camera": "chase",
            "cameras": cams,
            "hold_last": 0.0,
            "max_steps": self.cfg["max_steps"],
            "embodiment": self.embodiment(),
        }

    def embodiment(self) -> dict:
        v = self.rb["vmax"]
        kind = "quadruped" if self.cfg["robot"] == "go1" else "humanoid"
        return {
            "spec_version": "1",
            "name": f"unitree-{self.cfg['robot']}-playground-policy",
            "kind": kind,
            "step_s": ACT_S,
            "action_groups": [
                {
                    "name": "base.twist",
                    "components": ["vx", "vy", "wz"],
                    "low": [-v[0], -v[1], -v[2]],
                    "high": list(v),
                    "units": "m/s, m/s, rad/s (body frame)",
                    "mode": "base_twist",
                    "frame": "body",
                    "hold": "zero",
                    "doc": "velocity command to a learned joystick policy (Brax PPO, MuJoCo Playground)",
                }
            ],
            "sensors": {
                "cameras": [
                    {"name": "chase", "mount": "world", "width": IMG_W, "height": IMG_H, "calibrated": False},
                    {"name": "overview", "mount": "world", "width": IMG_W, "height": IMG_H, "calibrated": False},
                    {"name": "head", "mount": "head", "width": IMG_W, "height": IMG_H, "calibrated": False},
                ],
                "proprioception": [
                    {"name": "base", "shape": [4], "units": "m, m, m, deg"},
                    {"name": "velocity", "shape": [3]},
                    {"name": "fallen", "shape": []},
                ],
            },
            "skills": [{"name": "walk", "impl": "backend"}, {"name": "stand", "impl": "backend"}],
            "budgets": {"max_steps": self.cfg["max_steps"]},
            "reward": {"dense": "sparse", "success_mode": "final"},
        }

    def reset(self, seed: int | None = None) -> dict:
        jp = self.jp
        self.rngkey = self.jax.random.PRNGKey(int(self.seed if seed is None else seed))
        self.rngkey, sub = self.jax.random.split(self.rngkey)
        st = self.jreset(sub)
        q = np.asarray(self.env._init_q).copy()
        sx, sy, syaw = self.lay["start"]
        q[0], q[1] = sx, sy
        q[3:7] = [math.cos(syaw / 2), 0, 0, math.sin(syaw / 2)]
        data = st.data.replace(qpos=jp.asarray(q), qvel=jp.zeros_like(st.data.qvel), ctrl=jp.asarray(q[7:]))
        self.state = st.replace(data=data)
        self.t = 0.0
        self.fallen = False
        self.touched = 0
        self.next_wp = 0
        self.plan = None
        self._set_movers()
        self._run([0, 0, 0], 1.0)
        self.touched = 0
        return {"frame": jpeg(self._render("chase")), "hold_last": 0.0}

    def step(self, action) -> dict:
        a = [float(x) for x in action]
        self._run(a, ACT_S)
        return {"success": False, "frame": jpeg(self._render("chase")), "hold_last": 0.0}

    def skill(self, name: str, args: list) -> dict:
        self.frames = []
        self._recording = True
        try:
            ok = True
            if name == "walk":
                v = [float(a) for a in args]
                if len(v) != 4:
                    raise ValueError("walk VX VY WZ SECONDS")
                n = self._run(v[:3], min(5.0, max(0.02, v[3])), stop_on_event=True)
                msg = (
                    f"walked {n * self.dt:.2f} s"
                    + ("; stopped: touched an obstacle" if self.touched else "")
                    + ("; the robot has fallen" if self.fallen else "")
                )
            elif name == "stand":
                s = float(args[0]) if args else 1.0
                self._run([0, 0, 0], min(5.0, max(0.02, s)))
                msg = f"stood {s:.2f} s"
            else:
                raise ValueError(f"unknown skill {name!r}; skills: walk, stand")
        except ValueError as e:
            ok, msg = False, str(e)
        finally:
            self._recording = False
        out = {"ok": ok, "message": msg, "frames": self.frames, "frame": jpeg(self._render("chase")), "hold_last": 0.0}
        self.frames = []
        return out

    def observe(self, privileged: bool = False) -> dict:
        x, y, z, yaw, up = self._base()
        lv = np.asarray(self.env.get_local_linvel(self.state.data))
        st = {
            "base": {
                "x": round(float(x), 3),
                "y": round(float(y), 3),
                "z": round(float(z), 3),
                "yaw_deg": round(math.degrees(yaw), 1),
            },
            "velocity": {"vx": round(float(lv[0]), 3), "vy": round(float(lv[1]), 3)},
            "fallen": bool(self.fallen),
            "time_s": round(self.t, 2),
            "obstacle_contacts": int(self.touched),
        }
        if not self.cfg.get("vision"):
            st["goal"] = r3(self.lay["goal"], 3)
            st["obstacles"] = [
                {"center": [round(o[0], 3), round(o[1], 3)], "half_size": [round(o[2], 3), round(o[3], 3)]}
                for o in self.lay["obstacles"]
            ]
            if self.lay["movers"]:
                st["moving_blocks"] = [
                    {"x": m["cx"], "half_size": [m["hx"], m["hy"]], "y_now": round(self._mover_pos(i, self.t)[1], 3)}
                    for i, m in enumerate(self.lay["movers"])
                ]
            if self.lay["waypoints"]:
                st["waypoints"] = [r3(w, 3) for w in self.lay["waypoints"]]
                st["next_waypoint"] = int(self.next_wp)
        st["robot_radius_m"] = self.rb["radius"]
        if privileged:
            if self.plan is None:
                self.plan = self._oracle()
            try:
                st["_oracle"] = {"next": next(self.plan)}
            except StopIteration:
                st["_oracle"] = {"next": ["done"]}
        return st

    def _success(self) -> bool:
        x, y, z, yaw, up = self._base()
        g = self.lay["goal"]
        ok = (not self.fallen) and self.touched == 0 and math.hypot(x - g[0], y - g[1]) < self.cfg["goal_tol"]
        if self.lay["waypoints"]:
            ok = ok and self.next_wp >= len(self.lay["waypoints"])
        return bool(ok)

    def success(self) -> dict:
        return {"success": self._success()}

    # ---- rendering -------------------------------------------------------------------------------------------------
    def _render(self, cam: str) -> np.ndarray:
        mj = self.mujoco
        d = self.mjd
        d.qpos[:] = np.asarray(self.state.data.qpos)
        d.qvel[:] = np.asarray(self.state.data.qvel)
        if self.env.mj_model.nmocap:
            d.mocap_pos[:] = np.asarray(self.state.data.mocap_pos)
        mj.mj_forward(self.env.mj_model, d)
        x, y, z, yaw, _ = self._base()
        c = mj.MjvCamera()
        c.type = mj.mjtCamera.mjCAMERA_FREE
        if cam == "chase":
            c.lookat[:] = [x, y, 0.3]
            c.distance, c.azimuth, c.elevation = 3.2, math.degrees(yaw), -28
        elif cam == "overview":
            b = self.lay["bounds"]
            c.lookat[:] = [(b[0] + b[1]) / 2, (b[2] + b[3]) / 2, 0]
            c.distance, c.azimuth, c.elevation = max(b[1] - b[0], b[3] - b[2]) * 1.15, 90, -70
        elif cam == "head":
            h = self.rb["height"] + (0.1 if self.cfg["robot"] == "go1" else 0.45)
            f = 0.35 if self.cfg["robot"] == "go1" else 0.2
            c.lookat[:] = [x + (f + 1.0) * math.cos(yaw), y + (f + 1.0) * math.sin(yaw), h - 0.15]
            c.distance, c.azimuth, c.elevation = 1.0, math.degrees(yaw), -8
        else:
            raise ValueError(f"unknown camera {cam!r}")
        self.renderer.update_scene(d, camera=c)
        return self.renderer.render()

    def render(self, camera: str | None = None) -> dict:
        return {"jpeg": jpeg(self._render(camera or "chase"))}

    def camera_info(self, cameras: list[str]) -> list[dict]:
        return [{"name": c, "width": IMG_W, "height": IMG_H, "projection": None} for c in cameras]

    def close(self) -> None:
        try:
            os.unlink(self.scene_xml)
        except OSError:
            pass

    # ---- reference policy: grid A* on the privileged map + a heading controller --------------------------------------
    def _astar(self, start, goal, t_obs):
        res, r = 0.1, self.rb["radius"] + 0.12
        b = self.lay["bounds"]
        nx, ny = int((b[1] - b[0]) / res) + 1, int((b[3] - b[2]) / res) + 1
        occ = np.zeros((nx, ny), bool)
        xs = b[0] + np.arange(nx) * res
        ys = b[2] + np.arange(ny) * res
        X, Y = np.meshgrid(xs, ys, indexing="ij")
        for cx, cy, hx, hy, _ in t_obs:
            dx = np.maximum(np.abs(X - cx) - hx, 0)
            dy = np.maximum(np.abs(Y - cy) - hy, 0)
            occ |= dx * dx + dy * dy < r * r
        import heapq

        def idx(p):
            return (int(round((p[0] - b[0]) / res)), int(round((p[1] - b[2]) / res)))

        s, g = idx(start), idx(goal)
        occ[s] = False
        occ[g] = False
        openh = [(0.0, s)]
        came, cost = {s: None}, {s: 0.0}
        while openh:
            _, cur = heapq.heappop(openh)
            if cur == g:
                break
            for di in (-1, 0, 1):
                for dj in (-1, 0, 1):
                    if di == dj == 0:
                        continue
                    nb = (cur[0] + di, cur[1] + dj)
                    if not (0 <= nb[0] < nx and 0 <= nb[1] < ny) or occ[nb]:
                        continue
                    c2 = cost[cur] + math.hypot(di, dj)
                    if c2 < cost.get(nb, 1e9):
                        cost[nb] = c2
                        came[nb] = cur
                        heapq.heappush(openh, (c2 + math.hypot(nb[0] - g[0], nb[1] - g[1]), nb))
        if g not in came:
            return [goal]
        path, cur = [], g
        while cur is not None:
            path.append((xs[cur[0]], ys[cur[1]]))
            cur = came[cur]
        path = path[::-1]
        return path[::4] + [goal]

    def _drive(self, target, tol):
        """Commands (one act each) that walk the base to `target`."""
        for _ in range(60):
            x, y, _, yaw, _ = self._base()
            dx, dy = target[0] - x, target[1] - y
            d = math.hypot(dx, dy)
            if d < tol:
                return
            err = math.atan2(dy, dx) - yaw
            err = math.atan2(math.sin(err), math.cos(err))
            vmax = self.rb["vmax"]
            vx = min(vmax[0] * 0.6, 0.9 * d) if abs(err) < 0.5 else 0.0
            wz = max(-vmax[2], min(vmax[2], 2.0 * err))
            yield ["act", [round(vx, 3), 0.0, round(wz, 3)]]

    def _oracle(self):
        kind = self.cfg["layout"]
        if kind == "tour":
            for wp in self.lay["waypoints"]:
                x, y, *_ = self._base()
                for p in self._astar((x, y), wp, self.lay["obstacles"])[1:]:
                    yield from self._drive(p, 0.2)
                yield from self._drive(wp, 0.15)
        elif kind == "gates":
            x, y, *_ = self._base()
            for i, m in enumerate(self.lay["movers"]):
                # walk up to the lane, wait until the block will stay clear for the crossing time, then cross
                x, *_ = self._base()
                if x < m["cx"] - 1.3:
                    yield from self._drive((m["cx"] - 1.2, 0.0), 0.15)
                for _ in range(10):  # face +x before timing the crossing
                    yaw = self._base()[3]
                    if abs(yaw) < 0.08:
                        break
                    yield ["act", [0.0, 0.0, round(max(-1.2, min(1.2, -2.0 * yaw)), 3)]]
                yield ["skill", "stand", [0.5]]
                for _ in range(60):
                    t0 = self.t
                    clear = all(
                        abs(self._mover_pos(i, t0 + k * 0.25)[1]) > m["hy"] + self.rb["radius"] + 0.12
                        for k in range(1, 12)
                    )
                    if clear:
                        break
                    yield ["act", [0.0, 0.0, 0.0]]
                for _ in range(8):  # cross fast, straight
                    x, *_ = self._base()
                    if x > m["cx"] + 0.75:
                        break
                    yield ["act", [self.rb["vmax"][0], 0.0, round(-1.5 * self._base()[3], 3)]]
                yield ["skill", "stand", [0.5]]
            yield from self._drive(self.lay["goal"], 0.15)
        else:
            x, y, *_ = self._base()
            for p in self._astar((x, y), self.lay["goal"], self.lay["obstacles"])[1:]:
                yield from self._drive(p, 0.2)
            yield from self._drive(self.lay["goal"], 0.15)
        yield ["skill", "stand", [1.0]]
