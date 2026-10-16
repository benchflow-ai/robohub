"""Isaac Lab 3.0 Factory tasks for the GPU track (runs on the GPU worker; see server.py).

NVIDIA Isaac Lab 3.0.0-EA in its kit-less container (nvcr.io/nvidia/isaac-lab:3.0.0-rc1-kitless): no Isaac Sim and no
Kit. The Factory contact-rich assembly environments (Narang et al., "Factory: Fast Contact for Robotic Assembly", RSS
2022; Isaac Lab 3.0 registers them as `IsaacContrib-Factory-*-Direct` in `isaaclab_tasks.contrib.factory`) run with
PhysX through OvPhysX, the kit-less PhysX runtime. One episode = one process with one environment (num_envs=1).

Physics, per task (`opts: {physics: ...}`) or for the whole worker (ROBOUSE_IL_PHYSICS, which wins):
  physx    PhysX 5 through OvPhysX (the default).
  newton   Newton with the MuJoCo Warp solver. Not available for Factory: Isaac Lab 3.0 ships no Newton configuration
           for these tasks, and Newton 1.5.2 (in the 3.0.0-EA container) imports Factory's fixed parts without an
           articulation, which Isaac Lab needs (see NEWTON_FACTORY).

Images come from one of three renderers, chosen by ROBOUSE_IL_RENDERER when the worker starts and reported in `info()`
and `renderer()` (so in `robo info`, the trace and every episode's result.json):
  ovrtx              NVIDIA's RTX renderer without Kit (OVRTX), from one camera that moves between the views.
  newton-warp        Newton's Warp rasterizer, same camera.
  mujoco-schematic   a MuJoCo (OpenGL) scene of boxes and cylinders sized from Factory's asset configs, with the poses
                     copied from Isaac Lab every frame. The physics and the success check are Isaac Lab's either way.
ROBOUSE_IL_RENDERER=auto (the default) probes OVRTX, then the Newton renderer, in a child process when the worker starts
(a renderer that crashes takes only the probe down) and uses the first that gives an image, else the schematic.

Isaac Lab 3.0 orders quaternions (x, y, z, w); Isaac Lab 2.3 used (w, x, y, z). The agent still sees `quat_wxyz`
(converted here), so the observation format did not change.

Isaac Lab 3.0.0-EA's Factory environment still has three PhysX-with-Kit leftovers, patched here at run time (`_compat`):
it imports Kit's `carb` for one gravity call (release/3.0.0 replaced it with `physics_manager.set_gravity`), it sets
friction through PhysX's tensor API, and OvPhysX 0.5.11 fails to create the data buffers of an articulation without
joints (Factory's fixed and held parts). OvPhysX also reads the PhysX scene settings (solver, iteration counts,
friction offsets, GPU partitions) from the USD physics scene, which only Isaac Lab's Kit PhysX backend writes from the
task's PhysxCfg; `_scene_attributes` writes Factory's onto the scene for OvPhysX the same way.

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
import sys
import types

import numpy as np
from common import jpeg, pose_dict, r3

IMG_W, IMG_H = 640, 480
FRAME_EVERY = 2
ISAACLAB = "Isaac Lab 3.0.0-EA (kit-less container nvcr.io/nvidia/isaac-lab:3.0.0-rc1-kitless)"
PHYSICS = {"physx": "PhysX 5 through OvPhysX (kit-less)"}
NEWTON_FACTORY = (
    "Factory does not run on Newton in Isaac Lab 3.0.0-EA: Isaac Lab ships no Newton configuration for the Factory "
    "tasks (neither 3.0.0-EA nor release/3.0.0 on 2026-10-16), and Newton 1.5.2 imports Factory's fixed parts (a "
    "body with a fixed joint to the world) without an articulation, so Isaac Lab cannot open them"
)
RENDERER_MODES = ("auto", "ovrtx", "newton-warp", "mujoco-schematic")
RENDERERS = {
    "ovrtx": "NVIDIA's RTX renderer without Kit (OVRTX)",
    "newton-warp": "Newton's Warp rasterizer",
    "mujoco-schematic": "a MuJoCo schematic of the Factory parts (boxes and cylinders sized from Factory's asset "
    "configs, poses copied from Isaac Lab every frame); not Isaac Lab's renderer",
}
PROBE_ORDER = ("ovrtx", "newton-warp")
PROBE_TIMEOUT_S = float(os.environ.get("ROBOUSE_IL_PROBE_TIMEOUT_S", "900"))
RENDERER = os.environ.get("ROBOUSE_IL_RENDERER", "auto").strip().lower() or "auto"


# ---- quaternions: Isaac Lab 3.0 is (x, y, z, w) ----------------------------------------------------------------------
def il_quat_to_wxyz(q) -> np.ndarray:
    """An Isaac Lab 3.0 quaternion (x, y, z, w) as (w, x, y, z), the order MuJoCo and the agent's `quat_wxyz` use."""
    x, y, z, w = (float(c) for c in np.asarray(q, dtype=float).ravel()[:4])
    return np.array([w, x, y, z])


def yaw_deg_from_il_quat(q) -> float:
    """Yaw (rotation about world z, degrees) of an Isaac Lab 3.0 quaternion (x, y, z, w)."""
    x, y, z, w = (float(c) for c in np.asarray(q, dtype=float).ravel()[:4])
    return math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


# ---- renderer choice -------------------------------------------------------------------------------------------------
def prepare() -> None:
    """Worker start-up (server.py): resolve ROBOUSE_IL_RENDERER=auto with one probe per renderer in a child process.
    The episode processes inherit the result."""
    import subprocess

    mode = os.environ.get("ROBOUSE_IL_RENDERER", "auto").strip().lower() or "auto"
    if mode not in RENDERER_MODES:
        raise SystemExit(f"ROBOUSE_IL_RENDERER must be one of {', '.join(RENDERER_MODES)}, not {mode!r}")
    physics = os.environ.get("ROBOUSE_IL_PHYSICS", "")
    if physics and physics not in PHYSICS:
        raise SystemExit(
            f"ROBOUSE_IL_PHYSICS must be one of {', '.join(PHYSICS)}"
            + (f"; {NEWTON_FACTORY}" if physics == "newton" else "")
        )
    note = f"ROBOUSE_IL_RENDERER={mode}"
    if mode == "auto":
        tried = []
        mode = "mujoco-schematic"
        for cand in PROBE_ORDER:
            print(f"isaaclab3: probing the {cand} renderer (ROBOUSE_IL_RENDERER=auto)", flush=True)
            try:
                r = subprocess.run(
                    [sys.executable, os.path.abspath(__file__), "--probe", cand],
                    capture_output=True,
                    text=True,
                    timeout=PROBE_TIMEOUT_S,
                    env={**os.environ, "ROBOUSE_IL_RENDERER": cand},
                )
                said = [ln for ln in r.stdout.splitlines() if ln.startswith(("PROBE OK", "PROBE FAILED"))]
                ok = bool(said) and said[-1].startswith("PROBE OK")
                if said:
                    why = said[-1]
                elif r.returncode < 0:
                    why = f"the probe crashed with signal {-r.returncode} before rendering a frame"
                else:
                    tail = (r.stderr.strip().splitlines() or r.stdout.strip().splitlines() or [""])[-1]
                    why = f"exit code {r.returncode}: {tail[:300]}"
            except subprocess.TimeoutExpired:
                ok, why = False, f"no image within {PROBE_TIMEOUT_S:.0f} s"
            tried.append(f"{cand}: {why}")
            if ok:
                mode = cand
                break
        note = "ROBOUSE_IL_RENDERER=auto: " + "; ".join(tried)
        print(f"isaaclab3: {note}; images from {mode}", flush=True)
    os.environ["ROBOUSE_IL_RENDERER"] = mode
    os.environ["ROBOUSE_IL_RENDERER_NOTE"] = note
    global RENDERER
    RENDERER = mode


def _probe(renderer: str) -> int:
    """`python isaaclab3_sim.py --probe RENDERER`: open the peg-insertion environment, render one frame and say whether
    it is an image (not empty, not a single colour)."""
    global RENDERER
    RENDERER = renderer
    t = ILTask("factory-peg-insert", 0, {})
    for _ in range(4):
        t._env_step(np.zeros(6))
    img = t._render("scene")
    if img is None or img.ndim != 3 or float(np.asarray(img, dtype=float).std()) < 1.0:
        print("PROBE FAILED: the renderer returned an empty or single-colour image", flush=True)
        return 1
    print(f"PROBE OK: {img.shape[1]}x{img.shape[0]} frame from {renderer}", flush=True)
    return 0


# Height above the seated pose at which the reference policy centres the held part: clear of the socket (peg), the
# 20 mm shaft (gear) and the bolt's top (nut).
ALIGN_HEIGHT_M = {"peg_insert": 0.012, "gear_mesh": 0.026, "nut_thread": 0.016}

# ---- tasks -----------------------------------------------------------------------------------------------------------
TASKS: dict[str, dict] = {}


def task(key, **kw):
    TASKS[key] = kw


# gym_id: Isaac Lab 3.0's ID; legacy_gym_id: the same environment in Isaac Lab 2.3 (isaaclab_sim.py)
task(
    "factory-peg-insert",
    gym_id="IsaacContrib-Factory-PegInsert-Direct",
    legacy_gym_id="Isaac-Factory-PegInsert-Direct-v0",
    max_steps=300,
)
task(
    "factory-gear-mesh",
    gym_id="IsaacContrib-Factory-GearMesh-Direct",
    legacy_gym_id="Isaac-Factory-GearMesh-Direct-v0",
    max_steps=400,
)
task(
    "factory-nut-thread",
    gym_id="IsaacContrib-Factory-NutThread-Direct",
    legacy_gym_id="Isaac-Factory-NutThread-Direct-v0",
    max_steps=600,
)
task(
    "factory-peg-insert-noisy",
    gym_id="IsaacContrib-Factory-PegInsert-Direct",
    legacy_gym_id="Isaac-Factory-PegInsert-Direct-v0",
    max_steps=300,
    obs_noise=0.004,
)
task(
    "factory-nut-thread-noisy",
    gym_id="IsaacContrib-Factory-NutThread-Direct",
    legacy_gym_id="Isaac-Factory-NutThread-Direct-v0",
    max_steps=600,
    obs_noise=0.003,
)


def make(key: str, seed: int, opts: dict):
    if key not in TASKS:
        raise KeyError(f"unknown Isaac Lab task {key!r}; known: {sorted(TASKS)}")
    return ILTask(key, seed, opts)


def physics_choice(opts: dict | None) -> str:
    """The physics engine for one episode: ROBOUSE_IL_PHYSICS, else the task's `physics` option, else physx."""
    name = (os.environ.get("ROBOUSE_IL_PHYSICS") or (opts or {}).get("physics") or "physx").strip().lower()
    if name == "newton":
        raise ValueError(NEWTON_FACTORY)
    if name not in PHYSICS:
        raise ValueError(f"unknown physics {name!r}; known: {sorted(PHYSICS)}")
    return name


# ---- Isaac Lab 3.0.0-EA compatibility (see the module docstring) -----------------------------------------------------
_PATCHED = False


def _compat() -> None:
    global _PATCHED
    if _PATCHED:
        return
    import importlib.util

    import torch
    import warp as wp

    if importlib.util.find_spec("carb") is None:  # Factory's `carb.Float3(...)` for its gravity toggle
        carb = types.ModuleType("carb")
        carb.Float3 = lambda *a: tuple(float(x) for x in a)
        sys.modules["carb"] = carb

    from isaaclab.sim import SimulationContext

    class GravityView:
        """`physics_sim_view.set_gravity` (PhysX with Kit) through the kit-less physics manager."""

        def __init__(self, manager, inner):
            self.manager, self.inner = manager, inner

        def set_gravity(self, g):
            self.manager.set_gravity(tuple(float(x) for x in g))

        def __getattr__(self, k):
            return getattr(self.inner, k)

    view = SimulationContext.physics_sim_view
    SimulationContext.physics_sim_view = property(lambda self: GravityView(self.physics_manager, view.fget(self)))

    from isaaclab_ov.assets.articulation import articulation_data as ad

    pin = ad.ArticulationData._pin_proxy_arrays

    def pin_without_joints(self):
        """OvPhysX 0.5.11: an articulation without joints has an empty friction buffer (no pointer); give the strided
        views a one-element buffer to point at (they have zero columns either way)."""
        buf = self._joint_friction_props_buf
        if getattr(buf.data, "ptr", 1) is not None:
            return pin(self)
        stand_in = types.SimpleNamespace(
            data=wp.zeros((max(self._num_instances, 1), 1, 3), dtype=wp.float32, device=str(buf.data.device))
        )
        self._joint_friction_props_buf, self._robouse_stand_in = stand_in, stand_in
        try:
            return pin(self)
        finally:
            self._joint_friction_props_buf = buf

    ad.ArticulationData._pin_proxy_arrays = pin_without_joints

    from isaaclab_tasks.contrib.factory import factory_utils as fu

    def set_friction(asset, value, num_envs):
        """Factory's set_friction through OvPhysX's shape material binding (static and dynamic friction)."""
        import isaaclab_ov.tensor_types as tt

        view = asset.root_view
        m = wp.to_torch(view.get_attribute(tt.SHAPE_FRICTION_AND_RESTITUTION))
        m[..., 0] = value
        m[..., 1] = value
        rows = torch.arange(m.shape[0], dtype=torch.int32, device=m.device)
        view.set_attribute(
            tt.SHAPE_FRICTION_AND_RESTITUTION,
            wp.from_torch(m.contiguous(), dtype=wp.float32),
            indices=wp.from_torch(rows),
        )

    fu.set_friction = set_friction

    from isaaclab_ov.physics.ovphysx_manager import OvPhysxManager

    configure = OvPhysxManager._configure_physx_scene_prim

    def configure_with_task_settings(scene_prim, cfg, device):
        configure(scene_prim, cfg, device)
        for name, (sdf_type, value) in _SCENE_ATTRS.items():
            scene_prim.CreateAttribute(name, sdf_type).Set(value)

    OvPhysxManager._configure_physx_scene_prim = staticmethod(configure_with_task_settings)
    _PATCHED = True


_SCENE_ATTRS: dict = {}
# PhysxCfg fields that are not physxScene attributes, or that the kit-less backend sets itself
_SCENE_SKIP = {
    "deterministic",
    "solver_type",
    "enable_ccd",
    "solve_articulation_contact_last",
    "dt",
    "device",
    "render_interval",
    "gravity",
    "physics_prim_path",
    "use_fabric",
    "physics_material",
    "class_type",
    "enable_scene_query_support",
}


def _scene_attributes(physx_cfg, dt: float) -> dict:
    """The physxScene attributes Isaac Lab's Kit PhysX backend (PhysxManager) writes from a task's PhysxCfg, as
    {name: (Sdf type, value)}, for OvPhysX to read from the USD physics scene."""
    from isaaclab.utils.string import to_camel_case
    from pxr import Sdf

    out = {
        "physxScene:timeStepsPerSecond": (Sdf.ValueTypeNames.UInt, int(round(1.0 / dt))),
        "physxScene:solverType": (Sdf.ValueTypeNames.Token, "TGS" if physx_cfg.solver_type == 1 else "PGS"),
        "physxScene:solveArticulationContactLast": (
            Sdf.ValueTypeNames.Bool,
            bool(physx_cfg.solve_articulation_contact_last),
        ),
        "physxScene:enableCCD": (Sdf.ValueTypeNames.Bool, False),  # not supported with GPU dynamics
    }
    for key, value in physx_cfg.to_dict().items():
        if key in _SCENE_SKIP or value is None:
            continue
        name = "physxScene:" + to_camel_case("bounce_threshold" if key == "bounce_threshold_velocity" else key, "cC")
        if isinstance(value, bool):
            out[name] = (Sdf.ValueTypeNames.Bool, value)
        elif isinstance(value, int):
            out[name] = (Sdf.ValueTypeNames.UInt, value)
        elif isinstance(value, float):
            out[name] = (Sdf.ValueTypeNames.Float, value)
    return out


def _physics_cfg(name: str):
    from isaaclab_ov.physics import OvPhysxCfg

    # Factory's GPU buffer sizes (its PhysxCfg); OvPhysX reads the solver settings from the USD physics scene
    return OvPhysxCfg(
        gpu_max_rigid_contact_count=2**23, gpu_max_rigid_patch_count=2**23, gpu_collision_stack_size=2**28
    )


def _renderer_cfg(name: str):
    if name == "ovrtx":
        from isaaclab_ov.renderers import OVRTXRendererCfg

        return OVRTXRendererCfg()
    if name == "newton-warp":
        from isaaclab_newton.renderers import NewtonWarpRendererCfg

        return NewtonWarpRendererCfg()
    return None


def _env_class():
    """Factory's environment with one robouse camera (when a renderer of Isaac Lab's draws the images)."""
    import isaaclab.sim as sim_utils
    from isaaclab.sensors import Camera, CameraCfg
    from isaaclab_tasks.contrib.factory.factory_env import FactoryEnv

    class FactoryWithCamera(FactoryEnv):
        robouse_renderer = None

        def _setup_scene(self):
            super()._setup_scene()
            self.robouse_cam = None
            rcfg = _renderer_cfg(self.robouse_renderer)
            if rcfg is not None:
                self.robouse_cam = Camera(
                    CameraCfg(
                        prim_path="/World/envs/env_0/RobouseCamera",
                        width=IMG_W,
                        height=IMG_H,
                        data_types=["rgb"],
                        update_latest_camera_pose=True,
                        spawn=sim_utils.PinholeCameraCfg(focal_length=24.0, clipping_range=(0.01, 20.0)),
                        renderer_cfg=rcfg,
                    )
                )
                self.scene.sensors["robouse_camera"] = self.robouse_cam

    return FactoryWithCamera


class ILTask:
    def __init__(self, key: str, seed: int, opts: dict):
        self.key, self.cfg, self.seed = key, {**TASKS[key], **(opts or {})}, seed
        self.physics = physics_choice(opts)
        self.renderer_name = RENDERER if RENDERER in RENDERERS else "mujoco-schematic"
        _compat()
        import isaaclab_tasks  # noqa: F401  (registers the environments)
        import torch
        from isaaclab.app import launch_simulation
        from isaaclab_tasks.utils import parse_env_cfg

        self.torch = torch
        cfg = parse_env_cfg(self.cfg["gym_id"], device="cuda:0", num_envs=1)
        cfg.episode_length_s = 36000.0  # the episode server enforces the budget; never auto-reset
        _SCENE_ATTRS.clear()
        _SCENE_ATTRS.update(_scene_attributes(cfg.sim.physics, cfg.sim.dt))  # Factory's PhysxCfg, before it is replaced
        cfg.sim.physics = _physics_cfg(self.physics)
        if self.renderer_name == "ovrtx":
            import ovrtx

            ovrtx.register_schema_paths()  # as launch_simulation does for a config with an OVRTX camera
        self._launch = launch_simulation(cfg, {"headless": True})  # kit-less: no app, kept open for the process
        self._launch.__enter__()
        env_cls = _env_class()
        env_cls.robouse_renderer = self.renderer_name
        self.env = self.u = env_cls(cfg)
        self.name = self.u.cfg_task.name
        self.frames: list[str] = []
        self._recording = False
        self.used: set[str] = set()
        self.reset(seed)

    # ---- helpers -----------------------------------------------------------------------------------------------------
    def _np(self, t):
        return t[0].detach().cpu().numpy().astype(float)

    def _poses(self):
        """Held part base pose and its target, as positions and Isaac Lab (x, y, z, w) quaternions."""
        from isaaclab_tasks.contrib.factory import factory_utils as fu

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
        t = self.torch
        self.u.robouse_cam.set_world_poses_from_view(
            t.tensor([f + np.array(eye)], dtype=t.float32), t.tensor([f + np.array(tgt)], dtype=t.float32)
        )

    def _render(self, cam: str = "scene") -> np.ndarray:
        if self.u.robouse_cam is None:
            self.used.add("mujoco-schematic")
            return self._mj_render(cam)
        self._set_cam(cam)
        self.u.sim.render()
        self.u.robouse_cam.update(dt=0.0, force_recompute=True)
        img = self.u.robouse_cam.data.output["rgb"][0].detach().cpu().numpy()
        self.used.add(self.renderer_name)
        return np.asarray(img)[..., :3].astype(np.uint8)

    def _env_step(self, a) -> None:
        t = self.torch.tensor(np.asarray(a, dtype=np.float32)[None], device=self.u.device)
        self.env.step(t)
        self.n += 1
        if self._recording and self.n % FRAME_EVERY == 0 and len(self.frames) < 60:
            self.frames.append(jpeg(self._render("scene")))

    # ---- images without Isaac Lab's renderers: a MuJoCo scene of the parts' shapes --------------------------------
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
            self.mjd.mocap_quat[i] = il_quat_to_wxyz(q)  # MuJoCo is (w, x, y, z)
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
    def renderer(self) -> dict:
        """The renderer that draws this episode's images, and those that drew any so far (`used`)."""
        name = self.renderer_name
        out = {"name": name, "about": RENDERERS[name], "used": sorted(self.used)}
        if note := os.environ.get("ROBOUSE_IL_RENDERER_NOTE"):
            out["why"] = note
        return out

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
            "renderer": self.renderer(),
            "physics": {"name": self.physics, "about": PHYSICS[self.physics], "framework": ISAACLAB},
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
            "fingertip": pose_dict(
                self._np(u.fingertip_midpoint_pos), il_quat_to_wxyz(self._np(u.fingertip_midpoint_quat))
            ),
            "held_part_base": pose_dict(hb, il_quat_to_wxyz(hq)),
            "fixed_part_position_estimate": r3(f + self.fixed_noise, 4),
            "fixed_part_yaw_deg": round(yaw_deg_from_il_quat(self._np(u.fixed_quat)), 1),
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
        """Align the held part's base over its target at a height clear of the fixed part, then descend (nut: descend
        while turning). Isaac Lab 2.3's PhysX let the part slide into place when it touched the shaft or bolt a few
        millimetres off centre; on OvPhysX friction holds it there, so the alignment happens above the fixed part."""
        for phase, z_above in (("align", ALIGN_HEIGHT_M[self.name]), ("descend", -0.003)):
            for _ in range(250):
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
                    if self.name == "gear_mesh":  # turn slowly so the teeth find the mesh with the other gears
                        a[5] = 0.3
                        a[2] = -0.25
                else:
                    a[:3] = np.clip(e / 0.02 * 1.0, -1, 1)
                    if self.name == "nut_thread":
                        a[5] = -1.0  # no turn
                yield ["act", [round(float(x), 4) for x in a]]
        for _ in range(3):
            yield ["skill", "hold", [10]]


if __name__ == "__main__":
    if len(sys.argv) != 3 or sys.argv[1] != "--probe" or sys.argv[2] not in PROBE_ORDER:
        raise SystemExit(f"usage: python isaaclab3_sim.py --probe {{{','.join(PROBE_ORDER)}}}")
    try:
        rc = _probe(sys.argv[2])
    except Exception as e:  # reported to prepare() on stdout
        print(f"PROBE FAILED: {type(e).__name__}: {e}"[:400], flush=True)
        rc = 1
    sys.stdout.flush()
    os._exit(rc)  # skip the slow shutdown: the answer is already printed
