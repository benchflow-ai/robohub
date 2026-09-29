"""RoboCasa simulation worker. Runs inside the separate RoboCasa virtualenv (robosuite 1.5 + robocasa + MuJoCo 3.3.1),
because RoboCasa's pinned dependencies conflict with the main robouse environment (robosuite 1.4 for LIBERO).

The robouse `robocasa` backend (robocasa.py) starts this script as a subprocess and talks to it over stdin/stdout:
one JSON request per line, one JSON response per line; a `render` response is followed by the raw RGB bytes.
Only numpy, robosuite and robocasa are imported here (never the robouse package).
"""
from __future__ import annotations

import json
import os
import sys

sys.path = [p for p in sys.path if os.path.abspath(p or ".") != os.path.dirname(os.path.abspath(__file__))]

import numpy as np  # noqa: E402

STEP_M = .02        # metres per action unit per step (world frame)
OSC_MAX = .05       # robosuite OSC_POSE output_max for position (m per step at input 1)
LEAD_M = .05


def _rotvec(R):
    """Axis-angle vector of a rotation matrix."""
    c = np.clip((np.trace(R) - 1) / 2, -1, 1)
    a = np.arccos(c)
    if a < 1e-6:
        return np.zeros(3)
    v = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / (2 * np.sin(a))
    return v * a


def _rot(axis, ang):
    axis = np.asarray(axis, dtype=float) / np.linalg.norm(axis)
    K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + np.sin(ang) * K + (1 - np.cos(ang)) * K @ K


def _quat_to_mat(q_xyzw):
    x, y, z, w = q_xyzw
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _assets_root() -> str:
    import robocasa.models

    return os.path.abspath(robocasa.models.assets_root)


def _relative_paths(ep_meta: dict) -> dict:
    """Object model paths in an ep_meta, made relative to RoboCasa's asset root (objects/lightwheel/<cat>/<model>/model.xml),
    so a recorded layout does not depend on where RoboCasa is installed."""
    root = _assets_root()
    for c in ep_meta.get("object_cfgs", []):
        info = c.get("info") or {}
        p = info.get("mjcf_path")
        if isinstance(p, str):
            p = p.replace("\\", "/")
            if os.path.isabs(p):
                p = os.path.relpath(p, root) if p.startswith(root + os.sep) else "objects/" + p.split("/objects/")[-1]
            info["mjcf_path"] = p
    return ep_meta


def _resolve_paths(ep_meta: dict) -> dict:
    """The inverse of _relative_paths: absolute object model paths under this machine's RoboCasa asset root."""
    root = _assets_root()
    for c in ep_meta.get("object_cfgs", []):
        info = c.get("info") or {}
        p = info.get("mjcf_path")
        if isinstance(p, str) and not os.path.isabs(p):
            full = os.path.join(root, p)
            if not os.path.exists(full):
                raise FileNotFoundError(f"RoboCasa object model {p} is not installed under {root}")
            info["mjcf_path"] = full
    return ep_meta


class Worker:
    def __init__(self):
        self.env = None

    # ---- setup ---------------------------------------------------------------------------------------------------
    def make(self, env_name: str, layout: int, style: int, seed: int, tool: str = "down", kwargs: dict | None = None,
             ep_meta: dict | None = None) -> dict:
        import random

        import robocasa  # noqa: F401  (registers the kitchen environments)

        random.seed(int(seed))  # some RoboCasa placement code draws from the global generators
        np.random.seed(int(seed))
        import robosuite
        from robosuite.controllers import load_composite_controller_config

        from robosuite.environments.base import REGISTERED_ENVS

        # Only the lightwheel object registry is installed (see docs/suites/robocasa.md); object groups that exist only in
        # the objaverse registry are swapped for a lightwheel category of the same use.
        base = REGISTERED_ENVS[env_name]
        swap = {"mug": "glass_cup", "pan": "saucepan"}

        def _get_obj_cfgs(env_self):
            cfgs = base._get_obj_cfgs(env_self)
            for c in cfgs:
                g = c.get("obj_groups")
                if isinstance(g, str) and g in swap:
                    c["obj_groups"] = swap[g]
                # containers to nest an object in come from objaverse only; place the object directly instead
                c.get("placement", {}).pop("try_to_place_in", None)
            return cfgs

        patched = f"RobouseLW{env_name}"
        if patched not in REGISTERED_ENVS:
            type(patched, (base,), {"_get_obj_cfgs": _get_obj_cfgs})
        self.env = robosuite.make(
            patched, robots="PandaOmron",
            controller_configs=load_composite_controller_config(controller=None, robot="PandaOmron"),
            has_renderer=False, has_offscreen_renderer=True, use_camera_obs=False, ignore_done=True,
            layout_ids=[int(layout)], style_ids=[int(style)], obj_registries=("lightwheel",), seed=int(seed),
            translucent_robot=False, **(kwargs or {}))
        if ep_meta:  # replay a recorded episode layout exactly (RoboCasa's own demo-replay mechanism)
            self.env.set_ep_meta(_resolve_paths(ep_meta))
        self.obs = self.env.reset()
        self.grip = -1.0
        self.adim = self.env.action_spec[0].shape[0]
        R0 = self._eef_R()
        if tool == "forward":  # pitch the hand so the fingers point along the robot's forward direction
            fwd = self._base_R()[:, 0]
            left = np.cross([0, 0, 1], fwd)
            self.target_R = _rot(left, -np.pi / 2) @ R0 if R0[:, 2] @ fwd < .5 else R0
        else:
            self.target_R = R0
        self.target_p = np.asarray(self.obs["robot0_eef_pos"], dtype=float).copy()
        for _ in range(60):  # part of the reset: turn to the tool orientation, then hold it
            self._send()
        self.target_p = np.asarray(self.obs["robot0_eef_pos"], dtype=float).copy()
        return {"lang": self.env.get_ep_meta().get("lang", ""), "cameras": list(self.env.sim.model.camera_names)}

    def ep_meta(self) -> dict:
        meta = json.loads(json.dumps(self.env.get_ep_meta(), default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o)))
        return _relative_paths(meta)

    # ---- helpers -------------------------------------------------------------------------------------------------
    def _pos(self, name: str):
        m, d = self.env.sim.model, self.env.sim.data
        for kind, getter in (("geom", lambda i: d.geom_xpos[i]), ("body", lambda i: d.xpos[i]), ("site", lambda i: d.site_xpos[i])):
            try:
                i = getattr(m, f"{kind}_name2id")(name)
            except Exception:
                continue
            if i is not None and i >= 0:
                return [float(v) for v in getter(i)]
        return None

    def _geoms_containing(self, sub: str):
        m = self.env.sim.model
        return [n for n in m.geom_names if n and sub in n]

    def _handle_pos(self, fx):
        for name in (f"{fx.name}_door_handle_reg_main", f"{fx.name}_handle_reg_main", fx.handle_name, f"{fx.name}_door_handle_main"):
            p = self._pos(name)
            if p is not None:
                return p
        return None

    def _joint_geoms(self, joint: str, prefer: str = "") -> list[int]:
        """Collision geoms of the body a joint moves (and its child bodies); those whose name ends with `prefer` if any."""
        import mujoco

        m = self.env.sim.model._model
        j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, joint)
        if j < 0:
            return []
        b = int(m.jnt_bodyid[j])

        def under(k):
            while k > 0:
                if k == b:
                    return True
                k = int(m.body_parentid[k])
            return False

        gs = [g for g in range(m.ngeom) if m.geom_contype[g] and under(int(m.geom_bodyid[g]))]
        named = [g for g in gs if prefer and (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or "").endswith(prefer)]
        return named or gs

    def _geoms_box(self, gs: list[int]):
        """World axis-aligned bounding box (lo, hi) of geoms."""
        m, d = self.env.sim.model._model, self.env.sim.data._data
        pts = []
        for g in gs:
            R = d.geom_xmat[g].reshape(3, 3)
            c = d.geom_xpos[g] + R @ m.geom_aabb[g][:3]
            e = np.abs(R) @ m.geom_aabb[g][3:]
            pts += [c - e, c + e]
        pts = np.asarray(pts)
        return pts.min(0), pts.max(0)

    def _eef_R(self):
        return _quat_to_mat(self.obs["robot0_eef_quat"])

    def _send(self) -> None:
        """One control step toward the commanded gripper target (position self.target_p, orientation self.target_R)."""
        d_world = self.target_p - np.asarray(self.obs["robot0_eef_pos"], dtype=float)
        R_err = self.target_R @ self._eef_R().T  # world-frame rotation from current to target
        w_world = _rotvec(R_err)
        Rb = self._base_R()
        full = np.zeros(self.adim)
        full[0:3] = np.clip(Rb.T @ np.asarray(d_world) / OSC_MAX, -1, 1)
        full[3:6] = np.clip(Rb.T @ w_world / .5, -1, 1)
        full[6] = self.grip
        full[-1] = -1.0  # arm mode (base does not move)
        self.obs, _, _, _ = self.env.step(full)

    def _base_R(self):
        return _quat_to_mat(self.obs["robot0_base_quat"])  # robosuite quats are xyzw

    # ---- stepping ------------------------------------------------------------------------------------------------
    def step(self, action) -> dict:
        a = np.asarray(action, dtype=float)
        d_world = np.clip(a[:3], -1, 1) * STEP_M
        g = float(a[3]) if len(a) > 3 else 0.0
        if g > 1e-3:
            self.grip = 1.0
        elif g < -1e-3:
            self.grip = float(1.0 - 2.0 * min(1.0, -g))
        hand = np.asarray(self.obs["robot0_eef_pos"], dtype=float)
        tgt = self.target_p + d_world
        off = tgt - hand
        n = float(np.linalg.norm(off))
        if n > LEAD_M:  # the target stays within 5 cm of the hand so a blocked arm does not wind up
            tgt = hand + off * (LEAD_M / n)
        self.target_p = tgt
        self._send()
        return {"success": bool(self.env._check_success())}

    def success(self) -> dict:
        return {"success": bool(self.env._check_success())}

    # ---- observation ---------------------------------------------------------------------------------------------
    def observe(self, features: list[str]) -> dict:
        o, env = self.obs, self.env
        st = {"hand_pos": [round(float(v), 4) for v in o["robot0_eef_pos"]],
              "hand_target": [round(float(v), 4) for v in self.target_p],
              "gripper_open": round(float(np.clip(np.mean(np.abs(o["robot0_gripper_qpos"])) / .04, 0, 1)), 3),
              "robot_base_pos": [round(float(v), 4) for v in o["robot0_base_pos"]],
              "robot_forward": [round(float(v), 4) for v in self._base_R()[:, 0]]}
        r = lambda v: None if v is None else [round(float(x), 4) for x in v]
        for f in features:
            if f == "object":
                st["object_pos"] = r(o["obj_pos"])
                ob = env.objects["obj"]
                cat = ""
                for c in getattr(env, "object_cfgs", []):
                    if c.get("name") == "obj":
                        cat = c.get("info", {}).get("cat", "")
                st["object_category"] = cat
                st["object_size"] = r(getattr(ob, "size", [0, 0, 0]))
            elif f in ("sink", "cabinet"):
                fx = env.sink if f == "sink" else env.cab
                ints = fx.get_int_sites(relative=False)
                p0, px, py, pz = (np.asarray(v, dtype=float) for v in list(ints.values())[0][:4])
                centre = p0 + ((px - p0) + (py - p0) + (pz - p0)) / 2
                key = "sink_basin" if f == "sink" else "cabinet_interior"
                st[f"{key}_center"] = r(centre)
                st[f"{key}_half_size"] = r(np.abs(np.array([np.linalg.norm(px - p0), np.linalg.norm(py - p0), np.linalg.norm(pz - p0)])) / 2)
                st[f"{key}_bottom_z"] = round(float(p0[2]), 4)
                if f == "cabinet":
                    st["cabinet_interior_top_z"] = round(float(pz[2]), 4)
                    st["cabinet_front_normal"] = r(_quat_to_mat(np.r_[fx.quat[1:], fx.quat[0]])[:, 1] * -1)
            elif f == "faucet":
                import mujoco

                m, d = env.sim.model._model, env.sim.data._data
                pre = env.sink.naming_prefix
                hs = env.sink.get_handle_state(env=env)
                j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, f"{pre}handle_joint")
                lo, hi = self._geoms_box(self._joint_geoms(f"{pre}handle_joint", "handle_main"))
                st["faucet_handle_pos"] = r((lo + hi) / 2)
                st["faucet_handle_size"] = r(hi - lo)
                st["faucet_handle_hinge_pos"] = r(d.xanchor[j])
                st["faucet_handle_hinge_axis"] = r(d.xaxis[j])
                st["faucet_handle_angle"] = round(float(hs.get("handle_joint", 0.0)), 3)
                st["water_on"] = bool(hs.get("water_on", False))
            elif f == "coffee":
                cm = env.coffee_machine
                names = [f"{cm.naming_prefix}{n}" for n in cm._start_button_names]
                pts = [self._pos(n) for n in names]
                pts = [p for p in pts if p is not None]
                st["coffee_button_pos"] = r(np.mean(pts, axis=0)) if pts else None
                st["coffee_machine_on"] = bool(cm.get_state()["turned_on"])
            elif f == "microwave":
                mw = env.microwave
                st["microwave_start_button_pos"] = r(self._pos(f"{mw.name}_start_button"))
                st["microwave_stop_button_pos"] = r(self._pos(f"{mw.name}_stop_button"))
                st["microwave_on"] = bool(mw.get_state()["turned_on"])
            elif f == "toaster":
                ts = env.toaster
                sp = 0
                for pair in range(len(ts.get_state(env).keys())):
                    if ts.check_slot_contact(env, "obj", pair):
                        sp = pair
                        break
                lo, hi = self._geoms_box(self._joint_geoms(ts._joint_names[f"lever_{sp}"], "handle"))
                st["toaster_lever_pos"] = r((lo + hi) / 2)
                st["toaster_lever_size"] = r(hi - lo)
                st["toaster_lever_pressed"] = round(float(ts.get_state(env, slot_pair=sp)["lever"]), 3)
                st["toaster_on"] = bool(ts.get_state(env, slot_pair=sp)["turned_on"])
            elif f == "kettle":
                k = env.electric_kettle
                ks = k.get_state(env)
                for key, joint in (("switch", "switch"), ("lid", "lid")):
                    lo, hi = self._geoms_box(self._joint_geoms(k._joint_names[joint], "_main"))
                    st[f"kettle_{key}_pos"] = r((lo + hi) / 2)
                    st[f"kettle_{key}_size"] = r(hi - lo)
                import mujoco

                m, d = env.sim.model._model, env.sim.data._data
                j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, k._joint_names["lid"])
                st["kettle_pos"] = r(k.pos)
                st["kettle_lid_hinge_pos"] = r(d.xanchor[j])
                st["kettle_lid_hinge_axis"] = r(d.xaxis[j])
                st["kettle_lid_open"] = round(float(ks.get("lid", 0.0)), 3)
                st["kettle_on"] = bool(ks.get("turned_on", False))
            elif f == "stove":
                import mujoco

                m, d = env.sim.model._model, env.sim.data._data
                pre = env.stove.naming_prefix
                j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, f"{pre}knob_{env.knob}_joint")
                ridge = self._joint_geoms(f"{pre}knob_{env.knob}_joint", "_main")
                g = ridge[0]
                R = d.geom_xmat[g].reshape(3, 3)
                half = m.geom_aabb[g][3:]
                k = int(np.argmax(half))
                lo, hi = self._geoms_box(self._joint_geoms(f"{pre}knob_{env.knob}_joint"))
                st["stove_knob_pos"] = r(d.xanchor[j])
                st["stove_knob_axis"] = r(d.xaxis[j])
                st["stove_knob_ridge_dir"] = r(R[:, k])
                st["stove_knob_ridge_half_length"] = round(float(half[k]), 4)
                st["stove_knob_top_z"] = round(float(hi[2]), 4)
                st["stove_knob_angle"] = round(float(d.qpos[m.jnt_qposadr[j]]), 3)
                st["burner_on"] = bool(env.stove.is_burner_on(env=env, burner_loc=env.knob))
            elif f in ("drawer", "door"):
                fx = env.drawer if f == "drawer" else env.fxtr
                st[f"{f}_handle_pos"] = r(self._handle_pos(fx))
                if f == "drawer":  # the measure RoboCasa's drawer tasks judge (slide travel / 55 % of the drawer depth)
                    ds = fx.get_door_state(env)
                else:
                    ds = fx.get_joint_state(env, fx.door_joint_names)
                st[f"{f}_open_fraction"] = round(float(max(ds.values())), 3) if ds else None
                st[f"{f}_front_normal"] = r(_quat_to_mat(np.r_[fx.quat[1:], fx.quat[0]])[:, 1] * -1)
        return st

    # ---- rendering -----------------------------------------------------------------------------------------------
    def render(self, camera: str, width: int, height: int) -> bytes:
        img = self.env.sim.render(width=width, height=height, camera_name=camera)
        return np.ascontiguousarray(img[::-1]).astype(np.uint8).tobytes()

    def camera(self, name: str) -> dict:
        m, d = self.env.sim.model, self.env.sim.data
        i = m.camera_name2id(name)
        return {"pos": [float(v) for v in d.cam_xpos[i]], "xmat": [float(v) for v in d.cam_xmat[i]], "fovy": float(m.cam_fovy[i])}


def main() -> None:
    w = Worker()
    out = sys.stdout.buffer
    real_stdout = sys.stdout
    sys.stdout = sys.stderr  # library prints must not corrupt the protocol
    for line in sys.stdin:
        req = json.loads(line)
        cmd = req.pop("cmd")
        try:
            if cmd == "render":
                data = w.render(req["camera"], int(req["width"]), int(req["height"]))
                out.write((json.dumps({"ok": True, "nbytes": len(data)}) + "\n").encode())
                out.write(data)
                out.flush()
                continue
            if cmd == "close":
                break
            res = getattr(w, cmd)(**req)
            out.write((json.dumps({"ok": True, "result": res}) + "\n").encode())
        except Exception as e:  # report and keep serving
            import traceback

            out.write((json.dumps({"ok": False, "error": f"{type(e).__name__}: {e}", "trace": traceback.format_exc()[-2000:]}) + "\n").encode())
        out.flush()
    real_stdout.flush()


if __name__ == "__main__":
    main()
