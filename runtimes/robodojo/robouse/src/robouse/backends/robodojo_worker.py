"""RoboDojo simulator worker: one RoboDojo task in NVIDIA Isaac Sim, served over HTTP to the Robo Use `robodojo` backend.

Runs on a GPU machine inside RoboDojo's own conda environment (Isaac Sim 5.1, RoboDojo's Isaac Lab 2.3 fork, cuRobo),
from the root of a RoboDojo checkout (commit pinned in docs/suites/robodojo.md) with its Hugging Face `Assets/`:

  OMNI_KIT_ACCEPT_EULA=YES ROBOUSE_ROBODOJO_SECRET=<secret> python robodojo_worker.py --task stack_bowls --port 8801

The worker builds RoboDojo's own evaluation environment (`src/eval_client/eval_env.create_eval_env`) for one task with
one environment, exactly as `scripts/robodojo.sh eval` does, except that the policy WebSocket client is replaced: the
actions come from this HTTP interface instead of an XPolicyLab policy server. Everything else is RoboDojo's code path:
the evaluation layouts (`Assets/Eval_Layout/RoboDojo/arx_x5/0/<task>_<i>.json`), the layout stability check, the
end-effector action (`take_action` with `left_ee_pose` / `right_ee_pose` solved by RoboDojo's cuRobo IK, 10 physics
steps of 4 ms per action with RoboDojo's interpolation), the task's `step_lim`, and the task's own `run_reward()` stages
and queries, checked after every action by `reward_manager.step()` and `is_episode_end()`. Success is RoboDojo's: the
reward reaches 1 (all stages passed in order, no query violated) within `step_lim` actions.

RPC (POST /rpc, JSON {"method": ..., "args": {...}}, header X-Robouse-Secret): info, reset, step, observe, render,
success, debug_exec (only with ROBOUSE_ROBODOJO_DEBUG=1).
"""
from __future__ import annotations

import argparse
import base64
import io
import json
import os
import sys
import time
import traceback
from http.server import BaseHTTPRequestHandler, HTTPServer

ROOT = os.path.abspath(os.environ.get("ROBODOJO_ROOT", os.getcwd()))
sys.path.insert(0, ROOT)
sys.path.insert(1, os.path.join(ROOT, "XPolicyLab"))  # RoboDojo's eval client imports XPolicyLab's client_server
os.chdir(ROOT)

parser = argparse.ArgumentParser()
parser.add_argument("--task", required=True)
parser.add_argument("--port", type=int, default=8801)
parser.add_argument("--env_cfg_type", default="arx_x5")
parser.add_argument("--eval_seed", type=int, default=0, help="RoboDojo eval seed directory (Eval_Layout/.../<seed>)")
from isaaclab.app import AppLauncher  # noqa: E402

AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
args.headless = True
args.enable_cameras = True
exts = " --enable isaacsim.replicator.behavior --enable isaacsim.sensors.camera"
# Kit's task scheduler sizes its thread pool from the CPUs it sees; in a container that shows many more CPUs than its
# quota the idle workers spin and starve the simulation thread. ROBOUSE_ROBODOJO_KIT_THREADS caps the pool.
if os.environ.get("ROBOUSE_ROBODOJO_KIT_THREADS"):
    exts += f" --/plugins/carb.tasking.plugin/threadCount={int(os.environ['ROBOUSE_ROBODOJO_KIT_THREADS'])}"
args.kit_args = ((getattr(args, "kit_args", None) or "") + exts).strip()

from env.camera_manager.capture.render_sync import add_zero_delay_kit_args  # noqa: E402

add_zero_delay_kit_args(args)
app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

import numpy as np  # noqa: E402
from omegaconf import OmegaConf  # noqa: E402

import src.eval_client.eval_env as eval_env_mod  # noqa: E402
from env.global_configs import BENCHMARK, ENV_CONFIG_PATH  # noqa: E402
from utils.cluttered_generator import UnStableError  # noqa: E402
from utils.load_file import load_yaml  # noqa: E402
from utils.pipeline_utils import process_config, process_randomization  # noqa: E402

SECRET = os.environ.get("ROBOUSE_ROBODOJO_SECRET", "")
GRIPPER_BIAS = 0.145  # RoboDojo's x5 robot_config.yml gripper_bias: link6 origin to the grasp point, along link6 +x
DEBUG = os.environ.get("ROBOUSE_ROBODOJO_DEBUG") == "1"


class _NoPolicy:
    """Stands in for XPolicyLab's WebSocket model client: actions arrive over this worker's HTTP interface."""

    def __init__(self, *a, **k):
        pass

    def call(self, func_name=None, **k):
        return None

    def close(self):
        pass


eval_env_mod.WsModelClient = _NoPolicy


def build_env(task: str):
    task_registry = __import__(f"task.{BENCHMARK}.task_registry", fromlist=["x"])
    eval_cfg = load_yaml(os.path.join(ENV_CONFIG_PATH, args.env_cfg_type + ".yml"))
    eval_cfg.update({"task_name": task, "num_envs": 1, "device_id": 0, "eval_batch": False, "policy_name": "robouse",
                     "additional_info": "robouse", "seed": args.eval_seed, "physx_monitor_enabled": False})
    deploy_cfg = {"policy_name": "robouse", "port": 0, "host": "localhost", "protocol": "ws",
                  "policy_server_url": "ws://localhost:0", "evaluation_id": "robouse", "trial_id": f"{task}-robouse",
                  "action_case_id": f"{task}_case", "repeat_index": None}
    bench = os.path.join(ROOT, "task", BENCHMARK)
    env_cfg = OmegaConf.create({
        "sim": load_yaml(os.path.join(ENV_CONFIG_PATH, "sim", eval_cfg["config"]["sim"] + ".yml")),
        "scene": load_yaml(os.path.join(ENV_CONFIG_PATH, "scene", eval_cfg["config"]["scene"] + ".yml")),
        "camera": load_yaml(os.path.join(ENV_CONFIG_PATH, "camera", eval_cfg["config"]["camera"] + ".yml")),
        "robot": load_yaml(os.path.join(ENV_CONFIG_PATH, "robot", eval_cfg["config"]["robot"] + ".yml")),
        "task_env": load_yaml(task_registry.task_config_path(os.path.join(bench, "config"), task)),
        "eval_cfg": eval_cfg, "deploy_cfg": deploy_cfg,
    })
    OmegaConf.update(env_cfg, "sim.scene.num_envs", 1, force_add=True)
    OmegaConf.update(env_cfg, "eval_cfg.num_envs", 1, force_add=True)
    env_cfg = process_randomization(env_cfg)
    env_cfg, eval_num = process_config(env_cfg, task_name=task)
    OmegaConf.update(env_cfg, "camera.default_frequency", eval_cfg["observation"].get("collect_freq", 0), force_add=True)
    env_cfg.sim.seed = [0]
    env = eval_env_mod.create_eval_env(env_cfg, simulation_app)
    return env, int(eval_num)


def _np(x):
    if hasattr(x, "detach"):
        x = x.detach().cpu().numpy()
    return np.asarray(x, dtype=float).reshape(-1)


def _r(v, n=4):
    return [round(float(x), n) for x in np.asarray(v, dtype=float).reshape(-1)]


def _jpeg(img, quality=85) -> str:
    from PIL import Image

    buf = io.BytesIO()
    Image.fromarray(np.asarray(img, dtype=np.uint8)[:, :, :3]).save(buf, format="JPEG", quality=quality)
    return base64.b64encode(buf.getvalue()).decode()


def _quat_mat(q):
    w, x, y, z = [float(v) for v in q]
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class Worker:
    def __init__(self, task: str):
        self.task = task
        t = time.time()
        self.env, self.eval_num = build_env(task)
        self.load_s = round(time.time() - t, 1)
        self.layouts = len(self.env.seed_manager.seed_info)
        self.layout = None
        self.steps = 0
        self.ik_fail = {"left": 0, "right": 0}
        self.last_ik = {"left": "", "right": ""}
        self._wrap_ik()
        self.arms = {}
        for robot in self.env.robot_manager.robot_list:
            if robot.type == "target":
                self.arms[robot.arm_name.split("_")[0]] = robot
        self.home = {}
        self.episodes = 0
        self.broken = False
        self.has_articulation = bool(self.env.scene_manager.layout_manager.task_config.get("Articulation"))

    def _wrap_ik(self):
        rm = self.env.robot_manager
        orig = rm.solve_ik

        def solve_ik(target_pose, env_idx, robot, trans="world"):
            res = orig(target_pose=target_pose, env_idx=env_idx, robot=robot, trans=trans)
            side = robot.arm_name.split("_")[0]
            self.last_ik[side] = res.get("status", "")
            if res.get("status") != "Success":
                self.ik_fail[side] = self.ik_fail.get(side, 0) + 1
            return res

        rm.solve_ik = solve_ik

    # ---- episode ---------------------------------------------------------------------------------------------------
    def reset(self, layout: int = 0) -> dict:
        env = self.env
        layout = int(layout) % max(1, self.layouts)
        if self.episodes >= 1:
            # Every episode starts in a fresh process. Resetting RoboDojo's environment a second time in one process
            # after a policy has moved things was unreliable in our runs: tasks with articulated objects failed to
            # find them again (get_scene_object -> None) and rigid-object tasks were rejected by the layout
            # stability check. The client waits for the worker to come back and resets again.
            global RESTART
            RESTART = True
            return {"restarting": True}
        t = time.time()
        self.episodes += 1
        try:
            env.reset(seed=[layout])
        except UnStableError as e:
            self.broken = True
            raise RuntimeError(f"RoboDojo rejected layout {layout} as unstable: {e}") from e
        except Exception:
            self.broken = True
            raise
        # what run_eval() does before handing control to the policy
        env.run_reward()
        if hasattr(env, "get_score"):
            env.get_score()
        if getattr(env, "interact", False) and hasattr(env, "query_support_arm_traj"):
            env.query_support_arm_traj(env_idx=0)
        self.layout = layout
        self.steps = 0
        self.ik_fail = {"left": 0, "right": 0}
        self.home = {side: _r(env.robot_manager.get_real_endpose(robot)[0], 5) for side, robot in self.arms.items()}
        return {"layout": layout, "reset_s": round(time.time() - t, 1), "instruction": self.instruction(),
                "step_lim": int(env.step_lim), "ee": self.ee()}

    def instruction(self) -> str:
        ins = self.env.obs_manager.instruction
        return str(ins[0]) if ins else ""

    def status(self) -> dict:
        env = self.env
        done = bool(env.end_flag[0])
        success = done and bool(env.success[0])
        out = {"success": success, "ended": done, "failed": done and not success, "steps": int(env.take_action_cnt[0]),
               "step_lim": int(env.step_lim)}
        try:
            out["score"] = round(float(env.reward_manager.get_score()[0]), 2) if hasattr(env, "get_score") else None
        except Exception:
            out["score"] = None
        out["stages_left"] = len(env.reward_manager.check_list[0])
        return out

    def step(self, action, frame: bool = False, camera: str = "cam_head") -> dict:
        a = np.asarray(action, dtype=float).reshape(-1)
        if a.shape[0] != 16 or not np.all(np.isfinite(a)):
            raise ValueError("action must be 16 finite numbers: left x y z qw qx qy qz grip, right x y z qw qx qy qz grip")
        act = {}
        for i, side in enumerate(["left", "right"]):
            seg = a[i * 8:(i + 1) * 8]
            q = seg[3:7]
            n = float(np.linalg.norm(q))
            if n < 1e-6:
                raise ValueError(f"{side} quaternion must be non-zero")
            act[f"{side}_ee_pose"] = [float(x) for x in seg[:3]] + [float(x) for x in q / n]
            act[f"{side}_ee_joint_state"] = [float(np.clip(seg[7], 0.0, 1.0))]
        env = self.env
        before = int(env.take_action_cnt[0])
        self.last_ik = {"left": "", "right": ""}
        env.take_action(act)
        executed = int(env.take_action_cnt[0]) - before
        self.steps += executed
        out = {**self.status(), "executed": executed, "ik": dict(self.last_ik), "ee": self.ee()}
        if frame:
            out["frame"] = _jpeg(self._image(camera))
        return out

    # ---- observation -----------------------------------------------------------------------------------------------
    def ee(self) -> dict:
        rm = self.env.robot_manager
        out = {}
        for side, robot in self.arms.items():
            g = np.mean(rm.get_end_effector_real_val(robot)[0])
            sc = robot.gripper_scale
            opening = (g - sc[0]) / (sc[1] - sc[0]) if robot.gripper_move["sign"] == 1 else (sc[1] - g) / (sc[1] - sc[0])
            out[side] = _r(rm.get_real_endpose(robot)[0], 5) + [round(float(np.clip(opening, 0, 1)), 3)]
        return out

    def observe(self) -> dict:
        env = self.env
        rm = env.robot_manager
        robot_state = {}
        for side, robot in self.arms.items():
            pose = rm.get_real_endpose(robot)[0]
            g = np.mean(rm.get_end_effector_real_val(robot)[0])
            sc = robot.gripper_scale
            opening = (g - sc[0]) / (sc[1] - sc[0]) if robot.gripper_move["sign"] == 1 else (sc[1] - g) / (sc[1] - sc[0])
            R = _quat_mat(pose[3:7])
            robot_state[side] = {
                "ee_pos": _r(pose[:3]), "ee_quat": _r(pose[3:7]),
                "ee_axes": {"x": _r(R[:, 0], 3), "y": _r(R[:, 1], 3), "z": _r(R[:, 2], 3)},
                "tcp_pos": _r(np.asarray(pose[:3], float) + GRIPPER_BIAS * R[:, 0]),
                "gripper_open": round(float(np.clip(opening, 0, 1)), 3),
                "joints": _r(rm.get_joint(robot)[0]),
                "home_pos": self.home.get(side, [None])[:3], "home_quat": self.home.get(side, [None] * 7)[3:7],
                "base_pos": _r(robot.base_link_origin_pose[:3]) if getattr(robot, "base_link_origin_pose", None) is not None else None,
            }
        lm = env.scene_manager.layout_manager
        objects = []
        for t in ("Rigid", "Articulation", "Geometry", "Dynamic", "Garment", "Fluid"):
            recs = lm.object_records_by_type.get(t) if hasattr(lm.object_records_by_type, "get") else None
            if recs is None:
                continue
            for rec in recs.layout_records_by_env[0]:
                label = rec.get("label")
                inst = rec.get("inst_name")
                if not label or not inst:
                    continue
                o = {"label": label, "type": t.lower()}
                try:
                    meta = lm.get_instance_metadata(env_idx=0, inst_name=inst) or {}
                    o["category"] = meta.get("model_name")
                    o["model_id"] = meta.get("model_id")
                    desc = meta.get("description")
                    if isinstance(desc, list) and desc:
                        o["description"] = str(desc[0])
                    elif isinstance(desc, str):
                        o["description"] = desc
                    pos, rot = lm.get_instance_pose(env_idx=0, inst_name=inst)
                    if pos is not None:
                        pos, rot = _np(pos), _np(rot)
                        o["pos"], o["quat"] = _r(pos), _r(rot)
                        R = _quat_mat(rot)
                        o["up_axis"] = _r(R[:, 2], 3)
                        o["yaw_deg"] = round(float(np.degrees(np.arctan2(R[1, 0], R[0, 0]))), 1)
                        v = lm.get_instance_bbox_vertices(inst, 0)
                        if v is not None:
                            v = np.asarray(v, dtype=float).reshape(-1, 3)
                            w = (R @ v.T).T + pos  # oriented box corners (object frame) -> world
                            o["bbox_min"], o["bbox_max"] = _r(w.min(0)), _r(w.max(0))
                            o["size"] = _r(v.max(0) - v.min(0))
                    if t == "Articulation":
                        obj = lm.get_scene_object(env_idx=0, inst_name=inst)
                        info = obj.get_all_joints_info() if obj is not None and hasattr(obj, "get_all_joints_info") else None
                        if info:
                            js = {}
                            for jn, ji in info.items():
                                pos_, lo, hi = (float(_np(ji.get(k))[0]) if ji.get(k) is not None else None
                                                for k in ("position", "lower", "upper"))
                                js[jn] = {"position": round(pos_, 5), "lower": round(lo, 5), "upper": round(hi, 5)}
                                if pos_ is not None and lo is not None and hi is not None and hi != lo:
                                    js[jn]["ratio"] = round((pos_ - lo) / (hi - lo), 3)  # RoboDojo's joint "ratio"
                            o["joints"] = js
                except Exception as e:  # never fail a whole observation on one odd object
                    o["error"] = f"{type(e).__name__}: {e}"[:200]
                objects.append(o)
        return {"robot": robot_state, "objects": objects, "instruction": self.instruction(), **self.status(),
                "layout": self.layout}

    def _image(self, camera: str = "cam_head"):
        om = self.env.obs_manager
        om.render_for_capture()
        data = om.get_obs(env_idx_list=[0])[0]
        vis = data.get("vision", {})
        if camera not in vis:
            raise ValueError(f"unknown camera {camera!r}; cameras: {sorted(vis)}")
        return vis[camera]["color"]

    def render(self, camera: str = "cam_head") -> dict:
        return {"jpeg": _jpeg(self._image(camera))}

    def info(self) -> dict:
        return {"task": self.task, "layouts": self.layouts, "eval_num": self.eval_num, "load_s": self.load_s,
                "step_lim": int(self.env.step_lim), "cameras": ["cam_head", "cam_left_wrist", "cam_right_wrist"],
                "arms": sorted(self.arms), "episodes": self.episodes}


W: Worker | None = None
RESTART = False


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, *a):
        pass

    def _send(self, code: int, obj: dict):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        self._send(200, {"ok": True, "task": args.task, "ready": W is not None})

    def do_POST(self):
        n = int(self.headers.get("Content-Length", 0))
        raw = self.rfile.read(n)
        if SECRET and self.headers.get("X-Robouse-Secret") != SECRET:
            return self._send(403, {"ok": False, "error": "forbidden"})
        try:
            req = json.loads(raw)
            m, a = req.get("method"), req.get("args") or {}
            if m == "debug_exec":
                if not DEBUG:
                    raise PermissionError("debug_exec is disabled")
                g = {"W": W, "np": np, "env": W.env}
                exec(a["code"], g)
                res = {"out": repr(g.get("out"))[:20000]}
            elif m in ("info", "reset", "step", "observe", "render", "status"):
                res = getattr(W, m)(**a)
            elif m == "success":
                res = W.status()
            else:
                raise ValueError(f"unknown method {m!r}")
            self._send(200, {"ok": True, "result": res})
            if RESTART:
                self.close_connection = True  # end this kept-alive connection so the server loop can restart
        except Exception as e:
            traceback.print_exc()
            self._send(200, {"ok": False, "error": f"{type(e).__name__}: {e}"[:1000]})


if __name__ == "__main__":
    W = Worker(args.task)
    print(f"[robodojo_worker] {args.task}: {W.layouts} layouts, loaded in {W.load_s}s; listening on {args.port}", flush=True)
    srv = HTTPServer(("0.0.0.0", args.port), Handler)
    srv.timeout = 1.0
    while not RESTART:
        srv.handle_request()
    srv.server_close()
    print("[robodojo_worker] restarting for a fresh episode", flush=True)
    os.execv(sys.executable, [sys.executable] + sys.argv)
