"""Policy server for any LeRobot policy (SmolVLA, pi0, pi0.5, ACT, diffusion, MolmoAct2, ...), speaking the `http-json`
wire of the `vla` harness. Runs in a LeRobot virtualenv (CPU, CUDA or MPS); it is not imported by Robo Use itself.

  POST /reset  {"episode": id}
  POST /act    {"episode": id, "task": "<instruction>", "images": {slot: base64 PNG (upright)}, "state": {...}}
           ->  {"actions": [[...] x n_action_steps], "latency_s": ..., "chunk": k}
  GET  /health

Two observation modes:
  --env.type=libero ...   LIBERO, exactly as lerobot-eval builds it (LeRobot's LIBERO env processors): images are
                          flipped back to robosuite's raw orientation, state {"eef_pos", "eef_quat", "gripper_qpos"}.
  --joint-state           real arms and other joint-space robots: "observation.state" is the concatenation of the
                          request's state values in order, images go to "observation.images.<slot>" as float [0, 1] CHW.
One /act call runs select_action n_action_steps times (the first call predicts a chunk and the rest pop LeRobot's
action queue), so the actions returned are the ones lerobot-eval would apply between two observations.

Usage (arguments after `--` are lerobot-eval's):
  python lerobot_server.py --port 8000 -- --policy.path=HuggingFaceVLA/smolvla_libero --policy.device=cpu \
      --env.type=libero --env.task=libero_10 \
      --env.camera_name_mapping='{"agentview_image":"image","robot0_eye_in_hand_image":"image2"}'
"""
# no `from __future__ import annotations`: LeRobot's config parser reads main()'s real type annotation
import base64
import io
import json
import logging
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
import torch
from PIL import Image

PORT, JOINT = 8000, False
if "--port" in sys.argv:
    i = sys.argv.index("--port")
    PORT = int(sys.argv[i + 1])
    del sys.argv[i:i + 2]
if "--joint-state" in sys.argv:
    JOINT = True
    sys.argv.remove("--joint-state")
if "--" in sys.argv:
    sys.argv.remove("--")

from lerobot.configs import parser  # noqa: E402
from lerobot.configs.eval import EvalPipelineConfig  # noqa: E402
from lerobot.policies.factory import make_policy, make_pre_post_processors  # noqa: E402
from lerobot.utils.constants import ACTION  # noqa: E402
from lerobot.utils.random_utils import set_seed  # noqa: E402

STATE: dict = {}
LOCK = threading.Lock()


def _img(b64: str) -> np.ndarray:
    return np.asarray(Image.open(io.BytesIO(base64.b64decode(b64))).convert("RGB"), dtype=np.uint8)


def _quat2mat(q) -> np.ndarray:
    x, y, z, w = [float(v) for v in q]
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _libero_obs(req: dict) -> dict:
    from lerobot.envs.utils import preprocess_observation

    s = req["state"]
    b = lambda v: np.asarray(v, dtype=np.float64)[None]  # noqa: E731
    raw = {"pixels": {k: np.ascontiguousarray(_img(v)[::-1])[None] for k, v in req["images"].items()},  # upright -> raw
           "robot_state": {"eef": {"pos": b(s["eef_pos"]), "quat": b(s["eef_quat"]), "mat": b(_quat2mat(s["eef_quat"]))},
                           "gripper": {"qpos": b(s["gripper_qpos"]), "qvel": np.zeros((1, 2))},
                           "joints": {"pos": np.zeros((1, 7)), "vel": np.zeros((1, 7))}}}
    obs = preprocess_observation(raw)
    obs["task"] = [str(req["task"])]
    return STATE["env_pre"](obs)


def _joint_obs(req: dict) -> dict:
    vals = []
    for v in req["state"].values():
        vals.extend(np.asarray(list(v.values()) if isinstance(v, dict) else v, dtype=np.float32).ravel().tolist())
    obs = {"observation.state": torch.tensor([vals], dtype=torch.float32), "task": [str(req["task"])]}
    for slot, b64 in req["images"].items():
        img = torch.from_numpy(_img(b64)).permute(2, 0, 1).float().div(255.0)[None]
        obs[f"observation.images.{slot}"] = img
    return obs


def act(req: dict) -> dict:
    P = STATE
    policy = P["policy"]
    ep = P["episodes"].setdefault(str(req.get("episode", "default")), {"chunks": 0, "generator": None})
    t0 = time.time()
    with LOCK, torch.inference_mode():
        obs = _joint_obs(req) if JOINT else _libero_obs(req)
        obs = P["pre"](obs)
        if hasattr(policy, "_rollout_action_generator"):  # MolmoAct2: per-episode flow-matching generator
            policy._rollout_action_generator = ep["generator"]
        out = []
        for _ in range(P["n"]):
            a = policy.select_action(obs)
            a = P["post"](a)
            if not JOINT:
                a = P["env_post"]({ACTION: a})[ACTION]
            out.append([float(x) for x in a.to("cpu").float().numpy().reshape(-1)])
        if hasattr(policy, "_rollout_action_generator"):
            ep["generator"] = policy._rollout_action_generator
        ep["chunks"] += 1
    return {"actions": out, "latency_s": round(time.time() - t0, 3), "chunk": ep["chunks"]}


class Handler(BaseHTTPRequestHandler):
    def _send(self, code: int, obj: dict) -> None:
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path.startswith("/health"):
            return self._send(200, {"ok": True, "model": STATE.get("model"), "n_action_steps": STATE.get("n"), "mode": "joint" if JOINT else "libero"})
        self._send(404, {"error": "not found"})

    def do_POST(self):
        try:
            req = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))) or b"{}")
            if self.path.startswith("/reset"):
                with LOCK:
                    STATE["episodes"][str(req.get("episode", "default"))] = {"chunks": 0, "generator": None}
                    STATE["policy"].reset()
                    STATE["pre"].reset()
                    STATE["post"].reset()
                return self._send(200, {"ok": True})
            if self.path.startswith("/act"):
                return self._send(200, act(req))
            self._send(404, {"error": "not found"})
        except Exception as e:  # noqa: BLE001 - reported to the client
            logging.exception("request failed")
            self._send(500, {"error": f"{type(e).__name__}: {e}"[:500]})

    def log_message(self, fmt, *args):
        logging.info("%s %s", self.address_string(), fmt % args)


@parser.wrap()
def main(cfg: EvalPipelineConfig) -> None:
    set_seed(cfg.seed)
    policy = make_policy(cfg=cfg.policy, env_cfg=cfg.env, rename_map=cfg.rename_map)
    policy.eval()
    pre, post = make_pre_post_processors(
        policy_cfg=cfg.policy, pretrained_path=cfg.policy.pretrained_path,
        preprocessor_overrides={"device_processor": {"device": str(policy.config.device)},
                                "rename_observations_processor": {"rename_map": cfg.rename_map}})
    env_pre = env_post = None
    if not JOINT:
        from lerobot.envs.factory import make_env_pre_post_processors

        env_pre, env_post = make_env_pre_post_processors(env_cfg=cfg.env, policy_cfg=cfg.policy)
    STATE.update(policy=policy, pre=pre, post=post, env_pre=env_pre, env_post=env_post, episodes={},
                 model=str(cfg.policy.pretrained_path), n=int(policy.config.n_action_steps))
    logging.info("LeRobot policy server ready on port %d (%s, n_action_steps=%d)", PORT, STATE["model"], STATE["n"])
    ThreadingHTTPServer(("0.0.0.0", PORT), Handler).serve_forever()


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s")
    main()
