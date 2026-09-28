"""MolmoAct2 policy server for the robouse `molmoact2` harness. Runs on an NVIDIA GPU inside a LeRobot checkout.

It loads the policy exactly as `lerobot-eval` does (same config parser, `make_policy`, policy pre/post processors
and LIBERO env processor), and serves one action chunk per HTTP request:

  POST /reset  {"episode": id}                                   start an episode (fresh action queue and seed)
  POST /act    {"episode": id, "task": "<language instruction>",
                "images": {"image": <base64 PNG>, "wrist_image": <base64 PNG>},   upright images (top row = top)
                "state": {"eef_pos": [3], "eef_quat": [x, y, z, w], "gripper_qpos": [2]}}
           ->  {"actions": [[7 floats] x n_action_steps], "latency_s": ...}
  GET  /health

Observation mapping, identical to lerobot-eval on LIBERO: each upright image is flipped back to robosuite's raw
(upside-down) orientation and passed through LeRobot's LiberoProcessorStep, which rotates it 180 degrees and
builds the 8-D state [eef_pos, axis-angle(eef_quat), gripper_qpos]; the policy preprocessor then normalizes the
state (quantiles from the checkpoint) and the postprocessor un-normalizes the actions. One /act call runs
select_action n_action_steps times (the first call predicts the chunk, the rest pop the queue), so the actions
returned are exactly the ones lerobot-eval would apply between two observations. Episodes are independent: each
keeps its own per-episode flow-matching generator (per_episode_seed, eval_seed), swapped in under a lock.

Usage (in the LeRobot venv; the arguments are lerobot-eval's):
  python server.py --port 8000 -- --policy.path=<ckpt> --policy.inference_action_mode=continuous \
      --policy.dtype=bfloat16 --policy.device=cuda --policy.per_episode_seed=true --policy.eval_seed=1000 \
      --env.type=libero --env.task=libero_spatial \
      --env.camera_name_mapping='{"agentview_image":"image","robot0_eye_in_hand_image":"wrist_image"}' --seed=1000
"""

# no `from __future__ import annotations`: LeRobot's config parser reads main()'s real type annotation
import base64
import io
import json
import logging
import sys
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
import torch
from PIL import Image

PORT = 8000
if "--port" in sys.argv:
    i = sys.argv.index("--port")
    PORT = int(sys.argv[i + 1])
    del sys.argv[i:i + 2]
if "--" in sys.argv:
    sys.argv.remove("--")

from lerobot.configs import parser  # noqa: E402
from lerobot.configs.eval import EvalPipelineConfig  # noqa: E402
from lerobot.envs.factory import make_env_pre_post_processors  # noqa: E402
from lerobot.envs.utils import preprocess_observation  # noqa: E402
from lerobot.policies import make_policy, make_pre_post_processors  # noqa: E402
from lerobot.utils.constants import ACTION  # noqa: E402
from lerobot.utils.random_utils import set_seed  # noqa: E402

STATE: dict = {}
LOCK = threading.Lock()


def _quat2mat(q) -> np.ndarray:
    x, y, z, w = [float(v) for v in q]
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _raw_image(b64: str) -> np.ndarray:
    img = np.asarray(Image.open(io.BytesIO(base64.b64decode(b64))).convert("RGB"), dtype=np.uint8)
    return np.ascontiguousarray(img[::-1])  # upright -> robosuite's raw orientation (as LiberoEnv returns it)


def _observation(req: dict) -> dict:
    """The dict LeRobot's LiberoEnv._format_raw_obs returns, with a batch dimension of 1."""
    s = req["state"]
    b = lambda v: np.asarray(v, dtype=np.float64)[None]
    return {
        "pixels": {k: _raw_image(v)[None] for k, v in req["images"].items()},
        "robot_state": {
            "eef": {"pos": b(s["eef_pos"]), "quat": b(s["eef_quat"]), "mat": b(_quat2mat(s["eef_quat"]))},
            "gripper": {"qpos": b(s["gripper_qpos"]), "qvel": np.zeros((1, 2))},
            "joints": {"pos": np.zeros((1, 7)), "vel": np.zeros((1, 7))},  # not used by LiberoProcessorStep
        },
    }


def act(req: dict) -> dict:
    P = STATE
    policy = P["policy"]
    ep = P["episodes"].setdefault(str(req.get("episode", "default")), {"generator": None, "chunks": 0})
    t0 = time.time()
    with LOCK, torch.inference_mode():
        obs = preprocess_observation(_observation(req))
        obs["task"] = [str(req["task"])]
        obs = P["env_pre"](obs)
        obs = P["pre"](obs)
        # swap in this episode's rollout state
        policy._action_queue = deque(maxlen=policy.config.n_action_steps)
        policy._rollout_action_generator = ep["generator"]
        if ep["generator"] is None:
            policy._rollout_task_key = None  # first chunk of the episode: seed = eval_seed, whatever ran before
        out = []
        for _ in range(policy.config.n_action_steps):
            a = policy.select_action(obs)
            a = P["post"](a)
            a = P["env_post"]({ACTION: a})[ACTION]
            out.append([float(x) for x in a.to("cpu").float().numpy().reshape(-1)])
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
            return self._send(200, {"ok": True, "model": STATE.get("model"), "n_action_steps": STATE.get("n")})
        self._send(404, {"error": "not found"})

    def do_POST(self):
        try:
            req = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))) or b"{}")
            if self.path.startswith("/reset"):
                STATE["episodes"][str(req.get("episode", "default"))] = {"generator": None, "chunks": 0}
                with LOCK:
                    STATE["pre"].reset()
                    STATE["post"].reset()
                return self._send(200, {"ok": True})
            if self.path.startswith("/act"):
                return self._send(200, act(req))
            self._send(404, {"error": "not found"})
        except Exception as e:  # report to the client; the client decides whether to stop
            logging.exception("request failed")
            self._send(500, {"error": f"{type(e).__name__}: {e}"[:500]})

    def log_message(self, fmt, *args):  # keep the log to one short line per request
        logging.info("%s %s", self.address_string(), fmt % args)


@parser.wrap()
def main(cfg: EvalPipelineConfig) -> None:
    torch.backends.cudnn.benchmark = True
    torch.backends.cuda.matmul.allow_tf32 = True
    set_seed(cfg.seed)
    policy = make_policy(cfg=cfg.policy, env_cfg=cfg.env, rename_map=cfg.rename_map)
    policy.eval()
    pre, post = make_pre_post_processors(
        policy_cfg=cfg.policy, pretrained_path=cfg.policy.pretrained_path,
        preprocessor_overrides={"device_processor": {"device": str(policy.config.device)},
                                "rename_observations_processor": {"rename_map": cfg.rename_map}})
    env_pre, env_post = make_env_pre_post_processors(env_cfg=cfg.env, policy_cfg=cfg.policy)
    STATE.update(policy=policy, pre=pre, post=post, env_pre=env_pre, env_post=env_post, episodes={},
                 model=str(cfg.policy.pretrained_path), n=policy.config.n_action_steps)
    logging.info("MolmoAct2 server ready on port %d (n_action_steps=%d)", PORT, policy.config.n_action_steps)
    ThreadingHTTPServer(("0.0.0.0", PORT), Handler).serve_forever()


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s")
    main()
