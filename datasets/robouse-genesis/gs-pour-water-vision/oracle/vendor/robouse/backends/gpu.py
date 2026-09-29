"""GPU track: simulators that need an NVIDIA GPU (ManiSkill3, Genesis, MuJoCo Playground, Isaac Lab), run remotely.

The simulator runs in a worker on a GPU machine (backends/gpu_worker/server.py, one worker per simulator family, one
child process per episode). This backend is a thin HTTPS client for it; the episode server, budgets, trace, video,
verifier and harnesses all run locally as for any other backend. The agent never sees the worker's address: it is
read from a file under ~/benchflow (denied to model harnesses by the macOS sandbox profile), not from the
environment, and the worker rejects requests without the shared secret.

Worker endpoints come from $ROBOUSE_GPU_REMOTE (default: <repo>/runs/_gpu/remote.json):
  {"secret": "...", "workers": {"maniskill": "https://...", ...}, "worker_headers": {"maniskill": {...}, ...}}

Task spec (the `robouse:` block): backend: gpu, sim: <family>, env: <task key in that family's module>, seed, and
optional `opts` passed to the task's constructor on the worker. The worker's `info` supplies the action space, the
skills, the cameras and the embodiment (BenchFlow embodied spec, docs/embodied.md).
"""
from __future__ import annotations

import base64
import http.client
import io
import json
import os
import time
import urllib.parse
from pathlib import Path

import numpy as np

from .base import ActionSpec, Backend, StepInfo

REPO = Path(__file__).resolve().parents[3]
REMOTE_FILE = REPO / "runs" / "_gpu" / "remote.json"
RETRY_METHODS = {"info", "observe", "render", "success", "camera_info", "goal_status"}


def _remote() -> dict:
    p = Path(os.environ.get("ROBOUSE_GPU_REMOTE", str(REMOTE_FILE)))
    if not p.exists():
        raise RuntimeError(f"GPU worker endpoints not configured ({p}); see docs/suites/gpu-track.md")
    return json.loads(p.read_text())


def _decode(b64: str) -> np.ndarray:
    from PIL import Image

    return np.asarray(Image.open(io.BytesIO(base64.b64decode(b64))).convert("RGB"))


class _Conn:
    """One kept-alive HTTPS connection to a worker (a fresh TLS handshake per step would dominate the step time)."""

    def __init__(self, url: str, headers: dict, timeout: float):
        u = urllib.parse.urlparse(url)
        self.https = u.scheme == "https"
        self.host, self.port = u.hostname, u.port
        self.path = (u.path.rstrip("/") or "") + "/rpc"
        self.headers = headers
        self.timeout = timeout
        self.c = None

    def post(self, body: bytes, timeout: float | None = None) -> dict:
        if self.c is None:
            cls = http.client.HTTPSConnection if self.https else http.client.HTTPConnection
            self.c = cls(self.host, self.port, timeout=timeout or self.timeout)
        self.c.timeout = timeout or self.timeout
        if self.c.sock is not None:
            self.c.sock.settimeout(self.c.timeout)
        try:
            self.c.request("POST", self.path, body=body, headers=self.headers)
            r = self.c.getresponse()
            data = r.read()
            if r.status != 200:
                raise RuntimeError(f"worker HTTP {r.status}: {data[:200]!r}")
            return json.loads(data)
        except Exception:
            self.c.close()
            self.c = None
            raise


class GpuBackend(Backend):
    name = "gpu"

    def __init__(self, spec: dict):
        self.spec = spec
        self.sim = spec["sim"]
        self.task = spec["env"]
        r = _remote()
        url = r.get("workers", {}).get(self.sim)
        if not url:
            raise RuntimeError(f"no GPU worker for simulator {self.sim!r}")
        extra = r.get("worker_headers", {}).get(self.sim, r.get("headers", {}))  # e.g. the proxy's preview token
        headers = {"Content-Type": "application/json", "X-Robouse-Secret": r.get("secret", ""), **extra}
        self.timeout = float(spec.get("rpc_timeout_s", 600))
        self.conn = _Conn(url, headers, self.timeout)
        self.session = ""
        opened = self._call("open", task=self.task, seed=int(spec.get("seed", 0)), opts=spec.get("opts") or {},
                            load_timeout_s=float(spec.get("ready_timeout_s", 900)), _timeout=float(spec.get("ready_timeout_s", 900)) + 60)
        self.session = opened["session"]
        self.remote_info = opened["info"]
        a = self.remote_info["action"]
        self.action_spec = ActionSpec(list(a["names"]), list(a["low"]), list(a["high"]), a.get("doc", ""))
        self.max_steps = int(spec.get("max_steps", self.remote_info.get("max_steps", 200)))
        self.camera = self.remote_info.get("camera", "")
        self._grip = float(self.remote_info.get("hold_last", 0.0))
        self._skills = list(self.remote_info.get("skills", []))
        self._pending: list[np.ndarray] = []
        self._last_frame: np.ndarray | None = None
        self.last_step: dict = {}

    def _call(self, method: str, _timeout: float | None = None, **args):
        body = json.dumps({"method": method, "session": self.session, "args": args}).encode()
        tries = 3 if method in RETRY_METHODS else 1
        for i in range(tries):
            try:
                resp = self.conn.post(body, _timeout)
                break
            except Exception:
                if i == tries - 1:
                    raise
                time.sleep(1 + i)
        if not resp.get("ok"):
            raise RuntimeError(resp.get("error", "worker error"))
        return resp["result"]

    def _take_frames(self, r: dict) -> None:
        fr = r.pop("frames", None) or []
        self._pending.extend(_decode(f) for f in fr)
        if r.get("frame"):
            self._last_frame = _decode(r.pop("frame"))
        else:
            self._last_frame = None
        if "hold_last" in r:
            self._grip = float(r.pop("hold_last"))

    # ---- Backend interface -----------------------------------------------------------------------------------------
    def reset(self, seed: int) -> None:
        r = self._call("reset", seed=int(seed))
        self._take_frames(r or {})

    def step(self, action) -> StepInfo:
        r = self._call("step", action=[float(x) for x in np.asarray(action, dtype=float).ravel()])
        self._take_frames(r)
        self.last_step = r
        return StepInfo(success=bool(r.get("success")), reward=float(r.get("reward", 0.0)), extra=r.get("info") or {})

    def run_skill(self, name: str, args: list[str]) -> dict:
        r = self._call("skill", name=name, args=args)
        self._take_frames(r)
        return r

    def pop_frames(self) -> list[np.ndarray]:
        out, self._pending = self._pending, []
        return out

    def observe(self) -> dict:
        return self._call("observe")

    def observe_privileged(self) -> dict:
        """Full state plus `_oracle`, the reference policy's next action (only for requests with the oracle token)."""
        return self._call("observe", privileged=True)

    def render(self, width: int = 640, height: int = 480) -> np.ndarray:
        if self.camera == self.remote_info.get("camera") and self._last_frame is not None:
            return self._last_frame
        img = _decode(self._call("render", camera=self.camera)["jpeg"])
        if self.camera == self.remote_info.get("camera"):
            self._last_frame = img
        return img

    def camera_info(self, cameras: list[str]) -> list[dict]:
        return self._call("camera_info", cameras=list(cameras))

    def success(self) -> bool:
        return bool(self._call("success")["success"])

    def skills(self) -> list[str]:
        return self._skills

    def embodiment(self) -> dict:
        """The robot as a BenchFlow embodiment spec (docs/embodied.md), as the worker declares it."""
        return self.remote_info.get("embodiment", {})

    def close(self) -> None:
        if self.session:
            try:
                self._call("close", _timeout=60)
            except Exception:
                pass
            self.session = ""


def oracle_main(env: str) -> None:
    """Reference solution. Each task's scripted or planner-based policy runs next to the simulator, on the privileged
    state; the oracle's observations (oracle token) carry its next action as `_oracle.next`, and this loop sends that
    action through the episode socket exactly like an agent's `robo act` / `robo skill` call."""
    from ..agent_cli import _send

    for _ in range(2000):
        r = _send({"op": "observe"})
        if not r.get("ok"):
            print("observe failed:", r.get("error"), flush=True)
            return
        nxt = (r["result"]["state"].get("_oracle") or {}).get("next") or ["done"]
        if nxt[0] == "done":
            break
        if nxt[0] == "act":
            req = {"op": "act", "action": [float(x) for x in nxt[1]], "repeat": int(nxt[2]) if len(nxt) > 2 else 1}
        elif nxt[0] == "skill":
            req = {"op": "skill", "name": nxt[1], "args": [str(x) for x in (nxt[2] if len(nxt) > 2 else [])]}
        else:
            raise ValueError(f"bad oracle action {nxt!r}")
        resp = _send(req)
        res = resp.get("result") or {}
        print(nxt[0], nxt[1] if nxt[0] == "skill" else "", resp.get("ok"), res.get("message", "") or resp.get("error", ""), flush=True)
        if not resp.get("ok") or "episode" in res:
            return
    _send({"op": "done", "text": "oracle plan finished"})
