"""GPU simulator worker for Robo Use's GPU track. Runs on an NVIDIA RTX GPU machine, never on the Robo Use host.

One worker process per simulator family (ManiSkill3, Genesis, MuJoCo Playground, Isaac Lab) listens on one port.
Every episode gets its own child process (`open`), because GPU physics engines hold process-wide state (PhysX GPU,
Taichi/Genesis, JAX, Isaac Sim), so two episodes can never share one. The Robo Use `gpu` backend (backends/gpu.py)
talks to it over HTTPS (Daytona preview URLs) with a shared secret:

  POST /rpc  {"method": "...", "session": "...", "args": {...}}  ->  {"ok": true, "result": ...} | {"ok": false, "error": "..."}

Methods: open (args: task, seed) -> {"session", "info"}; close; and, per session, info, reset, step, observe, render,
success, skill, camera_info. Images travel as base64 JPEG. Requests must carry the shared secret from
$ROBOUSE_GPU_SECRET in the X-Robouse-Secret header. A session with no request for IDLE_S seconds is killed.

Usage (on the GPU machine):  ROBOUSE_GPU_SECRET=... python server.py --sim maniskill --port 8800
"""
from __future__ import annotations

import argparse
import hmac
import importlib
import json
import multiprocessing as mp
import os
import secrets
import sys
import threading
import time
import traceback
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

IDLE_S = float(os.environ.get("ROBOUSE_GPU_IDLE_S", "2400"))
MAX_SESSIONS = int(os.environ.get("ROBOUSE_GPU_MAX_SESSIONS", "10"))
SIMS = {"maniskill": "maniskill_sim", "genesis": "genesis_sim", "playground": "playground_sim", "isaaclab": "isaaclab_sim"}


def _child(sim: str, task: str, seed: int, opts: dict, conn) -> None:
    """Session process: build the task once, then answer requests from the pipe until `close`."""
    try:
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        mod = importlib.import_module(SIMS[sim])
        t = time.time()
        env = mod.make(task, seed, opts)
        conn.send({"ok": True, "result": {"info": env.info(), "load_s": round(time.time() - t, 1)}})
    except Exception as e:
        conn.send({"ok": False, "error": f"{type(e).__name__}: {e}", "trace": traceback.format_exc()[-3000:]})
        return
    while True:
        try:
            req = conn.recv()
        except EOFError:
            return
        m, a = req.get("method"), req.get("args") or {}
        if m == "close":
            try:
                env.close()
            finally:
                conn.send({"ok": True, "result": {}})
            return
        try:
            fn = getattr(env, m, None)
            if m.startswith("_") or not callable(fn):
                raise AttributeError(f"unknown method {m!r}")
            conn.send({"ok": True, "result": fn(**a)})
        except Exception as e:
            conn.send({"ok": False, "error": f"{type(e).__name__}: {e}"[:2000], "trace": traceback.format_exc()[-3000:]})


class Session:
    def __init__(self, sim: str, task: str, seed: int, opts: dict):
        ctx = mp.get_context("spawn")
        self.conn, child = ctx.Pipe()
        self.proc = ctx.Process(target=_child, args=(sim, task, seed, opts, child), daemon=True)
        self.proc.start()
        self.lock = threading.Lock()
        self.last = time.time()
        self.task = task

    def call(self, msg: dict, timeout: float = 900) -> dict:
        with self.lock:
            self.last = time.time()
            if msg is not None:
                self.conn.send(msg)
            if not self.conn.poll(timeout):
                self.kill()
                return {"ok": False, "error": "simulator session timed out"}
            try:
                r = self.conn.recv()
            except EOFError:
                return {"ok": False, "error": "simulator session died"}
            self.last = time.time()
            return r

    def kill(self) -> None:
        if self.proc.is_alive():
            self.proc.kill()
        self.proc.join(5)


class State:
    sim = ""
    secret = ""
    sessions: dict[str, Session] = {}
    lock = threading.Lock()


def _reaper() -> None:
    while True:
        time.sleep(30)
        with State.lock:
            for sid, s in list(State.sessions.items()):
                if (time.time() - s.last > IDLE_S and not s.lock.locked()) or not s.proc.is_alive():
                    s.kill()
                    State.sessions.pop(sid, None)
                    print("reaped", sid, s.task, flush=True)


def handle(req: dict) -> dict:
    m = req.get("method")
    a = req.get("args") or {}
    if m == "ping":
        return {"ok": True, "result": {"sim": State.sim, "sessions": len(State.sessions)}}
    if m == "open":
        with State.lock:
            if len(State.sessions) >= MAX_SESSIONS:
                return {"ok": False, "error": f"worker busy ({len(State.sessions)} sessions)"}
            sid = secrets.token_hex(8)
            s = Session(State.sim, str(a["task"]), int(a.get("seed", 0)), dict(a.get("opts") or {}))
            State.sessions[sid] = s
        r = s.call(None, timeout=float(a.get("load_timeout_s", 900)))
        if not r.get("ok"):
            s.kill()
            State.sessions.pop(sid, None)
            print("open failed", a, r.get("trace", ""), flush=True)
            return r
        print("opened", sid, a.get("task"), r["result"].get("load_s"), "s", flush=True)
        return {"ok": True, "result": {"session": sid, **r["result"]}}
    sid = req.get("session")
    s = State.sessions.get(sid or "")
    if s is None:
        return {"ok": False, "error": "no such session (closed or reaped)"}
    if m == "close":
        r = s.call({"method": "close"}, timeout=60)
        s.kill()
        with State.lock:
            State.sessions.pop(sid, None)
        return {"ok": True, "result": {}}
    r = s.call({"method": m, "args": a}, timeout=float(a.pop("timeout_s", 900)) if isinstance(a, dict) else 900)
    if not r.get("ok") and r.get("trace"):
        print("error in", m, r["trace"], flush=True)
    r.pop("trace", None)
    return r


def _jsonable(o):
    """numpy scalars and arrays (and anything with .tolist()) in simulator results."""
    if hasattr(o, "tolist"):
        return o.tolist()
    if hasattr(o, "item"):
        return o.item()
    raise TypeError(f"not JSON serializable: {type(o).__name__}")


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, *a):
        pass

    def _send(self, code: int, obj: dict) -> None:
        body = json.dumps(obj, default=_jsonable).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        self._send(200, {"ok": True, "result": "robouse gpu worker"})

    def do_POST(self):
        n = int(self.headers.get("Content-Length") or 0)
        raw = self.rfile.read(n)
        if not hmac.compare_digest(self.headers.get("X-Robouse-Secret", ""), State.secret):
            return self._send(403, {"ok": False, "error": "forbidden"})
        try:
            req = json.loads(raw or b"{}")
            resp = handle(req)
        except Exception as e:
            resp = {"ok": False, "error": f"{type(e).__name__}: {e}"[:1000]}
        self._send(200, resp)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--sim", required=True, choices=sorted(SIMS))
    ap.add_argument("--port", type=int, default=8800)
    a = ap.parse_args()
    State.sim = a.sim
    State.secret = os.environ["ROBOUSE_GPU_SECRET"]
    threading.Thread(target=_reaper, daemon=True).start()
    srv = ThreadingHTTPServer(("0.0.0.0", a.port), Handler)
    srv.daemon_threads = True
    print(f"robouse gpu worker: {a.sim} on :{a.port}", flush=True)
    srv.serve_forever()


if __name__ == "__main__":
    main()
