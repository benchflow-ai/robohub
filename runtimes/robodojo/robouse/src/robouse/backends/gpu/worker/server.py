"""GPU simulator worker for Robo Use's GPU track. Runs on an NVIDIA RTX GPU machine, never on the Robo Use host.

One worker process per simulator family (ManiSkill3, Genesis, MuJoCo Playground, Isaac Lab) listens on one port.
Every episode gets its own child process (`open`), because GPU physics engines hold process-wide state (PhysX GPU,
Taichi/Genesis, JAX, Isaac Sim), so two episodes can never share one. The Robo Use `gpu` backend (backends/gpu/backend.py)
talks to it over HTTPS (Daytona preview URLs) with a shared secret:

  POST /rpc  {"method": "...", "session": "...", "args": {...}}  ->  {"ok": true, "result": ...} | {"ok": false, "error": "..."}

Methods: open (args: task, seed) -> {"session", "info"}; close; and, per session, info, reset, step, observe, render,
success, skill, camera_info. Images travel as base64 JPEG. Requests must carry the shared secret from
$ROBOUSE_GPU_SECRET in the X-Robouse-Secret header (robouse.workers.wire.serve_http). A session with no request for IDLE_S seconds is killed.

Usage (on the GPU machine):  ROBOUSE_GPU_SECRET=... python server.py --sim maniskill --port 8800 [--host 127.0.0.1]

--host is the address to listen on: 127.0.0.1 (the default) when the episode server runs on the same machine,
0.0.0.0 (or one interface's address) when it runs elsewhere or in a container.

A simulator family is a module with `make(task, seed, opts)`, which returns an object with the methods above plus
`info()`, and optionally `prepare()`, which the worker calls once at start-up, before any episode. The built-in
families are listed in SIMS; `--module` serves another one without changing this file:
  python server.py --sim mysim --module my_pkg.mysim_worker --port 8805      (an importable module)
  python server.py --sim mysim --module /path/to/mysim_worker.py --port 8805 (a file; its folder goes on sys.path)
The module may import `common` (helpers next to this file); task folders name it with `sim: mysim`.
"""

from __future__ import annotations

import argparse
import importlib
import multiprocessing as mp
import os
import secrets
import sys
import threading
import time
import traceback

try:
    from wire import serve_http  # robouse/workers/wire.py, uploaded next to this file (tools/gpu/up.py)
except ImportError:
    from robouse.workers.wire import serve_http

IDLE_S = float(os.environ.get("ROBOUSE_GPU_IDLE_S", "2400"))
MAX_SESSIONS = int(os.environ.get("ROBOUSE_GPU_MAX_SESSIONS", "10"))
SIMS = {
    "maniskill": "maniskill_sim",
    "genesis": "genesis_sim",
    "playground": "playground_sim",
    "isaaclab": "isaaclab_sim",
}


def load_family(module: str):
    """The family module: a name in SIMS, an importable module name, or a path to a .py file."""
    here = os.path.dirname(os.path.abspath(__file__))
    if here not in sys.path:
        sys.path.insert(0, here)
    module = SIMS.get(module, module)
    if module.endswith(".py"):
        path = os.path.abspath(module)
        if not os.path.isfile(path):
            raise FileNotFoundError(f"no simulator family module at {path}")
        sys.path.insert(0, os.path.dirname(path))
        module = os.path.splitext(os.path.basename(path))[0]
    mod = importlib.import_module(module)
    if not callable(getattr(mod, "make", None)):
        raise AttributeError(f"simulator family module {module!r} has no make(task, seed, opts)")
    return mod


def _child(module: str, task: str, seed: int, opts: dict, conn) -> None:
    """Session process: build the task once, then answer requests from the pipe until `close`."""
    try:
        mod = load_family(module)
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
            conn.send(
                {"ok": False, "error": f"{type(e).__name__}: {e}"[:2000], "trace": traceback.format_exc()[-3000:]}
            )


class Session:
    def __init__(self, module: str, task: str, seed: int, opts: dict):
        ctx = mp.get_context("spawn")
        self.conn, child = ctx.Pipe()
        self.proc = ctx.Process(target=_child, args=(module, task, seed, opts, child), daemon=True)
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
    module = ""
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
            s = Session(State.module, str(a["task"]), int(a.get("seed", 0)), dict(a.get("opts") or {}))
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


def main(argv: list[str] | None = None) -> None:
    ap = argparse.ArgumentParser(description="Robo Use GPU simulator worker (one simulator family per process)")
    ap.add_argument(
        "--sim", required=True, help=f"the family's name in task folders (`sim:`); built in: {', '.join(sorted(SIMS))}"
    )
    ap.add_argument(
        "--module", default="", help="the family module, for a family that is not built in: a module name or a .py file"
    )
    ap.add_argument("--port", type=int, default=8800)
    ap.add_argument(
        "--host", default="127.0.0.1", help="address to listen on (default 127.0.0.1; 0.0.0.0 for every interface)"
    )
    a = ap.parse_args(argv)
    if not a.module and a.sim not in SIMS:
        ap.error(f"unknown --sim {a.sim!r}: built in are {', '.join(sorted(SIMS))}; give --module for another family")
    secret = os.environ.get("ROBOUSE_GPU_SECRET", "")
    if not secret:
        raise SystemExit("robouse gpu worker: set ROBOUSE_GPU_SECRET (the shared secret in the endpoints file)")
    State.sim, State.module = a.sim, a.module or a.sim
    mod = load_family(State.module)  # fails here, not at the first episode, when the module cannot be imported
    if callable(prepare := getattr(mod, "prepare", None)):
        prepare()
    threading.Thread(target=_reaper, daemon=True).start()
    serve_http(handle, a.port, secret, name=f"robouse gpu worker: {a.sim}", host=a.host)


if __name__ == "__main__":
    main()
