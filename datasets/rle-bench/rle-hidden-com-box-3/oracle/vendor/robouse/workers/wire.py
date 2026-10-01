"""Worker side of the simulator-worker protocol. Standard library and numpy only: this file runs inside simulator
virtualenvs and on remote GPU machines, where the robouse package is not installed.

Local workers (a simulator in its own virtualenv) speak JSON lines over stdin/stdout. The client starts

    <venv python> wire.py <worker script>

and this host loads the script by path (never putting its folder on sys.path, so a script named after its simulator
cannot shadow the simulator's package), creates its `Worker` class and serves it:

    request   {"cmd": NAME, ...keyword arguments}                       one JSON line
    response  {"ok": true, "result": VALUE}                             one JSON line
              {"ok": true, "array": {"shape": [...], "dtype": "uint8"}} one JSON line, then the raw array bytes
              {"ok": false, "error": "Type: message", "trace": "..."}   one JSON line

`cmd` is a public method of the Worker (never one starting with "_"); `close` calls `Worker.close()` if there is one
and ends the process. Anything the simulator prints goes to stderr: the protocol owns the real stdout.

Remote workers (a GPU machine) serve the same kind of calls over HTTP with `serve_http`: POST /rpc with
{"method": NAME, "args": {...}, ...} and the shared secret in the X-Robouse-Secret header; the reply is
{"ok": true, "result": VALUE} or {"ok": false, "error": ...}. Images in results are PNG base64 strings (`png_b64`).
"""

from __future__ import annotations

import base64
import hmac
import importlib.util
import io
import json
import os
import sys
import traceback
from collections.abc import Callable
from typing import Any

PROTOCOL = 1
SECRET_HEADER = "X-Robouse-Secret"


def jsonable(x: Any) -> Any:
    """json.dumps default: numpy scalars and arrays become plain numbers and lists."""
    if hasattr(x, "tolist"):
        return x.tolist()
    if hasattr(x, "item"):
        return x.item()
    raise TypeError(f"{type(x).__name__} is not JSON serializable")


def _error(e: BaseException) -> dict:
    return {"ok": False, "error": f"{type(e).__name__}: {e}"[:1000], "trace": traceback.format_exc()[-2000:]}


# ---- local workers: JSON lines over stdin/stdout ------------------------------------------------------------------


def serve_stdio(make_worker: Callable[[], Any]) -> None:
    """Serve the public methods of `make_worker()` on stdin/stdout until `close` or end of input. The protocol keeps the
    real stdout; from before the worker is made, everything else written to fd 1 (C libraries included) goes to
    stderr."""
    out = os.fdopen(os.dup(1), "wb")
    os.dup2(2, 1)
    sys.stdout = sys.stderr
    worker = make_worker()
    for line in sys.stdin.buffer:
        if not line.strip():
            continue
        try:
            req = json.loads(line)
            cmd = req.pop("cmd")
            if cmd == "close":
                close = getattr(worker, "close", None)
                if callable(close):
                    close()
                break
            fn = getattr(worker, cmd, None) if isinstance(cmd, str) and not cmd.startswith("_") else None
            if not callable(fn):
                raise ValueError(f"unknown command {cmd!r}")
            res = fn(**req)
            if hasattr(res, "tobytes") and hasattr(res, "shape"):
                data = res.tobytes()
                head = {"ok": True, "array": {"shape": list(res.shape), "dtype": str(res.dtype)}}
                out.write((json.dumps(head) + "\n").encode())
                out.write(data)
            else:
                out.write((json.dumps({"ok": True, "result": res}, default=jsonable) + "\n").encode())
        except Exception as e:  # noqa: BLE001 - reported to the client, the worker keeps serving
            out.write((json.dumps(_error(e)) + "\n").encode())
        out.flush()
    out.flush()


def load_worker(path: str) -> Any:
    """The `Worker` class of the script at `path`, loaded without adding its folder to sys.path."""
    spec = importlib.util.spec_from_file_location("robouse_sim_worker", path)
    if spec is None or spec.loader is None:
        raise ImportError(f"cannot load worker script {path}")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod.Worker


# ---- remote workers: HTTP -----------------------------------------------------------------------------------------


def png_b64(img: Any) -> str:
    """An RGB uint8 image as a base64 PNG string (for HTTP results)."""
    import numpy as np
    from PIL import Image

    buf = io.BytesIO()
    Image.fromarray(np.asarray(img, dtype=np.uint8)).save(buf, "PNG")
    return base64.b64encode(buf.getvalue()).decode()


def serve_http(
    handle: Callable[[dict], Any],
    port: int,
    secret: str,
    name: str = "robouse worker",
    host: str = "0.0.0.0",
    threaded: bool = True,
    stop: Callable[[], bool] | None = None,
) -> None:
    """Serve POST /rpc: `handle(request)` returns the result (wrapped as {"ok": true, "result": ...}), or a dict that
    already has an "ok" key. Every request must carry `secret` in the X-Robouse-Secret header (compared in constant
    time); the server refuses to start without a secret. GET / answers a health check.

    With `stop`, requests are served one at a time until `stop()` is true (checked after every request and once a
    second); the connection that made it true is closed after its reply. Without it, the server runs forever."""
    from http.server import BaseHTTPRequestHandler, HTTPServer, ThreadingHTTPServer

    if not secret:
        raise SystemExit(f"{name}: refusing to start without a shared secret")

    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *a: Any) -> None:
            pass

        def _send(self, code: int, obj: dict) -> None:
            body = json.dumps(obj, default=jsonable).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self) -> None:
            self._send(200, {"ok": True, "result": name, "protocol": PROTOCOL})

        def do_POST(self) -> None:
            raw = self.rfile.read(int(self.headers.get("Content-Length") or 0))
            if not hmac.compare_digest(self.headers.get(SECRET_HEADER, ""), secret):
                self._send(403, {"ok": False, "error": "forbidden"})
                return
            try:
                res = handle(json.loads(raw or b"{}"))
                resp = res if isinstance(res, dict) and "ok" in res else {"ok": True, "result": res}
            except Exception as e:  # noqa: BLE001 - reported to the client, the worker keeps serving
                traceback.print_exc()
                resp = {"ok": False, "error": f"{type(e).__name__}: {e}"[:1000]}
            self._send(200, resp)
            if stop is not None and stop():
                self.close_connection = True

    if stop is not None:
        one = HTTPServer((host, port), Handler)
        one.timeout = 1.0
        print(f"{name} on :{port}", flush=True)
        while not stop():
            one.handle_request()
        one.server_close()
        return
    srv: HTTPServer
    if threaded:
        srv = ThreadingHTTPServer((host, port), Handler)
        srv.daemon_threads = True
    else:
        srv = HTTPServer((host, port), Handler)
    print(f"{name} on :{port}", flush=True)
    srv.serve_forever()


if __name__ == "__main__":
    here = os.path.dirname(os.path.abspath(__file__))
    sys.path[:] = [p for p in sys.path if os.path.abspath(p or ".") != here]
    if len(sys.argv) != 2:
        raise SystemExit("usage: python wire.py WORKER_SCRIPT")
    serve_stdio(lambda: load_worker(sys.argv[1])())
