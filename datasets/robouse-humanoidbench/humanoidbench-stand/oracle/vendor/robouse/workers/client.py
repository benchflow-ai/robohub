"""Client side of the simulator-worker protocol (see `wire.py`): a simulator in its own virtualenv (`StdioWorker`) or
on a remote GPU machine (`HttpWorker`), called the same way: `worker.call(name, **kwargs)`."""

from __future__ import annotations

import http.client
import json
import os
import select
import subprocess
import time
import urllib.parse
from pathlib import Path
from typing import Any

import numpy as np

from .. import config
from .wire import SECRET_HEADER

WIRE = Path(__file__).with_name("wire.py")


class WorkerError(RuntimeError):
    """A worker call failed: the worker reported an error, died, or did not answer in time."""


class WorkerUnavailable(WorkerError):
    """A remote worker could not be reached or answered with an HTTP error (it may be starting or restarting)."""


class StdioWorker:
    """A simulator worker process in its own virtualenv, speaking JSON lines on stdin/stdout.

    `name` is the backend (used for messages and for $ROBOUSE_<NAME>_QUIET); `python` is the virtualenv's interpreter
    and `script` the worker script (a module with a `Worker` class). `setup` says where the setup is documented."""

    def __init__(
        self,
        name: str,
        python: Path,
        script: Path,
        *,
        env: dict[str, str] | None = None,
        cwd: Path | None = None,
        setup: str = "",
        timeout: float = 600,
    ):
        self.name, self.timeout = name, timeout
        if not Path(python).exists():
            raise WorkerError(
                f"{name}: simulator virtualenv not found at {python}"
                + (f"; see {setup}" if setup else "")
                + f" (or set ROBOUSE_{name.upper()}_PYTHON)"
            )
        var = f"ROBOUSE_{name.upper()}_QUIET"
        quiet = config.flag(var, default=True) if var in config.ENV else True
        self.proc = subprocess.Popen(
            [str(python), str(WIRE), str(script)],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL if quiet else None,
            cwd=str(cwd or Path(script).parent),
            env={**os.environ, "OPENBLAS_NUM_THREADS": "1", "OMP_NUM_THREADS": "1", **(env or {})},
        )
        self._buf = b""
        self._hint = f"; run with ROBOUSE_{name.upper()}_QUIET=0 to see its output" if quiet else ""

    @property
    def alive(self) -> bool:
        return self.proc.poll() is None

    def _fill(self) -> None:
        """Read what the worker has written so far (waiting at most `timeout`). Reads the pipe's file descriptor
        directly, so select() sees every byte that has not been consumed."""
        assert self.proc.stdout is not None
        fd = self.proc.stdout.fileno()
        ready, _, _ = select.select([fd], [], [], self.timeout)
        if not ready:
            self.proc.kill()
            raise WorkerError(f"{self.name} worker did not answer within {self.timeout:.0f} s")
        chunk = os.read(fd, 1 << 20)
        if not chunk:
            raise EOFError("no reply")
        self._buf += chunk

    def _readline(self) -> bytes:
        while b"\n" not in self._buf:
            self._fill()
        line, _, self._buf = self._buf.partition(b"\n")
        return line

    def _readexact(self, n: int) -> bytes:
        while len(self._buf) < n:
            self._fill()
        data, self._buf = self._buf[:n], self._buf[n:]
        return data

    def call(self, cmd: str, **kw: Any) -> Any:
        """Run one command; returns its result (a numpy array for array results)."""
        if not self.alive:
            raise WorkerError(f"{self.name} worker exited (code {self.proc.returncode}){self._hint}")
        assert self.proc.stdin is not None
        try:
            self.proc.stdin.write((json.dumps({"cmd": cmd, **kw}) + "\n").encode())
            self.proc.stdin.flush()
            head = json.loads(self._readline())
            if head.get("ok") and "array" in head:
                spec = head["array"]
                dtype = np.dtype(spec["dtype"])
                n = int(np.prod(spec["shape"])) * dtype.itemsize
                return np.frombuffer(self._readexact(n), dtype=dtype).reshape(spec["shape"])
        except (BrokenPipeError, EOFError, OSError, ValueError) as e:
            raise WorkerError(f"{self.name} worker died ({type(e).__name__}: {e}){self._hint}") from None
        if not head.get("ok"):
            raise WorkerError(f"{self.name} worker: {head.get('error')}")
        return head.get("result")

    def close(self) -> None:
        if not self.alive:
            return
        try:
            assert self.proc.stdin is not None
            self.proc.stdin.write(b'{"cmd": "close"}\n')
            self.proc.stdin.flush()
            self.proc.wait(timeout=10)
        except (OSError, subprocess.TimeoutExpired):
            self.proc.kill()


def load_endpoints(name: str, setup: str) -> dict:
    """The endpoints file of a remote worker pool (`config.remote_endpoints`), parsed:
    {"secret": "...", "workers": {KEY: URL | {"url": URL, "headers": {...}}}, "headers": {...},
     "worker_headers": {KEY: {...}}}."""
    p = config.remote_endpoints(name)
    if p is None:
        raise WorkerError(
            f"{name} worker endpoints are not configured: set ROBOUSE_{name.upper()}_REMOTE or write "
            f"{config.config_dir() / 'remote' / (name + '.json')}; see {setup}"
        )
    return json.loads(p.read_text())


def remote_worker(
    endpoints: dict, key: str, kind: str, timeout: float, retry: frozenset[str] = frozenset()
) -> HttpWorker:
    """The HttpWorker for `key` in an endpoints file: its URL plus the file's shared headers, then the worker's own."""
    w = endpoints.get("workers", {}).get(key)
    if not w:
        raise WorkerError(f"no {kind} worker for {key!r} in the endpoints file")
    url, own = (w, {}) if isinstance(w, str) else (w["url"], w.get("headers", {}))
    headers = {**endpoints.get("headers", {}), **endpoints.get("worker_headers", {}).get(key, {}), **own}
    return HttpWorker(url, endpoints.get("secret", ""), headers, timeout=timeout, retry=retry)


class HttpWorker:
    """A remote worker behind HTTP(S), on one kept-alive connection (a TLS handshake per step would dominate the step
    time). Calls in `retry` are idempotent and retried twice on a transport error; `session`, when set, is sent with
    every call."""

    def __init__(
        self,
        url: str,
        secret: str,
        headers: dict[str, str] | None = None,
        timeout: float = 300,
        retry: frozenset[str] = frozenset(),
    ):
        u = urllib.parse.urlparse(url)
        if u.scheme not in ("http", "https") or not u.hostname:
            raise WorkerError(f"bad worker URL {url!r}")
        self.https, self.host, self.port = u.scheme == "https", u.hostname, u.port
        self.path = (u.path.rstrip("/") or "") + "/rpc"
        self.headers = {"Content-Type": "application/json", SECRET_HEADER: secret, **(headers or {})}
        self.timeout, self.retry = timeout, retry
        self.session: str | None = None
        self._conn: http.client.HTTPConnection | None = None

    def _post(self, body: bytes, timeout: float) -> dict:
        if self._conn is None:
            cls = http.client.HTTPSConnection if self.https else http.client.HTTPConnection
            self._conn = cls(self.host, self.port, timeout=timeout)
        self._conn.timeout = timeout
        if self._conn.sock is not None:
            self._conn.sock.settimeout(timeout)
        try:
            self._conn.request("POST", self.path, body=body, headers=self.headers)
            r = self._conn.getresponse()
            data = r.read()
        except (OSError, http.client.HTTPException):
            self._conn.close()
            self._conn = None
            raise
        if r.status != 200:
            raise WorkerUnavailable(f"worker HTTP {r.status}: {data[:200]!r}")
        return json.loads(data)

    def call(self, method: str, _timeout: float | None = None, **args: Any) -> Any:
        req: dict[str, Any] = {"method": method, "args": args}
        if self.session is not None:
            req["session"] = self.session
        body = json.dumps(req).encode()
        tries = 3 if method in self.retry else 1
        for i in range(tries):
            try:
                resp = self._post(body, _timeout or self.timeout)
                break
            except (OSError, http.client.HTTPException) as e:
                if i == tries - 1:
                    raise WorkerUnavailable(f"worker unreachable ({type(e).__name__}: {e})") from None
                time.sleep(1 + i)
        if not resp.get("ok"):
            raise WorkerError(resp.get("error", "worker error"))
        return resp.get("result")

    def close(self) -> None:
        if self._conn is not None:
            self._conn.close()
            self._conn = None


def decode_png_b64(b64: str) -> np.ndarray:
    """A base64 PNG from an HTTP worker result as an RGB uint8 array."""
    import base64
    import io

    from PIL import Image

    return np.asarray(Image.open(io.BytesIO(base64.b64decode(b64))).convert("RGB"))
