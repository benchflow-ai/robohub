"""`robo mcp`: the `robo` operations as MCP tools. Standard library only.

A thin client of the episode server, on the agent's side of the trust boundary: every tool call becomes the same
episode request the `robo` CLI would send (`agent_cli.build_request`), over the same socket, with no token. It has no
more authority than the CLI: the server alone keeps the budgets, counts steps, records and judges. The runner's own
operations (status, shutdown) have no tool.

Transports
  robo mcp                                   stdio (one JSON-RPC message per line), for any MCP client
  robo mcp --http [HOST:]PORT [--path P]     streamable HTTP on 127.0.0.1 (JSON responses, no SSE stream);
           [--run CMD ...]                   with --run: serve while CMD runs, then exit with its return code

Tools: info, observe, act, move_to, grip, skill, done, give_up. A result's text is exactly what `robo` prints. When
the reply names camera images (`image_path`), the PNGs are attached as image content, so a model sees images through
MCP exactly when the CLI would have given it a path. The raw reply is in the result's `_meta["robouse/reply"]`.
Requests carry `"via": "mcp"`, which the episode server ignores and records in its trace.
"""

from __future__ import annotations

import atexit
import base64
import json
import os
import subprocess
import sys
import threading
from typing import Any

PROTOCOL_VERSIONS = ("2025-11-25", "2025-06-18", "2025-03-26", "2024-11-05")
SERVER_INFO = {"name": "robo", "title": "Robo Use episode", "version": "1"}
INSTRUCTIONS = (
    "These tools control the robot of one Robo Use episode. Start with `info` (action space, skills, step budget) and "
    "`observe`. Motion tools (act, move_to, grip, skill) use the step budget. Call `done` once when the task is "
    "complete, or `give_up`. Both end the episode; the server decides success from the simulator state."
)
_NUM = {"type": "number"}
_INT = {"type": "integer", "minimum": 1}


def _schema(props: dict, required: tuple[str, ...] = ()) -> dict:
    return {"type": "object", "properties": props, "required": list(required), "additionalProperties": False}


TOOLS: list[dict] = [
    {
        "name": "info",
        "title": "Episode info",
        "description": "The task's action space (names, bounds, meaning), named skills, observation mode and step "
        "budget. Same as `robo info`.",
        "inputSchema": _schema({}),
        "annotations": {"readOnlyHint": True, "idempotentHint": True, "openWorldHint": False},
    },
    {
        "name": "observe",
        "title": "Observe",
        "description": "Robot and object state. image=true also returns a camera image (and its saved path); camera "
        "picks a camera listed by `info`. Same as `robo observe [--image] [--camera C]`. Uses no steps.",
        "inputSchema": _schema(
            {
                "image": {"type": "boolean", "description": "also return a camera image", "default": False},
                "camera": {"type": "string", "description": "camera name (vision tasks list them in info)"},
            }
        ),
        "annotations": {"readOnlyHint": True, "openWorldHint": False},
    },
    {
        "name": "act",
        "title": "Act",
        "description": "Apply one low-level action (one number per action dimension from `info`) `repeat` times "
        "(at most 50). Uses one step per repeat. Same as `robo act A1 A2 ... [--repeat N]`.",
        "inputSchema": _schema(
            {
                "action": {"type": "array", "items": _NUM, "description": "one value per action dimension"},
                "repeat": {**_INT, "default": 1, "description": "apply the action this many times"},
            },
            ("action",),
        ),
        "annotations": {"readOnlyHint": False, "destructiveHint": False, "openWorldHint": False},
    },
    {
        "name": "move_to",
        "title": "Move the hand to a point",
        "description": "Skill: move the hand toward (x, y, z) in the task's frame, holding the gripper at `grip` "
        "(+1 close, -1 open). Uses one step per control step. Only on tasks whose `info` lists move_to. "
        "Same as `robo move-to X Y Z [--grip G] [--max-steps N] [--tol T]`.",
        "inputSchema": _schema(
            {
                "x": _NUM,
                "y": _NUM,
                "z": _NUM,
                "grip": {**_NUM, "description": "gripper command while moving (default: keep the current one)"},
                "max_steps": {**_INT, "default": 100},
                "tol": {"type": "number", "exclusiveMinimum": 0, "default": 0.01},
            },
            ("x", "y", "z"),
        ),
        "annotations": {"readOnlyHint": False, "destructiveHint": False, "openWorldHint": False},
    },
    {
        "name": "grip",
        "title": "Set the gripper",
        "description": "Skill: hold position and set the gripper (+1 close, -1 open) for `steps` steps. Only on tasks "
        "whose `info` lists grip. Same as `robo grip G [--steps N]`.",
        "inputSchema": _schema({"value": _NUM, "steps": {**_INT, "default": 15}}, ("value",)),
        "annotations": {"readOnlyHint": False, "destructiveHint": False, "openWorldHint": False},
    },
    {
        "name": "skill",
        "title": "Run a named skill",
        "description": "Run a named high-level skill listed by `info`, with string arguments. Same as "
        "`robo skill NAME [ARG ...]`.",
        "inputSchema": _schema(
            {"name": {"type": "string"}, "args": {"type": "array", "items": {"type": "string"}, "default": []}},
            ("name",),
        ),
        "annotations": {"readOnlyHint": False, "destructiveHint": False, "openWorldHint": False},
    },
    {
        "name": "done",
        "title": "Finish: task complete",
        "description": "End the episode and ask for scoring. Call once, when the task is complete. The server judges "
        "success from the simulator state. Same as `robo done [message]`.",
        "inputSchema": _schema({"message": {"type": "string", "default": ""}}),
        "annotations": {
            "readOnlyHint": False,
            "destructiveHint": True,
            "idempotentHint": False,
            "openWorldHint": False,
        },
    },
    {
        "name": "give_up",
        "title": "Finish: give up",
        "description": "End the episode without claiming success. Same as `robo give-up [message]`.",
        "inputSchema": _schema({"message": {"type": "string", "default": ""}}),
        "annotations": {
            "readOnlyHint": False,
            "destructiveHint": True,
            "idempotentHint": False,
            "openWorldHint": False,
        },
    },
]
_BY_NAME = {t["name"]: t for t in TOOLS}


class ArgError(ValueError):
    """Tool arguments that do not match the tool's input schema."""


def _check(name: str, args: Any) -> dict:
    """Validate arguments against the tool's schema (the subset of JSON Schema the tools use)."""
    schema = _BY_NAME[name]["inputSchema"]
    if args is None:
        args = {}
    if not isinstance(args, dict):
        raise ArgError("arguments must be an object")
    props = schema["properties"]
    extra = sorted(set(args) - set(props))
    if extra:
        raise ArgError(f"unknown argument(s) {', '.join(extra)}; {name} takes: {', '.join(props) or 'nothing'}")
    missing = [k for k in schema["required"] if k not in args]
    if missing:
        raise ArgError(f"missing argument(s) {', '.join(missing)}")
    for k, v in args.items():
        _check_value(k, v, props[k])
    return args


def _is_num(v: Any) -> bool:
    return isinstance(v, int | float) and not isinstance(v, bool)


def _check_value(key: str, v: Any, s: dict) -> None:
    t = s.get("type")
    ok = {
        "number": _is_num(v),
        "integer": isinstance(v, int) and not isinstance(v, bool),
        "boolean": isinstance(v, bool),
        "string": isinstance(v, str),
        "array": isinstance(v, list),
    }.get(t, True)
    if t == "integer" and isinstance(v, float) and v.is_integer():
        ok = True
    if not ok:
        raise ArgError(f"{key} must be {'an' if t in ('integer', 'array') else 'a'} {t}")
    if t == "array":
        for i, x in enumerate(v):
            _check_value(f"{key}[{i}]", x, s["items"])
    if "minimum" in s and v < s["minimum"]:
        raise ArgError(f"{key} must be at least {s['minimum']}")
    if "exclusiveMinimum" in s and v <= s["exclusiveMinimum"]:
        raise ArgError(f"{key} must be greater than {s['exclusiveMinimum']}")


def _num(v: Any) -> str:
    return repr(float(v))


def tool_argv(name: str, args: dict) -> tuple[str, list[str]]:
    """The `robo` command line equivalent to a tool call: (command, arguments)."""
    a = _check(name, args)
    if name == "info":
        return "info", []
    if name == "observe":
        out = ["--image"] if a.get("image") else []
        return "observe", out + (["--camera", a["camera"]] if a.get("camera") else [])
    if name == "act":
        return "act", [_num(x) for x in a["action"]] + ["--repeat", str(int(a.get("repeat", 1)))]
    if name == "move_to":
        out = [_num(a["x"]), _num(a["y"]), _num(a["z"])]
        if a.get("grip") is not None:
            out += ["--grip", _num(a["grip"])]
        return "move-to", out + ["--max-steps", str(int(a.get("max_steps", 100))), "--tol", _num(a.get("tol", 0.01))]
    if name == "grip":
        return "grip", [_num(a["value"]), "--steps", str(int(a.get("steps", 15)))]
    if name == "skill":
        return "skill", [a["name"], *a.get("args", [])]
    if name in ("done", "give_up"):
        return name.replace("_", "-"), [a["message"]] if a.get("message") else []
    raise KeyError(name)


def _cli():
    """The `robo` client module: robo_cli.py next to this file in a workspace, else the package's agent_cli."""
    here = os.path.dirname(os.path.abspath(__file__))
    if os.path.isfile(os.path.join(here, "robo_cli.py")):
        sys.path.insert(0, here)
        import robo_cli  # type: ignore[import-not-found]

        return robo_cli
    from robouse import agent_cli

    return agent_cli


def call_tool(name: str, args: Any, send=None) -> dict:
    """One tools/call result. Bad arguments and episode errors are tool errors (isError), which the model can read."""
    cli = _cli()
    send = send or _socket_send
    try:
        cmd, rest = tool_argv(name, args)
        req = {**cli.build_request(cmd, rest), "via": "mcp"}
    except ArgError as e:
        return {"content": [{"type": "text", "text": f"error: bad arguments ({e})"}], "isError": True}
    except (ValueError, IndexError) as e:
        return {"content": [{"type": "text", "text": f"error: bad arguments ({e})"}], "isError": True}
    resp = send(req)
    text, rc = cli.format_reply(resp)
    content: list[dict] = [{"type": "text", "text": text}]
    r = resp.get("result") if isinstance(resp.get("result"), dict) else {}
    paths = r.get("image_path") if resp.get("ok") else None
    for p in [paths] if isinstance(paths, str) else paths or []:
        try:
            with open(p, "rb") as f:
                content.append({"type": "image", "data": base64.b64encode(f.read()).decode(), "mimeType": "image/png"})
        except OSError:
            content.append({"type": "text", "text": f"(image {p} could not be read)"})
    return {"content": content, "isError": rc != 0, "_meta": {"robouse/reply": resp}}


def _socket_send(req: dict) -> dict:
    path = _socket_path()
    if not path:
        return {"ok": False, "error": "no episode: ROBOUSE_SOCKET is not set"}
    return _cli()._send(req, path=path)


def _socket_path() -> str | None:
    """The episode socket: $ROBOUSE_SOCKET, or $ROBO_SOCKET, else .robouse/socket in the workspace (the default)."""
    p = os.environ.get("ROBOUSE_SOCKET") or os.environ.get("ROBO_SOCKET")
    if p:
        return p
    f = os.path.join(os.getcwd(), ".robouse", "socket")
    if os.path.exists(f):
        return open(f).read().strip()
    return None


# ---- JSON-RPC --------------------------------------------------------------------------------------------------------


def _error(id_: Any, code: int, message: str) -> dict:
    return {"jsonrpc": "2.0", "id": id_, "error": {"code": code, "message": message}}


def handle(msg: Any, send=None) -> dict | None:
    """One JSON-RPC message in, its response out (None for notifications and responses)."""
    if not isinstance(msg, dict) or msg.get("jsonrpc") != "2.0":
        return _error(msg.get("id") if isinstance(msg, dict) else None, -32600, "invalid request")
    method, id_ = msg.get("method"), msg.get("id")
    if method is None:  # a response to a request we never made
        return None
    if "id" not in msg:  # a notification: initialized, cancelled, ...
        return None
    params = msg.get("params") or {}
    if not isinstance(params, dict):
        return _error(id_, -32602, "params must be an object")
    if method == "initialize":
        asked = params.get("protocolVersion")
        version = asked if asked in PROTOCOL_VERSIONS else PROTOCOL_VERSIONS[0]
        return {
            "jsonrpc": "2.0",
            "id": id_,
            "result": {
                "protocolVersion": version,
                "capabilities": {"tools": {"listChanged": False}},
                "serverInfo": SERVER_INFO,
                "instructions": INSTRUCTIONS,
            },
        }
    if method == "ping":
        return {"jsonrpc": "2.0", "id": id_, "result": {}}
    if method == "tools/list":
        return {"jsonrpc": "2.0", "id": id_, "result": {"tools": TOOLS}}
    if method == "tools/call":
        name = params.get("name")
        if name not in _BY_NAME:
            return _error(id_, -32602, f"unknown tool {name!r}; tools: {', '.join(_BY_NAME)}")
        return {"jsonrpc": "2.0", "id": id_, "result": call_tool(name, params.get("arguments"), send)}
    return _error(id_, -32601, f"method not found: {method}")


def handle_payload(raw: bytes | str, send=None) -> Any:
    """A message or a batch (a JSON array); None when nothing needs a reply."""
    try:
        msg = json.loads(raw)
    except ValueError:
        return _error(None, -32700, "parse error")
    if isinstance(msg, list):
        if not msg:
            return _error(None, -32600, "empty batch")
        out = [r for r in (handle(m, send) for m in msg) if r is not None]
        return out or None
    return handle(msg, send)


def serve_stdio(stdin=None, stdout=None) -> int:
    stdin = stdin or sys.stdin.buffer
    stdout = stdout or sys.stdout.buffer
    for line in stdin:
        if not line.strip():
            continue
        reply = handle_payload(line)
        if reply is not None:
            stdout.write(json.dumps(reply).encode() + b"\n")
            stdout.flush()
    return 0


# ---- streamable HTTP -------------------------------------------------------------------------------------------------

LOCAL_HOSTS = ("127.0.0.1", "localhost", "[::1]", "::1")


def http_server(host: str, port: int, path: str = "/mcp"):
    """A streamable HTTP MCP endpoint at http://host:port{path}. Stateless: every POST is answered with JSON; GET
    (a server-sent event stream) is not offered. A browser page cannot call it (the Origin header must be local)."""
    from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
    from urllib.parse import urlparse

    class H(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *a) -> None:  # quiet
            pass

        def _reply(self, code: int, body: Any = None, allow: str | None = None) -> None:
            data = b"" if body is None else json.dumps(body).encode()
            self.send_response(code)
            if body is not None:
                self.send_header("Content-Type", "application/json")
            if allow:
                self.send_header("Allow", allow)
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def _allowed(self) -> bool:
            if self.path.split("?")[0] != path:
                self._reply(404, _error(None, -32600, "not found"))
                return False
            origin = self.headers.get("Origin")
            if origin and (urlparse(origin).hostname or "") not in LOCAL_HOSTS:
                self._reply(403, _error(None, -32600, "origin not allowed"))
                return False
            return True

        def do_POST(self) -> None:
            if not self._allowed():
                return
            n = int(self.headers.get("Content-Length") or 0)
            if n > 1_000_000:
                self._reply(413, _error(None, -32600, "request too large"))
                return
            reply = handle_payload(self.rfile.read(n))
            if reply is None:
                self._reply(202)
            else:
                self._reply(200, reply)

        def do_GET(self) -> None:
            if self._allowed():
                self._reply(405, allow="POST")

        def do_DELETE(self) -> None:
            if self._allowed():
                self._reply(405, allow="POST")

    return ThreadingHTTPServer((host, port), H)


def _host_port(spec: str) -> tuple[str, int]:
    host, _, port = spec.rpartition(":")
    return host or "127.0.0.1", int(port)


def main(argv: list[str] | None = None) -> int:
    args = list(sys.argv[1:] if argv is None else argv)
    if args[:1] in (["-h"], ["--help"]):
        print(__doc__)
        return 0
    if "--http" not in args:
        return serve_stdio()
    i = args.index("--http")
    host, port = _host_port(args[i + 1])
    path = args[args.index("--path") + 1] if "--path" in args else "/mcp"
    run = args[args.index("--run") + 1 :] if "--run" in args else []
    srv = http_server(host, port, path)
    if not run:
        print(f"robo mcp: serving at http://{host}:{srv.server_address[1]}{path}", file=sys.stderr, flush=True)
        try:
            srv.serve_forever()
        except KeyboardInterrupt:
            pass
        return 0
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    try:
        return subprocess.call(run)
    finally:
        srv.shutdown()


# ---- a minimal client: reference solutions through MCP, and tests ----------------------------------------------------


class StdioClient:
    """Spawns `robo mcp` (stdio) and calls its tools."""

    def __init__(self, cmd: list[str] | None = None, env: dict | None = None):
        cmd = cmd or [sys.executable, os.path.abspath(__file__)]
        self.proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, env=env)
        self._id = 0
        self._lock = threading.Lock()
        init = self.request(
            "initialize",
            {
                "protocolVersion": PROTOCOL_VERSIONS[0],
                "capabilities": {},
                "clientInfo": {"name": "robouse", "version": "1"},
            },
        )
        if "result" not in init:
            raise RuntimeError(f"MCP initialize failed: {init}")
        self.notify("notifications/initialized")

    def _write(self, msg: dict) -> None:
        assert self.proc.stdin is not None
        self.proc.stdin.write(json.dumps(msg).encode() + b"\n")
        self.proc.stdin.flush()

    def notify(self, method: str, params: dict | None = None) -> None:
        self._write({"jsonrpc": "2.0", "method": method, **({"params": params} if params else {})})

    def request(self, method: str, params: dict | None = None) -> dict:
        with self._lock:
            self._id += 1
            self._write({"jsonrpc": "2.0", "id": self._id, "method": method, **({"params": params} if params else {})})
            assert self.proc.stdout is not None
            line = self.proc.stdout.readline()
            if not line:
                raise RuntimeError("the MCP server exited")
            return json.loads(line)

    def call(self, name: str, arguments: dict) -> dict:
        r = self.request("tools/call", {"name": name, "arguments": arguments})
        if "error" in r:
            raise RuntimeError(r["error"]["message"])
        return r["result"]

    def send(self, req: dict) -> dict:
        """An episode request (as the reference solutions write them) through the matching tool; the raw reply."""
        name, args = request_tool(req)
        return self.call(name, args)["_meta"]["robouse/reply"]

    def close(self) -> None:
        if self.proc.poll() is None:
            assert self.proc.stdin is not None
            self.proc.stdin.close()
            try:
                self.proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.proc.kill()


def request_tool(req: dict) -> tuple[str, dict]:
    """The tool call for an episode request: the inverse of the tools' request building."""
    op = req.get("op")
    if op == "info":
        return "info", {}
    if op == "observe":
        a: dict = {"image": bool(req.get("image"))}
        if req.get("camera"):
            a["camera"] = req["camera"]
        return "observe", a
    if op == "act":
        return "act", {"action": list(req.get("action") or []), "repeat": int(req.get("repeat", 1))}
    if op == "move_to":
        x, y, z = (list(req.get("pos") or []) + [None] * 3)[:3]
        a = {"x": x, "y": y, "z": z, "max_steps": int(req.get("max_steps", 100)), "tol": req.get("tol", 0.01)}
        if req.get("grip") is not None:
            a["grip"] = req["grip"]
        return "move_to", a
    if op == "grip":
        return "grip", {"value": req.get("value", 1.0), "steps": int(req.get("steps", 15))}
    if op == "skill":
        return "skill", {"name": req.get("name"), "args": [str(x) for x in req.get("args") or []]}
    if op in ("done", "give_up"):
        return op, {"message": str(req.get("text", ""))}
    raise ValueError(f"no MCP tool for op {op!r}")


_client: StdioClient | None = None


def send(req: dict) -> dict:
    """`agent_cli._send` when $ROBOUSE_SEND_VIA=mcp: requests go through one `robo mcp` server for this process's
    life. Requests with no tool (the runner's status) keep using the socket directly."""
    global _client
    try:
        request_tool(req)
    except ValueError:
        return _socket_send(req)
    if _client is None:
        env = {k: v for k, v in os.environ.items() if k != "ROBOUSE_SEND_VIA"}
        _client = StdioClient(env=env)
        atexit.register(_client.close)
    return _client.send(req)


if __name__ == "__main__":
    sys.exit(main())
