"""`robo`: the only way an agent touches the robot. Standard library only.

Finds the episode socket from $ROBOUSE_SOCKET (set by the runner; $ROBO_SOCKET on BenchFlow) and sends one JSON
request.

  robo info                          action space, skills, step budget
  robo observe [--image] [--camera C] robot and object state; --image also saves a camera PNG and prints its path
  robo act A1 A2 ... [--repeat N]    apply one low-level action N times (N <= 50)
  robo move-to X Y Z [--grip G]      move the hand toward a point (skill; may be disabled per task)
  robo grip G [--steps N]            hold position and set the gripper (+1 close, -1 open)
  robo skill NAME [ARG ...]          run a named high-level skill listed by `robo info` (only some tasks have them)
  robo done ["message"]              end the episode and ask for scoring
  robo give-up ["message"]           end the episode without claiming success
  robo mcp [--http [HOST:]PORT]      serve these operations as MCP tools (stdio; --http: streamable HTTP)
Add --json to any command for raw JSON output.
"""

from __future__ import annotations

import json
import os
import socket
import sys

STEPPING_OPS = ("act", "move_to", "grip", "skill", "done", "give_up")
STEPPING_TIMEOUT_S = 3600


def _send(req: dict, path: str | None = None) -> dict:
    if path is None and os.environ.get("ROBOUSE_SEND_VIA") == "mcp":  # reference solutions run through MCP
        return _mcp().send(req)
    path = path or os.environ.get("ROBOUSE_SOCKET") or os.environ.get("ROBO_SOCKET") or _find_socket()
    if not path:
        return {"ok": False, "error": "no episode: ROBOUSE_SOCKET is not set"}
    token = os.environ.get("ROBOUSE_ORACLE_TOKEN") or os.environ.get("ROBO_ORACLE_TOKEN")  # the reference solution's
    if token:
        req = {**req, "token": token}
    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    # requests that step the simulator wait as long as the episode may run: a long skill steps hundreds of times and
    # the server renders a video frame every other step (0.6 s each with software OpenGL on a busy CPU); the server
    # bounds them itself (per-skill step caps and the episode's wall-clock budget)
    s.settimeout(STEPPING_TIMEOUT_S if req.get("op") in STEPPING_OPS else 120)
    try:
        s.connect(path)
        s.sendall((json.dumps(req) + "\n").encode())
        buf = b""
        while not buf.endswith(b"\n"):
            chunk = s.recv(65536)
            if not chunk:
                break
            buf += chunk
        return json.loads(buf.decode() or '{"ok": false, "error": "empty response"}')
    except (FileNotFoundError, ConnectionRefusedError):
        return {"ok": False, "error": "episode server is not running (episode may have finished)"}
    except TimeoutError:
        return {"ok": False, "error": f"no answer from the episode server within {s.gettimeout():.0f} s"}
    finally:
        s.close()


def _mcp():
    """The MCP module: robo_mcp.py next to this script (the workspace shim), else the installed package's."""
    here = os.path.dirname(os.path.abspath(__file__))
    if os.path.isfile(os.path.join(here, "robo_mcp.py")):
        sys.path.insert(0, here)
        import robo_mcp  # type: ignore[import-not-found]

        return robo_mcp
    from robouse import mcp_server

    return mcp_server


def _find_socket() -> str | None:
    p = os.path.join(os.getcwd(), ".robouse", "socket")
    if os.path.exists(p):
        return open(p).read().strip()
    return None


def _fmt_state(st: dict) -> str:
    keys = [k for k in st if k != "obs_vector"]
    return "\n".join(f"  {k}: {st[k]}" for k in keys)


def format_reply(resp: dict) -> tuple[str, int]:
    """What `robo` prints for a reply, and its exit code."""
    if not resp.get("ok"):
        return f"error: {resp.get('error')}", 1
    r = resp.get("result", {})
    if not isinstance(r, dict):
        return str(r), 0
    lines = []
    for k, v in r.items():
        if k == "state" and isinstance(v, dict):
            lines += ["state:", _fmt_state(v)]
        elif k == "action" and isinstance(v, dict):
            lines.append(f"action: {v['names']} low={v['low']} high={v['high']}")
            if v.get("doc"):
                lines.append(f"  {v['doc']}")
        else:
            lines.append(f"{k}: {v}")
    return "\n".join(lines), 0


def _print(resp: dict, as_json: bool) -> int:
    if as_json:
        print(json.dumps(resp))
        return 0 if resp.get("ok") else 1
    text, rc = format_reply(resp)
    print(text)
    return rc


def build_request(cmd: str, rest: list[str]) -> dict:
    """The episode request for `robo CMD REST...`. Raises ValueError or IndexError for bad arguments and KeyError for an
    unknown command. The MCP tools build their requests through this too, so both interfaces send the same thing."""
    args = list(rest)

    def opt(name: str, default=None, cast=float):
        if name in args:
            i = args.index(name)
            v = cast(args[i + 1])
            del args[i : i + 2]
            return v
        return default

    if cmd == "info":
        return {"op": "info"}
    if cmd == "observe":
        cam = opt("--camera", None, str)
        return {"op": "observe", "image": "--image" in args, "camera": cam}
    if cmd == "act":
        rep = opt("--repeat", 1, int)
        return {"op": "act", "action": [float(x) for x in args], "repeat": rep}
    if cmd in ("move-to", "move_to"):
        grip = opt("--grip", None, float)
        ms = opt("--max-steps", 100, int)
        tol = opt("--tol", 0.01, float)
        return {"op": "move_to", "pos": [float(x) for x in args[:3]], "grip": grip, "max_steps": ms, "tol": tol}
    if cmd == "grip":
        steps = opt("--steps", 15, int)
        return {"op": "grip", "value": float(args[0]), "steps": steps}
    if cmd == "skill":
        if not args:
            raise IndexError("skill needs a name")
        return {"op": "skill", "name": args[0], "args": args[1:]}
    if cmd == "done":
        return {"op": "done", "text": " ".join(args)}
    if cmd in ("give-up", "give_up"):
        return {"op": "give_up", "text": " ".join(args)}
    raise KeyError(cmd)


def main(argv: list[str] | None = None) -> int:
    args = list(sys.argv[1:] if argv is None else argv)
    if args[:1] == ["mcp"]:
        return _mcp().main(args[1:])
    as_json = "--json" in args
    args = [a for a in args if a != "--json"]
    if not args or args[0] in ("-h", "--help", "help"):
        print(__doc__)
        return 0
    cmd, rest = args[0], args[1:]
    try:
        req = build_request(cmd, rest)
    except (ValueError, IndexError) as e:
        print(f"error: bad arguments ({e}); run `robo help`")
        return 2
    except KeyError:
        print(f"error: unknown command {cmd!r}; run `robo help`")
        return 2
    return _print(_send(req), as_json)


if __name__ == "__main__":
    sys.exit(main())
