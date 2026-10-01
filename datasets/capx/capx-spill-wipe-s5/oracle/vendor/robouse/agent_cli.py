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
Add --json to any command for raw JSON output.
"""

from __future__ import annotations

import json
import os
import socket
import sys


def _send(req: dict, path: str | None = None) -> dict:
    path = path or os.environ.get("ROBOUSE_SOCKET") or os.environ.get("ROBO_SOCKET") or _find_socket()
    if not path:
        return {"ok": False, "error": "no episode: ROBOUSE_SOCKET is not set"}
    token = os.environ.get("ROBOUSE_ORACLE_TOKEN") or os.environ.get("ROBO_ORACLE_TOKEN")  # the reference solution's
    if token:
        req = {**req, "token": token}
    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    s.settimeout(120)
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
    finally:
        s.close()


def _find_socket() -> str | None:
    p = os.path.join(os.getcwd(), ".robouse", "socket")
    if os.path.exists(p):
        return open(p).read().strip()
    return None


def _fmt_state(st: dict) -> str:
    keys = [k for k in st if k != "obs_vector"]
    return "\n".join(f"  {k}: {st[k]}" for k in keys)


def _print(resp: dict, as_json: bool) -> int:
    if as_json:
        print(json.dumps(resp))
        return 0 if resp.get("ok") else 1
    if not resp.get("ok"):
        print(f"error: {resp.get('error')}")
        return 1
    r = resp.get("result", {})
    if not isinstance(r, dict):
        print(r)
        return 0
    for k, v in r.items():
        if k == "state" and isinstance(v, dict):
            print("state:")
            print(_fmt_state(v))
        elif k == "action" and isinstance(v, dict):
            print(f"action: {v['names']} low={v['low']} high={v['high']}")
            if v.get("doc"):
                print(f"  {v['doc']}")
        else:
            print(f"{k}: {v}")
    return 0


def main(argv: list[str] | None = None) -> int:
    args = list(sys.argv[1:] if argv is None else argv)
    as_json = "--json" in args
    args = [a for a in args if a != "--json"]
    if not args or args[0] in ("-h", "--help", "help"):
        print(__doc__)
        return 0

    def opt(name: str, default=None, cast=float):
        if name in args:
            i = args.index(name)
            v = cast(args[i + 1])
            del args[i : i + 2]
            return v
        return default

    cmd, rest = args[0], args[1:]
    try:
        if cmd == "info":
            return _print(_send({"op": "info"}), as_json)
        if cmd == "observe":
            args = rest
            cam = opt("--camera", None, str)
            if os.environ.get("ROBOUSE_TEXT_ONLY") and ("--image" in args or cam):
                print(
                    "note: this harness's model cannot read images, so --image and --camera are ignored",
                    file=sys.stderr,
                )
                return _print(_send({"op": "observe", "image": False, "camera": None}), as_json)
            return _print(_send({"op": "observe", "image": "--image" in args, "camera": cam}), as_json)
        if cmd == "act":
            args = rest
            rep = opt("--repeat", 1, int)
            return _print(_send({"op": "act", "action": [float(x) for x in args], "repeat": rep}), as_json)
        if cmd in ("move-to", "move_to"):
            args = rest
            grip = opt("--grip", None, float)
            ms = opt("--max-steps", 100, int)
            tol = opt("--tol", 0.01, float)
            return _print(
                _send(
                    {"op": "move_to", "pos": [float(x) for x in args[:3]], "grip": grip, "max_steps": ms, "tol": tol}
                ),
                as_json,
            )
        if cmd == "grip":
            args = rest
            steps = opt("--steps", 15, int)
            return _print(_send({"op": "grip", "value": float(args[0]), "steps": steps}), as_json)
        if cmd == "skill":
            if not rest:
                raise IndexError("skill needs a name")
            return _print(_send({"op": "skill", "name": rest[0], "args": rest[1:]}), as_json)
        if cmd == "done":
            return _print(_send({"op": "done", "text": " ".join(rest)}), as_json)
        if cmd in ("give-up", "give_up"):
            return _print(_send({"op": "give_up", "text": " ".join(rest)}), as_json)
    except (ValueError, IndexError) as e:
        print(f"error: bad arguments ({e}); run `robo help`")
        return 2
    print(f"error: unknown command {cmd!r}; run `robo help`")
    return 2


if __name__ == "__main__":
    sys.exit(main())
