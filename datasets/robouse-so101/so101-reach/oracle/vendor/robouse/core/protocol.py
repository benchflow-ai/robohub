"""The episode protocol: one JSON request per Unix-socket connection, one JSON line back.

Requests carry an `op`: info, status, observe (image, camera), act (action, repeat), move_to (pos, grip, max_steps, tol),
grip (value, steps), skill (name, args), done (text), give_up (text), and the runner's shutdown. Replies are
{"ok": true, "result": ...} or {"ok": false, "error": "..."}. The `robo` client (agent_cli.py) is the other end; it is a
standalone script, so it keeps its own copy of the client side.
"""

from __future__ import annotations

import hashlib
import hmac
import json
import socket

MAX_REQUEST_BYTES = 1_000_000
MOTION_OPS = ("act", "move_to", "grip", "skill")
# why an episode ended, as the agent is told in the reply to its last motion
FINISH_MESSAGES = {
    "budget_exhausted": "step budget exhausted",
    "success_reached": "goal reached, episode scored as solved",
    "wall_time_exhausted": "wall-clock budget exhausted",
    "safety_stop": "stopped by the safety layer (the robot holds; a human must check it)",
}

# the runner's own give-up (the agent timed out or exited without `robo done`) starts with this
RUNNER_PREFIX = "[runner]"


def is_runner_request(req: dict) -> bool:
    """A request the runner sent, not the agent: its status check (`"runner": true`) or its give-up."""
    return bool(req.get("runner")) or (
        req.get("op") == "give_up" and str(req.get("text", "")).startswith(RUNNER_PREFIX)
    )


def agent_refused(outcome: str | None, text: str) -> bool:
    """The agent itself gave up (`robo give-up`), as opposed to the runner closing the episode for it."""
    return outcome == "gave_up" and not str(text).startswith(RUNNER_PREFIX)


def read_request(conn: socket.socket) -> dict:
    """One request line from a connection (at most MAX_REQUEST_BYTES)."""
    buf = b""
    while not buf.endswith(b"\n") and len(buf) < MAX_REQUEST_BYTES:
        chunk = conn.recv(65536)
        if not chunk:
            break
        buf += chunk
    return json.loads(buf.decode() or "{}")


def send_reply(conn: socket.socket, reply: dict) -> bool:
    """Send one reply line; False when the client has gone away (e.g. its shell command timed out)."""
    try:
        conn.sendall((json.dumps(reply) + "\n").encode())
        return True
    except OSError:
        return False


# ---- the signed verdict ------------------------------------------------------------------------------------------
# The runner gives the episode server a per-trial key on its stdin (never in its environment or command line, which
# other processes of the same user can read). The server signs result.json with it, and the runner accepts the
# verdict only if the signature matches, so an agent that rewrites result.json cannot change its score.


def _canonical(result: dict) -> bytes:
    return json.dumps({k: v for k, v in result.items() if k != "signature"}, sort_keys=True).encode()


def sign_result(result: dict, key: str) -> str:
    return hmac.new(key.encode(), _canonical(result), hashlib.sha256).hexdigest()


def verify_result(result: dict, key: str) -> bool:
    sig = result.get("signature")
    return isinstance(sig, str) and hmac.compare_digest(sig, sign_result(result, key))
