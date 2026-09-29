"""`molmoact2` harness: drive a robouse episode with MolmoAct2 through the `robo` socket. Standard library only.

The runner copies this file into the trial workspace and runs it like any other harness (same sandbox, same
socket, no access to the simulator). Each cycle:
  1. `observe` with images: the front camera and the wrist camera (LIBERO: `agentview`, `robot0_eye_in_hand`),
     saved as PNG by the episode server; plus the robot state.
  2. POST the two PNGs, the task's language instruction and the state to the MolmoAct2 policy server
     (src/robouse/molmoact2/server.py on a GPU); get back an action chunk (10 actions for MolmoAct2-LIBERO).
  3. Apply the actions one by one with `act`, stopping as soon as the episode ends.
Until the episode ends (success under success_mode "first", or the step budget); then `done` if still open.

Environment: ROBOUSE_SOCKET (set by the runner), ROBOUSE_MOLMOACT2_URL (server base URL),
ROBOUSE_MOLMOACT2_TOKEN (optional; sent as the Daytona preview token header), ROBOUSE_TASK_LANGUAGE (the
instruction given to the policy). One JSON line per chunk is printed to stdout (agent/stdout.jsonl).

Backends other than LIBERO (zero-shot, out of distribution; see docs/molmoact2.md): the task's single camera
fills both image slots, the state and actions are mapped between the LIBERO frame (+x forward from the robot,
+y to its left) and the robouse frame (+y away from the robot/front camera, +x to its right):
LIBERO (dx, dy, dz) -> robouse (-dy, dx, dz); rotation deltas are dropped; the gripper command passes through
(both use -1 open, +1 close). Hand positions are re-centred so the episode's first hand position maps to
LIBERO's usual start pose.
"""
from __future__ import annotations

import base64
import json
import os
import socket
import sys
import time
import urllib.request

LIBERO_CAMS = ("agentview", "robot0_eye_in_hand")
LIBERO_START = (-0.21, 0.0, 1.17)       # typical LIBERO end-effector start position
DOWN_QUAT = (1.0, 0.0, 0.0, 0.0)        # LIBERO's gripper-pointing-down orientation (x, y, z, w)


def robo(req: dict) -> dict:
    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    s.settimeout(300)
    try:
        s.connect(os.environ["ROBOUSE_SOCKET"])
        s.sendall((json.dumps(req) + "\n").encode())
        buf = b""
        while not buf.endswith(b"\n"):
            chunk = s.recv(1 << 20)
            if not chunk:
                break
            buf += chunk
        return json.loads(buf.decode() or '{"ok": false, "error": "empty response"}')
    except (FileNotFoundError, ConnectionRefusedError):
        return {"ok": False, "error": "episode server is not running"}
    finally:
        s.close()


def post(path: str, body: dict, timeout: float = 120) -> dict:
    url = os.environ["ROBOUSE_MOLMOACT2_URL"].rstrip("/") + path
    headers = {"Content-Type": "application/json"}
    if os.environ.get("ROBOUSE_MOLMOACT2_TOKEN"):
        headers["x-daytona-preview-token"] = os.environ["ROBOUSE_MOLMOACT2_TOKEN"]
    last = None
    for attempt in range(4):  # the preview proxy occasionally drops a request
        try:
            req = urllib.request.Request(url, data=json.dumps(body).encode(), headers=headers, method="POST")
            with urllib.request.urlopen(req, timeout=timeout) as r:
                return json.loads(r.read().decode())
        except Exception as e:  # noqa: BLE001
            last = e
            time.sleep(2 * (attempt + 1))
    raise RuntimeError(f"policy server unreachable: {last}")


def b64(path: str) -> str:
    with open(path, "rb") as f:
        return base64.b64encode(f.read()).decode()


def emit(**kw) -> None:
    print(json.dumps(kw), flush=True)


def main() -> int:
    info = robo({"op": "info"})
    if not info.get("ok"):
        emit(type="error", error=info.get("error"))
        return 1
    info = info["result"]
    backend, dim = info["backend"], len(info["action"]["names"])
    language = os.environ.get("ROBOUSE_TASK_LANGUAGE", "").strip()
    episode = f"{info.get('task')}-{os.getpid()}-{int(time.time())}"
    emit(type="start", task=info.get("task"), backend=backend, action_dim=dim, max_steps=info["max_steps"],
         language=language, server=os.environ.get("ROBOUSE_MOLMOACT2_URL", ""))
    post("/reset", {"episode": episode})
    offset = None
    chunk = 0
    while True:
        # 1. observe: state + the two camera images
        cams = LIBERO_CAMS if backend == "libero" else (None,)
        paths, st = [], None
        for cam in cams:
            r = robo({"op": "observe", "image": True, "camera": cam})
            if not r.get("ok"):
                emit(type="end", reason=r.get("error"))
                return 0
            st = r["result"]["state"]
            p = r["result"]["image_path"]
            paths.append(p if isinstance(p, str) else p[0])
        if len(paths) == 1:
            paths.append(paths[0])  # no wrist camera: the scene camera fills both slots
        if backend == "libero":
            state = {"eef_pos": st["hand_pos"], "eef_quat": st["eef_quat"], "gripper_qpos": st["gripper_qpos"]}
        else:
            h = st["hand_pos"]
            if offset is None:
                offset = [LIBERO_START[0] - h[1], LIBERO_START[1] + h[0], LIBERO_START[2] - h[2]]
            g = float(st.get("gripper_open", 1.0))
            g = max(0.0, min(1.0, g if g <= 1.0 else g / 0.08))
            state = {"eef_pos": [h[1] + offset[0], -h[0] + offset[1], h[2] + offset[2]], "eef_quat": list(DOWN_QUAT),
                     "gripper_qpos": [0.04 * g, -0.04 * g]}
        # 2. policy
        t0 = time.time()
        out = post("/act", {"episode": episode, "task": language, "state": state,
                            "images": {"image": b64(paths[0]), "wrist_image": b64(paths[1])}})
        if "actions" not in out:
            emit(type="error", error=out.get("error"))
            break
        chunk += 1
        emit(type="chunk", chunk=chunk, steps_used=r["result"].get("steps_used"), state=state,
             server_latency_s=out.get("latency_s"), roundtrip_s=round(time.time() - t0, 3), actions=out["actions"])
        # 3. act
        for a in out["actions"]:
            if backend == "libero":
                cmd = [max(-1.0, min(1.0, float(x))) for x in a[:dim]]
            else:  # LIBERO (dx, dy, dz, ..., grip) -> robouse (dx, dy, dz, grip)
                cmd = [-a[1], a[0], a[2], a[6]]
                cmd = [max(-1.0, min(1.0, float(x))) for x in cmd] + [0.0] * (dim - 4)
            res = robo({"op": "act", "action": cmd, "repeat": 1})
            if not res.get("ok"):
                emit(type="end", reason=res.get("error"))
                return 0
            if "episode" in res["result"]:
                emit(type="end", reason=res["result"]["episode"], steps_used=res["result"].get("steps_used"))
                return 0
    robo({"op": "done", "text": "molmoact2 stopped"})
    return 0


if __name__ == "__main__":
    sys.exit(main())
