"""Trusted episode server.

`robouse serve` loads one task, owns the simulator, and answers one JSON request per Unix-socket
connection (the `robo` CLI is the client). It enforces the step and wall-clock budgets, records every
frame with a timestamp, and decides success itself when the episode ends. The agent only sees what
`observe` returns.

Episode files written to the run directory:
  result.json            success, outcome, steps used, agent's final message
  trace.jsonl            every request and response (images referenced by path)
  frames.jsonl           wall-clock time and env step for each video frame
  recording.mp4          the episode video
"""
from __future__ import annotations

import json
import os
import socket
import time
from pathlib import Path
from typing import Any

import numpy as np

from .backends import make_backend

SETTLE_STEPS = 10  # after `done`, hold still this many steps before judging success
MAX_REPEAT = 50
MAX_SKILL_STEPS = 150
FPS = 30


class Episode:
    def __init__(self, spec: dict, run_dir: Path, workspace: Path | None = None, seed: int | None = None):
        self.spec = spec
        self.run_dir = Path(run_dir)
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self.workspace = Path(workspace) if workspace else None
        self.backend = make_backend(spec)
        self.seed = int(spec.get("seed", 0) if seed is None else seed)
        self.max_steps = int(spec.get("max_steps", getattr(self.backend, "max_steps", 500)))
        self.frame_every = int(spec.get("frame_every", 2))
        self.skills_enabled = bool(spec.get("skills", True))
        # "final": judged after `robo done` and a settle period (default).
        # "first": the episode ends as solved the first time the task's own success signal fires
        #          (Meta-World's evaluation protocol).
        self.success_mode = str(spec.get("success_mode", "final"))
        # "state" (default): observe returns all public state fields.
        # "vision": observe returns only `visible_fields` (the robot's own proprioception by default) plus camera
        #           images; objects and goals must be found in the images. Requests carrying the per-episode
        #           oracle token (given only to the reference solution) still get the full state.
        self.obs_mode = str(spec.get("obs_mode", "state"))
        self.visible_fields = list(spec.get("visible_fields", ["hand_pos", "gripper_open"]))
        self.cameras = list(spec.get("cameras", [getattr(self.backend, "camera", "")]))
        self._oracle_token = os.environ.get("ROBOUSE_ORACLE_TOKEN", "")
        # optional per-task limit on `act --repeat` (and `grip --steps`); default unchanged
        self.max_repeat = max(1, min(MAX_REPEAT, int(spec.get("max_repeat", MAX_REPEAT))))
        # cameras whose projection matrix is not given to the agent (e.g. a camera that moves with the gripper)
        self.uncalibrated = set(spec.get("uncalibrated_cameras", []))
        self.backend.reset(self.seed)
        self._save_workspace_images()
        self.steps = 0
        self.requests = 0
        self.success_ever = False
        self.finished = False
        self.outcome = None
        self.agent_text = ""
        self.t0 = time.time()
        self._frames: list[np.ndarray] = []
        self._frame_meta: list[dict] = []
        self._trace = open(self.run_dir / "trace.jsonl", "w")
        self._obs_count = 0
        self._record_frame(force=True)

    def _save_workspace_images(self) -> None:
        """Optional backend hook: images placed in the agent's workspace at the start (e.g. rule examples)."""
        hook = getattr(self.backend, "workspace_images", None)
        imgs = hook() if callable(hook) else {}
        if not imgs:
            return
        from PIL import Image

        for d in [self.run_dir] + ([self.workspace] if self.workspace else []):
            (d / "observations").mkdir(parents=True, exist_ok=True)
            for name, img in imgs.items():
                Image.fromarray(np.asarray(img, dtype=np.uint8)).save(d / "observations" / name)

    # ---- recording -------------------------------------------------------------------------
    def _record_frame(self, force: bool = False) -> None:
        if force or self.steps % self.frame_every == 0:
            img = self.backend.render()
            self._frames.append(np.asarray(img, dtype=np.uint8))
            self._frame_meta.append({"i": len(self._frames) - 1, "t": round(time.time() - self.t0, 3), "step": self.steps})

    def _env_step(self, action) -> Any:
        info = self.backend.step(action)
        self.steps += 1
        self.success_ever = self.success_ever or info.success
        self._record_frame()
        if info.success and self.success_mode == "first" and not self.finished:
            self.finish("success_reached", "", settle=False)
        return info

    def _state(self, privileged: bool = False) -> dict:
        hook = getattr(self.backend, "observe_privileged", None)  # reference-solution hints (oracle token only)
        st = hook() if privileged and callable(hook) else self.backend.observe()
        if self.obs_mode == "vision" and not privileged:
            st = {k: v for k, v in st.items() if k in self.visible_fields}
        return st

    def camera_info(self) -> list[dict]:
        """Per camera: a 3x4 projection matrix P with [u*w, v*w, w] = P @ [x, y, z, 1] in the saved image's pixels."""
        out = []
        hook = getattr(self.backend, "camera_info", None)  # backends that are not MuJoCo compute their own calibration
        if callable(hook):
            return [c if c.get("name") not in self.uncalibrated else {**c, "projection": None, "position": None}
                    for c in hook(self.cameras)]
        m_d = getattr(self.backend, "mj_model_data", lambda: None)()
        if not m_d:
            return out
        model, data = m_d
        import mujoco

        for name in self.cameras:
            cid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, name)
            if cid < 0:
                continue
            img = self._render(name)
            h, w = img.shape[:2]
            fovy = float(model.cam_fovy[cid])
            if name in self.uncalibrated:
                out.append({"name": name, "width": w, "height": h, "fovy_deg": round(fovy, 2), "projection": None})
                continue
            f = (h / 2) / np.tan(np.deg2rad(fovy) / 2)
            R = data.cam_xmat[cid].reshape(3, 3)
            t = data.cam_xpos[cid]
            ext = np.hstack([R.T, (-R.T @ t)[:, None]])
            K = np.array([[f, 0, -w / 2], [0, -f, -h / 2], [0, 0, -1.0]])
            P = K @ ext
            if getattr(self.backend, "image_flipped", False):
                P[1] = (h - 1) * P[2] - P[1]
            out.append({"name": name, "width": w, "height": h, "fovy_deg": round(fovy, 2),
                        "position": [round(float(x), 4) for x in t],
                        "projection": [[round(float(x), 5) for x in row] for row in P]})
        return out

    def _render(self, camera: str | None = None):
        if not camera or camera == getattr(self.backend, "camera", None):
            return self.backend.render()
        old = self.backend.camera
        try:
            self.backend.camera = camera
            return self.backend.render()
        finally:
            self.backend.camera = old

    # ---- request handling ------------------------------------------------------------------
    def handle(self, req: dict) -> dict:
        self.requests += 1
        privileged = bool(self._oracle_token) and req.get("token") == self._oracle_token
        req = {k: v for k, v in req.items() if k != "token"}  # never write the token to the trace
        op = req.get("op")
        t = round(time.time() - self.t0, 3)
        try:
            if self.finished and op not in ("info", "status"):
                resp = {"ok": False, "error": f"episode finished ({self.outcome})"}
            elif op == "info":
                resp = {"ok": True, "result": self.info()}
            elif op == "status":
                resp = {"ok": True, "result": {"finished": self.finished, "outcome": self.outcome, "steps": self.steps}}
            elif op == "observe":
                resp = {"ok": True, "result": self.observe(bool(req.get("image", False)), privileged, req.get("camera"))}
            elif op == "act":
                resp = self.act(req.get("action"), int(req.get("repeat", 1)))
            elif op == "move_to":
                resp = self.move_to(req.get("pos"), req.get("grip"), int(req.get("max_steps", 100)), float(req.get("tol", 0.01)))
            elif op == "grip":
                resp = self.grip(float(req.get("value", 1.0)), int(req.get("steps", 15)))
            elif op == "skill":
                resp = self.skill(req.get("name"), req.get("args") or [])
            elif op in ("done", "give_up"):
                resp = {"ok": True, "result": self.finish("done" if op == "done" else "gave_up", str(req.get("text", ""))[:4000])}
            else:
                resp = {"ok": False, "error": f"unknown op {op!r}"}
        except Exception as e:  # bounded error back to the agent
            resp = {"ok": False, "error": f"{type(e).__name__}: {e}"[:500]}
        if not self.finished and self.steps >= self.max_steps:
            self.finish("budget_exhausted", "")
        if self.finished and op in ("act", "move_to", "grip", "skill") and resp.get("ok") and isinstance(resp.get("result"), dict):
            msg = {"budget_exhausted": "step budget exhausted", "success_reached": "goal reached, episode scored as solved",
                   "wall_time_exhausted": "wall-clock budget exhausted"}.get(self.outcome, self.outcome)
            resp["result"]["episode"] = f"finished: {msg}"
        self._trace.write(json.dumps({"t": t, "steps": self.steps, "req": req, "resp": _strip(resp)}) + "\n")
        self._trace.flush()
        return resp

    def info(self) -> dict:
        s = self.backend.action_spec
        return {
            "task": self.spec.get("id"), "backend": self.backend.name,
            "action": {"names": s.names, "low": s.low, "high": s.high, "doc": s.doc},
            "skills": self.backend.skills() if self.skills_enabled else [],
            "steps_used": self.steps, "max_steps": self.max_steps,
            "observation_mode": self.obs_mode,
            **({"max_repeat": self.max_repeat} if "max_repeat" in self.spec else {}),
            **({"visible_fields": self.visible_fields, "cameras": self.camera_info()} if self.obs_mode == "vision" else {}),
            **({"embodiment": self.backend.embodiment()} if callable(getattr(self.backend, "embodiment", None)) else {}),
        }

    def observe(self, image: bool, privileged: bool = False, camera: str | None = None) -> dict:
        out = {"state": self._state(privileged), "steps_used": self.steps, "max_steps": self.max_steps}
        if image or self.obs_mode == "vision":
            from PIL import Image

            cams = [camera] if camera in self.cameras else (self.cameras if self.obs_mode == "vision" else [None])
            paths = []
            for cam in cams:
                self._obs_count += 1
                img = Image.fromarray(np.asarray(self._render(cam), dtype=np.uint8))
                name = f"obs_{self._obs_count:03d}" + (f"_{cam}" if cam else "") + ".png"
                (self.run_dir / "observations").mkdir(exist_ok=True)
                img.save(self.run_dir / "observations" / name)
                if self.workspace:
                    (self.workspace / "observations").mkdir(exist_ok=True)
                    img.save(self.workspace / "observations" / name)
                    paths.append(str(self.workspace / "observations" / name))
                else:
                    paths.append(str(self.run_dir / "observations" / name))
            out["image_path"] = paths[0] if len(paths) == 1 else paths
        return out

    def act(self, action, repeat: int) -> dict:
        spec = self.backend.action_spec
        if not isinstance(action, list) or len(action) != spec.dim or not all(isinstance(x, (int, float)) for x in action):
            return {"ok": False, "error": f"action must be a list of {spec.dim} numbers {spec.names}"}
        repeat = max(1, min(self.max_repeat, repeat))
        n = 0
        for _ in range(repeat):
            if self.steps >= self.max_steps or self.finished:
                break
            self._env_step(action)
            n += 1
        return {"ok": True, "result": {"executed_steps": n, "state": self._state(), "steps_used": self.steps}}

    def move_to(self, pos, grip, max_steps: int, tol: float) -> dict:
        if not self.skills_enabled or "move_to" not in self.backend.skills():
            return {"ok": False, "error": "move_to is not available for this task; use act"}
        if not isinstance(pos, list) or len(pos) != 3:
            return {"ok": False, "error": "pos must be [x, y, z]"}
        target = np.asarray(pos, dtype=float)
        g = float(getattr(self.backend, "_grip", -1.0) if grip is None else grip)
        n, dist, stalled = 0, float("inf"), False
        recent: list[np.ndarray] = []
        for _ in range(max(1, min(MAX_SKILL_STEPS, max_steps))):
            if self.steps >= self.max_steps or self.finished:
                break
            hp = self.backend.hand_pos()
            d = target - hp
            dist = float(np.linalg.norm(d))
            if dist < tol:
                break
            recent.append(hp)
            if len(recent) > 8 and float(np.linalg.norm(recent[-1] - recent[-9])) < 0.002:
                stalled = True  # the hand has stopped moving: the target is out of reach or blocked
                break
            self._env_step(list(np.clip(d * 10.0, -1, 1)) + [g])
            n += 1
        dist = float(np.linalg.norm(target - self.backend.hand_pos()))
        res = {"reached": dist < tol * 1.5, "distance": round(dist, 4), "executed_steps": n,
               "state": self._state(), "steps_used": self.steps}
        if stalled:
            res["stalled"] = "the hand stopped moving before reaching the target (out of reach or blocked)"
        return {"ok": True, "result": res}

    def grip(self, value: float, steps: int) -> dict:
        if not self.skills_enabled or "grip" not in self.backend.skills():
            return {"ok": False, "error": "grip is not available for this task; use act"}
        n = 0
        for _ in range(max(1, min(self.max_repeat, steps))):
            if self.steps >= self.max_steps or self.finished:
                break
            self._env_step([0.0, 0.0, 0.0, max(-1.0, min(1.0, value))])
            n += 1
        return {"ok": True, "result": {"executed_steps": n, "state": self._state(), "steps_used": self.steps}}

    def skill(self, name, args) -> dict:
        """Named high-level skill implemented by the backend (optional `run_skill(name, args)` hook, e.g. BEHAVIOR's
        symbolic primitives). One skill call counts as one step of the budget. Backends may also provide
        `pop_frames()` with the video frames rendered while the skill ran."""
        run = getattr(self.backend, "run_skill", None)
        if not self.skills_enabled or not callable(run):
            return {"ok": False, "error": "no named skills in this task; see `robo info`"}
        if not isinstance(name, str) or not isinstance(args, list):
            return {"ok": False, "error": "skill needs a name and a list of arguments"}
        if self.steps >= self.max_steps:
            return {"ok": False, "error": "step budget exhausted"}
        out = run(name, [str(a) for a in args])
        self.steps += 1
        pop = getattr(self.backend, "pop_frames", None)
        for fr in (pop() if callable(pop) else []):
            self._frames.append(np.asarray(fr, dtype=np.uint8))
            self._frame_meta.append({"i": len(self._frames) - 1, "t": round(time.time() - self.t0, 3), "step": self.steps})
        self._record_frame(force=True)
        return {"ok": True, "result": {**out, "steps_used": self.steps, "max_steps": self.max_steps}}

    def finish(self, outcome: str, text: str, settle: bool = True) -> dict:
        if self.finished:
            return {"outcome": self.outcome}
        g = float(getattr(self.backend, "_grip", 0.0))
        dim = self.backend.action_spec.dim
        hold = [0.0] * (dim - 1) + [g] if dim >= 1 else []
        for _ in range(SETTLE_STEPS if settle else 0):  # settle; success must still hold afterwards
            try:
                self.backend.step(hold)
            except Exception:  # a backend that cannot step further is judged on its current state
                break
            self._record_frame(force=True)
        judge = getattr(self.backend, "judge", None)  # optional: verdict that depends on how the episode ended
        if callable(judge):  # e.g. safety tasks, where refusing (give_up) is the rewarded behaviour
            success = bool(judge(outcome, text))
        elif self.success_mode == "first":
            success = bool(self.success_ever)
        else:  # only an explicit `robo done` is judged; give-up, timeouts and budget endings score 0
            success = outcome == "done" and bool(self.backend.success())
        self.finished, self.outcome, self.agent_text = True, outcome, text
        self.result = {
            "task": self.spec.get("id"), "seed": self.seed, "success": success, "success_ever": self.success_ever,
            "success_mode": self.success_mode,
            "outcome": outcome, "steps_used": self.steps, "max_steps": self.max_steps, "requests": self.requests,
            "agent_text": text, "wall_time_s": round(time.time() - self.t0, 2),
        }
        (self.run_dir / "result.json").write_text(json.dumps(self.result, indent=2))
        self._write_video()
        return {"outcome": outcome, "episode": "finished"}

    def _write_video(self) -> None:
        with open(self.run_dir / "frames.jsonl", "w") as f:
            for m in self._frame_meta:
                f.write(json.dumps(m) + "\n")
        if not self._frames:
            return
        import imageio.v2 as imageio

        with imageio.get_writer(self.run_dir / "recording.mp4", fps=FPS, codec="libx264", quality=7,
                                macro_block_size=16, ffmpeg_log_level="error") as w:
            for fr in self._frames:
                w.append_data(fr)
        self._frames = []

    def close(self) -> None:
        if not self.finished:
            self.finish("agent_exited", "")
        self._trace.close()
        self.backend.close()


def _strip(resp: dict) -> dict:
    """Drop the raw obs vector from the trace to keep it readable."""
    r = json.loads(json.dumps(resp))
    st = (r.get("result") or {}).get("state") if isinstance(r.get("result"), dict) else None
    if isinstance(st, dict):
        st.pop("obs_vector", None)
    return r


def serve(spec: dict, run_dir: Path, sock_path: str, workspace: Path | None = None, max_wall_s: float = 1800,
          ready_file: Path | None = None) -> dict:
    ep = Episode(spec, run_dir, workspace)
    if os.path.exists(sock_path):
        os.unlink(sock_path)
    srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    srv.bind(sock_path)
    srv.listen(8)
    srv.settimeout(1.0)
    if ready_file:
        Path(ready_file).write_text(sock_path)
    deadline = time.time() + max_wall_s
    stop_after_finish = None
    try:
        while True:
            if time.time() > deadline and not ep.finished:
                ep.finish("wall_time_exhausted", "")
            if ep.finished and stop_after_finish is None:
                stop_after_finish = time.time() + float(spec.get("linger_s", 5))
            if stop_after_finish and time.time() > stop_after_finish:
                break
            try:
                conn, _ = srv.accept()
            except socket.timeout:
                continue
            with conn:
                conn.settimeout(10)
                buf = b""
                try:
                    while not buf.endswith(b"\n") and len(buf) < 1_000_000:
                        chunk = conn.recv(65536)
                        if not chunk:
                            break
                        buf += chunk
                    req = json.loads(buf.decode() or "{}")
                    if req.get("op") == "shutdown":
                        conn.sendall(b'{"ok": true}\n')
                        break
                    resp = ep.handle(req)
                except Exception as e:
                    resp = {"ok": False, "error": f"bad request: {e}"[:300]}
                try:
                    conn.sendall((json.dumps(resp) + "\n").encode())
                except OSError:  # the client went away (e.g. its shell command timed out); keep serving the episode
                    pass
    finally:
        ep.close()
        srv.close()
        if os.path.exists(sock_path):
            os.unlink(sock_path)
    return ep.result
