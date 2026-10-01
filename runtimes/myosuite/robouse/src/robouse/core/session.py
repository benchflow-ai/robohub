"""The trusted episode server.

`robouse serve` loads one task, owns the simulator, and answers one JSON request per Unix-socket connection (the `robo`
CLI is the client; core/protocol.py). It enforces the step and wall-clock budgets, records every frame with a timestamp,
and decides success itself when the episode ends. The agent only sees what `observe` returns.

Episode files written to the run directory:
  result.json            success, outcome, steps used, the agent's final message, metrics
  trace.jsonl            every request and response (images referenced by path)
  frames.jsonl           wall-clock time and env step for each video frame
  physics.jsonl          per-step physics samples behind the metrics
  recording.mp4          the episode video

The video is rendered at the backend's native size unless ROBOUSE_RECORD_SIZE=WIDTHxHEIGHT is set (or `--record-size`
on `serve`, `run` and `run-many`); then each frame comes from a separate renderer at that size, with the same camera.
Agent observation images are never affected.
"""

from __future__ import annotations

import json
import logging
import os
import socket
import time
from collections.abc import Callable, Generator
from pathlib import Path
from typing import Any

import numpy as np

from .. import config
from ..backends import make_backend
from ..backends.base import embodiment_dict
from ..lib.text import clip
from .cameras import mujoco_camera
from .probe import PhysicsProbe
from .protocol import FINISH_MESSAGES, MOTION_OPS, is_runner_request, read_request, send_reply, sign_result
from .recording import VideoRecorder, record_size

log = logging.getLogger(__name__)

SETTLE_STEPS = 10  # after `done`, hold still this many steps before judging success
MAX_REPEAT = 50
MAX_SKILL_STEPS = 150


def _hook(obj: Any, name: str) -> Callable | None:
    """An optional backend method, or None."""
    fn = getattr(obj, name, None)
    return fn if callable(fn) else None


class Episode:
    def __init__(
        self,
        spec: dict,
        run_dir: Path,
        workspace: Path | None = None,
        seed: int | None = None,
        secrets: dict | None = None,
    ):
        self.spec = spec
        self.run_dir = Path(run_dir)
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self.workspace = Path(workspace) if workspace else None
        self.backend = make_backend(spec)
        if attach := _hook(self.backend, "attach_run_dir"):  # real robots: session log next to the episode record
            attach(self.run_dir)
        self.mode = str(getattr(self.backend, "mode", "sim"))  # sim | real | hil-mock (the sim/real axis)
        self.seed = int(spec.get("seed", 0) if seed is None else seed)
        self.max_steps = int(spec.get("max_steps", getattr(self.backend, "max_steps", 500)))
        self.frame_every = int(spec.get("frame_every", 2))
        self.skills_enabled = bool(spec.get("skills", True))
        # "final": judged after `robo done` and a settle period (default).
        # "first": the episode ends as solved the first time the task's own success signal fires (Meta-World's protocol).
        self.success_mode = str(spec.get("success_mode", "final"))
        # "state" (default): observe returns all public state fields.
        # "vision": observe returns only `visible_fields` (the robot's own proprioception by default) plus camera images;
        #           requests carrying the per-episode oracle token (the reference solution's) still get the full state.
        self.obs_mode = str(spec.get("obs_mode", "state"))
        self.visible_fields = list(spec.get("visible_fields", ["hand_pos", "gripper_open"]))
        self.cameras = list(spec.get("cameras", [getattr(self.backend, "camera", "")]))
        # the reference solution's token and the key that signs result.json (from the runner, on the server's stdin);
        # a server started by another tool (a hub runtime) may get the token from its environment instead
        secrets = secrets or {}
        self._oracle_token = str(secrets.get("oracle_token") or config.env("ROBOUSE_ORACLE_TOKEN"))
        self._result_key = str(secrets.get("result_key") or "")
        self.max_repeat = max(
            1, min(MAX_REPEAT, int(spec.get("max_repeat", MAX_REPEAT)))
        )  # `act --repeat`, `grip --steps`
        # cameras whose projection matrix is not given to the agent (e.g. a camera that moves with the gripper)
        self.uncalibrated = set(spec.get("uncalibrated_cameras", []))
        self.backend.reset(self.seed)
        self._save_workspace_images()
        self.steps = 0
        self.requests = 0
        self.success_ever = False
        self.finished = False
        self.outcome: str | None = None
        self.agent_text = ""
        self.result: dict = {}
        self.t0 = time.time()
        self.record_size = record_size()
        self.video = VideoRecorder(self.run_dir, self.t0, exact=self.record_size is not None)
        self.probe = PhysicsProbe(self.backend, spec)
        self._trace = open(self.run_dir / "trace.jsonl", "w")  # noqa: SIM115 - open for the whole episode
        self._obs_count = 0
        self._record_frame(force=True)

    def _save_workspace_images(self) -> None:
        """Optional backend hook: images placed in the agent's workspace at the start (e.g. rule examples)."""
        hook = _hook(self.backend, "workspace_images")
        imgs = hook() if hook else {}
        if not imgs:
            return
        from PIL import Image

        for d in [self.run_dir] + ([self.workspace] if self.workspace else []):
            (d / "observations").mkdir(parents=True, exist_ok=True)
            for name, img in imgs.items():
                Image.fromarray(np.asarray(img, dtype=np.uint8)).save(d / "observations" / name)

    # ---- recording ---------------------------------------------------------------------------------------------------
    def _record_frame(self, force: bool = False) -> None:
        if force or self.steps % self.frame_every == 0:
            self.video.add(self._video_image(), self.steps)

    def _video_image(self) -> np.ndarray:
        if self.record_size:
            img = None
            try:
                img = self.backend.render_record(*self.record_size)
            except Exception as e:  # noqa: BLE001 - a backend that cannot: keep recording at its native size
                log.warning(
                    "record size %s not available (%s: %s); recording at the native size",
                    self.record_size,
                    type(e).__name__,
                    e,
                )
            if img is not None:
                return img
            self.record_size = None
            self.video.exact = False
        rv = _hook(self.backend, "render_video")  # a backend may record a different view than agents see
        return rv() if rv else self.backend.render()

    def _env_step(self, action: Any) -> Any:
        info = self.backend.step(action)
        self.steps += 1
        self.probe.sample(self.steps)
        self.success_ever = self.success_ever or info.success
        self._record_frame()
        if info.success and self.success_mode == "first" and not self.finished:
            self.finish("success_reached", "", settle=False)
        return info

    def _state(self, privileged: bool = False) -> dict:
        hook = _hook(self.backend, "observe_privileged")  # reference-solution hints (oracle token only)
        st = hook() if privileged and hook else self.backend.observe()
        if self.obs_mode == "vision" and not privileged:
            st = {k: v for k, v in st.items() if k in self.visible_fields}
        if privileged and (ps := _hook(self.backend, "privileged_state")):  # what only the reference solution sees
            st = {**st, **ps()}
        return st

    def camera_info(self) -> list[dict]:
        """Per camera: a 3x4 projection matrix P with [u*w, v*w, w] = P @ [x, y, z, 1] in the saved image's pixels."""
        if hook := _hook(self.backend, "camera_info"):  # backends that are not MuJoCo compute their own calibration
            return [
                c if c.get("name") not in self.uncalibrated else {**c, "projection": None, "position": None}
                for c in hook(self.cameras)
            ]
        m_d = getattr(self.backend, "mj_model_data", lambda: None)()
        if not m_d:
            return []
        model, data = m_d
        out = []
        for name in self.cameras:
            h, w = self._render(name).shape[:2]
            cam = mujoco_camera(
                model,
                data,
                name,
                w,
                h,
                image_flipped=bool(getattr(self.backend, "image_flipped", False)),
                calibrated=name not in self.uncalibrated,
            )
            if cam is not None:
                out.append(cam)
        return out

    def _render(self, camera: str | None = None) -> np.ndarray:
        if not camera or camera == getattr(self.backend, "camera", None):
            return self.backend.render()
        old = self.backend.camera
        try:
            self.backend.camera = camera
            return self.backend.render()
        finally:
            self.backend.camera = old

    # ---- request handling --------------------------------------------------------------------------------------------
    def handle(self, req: dict) -> dict:
        # `requests` in the result counts the agent's requests up to the one that ended the episode; the runner's own
        # (its status check and its give-up after the agent exits) are not the agent's
        if not (self.finished or is_runner_request(req)):
            self.requests += 1
        privileged = bool(self._oracle_token) and req.get("token") == self._oracle_token
        req = {k: v for k, v in req.items() if k != "token"}  # never write the token to the trace
        op = req.get("op")
        t = round(time.time() - self.t0, 3)
        self._check_halt()
        try:
            resp = self._dispatch(op, req, privileged)
        except Exception as e:  # noqa: BLE001 - a bounded error goes back to the agent
            resp = {"ok": False, "error": clip(f"{type(e).__name__}: {e}", 500)}
        self._check_halt()
        if not self.finished and self.steps >= self.max_steps:
            self.finish("budget_exhausted", "")
        if self.finished and op in MOTION_OPS and resp.get("ok") and isinstance(resp.get("result"), dict):
            resp["result"]["episode"] = f"finished: {FINISH_MESSAGES.get(self.outcome or '', self.outcome)}"
        self._trace.write(json.dumps({"t": t, "steps": self.steps, "req": req, "resp": _strip(resp)}) + "\n")
        self._trace.flush()
        return resp

    def _dispatch(self, op: Any, req: dict, privileged: bool) -> dict:
        if self.finished and op not in ("info", "status"):
            return {"ok": False, "error": f"episode finished ({self.outcome})"}
        if op == "info":
            return {"ok": True, "result": self.info()}
        if op == "status":
            return {"ok": True, "result": {"finished": self.finished, "outcome": self.outcome, "steps": self.steps}}
        if op == "observe":
            return {"ok": True, "result": self.observe(bool(req.get("image", False)), privileged, req.get("camera"))}
        if op == "act":
            return self.act(req.get("action"), int(req.get("repeat", 1)))
        if op == "move_to":
            return self.move_to(
                req.get("pos"), req.get("grip"), int(req.get("max_steps", 100)), float(req.get("tol", 0.01))
            )
        if op == "grip":
            return self.grip(float(req.get("value", 1.0)), int(req.get("steps", 15)))
        if op == "skill":
            return self.skill(req.get("name"), req.get("args") or [])
        if op in ("done", "give_up"):
            return {
                "ok": True,
                "result": self.finish("done" if op == "done" else "gave_up", str(req.get("text", ""))[:4000]),
            }
        return {"ok": False, "error": f"unknown op {op!r}"}

    def _check_halt(self) -> None:
        """Real robots: an e-stop, over-temperature or hardware fault ends the episode at once (no settle steps)."""
        halted = getattr(self.backend, "halted", None)
        if halted and not self.finished:
            self.finish("safety_stop", clip(halted, 500), settle=False)

    def info(self) -> dict:
        s = self.backend.action_spec
        extra = _hook(self.backend, "info_extra")
        return {
            "task": self.spec.get("id"),
            "backend": self.backend.name,
            "mode": self.mode,
            "action": {"names": s.names, "low": s.low, "high": s.high, "doc": s.doc},
            "skills": self.backend.skills() if self.skills_enabled else [],
            "steps_used": self.steps,
            "max_steps": self.max_steps,
            "observation_mode": self.obs_mode,
            **({"max_repeat": self.max_repeat} if "max_repeat" in self.spec else {}),
            **(extra() if extra else {}),
            **(
                {"visible_fields": self.visible_fields, "cameras": self.camera_info()}
                if self.obs_mode == "vision"
                else {}
            ),
            **self._embodiment_info(),
        }

    def _embodiment_info(self) -> dict:
        try:
            return {"embodiment": embodiment_dict(self.backend)}
        except Exception as e:  # noqa: BLE001 - a declaration bug must not break `robo info`
            log.warning("embodiment of %s: %s: %s", self.backend.name, type(e).__name__, e)
            return {}

    def observe(self, image: bool, privileged: bool = False, camera: str | None = None) -> dict:
        out: dict[str, Any] = {"state": self._state(privileged), "steps_used": self.steps, "max_steps": self.max_steps}
        if image or self.obs_mode == "vision":
            from PIL import Image

            cams = [camera] if camera in self.cameras else (self.cameras if self.obs_mode == "vision" else [None])
            paths = []
            for cam in cams:
                self._obs_count += 1
                img = Image.fromarray(np.asarray(self._render(cam), dtype=np.uint8))
                name = f"obs_{self._obs_count:03d}" + (f"_{str(cam).replace('/', '_')}" if cam else "") + ".png"
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

    def _budget_left(self) -> bool:
        return self.steps < self.max_steps and not self.finished

    def act(self, action: Any, repeat: int) -> dict:
        spec = self.backend.action_spec
        if (
            not isinstance(action, list)
            or len(action) != spec.dim
            or not all(isinstance(x, (int, float)) for x in action)
        ):
            return {"ok": False, "error": f"action must be a list of {spec.dim} numbers {spec.names}"}
        if not all(np.isfinite(x) for x in action):
            return {"ok": False, "error": "action values must be finite numbers (no nan or inf)"}
        n = 0
        for _ in range(max(1, min(self.max_repeat, repeat))):
            if not self._budget_left():
                break
            self._env_step(action)
            n += 1
        return {"ok": True, "result": {"executed_steps": n, "state": self._state(), "steps_used": self.steps}}

    def move_to(self, pos: Any, grip: Any, max_steps: int, tol: float) -> dict:
        if not self.skills_enabled or "move_to" not in self.backend.skills():
            return {"ok": False, "error": "move_to is not available for this task; use act"}
        if not isinstance(pos, list) or len(pos) != 3 or not all(_finite(x) for x in pos):
            return {"ok": False, "error": "pos must be [x, y, z], finite numbers"}
        if grip is not None and not _finite(grip):
            return {"ok": False, "error": "grip must be a finite number"}
        if not _finite(tol) or tol <= 0:
            return {"ok": False, "error": "tol must be a positive number"}
        target = np.asarray(pos, dtype=float)
        g = float(getattr(self.backend, "_grip", -1.0) if grip is None else grip)
        n, stalled = 0, False
        recent: list[np.ndarray] = []
        for _ in range(max(1, min(MAX_SKILL_STEPS, max_steps))):
            if not self._budget_left():
                break
            hp = self.backend.hand_pos()  # type: ignore[attr-defined]  # move_to is only offered by arms
            d = target - hp
            if float(np.linalg.norm(d)) < tol:
                break
            recent.append(hp)
            if len(recent) > 8 and float(np.linalg.norm(recent[-1] - recent[-9])) < 0.002:
                stalled = True  # the hand has stopped moving: the target is out of reach or blocked
                break
            self._env_step(list(np.clip(d * 10.0, -1, 1)) + [g])
            n += 1
        dist = float(np.linalg.norm(target - self.backend.hand_pos()))  # type: ignore[attr-defined]
        res = {
            "reached": dist < tol * 1.5,
            "distance": round(dist, 4),
            "executed_steps": n,
            "state": self._state(),
            "steps_used": self.steps,
        }
        if stalled:
            res["stalled"] = "the hand stopped moving before reaching the target (out of reach or blocked)"
        return {"ok": True, "result": res}

    def grip(self, value: float, steps: int) -> dict:
        if not self.skills_enabled or "grip" not in self.backend.skills():
            return {"ok": False, "error": "grip is not available for this task; use act"}
        if not _finite(value):
            return {"ok": False, "error": "the grip value must be a finite number"}
        n = 0
        for _ in range(max(1, min(self.max_repeat, steps))):
            if not self._budget_left():
                break
            self._env_step([0.0, 0.0, 0.0, max(-1.0, min(1.0, value))])
            n += 1
        return {"ok": True, "result": {"executed_steps": n, "state": self._state(), "steps_used": self.steps}}

    def skill(self, name: Any, args: Any) -> dict:
        """A named skill. Embodiment backends (`start_skill`) run it closed-loop, step by step; others run it in one go
        (`run_skill`, e.g. BEHAVIOR's symbolic primitives), counted as one step, with the frames rendered meanwhile
        from `pop_frames()`."""
        if not self.skills_enabled:
            return {"ok": False, "error": "no named skills in this task; see `robo info`"}
        if not isinstance(name, str) or not isinstance(args, list):
            return {"ok": False, "error": "skill needs a name and a list of arguments"}
        if start := _hook(self.backend, "start_skill"):
            return self._closed_loop_skill(start, name, args)
        run = _hook(self.backend, "run_skill")
        if run is None:
            return {"ok": False, "error": "no named skills in this task; see `robo info`"}
        if self.steps >= self.max_steps:
            return {"ok": False, "error": "step budget exhausted"}
        out = run(name, [str(a) for a in args])
        self.steps += 1
        pop = _hook(self.backend, "pop_frames")
        for fr in pop() if pop else []:
            self.video.add(fr, self.steps)
        self._record_frame(force=True)
        return {"ok": True, "result": {**out, "steps_used": self.steps, "max_steps": self.max_steps}}

    def _closed_loop_skill(self, start: Callable[[str, list[str]], Generator], name: str, args: list) -> dict:
        """A skill as a generator of one action per control step; each runs as an ordinary step (budget, video, success
        checks) until the skill returns."""
        try:
            gen = start(name, [str(a) for a in args])
        except ValueError as e:
            return {"ok": False, "error": str(e)}
        n, out = 0, {}
        while True:
            if not self._budget_left():
                gen.close()
                out = {"stopped": "step budget exhausted" if self.steps >= self.max_steps else "episode finished"}
                break
            try:
                action = next(gen)
            except StopIteration as e:
                out = e.value or {}
                break
            except Exception as e:  # noqa: BLE001 - a skill that fails part-way reports it; its steps still count
                gen.close()
                out = {"error": clip(f"{type(e).__name__}: {e}", 300)}
                break
            self._env_step(action)
            n += 1
        return {
            "ok": True,
            "result": {
                "skill": name,
                **out,
                "executed_steps": n,
                "state": self._state(),
                "steps_used": self.steps,
                "max_steps": self.max_steps,
            },
        }

    # ---- judging -----------------------------------------------------------------------------------------------------
    def _hold(self) -> list[float]:
        if hold := _hook(self.backend, "hold_action"):  # embodiment backends say what "hold still" is
            return list(hold())
        dim = self.backend.action_spec.dim
        return [0.0] * (dim - 1) + [float(getattr(self.backend, "_grip", 0.0))] if dim >= 1 else []

    def _verdict(self, outcome: str, text: str) -> tuple[bool, str | None]:
        """(success, judge error). Only an explicit `robo done` is judged, unless the backend has its own `judge` (e.g.
        safety tasks, where refusing is rewarded) or success counts the first time it happens."""
        try:
            if judge := _hook(self.backend, "judge"):
                return bool(judge(outcome, text)), None
            if self.success_mode == "first":
                return bool(self.success_ever), None
            return outcome == "done" and bool(self.backend.success()), None
        except Exception as e:  # noqa: BLE001 - a simulator that died cannot be judged: score 0 and record why
            return False, clip(f"{type(e).__name__}: {e}", 300)

    def finish(self, outcome: str, text: str, settle: bool = True) -> dict:
        if self.finished:
            return {"outcome": self.outcome}
        if getattr(self.backend, "halted", None):
            settle = False
        hold = self._hold()
        for _ in range(SETTLE_STEPS if settle else 0):  # settle; success must still hold afterwards
            try:
                self.backend.step(hold)
            except Exception:  # noqa: BLE001 - a backend that cannot step further is judged on its current state
                break
            self._record_frame(force=True)
        success, judge_error = self._verdict(outcome, text)
        self.finished, self.outcome, self.agent_text = True, outcome, text
        b = self.backend
        summary = _hook(b, "session_summary")
        self.result = {
            "task": self.spec.get("id"),
            "seed": self.seed,
            "success": success,
            "success_ever": self.success_ever,
            "success_mode": self.success_mode,
            "outcome": outcome,
            "steps_used": self.steps,
            "max_steps": self.max_steps,
            "requests": self.requests,
            "agent_text": text,
            "wall_time_s": round(time.time() - self.t0, 2),
            "embodiment_mode": self.mode,
            **({"safety_events": list(events)} if (events := getattr(b, "events", None)) else {}),
            **({"session": summary()} if summary else {}),
            **({"judge_error": judge_error} if judge_error else {}),
            **({"placement": placement} if (placement := getattr(b, "placement_record", None)) else {}),
            **({"judge_detail": detail} if (detail := getattr(b, "last_judge", None)) else {}),
        }
        self._add_metrics()
        keep_video = self.steps > 0  # no step ran: no recording (it would be a zero-length video)
        if self.video.size and keep_video:
            self.result["video_size"] = list(self.video.size)
        if self._result_key:
            self.result["signature"] = sign_result(self.result, self._result_key)
        (self.run_dir / "result.json").write_text(json.dumps(self.result, indent=2))
        self.video.close(keep=keep_video)
        return {"outcome": outcome, "episode": "finished"}

    def _add_metrics(self) -> None:
        b = self.backend
        try:
            from .metrics import episode_metrics

            prog = progress() if (progress := _hook(b, "progress")) else None
            stages = len(getattr(b, "stages", None) or []) or None
            extra = em() if (em := _hook(b, "episode_metrics")) else None
            self.result["metrics"] = episode_metrics(
                self.result, self.probe.samples, self.probe.control_dt(), prog, stages, extra
            )
            self.probe.write(self.run_dir / "physics.jsonl")
        except Exception as e:  # noqa: BLE001 - metrics never change the verdict
            self.result["metrics_error"] = clip(f"{type(e).__name__}: {e}", 300)

    def close(self) -> None:
        if not self.finished:
            self.finish("agent_exited", "")
        self._trace.close()
        self.backend.close()


def _finite(x: Any) -> bool:
    return isinstance(x, int | float) and not isinstance(x, bool) and bool(np.isfinite(x))


def _strip(resp: dict) -> dict:
    """The response without the raw observation vector, to keep the trace readable."""
    r = json.loads(json.dumps(resp))
    st = (r.get("result") or {}).get("state") if isinstance(r.get("result"), dict) else None
    if isinstance(st, dict):
        st.pop("obs_vector", None)
    return r


def serve(
    spec: dict,
    run_dir: Path,
    sock_path: str,
    workspace: Path | None = None,
    max_wall_s: float = 1800,
    ready_file: Path | None = None,
    secrets: dict | None = None,
) -> dict:
    """Run one episode on a Unix socket until it finishes (plus `linger_s`) or the runner sends `shutdown`. `secrets`:
    the reference solution's token and the key that signs result.json."""
    ep = Episode(spec, run_dir, workspace, secrets=secrets)
    if os.path.exists(sock_path):
        os.unlink(sock_path)
    srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    srv.bind(sock_path)
    srv.listen(8)
    srv.settimeout(1.0)
    if ready_file:
        Path(ready_file).write_text(sock_path)
    deadline = time.time() + max_wall_s
    stop_after: float | None = None
    try:
        while True:
            if time.time() > deadline and not ep.finished:
                ep.finish("wall_time_exhausted", "")
            if ep.finished and stop_after is None:
                stop_after = time.time() + float(spec.get("linger_s", 5))
            if stop_after and time.time() > stop_after:
                break
            try:
                conn, _ = srv.accept()
            except TimeoutError:
                continue
            with conn:
                conn.settimeout(10)
                try:
                    req = read_request(conn)
                    if req.get("op") == "shutdown":
                        send_reply(conn, {"ok": True})
                        break
                    resp = ep.handle(req)
                except Exception as e:  # noqa: BLE001 - a malformed request gets an error reply
                    resp = {"ok": False, "error": clip(f"bad request: {e}", 300)}
                send_reply(conn, resp)  # a client that went away does not end the episode
    finally:
        ep.close()
        srv.close()
        if os.path.exists(sock_path):
            os.unlink(sock_path)
    return ep.result
