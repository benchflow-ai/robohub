"""Run trials: task x harness x model -> a BenchFlow trial directory (what the BenchFlow viewer reads).

<job>/<task>__<harness>__<id>/     (<id>: 6 random hex characters)
  config.json                 task, harness, model, seed, timeouts, and the files the agent's workspace held at start
  result.json                 result (reward, timing, agent info, exception)
  agent/stdout.jsonl          raw harness output (stream-json / codex --json / text)
  agent/stderr.txt
  agent/trajectory.json       ATIF trajectory converted from the raw output (see atif.py)
  agent/mini_traj.json        mini-swe-agent's own trajectory (mini-swe-agent only)
  episode/                    trusted episode server output (result.json, trace.jsonl, frames.jsonl)
  verifier/reward.txt         0 or 1, written by the task's verifier/test.sh
  verifier/test-stdout.txt
  artifacts/recording.mp4     episode video
  artifacts/recording_index.json   video time for each trajectory step
"""

from __future__ import annotations

import json
import os
import re
import secrets
import shutil
import signal
import subprocess
import sys
import tempfile
import threading
import time
import uuid
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

from .. import harnesses
from ..core.protocol import RUNNER_PREFIX, verify_result
from ..harnesses import interface as iface
from ..lib.text import clip
from ..tasks import Task
from . import sandbox
from .cost import trial_cost

AGENT_CLI = Path(__file__).resolve().parents[1] / "agent_cli.py"
MCP_SERVER = Path(__file__).resolve().parents[1] / "mcp_server.py"
SRC = Path(__file__).resolve().parents[2]  # the folder holding the robouse package


def provenance() -> dict:
    """What produced this trial: Robo Use version and git revision, Python, key simulator package versions."""
    import importlib.metadata as md
    import platform

    from .. import __version__

    out: dict[str, Any] = {"robouse": __version__, "python": platform.python_version(), "platform": platform.platform()}
    try:
        root = SRC.parent
        rev = subprocess.run(["git", "-C", str(root), "rev-parse", "HEAD"], capture_output=True, text=True, timeout=5)
        dirty = subprocess.run(
            ["git", "-C", str(root), "status", "--porcelain", "--untracked-files=no"],
            capture_output=True,
            text=True,
            timeout=5,
        )
        if rev.returncode == 0:
            out["git_commit"] = rev.stdout.strip()
            out["git_dirty"] = bool(dirty.stdout.strip())
    except Exception:  # noqa: BLE001
        pass
    for pkg in ("mujoco", "numpy", "metaworld", "robosuite", "gymnasium-robotics"):
        try:
            out[pkg] = md.version(pkg)
        except md.PackageNotFoundError:
            pass
    return out


def _variant(task: Task, split: str | None, extra: str, learnings: str | None) -> Task:
    """The same task with a placement split, an extra instruction or prior learnings appended (recorded in config.json)."""
    import copy

    meta = copy.deepcopy(task.meta)
    body = task.body
    if split:
        meta.setdefault("robouse", {})["placement"] = {**(meta["robouse"].get("placement") or {}), "split": split}
    if learnings:
        body += "\n\n## Notes from earlier attempts\n\n" + Path(learnings).read_text().strip() + "\n"
    if extra:
        body += "\n\n" + extra.strip() + "\n"
    return Task(task.path, meta, body)


def now() -> str:
    return datetime.now(UTC).isoformat()


def _workspace_files(ws: Path, limit: int = 500) -> list[str]:
    """What a workspace holds, relative to it, folders with a trailing slash (the `robo` client in .robouse/bin, the
    socket path, harness files, observations/ and the images a task starts with), at most `limit` entries."""
    files = sorted(str(p.relative_to(ws)) + ("/" if p.is_dir() else "") for p in ws.rglob("*"))
    return files[:limit] + ([f"... {len(files) - limit} more"] if len(files) > limit else [])


def _make_workspace(task: Task) -> Path:
    ws = Path(tempfile.mkdtemp(prefix=f"robouse-ws-{task.id[:24]}-"))
    (ws / ".robouse" / "bin").mkdir(parents=True)
    shim = ws / ".robouse" / "bin" / "robo"
    shim.write_text("#!/usr/bin/env python3\n" + AGENT_CLI.read_text())
    shim.chmod(0o755)
    # `robo mcp` (the MCP front end) and the client module it builds requests with, next to the shim
    (ws / ".robouse" / "bin" / "robo_mcp.py").write_text(MCP_SERVER.read_text())
    (ws / ".robouse" / "bin" / "robo_cli.py").write_text(AGENT_CLI.read_text())
    (ws / "observations").mkdir()
    return ws


def _wait_ready(p: Path, proc: subprocess.Popen, timeout: float = 120, log: Path | None = None) -> None:
    t = time.time() + timeout
    while time.time() < t:
        if p.exists():
            return
        if proc.poll() is not None:
            last = ""
            if (
                log is not None and log.exists()
            ):  # the server's last line usually names the cause (e.g. a missing simulator)
                lines = [l for l in log.read_text(errors="replace").splitlines() if l.strip()]
                last = f": {clip(lines[-1], 1000)}" if lines else ""
            raise RuntimeError(f"episode server exited early ({proc.returncode}){last}")
        time.sleep(0.2)
    raise TimeoutError("episode server did not start")


def _verdict(path: Path, key: str) -> tuple[dict, bool]:
    """The episode server's result.json and whether it carries the server's signature (the agent cannot forge it)."""
    try:
        episode = json.loads(path.read_text())
    except (OSError, ValueError):
        return {}, False
    return episode, verify_result(episode, key)


def _verify(task: Task, episode: dict, tdir: Path) -> float:
    """The task's verifier/test.sh turns the verdict into reward.txt. It reads a private copy of the checked verdict
    (outside the trial folder), and a reward that contradicts the verdict is not taken."""
    with tempfile.TemporaryDirectory(prefix="robouse-verdict-") as private:
        (Path(private) / "result.json").write_text(json.dumps(episode))
        vt = subprocess.run(
            ["bash", str(task.verifier_script)],
            capture_output=True,
            text=True,
            env={**os.environ, "ROBOUSE_EPISODE_DIR": private, "ROBOUSE_VERIFIER_DIR": str(tdir / "verifier")},
            check=False,
        )
    (tdir / "verifier" / "test-stdout.txt").write_text(vt.stdout + vt.stderr)
    rf = tdir / "verifier" / "reward.txt"
    reward = float(rf.read_text().strip()) if rf.exists() and rf.read_text().strip() else 0.0
    if reward > 0 and not episode.get("success"):
        (tdir / "verifier" / "test-stdout.txt").write_text(
            vt.stdout + vt.stderr + "\nreward ignored: the episode server's verdict is not a success\n"
        )
        return 0.0
    return reward


# harnesses that drive a language model (the others run scripted code or a VLA policy)
LLM_HARNESSES = ("claude-code", "codex", "mini-swe-agent", "dimcode")


def image_input_plan(harness: str, model: str) -> dict:
    """The canonical harness and model of a trial, and whether its model gets image input (`image_input`), with the
    reason. Every task is open to every harness and model; this only records what the model could see.

    No image input when the harness never passes images to its model (mini-swe-agent), the model registry lists the
    model as text-only or does not describe it (harnesses/models.py), or ROBOUSE_AGENT_IMAGES=0 turns images off for
    the run. The trial's model gateway then replaces every image part before it reaches the provider. The task itself
    is unchanged: the agent's workspace still holds the camera images as files (code may read their pixels), and a
    vision task never falls back to the object state it hides. Harnesses without a language model record None."""
    from .. import config
    from ..harnesses import models

    canon, mid = models.legacy(harness, model)
    if canon not in LLM_HARNESSES:
        return {"harness": canon, "model": mid, "image_input": None, "image_input_reason": "no language model"}
    mid = mid or models.DEFAULT_MODELS.get(canon, "")
    ref = models.resolve(canon, mid)
    accepts = models.accepts_images(ref)
    plan = {"harness": canon, "model": ref.id if models.split(mid)[0] else mid, "accepts_images": accepts}
    if canon in models.NO_IMAGE_HARNESSES:
        plan.update(image_input=False, image_input_reason=f"{canon} passes no images to its model")
    elif accepts is not True:
        why = (
            "the model registry lists it as text-only"
            if accepts is False
            else "the model registry lists no image input"
        )
        plan.update(image_input=False, image_input_reason=why)
    elif not config.flag("ROBOUSE_AGENT_IMAGES", default=True):
        plan.update(image_input=False, image_input_reason="images turned off for this run (ROBOUSE_AGENT_IMAGES=0)")
    else:
        plan.update(image_input=True, image_input_reason="the model takes images")
    if not plan["image_input"] and models.NATIVE_VENDORS.get(canon) == ref.provider:
        # a harness on its own vendor has no gateway, so nothing could remove the images
        raise ValueError(f"{canon} with {ref.id} cannot run without image input: it does not use the model gateway")
    return plan


def spend_cap_usd() -> float:
    """The per-trial model spend cap in USD (ROBOUSE_SPEND_CAP_USD, default 5; 0 turns it off). The model gateway
    enforces it; harnesses on their own vendor (no gateway) are not capped."""
    from .. import config

    return max(0.0, float(config.env("ROBOUSE_SPEND_CAP_USD", "5")))


def _secret_env_names() -> set[str]:
    from ..harnesses import models

    return models.secret_env_names()


def _wait_agent(agent: subprocess.Popen, timeout: float, stop_file: Path | None) -> tuple[int | None, bool]:
    """(return code, timed out). Waits for the agent; stops it at the timeout, or as soon as its model gateway writes
    `stop_file` (the trial's spend cap is reached)."""
    deadline = time.time() + timeout
    while True:
        try:
            return agent.wait(timeout=max(0.0, min(1.0, deadline - time.time()))), False
        except subprocess.TimeoutExpired:
            if stop_file is not None and stop_file.exists():
                _kill_group(agent)
                return agent.returncode, False
            if time.time() >= deadline:
                _kill_group(agent)
                return None, True


def _kill_group(agent: subprocess.Popen) -> None:
    for sig, wait in ((signal.SIGTERM, 15), (signal.SIGKILL, None)):
        try:
            os.killpg(agent.pid, sig)
        except OSError:  # the group is gone already
            pass
        try:
            agent.wait(timeout=wait)
            return
        except subprocess.TimeoutExpired:
            continue


def _read_budget_stop(stop_file: Path | None) -> dict | None:
    """The gateway's record of a spend-cap stop ({cap_usd, spent_usd, ...}), or None."""
    if stop_file is None or not stop_file.exists():
        return None
    try:
        return json.loads(stop_file.read_text())
    except (OSError, ValueError):
        return {"cap_usd": None, "spent_usd": None}


# Ctrl-C: the agents of the trials running now (their process groups), so an interrupt stops them all
_ACTIVE_AGENTS: set[int] = set()
INTERRUPTED = threading.Event()
INTERRUPT_TEXT = "Cancelled: the run was interrupted (Ctrl-C) before the trial finished"


def interrupt_all() -> None:
    """Stop every running agent; their trials are recorded as interrupted (an infrastructure failure, rerun by
    `run-many --resume`)."""
    INTERRUPTED.set()
    for pid in list(_ACTIVE_AGENTS):
        try:
            os.killpg(pid, signal.SIGTERM)
        except OSError:
            pass


def _harness_error(tdir: Path, harness: str, agent_rc: int | None, episode: dict) -> str | None:
    """An agent that never ended the episode itself (no `robo done` or `robo give-up`) and whose harness reported an
    error: a non-zero exit, or Claude Code's error result event (an API or credential failure). That is a failed run of
    the harness, not a result of the model."""
    if harness == "noop" or not (
        episode.get("outcome") == "gave_up" and str(episode.get("agent_text", "")).startswith(RUNNER_PREFIX)
    ):
        return None
    detail = ""
    try:
        for line in (tdir / "agent" / "stdout.jsonl").read_text(errors="replace").splitlines()[::-1]:
            ev = json.loads(line) if line.startswith("{") else None
            if isinstance(ev, dict) and ev.get("type") == "result":
                if ev.get("is_error"):
                    detail = clip(ev.get("result") or ev.get("subtype") or "error result", 300)
                break
    except (OSError, ValueError):
        pass
    if agent_rc not in (0, None):
        return f"AgentError: the {harness} harness exited with code {agent_rc} without ending the episode" + (
            f" ({detail})" if detail else ""
        )
    if detail:
        return f"AgentError: the {harness} harness reported an error without ending the episode ({detail})"
    return None


def run_trial(
    task: Task,
    harness: str,
    model: str,
    job_dir: Path,
    timeout: float | None = None,
    seed: int | None = None,
    trial_name: str | None = None,
    epoch: int = 0,
    split: str | None = None,
    extra_instruction: str = "",
    prior_learnings: str | None = None,
    attempt: int = 0,
    interface: str | None = None,
) -> dict:
    plan = image_input_plan(harness, model)
    cap = spend_cap_usd()
    # recorded and launched under the canonical names: harness = the agent program, model = provider-prefixed id
    requested = (harness, model)
    harness, model = plan["harness"], plan["model"] or ""
    job_dir.mkdir(parents=True, exist_ok=True)
    if split or extra_instruction or prior_learnings:
        task = _variant(task, split, extra_instruction, prior_learnings)
    interface = iface.resolve(task, interface)
    iface.check(harness, interface)
    trial_name = trial_name or f"{task.id}__{harness}__{uuid.uuid4().hex[:6]}"
    tdir = job_dir / trial_name
    for sub in ("agent", "episode", "verifier", "artifacts"):
        (tdir / sub).mkdir(parents=True, exist_ok=True)
    timeout = float(timeout or task.agent_timeout_s)
    ws = _make_workspace(task)
    sock = f"/tmp/robouse-{uuid.uuid4().hex[:10]}.sock"
    ready = ws / ".robouse" / "ready"
    started = now()
    t0 = time.time()
    cfg: dict[str, Any] = {
        "task": task.id,
        "task_path": str(task.path),
        "harness": harness,
        "model": model,
        "interface": interface,
        "seed": seed,
        "agent_timeout_s": timeout,
        "trial_name": trial_name,
        "started_at": started,
        "workspace": str(ws),
        "metadata": task.metadata,
        "epoch": epoch,
        "split": split,
        "attempt": attempt,
        "extra_instruction": extra_instruction or None,
        "prior_learnings": prior_learnings,
        "requested": {"harness": requested[0], "model": requested[1]} if requested != (harness, model) else None,
        "image_input": plan["image_input"],
        "image_input_reason": plan["image_input_reason"],
        **({"accepts_images": plan["accepts_images"]} if "accepts_images" in plan else {}),
        "spend_cap_usd": None,  # set once the model gateway is up (harnesses on their own vendor are not capped)
        "provenance": provenance(),
    }
    (tdir / "config.json").write_text(json.dumps(cfg, indent=2))

    py = sys.executable
    server_cmd = [
        py,
        "-m",
        "robouse.cli",
        "serve",
        "--task",
        str(task.path),
        "--run-dir",
        str(tdir / "episode"),
        "--socket",
        sock,
        "--workspace",
        str(ws),
        "--ready-file",
        str(ready),
        "--max-wall-s",
        str(timeout + 60),
    ]
    if seed is not None:
        server_cmd += ["--seed", str(seed)]
    if split:
        server_cmd += ["--split", split]
    server_cmd.append("--secrets-stdin")
    # the reference solution's token and the key that signs the verdict go to the server on its stdin: never in an
    # environment or a command line, which other processes of this user can read
    oracle_token, result_key = secrets.token_hex(16), secrets.token_hex(32)
    server_env = {k: v for k, v in os.environ.items() if k != "ROBOUSE_ORACLE_TOKEN"}
    with open(tdir / "episode" / "server.log", "w") as server_log:  # the child keeps its own copy of the handle
        server = subprocess.Popen(
            server_cmd, env=server_env, stdin=subprocess.PIPE, stdout=server_log, stderr=subprocess.STDOUT
        )
    assert server.stdin is not None
    server.stdin.write(json.dumps({"oracle_token": oracle_token, "result_key": result_key}).encode() + b"\n")
    server.stdin.close()
    exc = None
    launch = None
    agent_rc = None
    timed_out = False
    budget_stop = None
    interrupted = False
    try:
        _wait_ready(
            ready, server, timeout=float(task.spec.get("ready_timeout_s", 300)), log=tdir / "episode" / "server.log"
        )
        (ws / ".robouse" / "socket").write_text(sock)
        # no provider key reaches an agent: a harness adds its own vendor's login back; others use the gateway
        secret_names = _secret_env_names()
        base_env = {
            k: v
            for k, v in os.environ.items()
            if not k.startswith(("ANTHROPIC_", "CLAUDE_CODE_OAUTH", "OPENAI_", "BASETEN", "OPENROUTER"))
            and k not in secret_names
        }
        base_env["PATH"] = (
            f"{ws / '.robouse' / 'bin'}:{Path(py).parent}:{base_env.get('PATH', '')}"
            if harness == "oracle"
            else f"{ws / '.robouse' / 'bin'}:{base_env.get('PATH', '')}"
        )
        base_env["ROBOUSE_SOCKET"] = sock
        base_env["_ROBOUSE_TRIAL_DIR"] = str(tdir)  # read and removed by the harness builder (model gateway)
        base_env["_ROBOUSE_IMAGE_INPUT"] = "0" if plan["image_input"] is False else "1"
        base_env["_ROBOUSE_SPEND_CAP_USD"] = str(cap)
        base_env["PYTHONPATH"] = str(SRC) if harness == "oracle" else ""
        base_env.pop("ROBOUSE_ORACLE_TOKEN", None)
        if harness == "oracle":  # only the reference solution may see privileged state in vision tasks
            base_env["ROBOUSE_ORACLE_TOKEN"] = oracle_token
        base_env["ROBOUSE_INTERFACE"] = interface
        launch = harnesses.build(harness, model, task.instruction, ws, task, base_env)
        if launch.stop_file is not None:  # a model gateway: it enforces the spend cap
            cfg["spend_cap_usd"] = cap or None
        iface.apply(harness, launch, interface, ws, sock)
        if harness in ("oracle", "noop"):  # the trusted reference solution and the negative control
            cfg["sandbox"] = "none"
        else:
            ports = tuple(
                int(m) for v in launch.env.values() for m in re.findall(r"//(?:127\.0\.0\.1|localhost):(\d+)", str(v))
            )
            launch.cmd, cfg["sandbox"] = sandbox.wrap(launch.cmd, ws, sock, [job_dir, task.path], ports)
        # what the agent was given besides its prompt (the task.md body): the files in its workspace
        cfg["workspace_files"] = _workspace_files(ws)
        (tdir / "config.json").write_text(json.dumps(cfg, indent=2))
        with open(tdir / "agent" / "stdout.jsonl", "w") as out, open(tdir / "agent" / "stderr.txt", "w") as err:
            agent = subprocess.Popen(
                launch.cmd,
                cwd=ws,
                env=launch.env,
                stdout=out,
                stderr=err,
                stdin=subprocess.DEVNULL,
                start_new_session=True,
            )
            _ACTIVE_AGENTS.add(agent.pid)
            try:
                if INTERRUPTED.is_set():  # started just as another trial was interrupted
                    raise KeyboardInterrupt
                # stops the agent at the timeout, or when its model gateway reaches the trial's spend cap
                agent_rc, timed_out = _wait_agent(agent, timeout, launch.stop_file)
            except KeyboardInterrupt:
                interrupted = True
                interrupt_all()
                try:
                    agent.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(agent.pid, signal.SIGKILL)
                    agent.wait()
            finally:
                _ACTIVE_AGENTS.discard(agent.pid)
    except KeyboardInterrupt:  # before the agent started (e.g. while the simulator loads)
        interrupted = True
        interrupt_all()
    except Exception as e:  # noqa: BLE001 - an infrastructure failure, recorded in result.json
        exc = f"{type(e).__name__}: {e}"
    finally:
        # close the episode if the agent did not call `robo done`
        from ..agent_cli import _send

        try:
            st = _send({"op": "status", "runner": True}, path=sock)
        except OSError:  # the server is still busy (e.g. a slow remote simulator step); it ends on its own clock
            st = {}
        if st.get("ok") and not st["result"]["finished"]:
            # the agent did not end the episode: a spend-cap stop when its gateway reached the cap
            budget_stop = _read_budget_stop(launch.stop_file if launch is not None else None)
            try:
                _send(
                    {
                        "op": "give_up",
                        "text": f"{RUNNER_PREFIX} spend cap reached"
                        if budget_stop
                        else f"{RUNNER_PREFIX} agent timed out"
                        if timed_out
                        else f"{RUNNER_PREFIX} agent exited without robo done",
                    },
                    path=sock,
                )
            except OSError:  # the server went away (e.g. it got the same Ctrl-C)
                pass
        try:
            server.wait(timeout=5 if interrupted or INTERRUPTED.is_set() else 60)
        except subprocess.TimeoutExpired:
            server.kill()
            server.wait()
        if launch is not None and launch.stop is not None:
            launch.stop()  # the model gateway: its request log is summarized into proxy_stats

    interrupted = interrupted or INTERRUPTED.is_set()
    episode, verified = _verdict(tdir / "episode" / "result.json", result_key)
    if interrupted:
        exc = INTERRUPT_TEXT
    if not verified and exc is None:
        exc = "VerdictError: the episode's result.json is missing or does not carry the server's signature"
    if exc is None and not timed_out and not budget_stop:  # a harness stopped at the spend cap did not fail
        exc = _harness_error(tdir, harness, agent_rc, episode)
    if timed_out and episode.get("outcome") == "gave_up":
        episode["outcome"] = "agent_timeout"
    if budget_stop and episode.get("outcome") in ("gave_up", "agent_timeout"):
        # stopped at the spend cap: neither a model failure nor an infrastructure one
        episode["outcome"] = "budget_stop"
    reward = _verify(task, episode, tdir) if verified else 0.0

    # artifacts: video + observation images the agent saw
    if (tdir / "episode" / "recording.mp4").exists():
        shutil.copy(tdir / "episode" / "recording.mp4", tdir / "artifacts" / "recording.mp4")
    for f in sorted((ws / "observations").glob("*.png")):
        (tdir / "artifacts" / "observations").mkdir(exist_ok=True)
        shutil.copy(f, tdir / "artifacts" / "observations" / f.name)
    if harness.startswith("mini-swe-agent") and (ws / ".robouse" / "mini_traj.json").exists():
        shutil.copy(ws / ".robouse" / "mini_traj.json", tdir / "agent" / "mini_traj.json")
    for src, dst in (launch.notes.get("collect") or []) if launch else []:  # a harness's own transcript
        found = sorted((ws / ".robouse" / src).rglob("*.jsonl"), key=lambda f: f.stat().st_mtime)
        if found:
            shutil.copy(found[-1], tdir / dst)

    try:
        from .atif import write_trajectory

        write_trajectory(tdir, harness)
    except Exception as e:  # noqa: BLE001 - trajectory conversion is best effort; the raw output is kept
        (tdir / "agent" / "trajectory_error.txt").write_text(f"{type(e).__name__}: {e}")

    cost = trial_cost(
        tdir, (launch.notes.get("model") if launch else model) or "", launch.notes.get("provider") if launch else None
    )
    res = {
        "task_name": task.id,
        "trial_name": trial_name,
        # completed: the trial ran to a result (any reward); error: an infrastructure failure, not scored; cancelled: the
        # run was interrupted. `run-many --resume` reruns error and cancelled trials
        "status": "cancelled" if interrupted else ("error" if exc else "completed"),
        "source": task.metadata.get("source_benchmark"),
        "agent_info": {
            "name": harness,
            "model_info": {
                "name": (launch.notes.get("model_id") or launch.notes.get("model") if launch else model),
                "provider": launch.notes.get("provider") if launch else None,
            },
            "interface": interface,
        },
        "verifier_result": {"rewards": {"reward": reward}},
        "episode": episode,
        "agent_return_code": agent_rc,
        "agent_timed_out": timed_out,
        "exception_info": {"exception_type": exc.split(":")[0], "exception_message": clip(exc, 4000)} if exc else None,
        "started_at": started,
        "finished_at": now(),
        "wall_time_s": round(time.time() - t0, 1),
        "embodiment_mode": episode.get("embodiment_mode", "sim"),
        "cost": cost,
        "image_input": plan["image_input"],
        "budget_stop": budget_stop,
    }
    if launch and isinstance(launch.helper, dict):
        res["proxy_stats"] = launch.helper
    (tdir / "result.json").write_text(json.dumps(res, indent=2))
    shutil.rmtree(ws, ignore_errors=True)
    if interrupted and threading.current_thread() is threading.main_thread():
        raise KeyboardInterrupt
    return {
        "trial": trial_name,
        "reward": reward,
        "outcome": episode.get("outcome"),
        "steps": episode.get("steps_used"),
        "wall_s": res["wall_time_s"],
        "exception": exc,
    }
