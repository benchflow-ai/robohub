"""Run trials: task x harness x model -> a BenchFlow trial directory (what the BenchFlow viewer reads).

<job>/<task>__<harness>__<id>/     (<id>: 6 random hex characters)
  config.json                 task, harness, model, seed, timeouts
  result.json                 result (reward, timing, agent info, exception)
  agent/stdout.jsonl          raw harness output (stream-json / codex --json / text)
  agent/stderr.txt
  agent/trajectory.json       ATIF trajectory converted from the raw output (see atif.py)
  agent/mini_traj.json        mini-swe-agent's own trajectory (mini-swe-agent-glm only)
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
import time
import uuid
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

from .. import harnesses
from ..core.protocol import RUNNER_PREFIX, verify_result
from ..tasks import Task
from . import sandbox
from .cost import trial_cost

AGENT_CLI = Path(__file__).resolve().parents[1] / "agent_cli.py"
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


def _make_workspace(task: Task) -> Path:
    ws = Path(tempfile.mkdtemp(prefix=f"robouse-ws-{task.id[:24]}-"))
    (ws / ".robouse" / "bin").mkdir(parents=True)
    shim = ws / ".robouse" / "bin" / "robo"
    shim.write_text("#!/usr/bin/env python3\n" + AGENT_CLI.read_text())
    shim.chmod(0o755)
    (ws / "instruction.md").write_text(task.instruction)
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
                last = f": {lines[-1].strip()[:300]}" if lines else ""
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
) -> dict:
    job_dir.mkdir(parents=True, exist_ok=True)
    if split or extra_instruction or prior_learnings:
        task = _variant(task, split, extra_instruction, prior_learnings)
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
    cfg = {
        "task": task.id,
        "task_path": str(task.path),
        "harness": harness,
        "model": model,
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
    try:
        _wait_ready(
            ready, server, timeout=float(task.spec.get("ready_timeout_s", 300)), log=tdir / "episode" / "server.log"
        )
        (ws / ".robouse" / "socket").write_text(sock)
        base_env = {
            k: v
            for k, v in os.environ.items()
            if not k.startswith(("ANTHROPIC_", "CLAUDE_CODE_OAUTH", "OPENAI_", "BASETEN"))
        }
        base_env["PATH"] = (
            f"{ws / '.robouse' / 'bin'}:{Path(py).parent}:{base_env.get('PATH', '')}"
            if harness == "oracle"
            else f"{ws / '.robouse' / 'bin'}:{base_env.get('PATH', '')}"
        )
        base_env["ROBOUSE_SOCKET"] = sock
        base_env["PYTHONPATH"] = str(SRC) if harness == "oracle" else ""
        base_env.pop("ROBOUSE_ORACLE_TOKEN", None)
        if harness == "oracle":  # only the reference solution may see privileged state in vision tasks
            base_env["ROBOUSE_ORACLE_TOKEN"] = oracle_token
        launch = harnesses.build(harness, model, task.instruction, ws, task, base_env)
        if harness in ("oracle", "noop"):  # the trusted reference solution and the negative control
            cfg["sandbox"] = "none"
        else:
            ports = tuple(
                int(m) for v in launch.env.values() for m in re.findall(r"//(?:127\.0\.0\.1|localhost):(\d+)", str(v))
            )
            launch.cmd, cfg["sandbox"] = sandbox.wrap(launch.cmd, ws, sock, [job_dir, task.path], ports)
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
            try:
                agent_rc = agent.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                timed_out = True
                os.killpg(agent.pid, signal.SIGTERM)
                try:
                    agent.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(agent.pid, signal.SIGKILL)
    except Exception as e:  # noqa: BLE001 - an infrastructure failure, recorded in result.json
        exc = f"{type(e).__name__}: {e}"
    finally:
        # close the episode if the agent did not call `robo done`
        from ..agent_cli import _send

        try:
            st = _send({"op": "status"}, path=sock)
        except OSError:  # the server is still busy (e.g. a slow remote simulator step); it ends on its own clock
            st = {}
        if st.get("ok") and not st["result"]["finished"]:
            _send(
                {
                    "op": "give_up",
                    "text": f"{RUNNER_PREFIX} agent timed out"
                    if timed_out
                    else f"{RUNNER_PREFIX} agent exited without robo done",
                },
                path=sock,
            )
        try:
            server.wait(timeout=60)
        except subprocess.TimeoutExpired:
            server.kill()

    episode, verified = _verdict(tdir / "episode" / "result.json", result_key)
    if not verified and exc is None:
        exc = "VerdictError: the episode's result.json is missing or does not carry the server's signature"
    if timed_out and episode.get("outcome") == "gave_up":
        episode["outcome"] = "agent_timeout"
    reward = _verify(task, episode, tdir) if verified else 0.0

    # artifacts: video + observation images the agent saw
    if (tdir / "episode" / "recording.mp4").exists():
        shutil.copy(tdir / "episode" / "recording.mp4", tdir / "artifacts" / "recording.mp4")
    for f in sorted((ws / "observations").glob("*.png")):
        (tdir / "artifacts" / "observations").mkdir(exist_ok=True)
        shutil.copy(f, tdir / "artifacts" / "observations" / f.name)
    if harness == "mini-swe-agent-glm" and (ws / ".robouse" / "mini_traj.json").exists():
        shutil.copy(ws / ".robouse" / "mini_traj.json", tdir / "agent" / "mini_traj.json")

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
        "source": task.metadata.get("source_benchmark"),
        "agent_info": {"name": harness, "model_info": {"name": (launch.notes.get("model") if launch else model)}},
        "verifier_result": {"rewards": {"reward": reward}},
        "episode": episode,
        "agent_return_code": agent_rc,
        "agent_timed_out": timed_out,
        "exception_info": {"exception_type": exc.split(":")[0], "exception_message": exc} if exc else None,
        "started_at": started,
        "finished_at": now(),
        "wall_time_s": round(time.time() - t0, 1),
        "embodiment_mode": episode.get("embodiment_mode", "sim"),
        "cost": cost,
    }
    if launch and isinstance(launch.helper, dict):
        res["proxy_stats"] = launch.helper
    (tdir / "result.json").write_text(json.dumps(res, indent=2))
    shutil.rmtree(ws, ignore_errors=True)
    return {
        "trial": trial_name,
        "reward": reward,
        "outcome": episode.get("outcome"),
        "steps": episode.get("steps_used"),
        "wall_s": res["wall_time_s"],
        "exception": exc,
    }
