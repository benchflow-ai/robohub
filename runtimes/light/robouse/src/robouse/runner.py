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

import concurrent.futures as cf
import json
import os
import re
import shutil
import signal
import subprocess
import sys
import tempfile
import time
import uuid
from datetime import datetime, timezone
from pathlib import Path

from . import harnesses
from .tasks import Task, find_tasks

AGENT_CLI = Path(__file__).with_name("agent_cli.py")


def sandbox_profile(workspace: str | Path | None = None, local_ports: tuple[int, ...] = ()) -> str:
    """macOS Seatbelt profile for agent processes. Everything is allowed except:

    - reading or writing the paths in ROBOUSE_SANDBOX_DENY (colon-separated), plus the maintainers' workspace and
      credential folders when they exist;
    - reading any robo-use source or install (a `robouse` package directory anywhere, e.g. another venv under /tmp);
    - reading other trials' workspaces (only this trial's workspace is allowed);
    - TCP connections to this machine, except `local_ports` (a harness's own local proxy). This keeps agents away
      from local viewers and servers (e.g. a runs viewer on localhost) while the episode Unix socket and the
      network (model APIs) stay reachable.
    Set ROBOUSE_SANDBOX=0 to disable."""
    home = Path.home()
    op_dir = Path(os.environ.get("ROBOUSE_OPERATOR_DIR", home / ".config" / "robouse-operator"))  # real-robot e-stop and operator
    builtin = [str(p) for p in (home / "benchflow", home / ".config" / "bf-agents", op_dir) if p.exists()]
    denied = builtin + [p for p in os.environ.get("ROBOUSE_SANDBOX_DENY", "").split(":") if p]
    denied = [os.path.realpath(os.path.expanduser(d)) for d in denied]  # Seatbelt matches resolved paths (/tmp -> /private/tmp)
    rules = "".join(f'(deny file-read* file-write* (subpath "{d}"))' for d in dict.fromkeys(denied))
    rules += '(deny file-read* (regex #"/site-packages/robouse(/|$)") (regex #"/src/robouse(/|$)"))'
    rules += '(deny file-read* file-write* (regex #"/robouse-ws-[^/]*(/|$)"))'
    if workspace:
        rules += f'(allow file-read* file-write* (subpath "{os.path.realpath(workspace)}"))'
    rules += '(deny network-outbound (remote ip "localhost:*"))'
    rules += "".join(f'(allow network-outbound (remote ip "localhost:{p}"))' for p in local_ports)
    return "(version 1)(allow default)" + rules


def provenance() -> dict:
    """What produced this trial: Robo Use version and git revision, Python, key simulator package versions."""
    import importlib.metadata as md
    import platform

    from . import __version__

    out = {"robouse": __version__, "python": platform.python_version(), "platform": platform.platform()}
    try:
        root = Path(__file__).resolve().parents[2]
        rev = subprocess.run(["git", "-C", str(root), "rev-parse", "HEAD"], capture_output=True, text=True, timeout=5)
        dirty = subprocess.run(["git", "-C", str(root), "status", "--porcelain", "--untracked-files=no"], capture_output=True,
                               text=True, timeout=5)
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


def _now() -> str:
    return datetime.now(timezone.utc).isoformat()


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
            if log is not None and log.exists():  # the server's last line usually names the cause (e.g. a missing simulator)
                lines = [l for l in log.read_text(errors="replace").splitlines() if l.strip()]
                last = f": {lines[-1].strip()[:300]}" if lines else ""
            raise RuntimeError(f"episode server exited early ({proc.returncode}){last}")
        time.sleep(0.2)
    raise TimeoutError("episode server did not start")


def run_trial(task: Task, harness: str, model: str, job_dir: Path, timeout: float | None = None,
              seed: int | None = None, trial_name: str | None = None, epoch: int = 0, split: str | None = None,
              extra_instruction: str = "", prior_learnings: str | None = None, attempt: int = 0) -> dict:
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
    started = _now()
    t0 = time.time()
    cfg = {"task": task.id, "task_path": str(task.path), "harness": harness, "model": model, "seed": seed,
           "agent_timeout_s": timeout, "trial_name": trial_name, "started_at": started, "workspace": str(ws),
           "metadata": task.metadata, "epoch": epoch, "split": split, "attempt": attempt,
           "extra_instruction": extra_instruction or None, "prior_learnings": prior_learnings,
           "provenance": provenance()}
    (tdir / "config.json").write_text(json.dumps(cfg, indent=2))

    py = sys.executable
    server_cmd = [py, "-m", "robouse.cli", "serve", "--task", str(task.path), "--run-dir", str(tdir / "episode"),
                  "--socket", sock, "--workspace", str(ws), "--ready-file", str(ready), "--max-wall-s", str(timeout + 60)]
    if seed is not None:
        server_cmd += ["--seed", str(seed)]
    if split:
        server_cmd += ["--split", split]
    oracle_token = uuid.uuid4().hex
    server = subprocess.Popen(server_cmd, env={**os.environ, "ROBOUSE_ORACLE_TOKEN": oracle_token}, stdout=open(tdir / "episode" / "server.log", "w"), stderr=subprocess.STDOUT)
    exc = None
    launch = None
    agent_rc = None
    timed_out = False
    try:
        _wait_ready(ready, server, timeout=float(task.spec.get("ready_timeout_s", 300)), log=tdir / "episode" / "server.log")
        (ws / ".robouse" / "socket").write_text(sock)
        base_env = {k: v for k, v in os.environ.items() if not k.startswith(("ANTHROPIC_", "CLAUDE_CODE_OAUTH", "OPENAI_", "BASETEN"))}
        base_env["PATH"] = f"{ws / '.robouse' / 'bin'}:{Path(py).parent}:{base_env.get('PATH', '')}" if harness == "oracle" \
            else f"{ws / '.robouse' / 'bin'}:{base_env.get('PATH', '')}"
        base_env["ROBOUSE_SOCKET"] = sock
        base_env["PYTHONPATH"] = str(Path(__file__).resolve().parents[1]) if harness == "oracle" else ""
        base_env.pop("ROBOUSE_ORACLE_TOKEN", None)
        if harness == "oracle":  # only the reference solution may see privileged state in vision tasks
            base_env["ROBOUSE_ORACLE_TOKEN"] = oracle_token
        launch = harnesses.build(harness, model, task.instruction, ws, task, base_env)
        sandboxed = harness not in ("oracle", "noop") and sys.platform == "darwin" and os.environ.get("ROBOUSE_SANDBOX", "1") != "0"
        if sandboxed:
            ports = tuple(int(m) for v in launch.env.values() for m in re.findall(r"//(?:127\.0\.0\.1|localhost):(\d+)", str(v)))
            launch.cmd = ["sandbox-exec", "-p", sandbox_profile(ws, ports), *launch.cmd]
        cfg["sandbox"] = "macos-seatbelt" if sandboxed else "none"
        (tdir / "config.json").write_text(json.dumps(cfg, indent=2))
        with open(tdir / "agent" / "stdout.jsonl", "w") as out, open(tdir / "agent" / "stderr.txt", "w") as err:
            agent = subprocess.Popen(launch.cmd, cwd=ws, env=launch.env, stdout=out, stderr=err, stdin=subprocess.DEVNULL,
                                     start_new_session=True)
            try:
                agent_rc = agent.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                timed_out = True
                os.killpg(agent.pid, signal.SIGTERM)
                try:
                    agent.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(agent.pid, signal.SIGKILL)
    except Exception as e:  # infrastructure failure, recorded in result.json
        exc = f"{type(e).__name__}: {e}"
    finally:
        # close the episode if the agent did not call `robo done`
        from .agent_cli import _send

        os.environ["ROBOUSE_SOCKET"] = sock
        os.environ.pop("ROBOUSE_ORACLE_TOKEN", None)
        st = _send({"op": "status"})
        if st.get("ok") and not st["result"]["finished"]:
            _send({"op": "give_up", "text": "[runner] agent timed out" if timed_out else "[runner] agent exited without robo done"})
        try:
            server.wait(timeout=60)
        except subprocess.TimeoutExpired:
            server.kill()

    ep = tdir / "episode" / "result.json"
    episode = json.loads(ep.read_text()) if ep.exists() else {}
    if timed_out and episode.get("outcome") == "gave_up":
        episode["outcome"] = "agent_timeout"
    # verifier: the task's own verifier/test.sh turns the trusted verdict into reward.txt
    vt = subprocess.run(["bash", str(task.verifier_script)], capture_output=True, text=True,
                        env={**os.environ, "ROBOUSE_EPISODE_DIR": str(tdir / "episode"), "ROBOUSE_VERIFIER_DIR": str(tdir / "verifier")})
    (tdir / "verifier" / "test-stdout.txt").write_text(vt.stdout + vt.stderr)
    rf = tdir / "verifier" / "reward.txt"
    reward = float(rf.read_text().strip()) if rf.exists() and rf.read_text().strip() else 0.0

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
    except Exception as e:  # trajectory conversion is best effort; raw output is kept
        (tdir / "agent" / "trajectory_error.txt").write_text(f"{type(e).__name__}: {e}")

    cost = trial_cost(tdir, (launch.notes.get("model") if launch else model) or "", launch.notes.get("provider") if launch else None)
    res = {
        "task_name": task.id, "trial_name": trial_name, "source": task.metadata.get("source_benchmark"),
        "agent_info": {"name": harness, "model_info": {"name": (launch.notes.get("model") if launch else model)}},
        "verifier_result": {"rewards": {"reward": reward}},
        "episode": episode, "agent_return_code": agent_rc, "agent_timed_out": timed_out,
        "exception_info": {"exception_type": exc.split(":")[0], "exception_message": exc} if exc else None,
        "started_at": started, "finished_at": _now(), "wall_time_s": round(time.time() - t0, 1),
        "embodiment_mode": episode.get("embodiment_mode", "sim"), "cost": cost,
    }
    if launch and isinstance(launch.helper, dict):
        res["proxy_stats"] = launch.helper
    (tdir / "result.json").write_text(json.dumps(res, indent=2))
    shutil.rmtree(ws, ignore_errors=True)
    return {"trial": trial_name, "reward": reward, "outcome": episode.get("outcome"), "steps": episode.get("steps_used"),
            "wall_s": res["wall_time_s"], "exception": exc}


def trial_cost(tdir: Path, model: str, provider: str | None) -> dict:
    """Tokens and USD for one trial, from the ATIF trajectory's final metrics; USD from the harness when it reports one,
    else from pricing.json (per 1M tokens). Scripted harnesses (oracle, noop, VLAs) cost 0 model tokens."""
    out = {"prompt_tokens": None, "completion_tokens": None, "cached_tokens": None, "cost_usd": None, "cost_source": None}
    try:
        fm = json.loads((tdir / "agent" / "trajectory.json").read_text()).get("final_metrics") or {}
    except Exception:  # noqa: BLE001
        fm = {}
    out.update(prompt_tokens=fm.get("total_prompt_tokens"), completion_tokens=fm.get("total_completion_tokens"),
               cached_tokens=fm.get("total_cached_tokens"))
    prices = json.loads((Path(os.environ["ROBOUSE_PRICING"]) if os.environ.get("ROBOUSE_PRICING") else Path(__file__).with_name("pricing.json")).read_text())
    price = prices.get(model)
    if price and out["prompt_tokens"] is not None:
        cached = out["cached_tokens"] or 0
        inp = (out["prompt_tokens"] - cached) * price["input"] + cached * price.get("cached_input", price["input"])
        out["cost_usd"] = round((inp + (out["completion_tokens"] or 0) * price["output"]) / 1e6, 5)
        out["cost_source"] = f"pricing.json ({price.get('source', '')})"
    elif fm.get("total_cost_usd") is not None and provider is None:
        out["cost_usd"], out["cost_source"] = round(float(fm["total_cost_usd"]), 5), "harness"
    elif out["prompt_tokens"] is None and (model in ("scripted", "none", "") or provider == "vla"):
        out.update(prompt_tokens=0, completion_tokens=0, cost_usd=0.0, cost_source="no model")
    return out


def _parse_seeds(spec: str | None) -> list[int | None]:
    """"0-4" -> [0..4]; "0,3,7" -> [0, 3, 7]; None -> [None] (the task's own seed)."""
    if not spec:
        return [None]
    out: list[int | None] = []
    for part in str(spec).split(","):
        a, _, b = part.partition("-")
        out += list(range(int(a), int(b) + 1)) if b else [int(a)]
    return out


def select_tasks(root: Path, filt: str = "", ids: list[str] | None = None, include: list[str] | None = None,
                 exclude: list[str] | None = None, limit: int = 0, sample: int = 0, sample_seed: int = 0,
                 mode: str | None = None) -> list[Task]:
    """Task subset: substring filter, id list, glob include/exclude (fnmatch on the id, e.g. 'libero-10-*'), sim/real
    mode, then a seeded random sample of N and/or the first `limit`."""
    import fnmatch
    import random

    tasks = [t for t in find_tasks(root) if filt in t.id and (ids is None or t.id in ids)]
    if include:
        tasks = [t for t in tasks if any(fnmatch.fnmatch(t.id, g) for g in include)]
    if exclude:
        tasks = [t for t in tasks if not any(fnmatch.fnmatch(t.id, g) for g in exclude)]
    if mode:
        tasks = [t for t in tasks if (t.metadata.get("embodiment_mode") or ("real" if t.spec.get("backend") == "real" else "sim")) == mode]
    if sample and sample < len(tasks):
        tasks = sorted(random.Random(sample_seed).sample(tasks, sample), key=lambda t: t.id)
    if limit:
        tasks = tasks[:limit]
    return tasks


def run_many(root: Path, harness: str, model: str, job_dir: Path, filt: str = "", limit: int = 0,
             concurrency: int = 1, timeout: float | None = None, ids: list[str] | None = None,
             resume: bool = False, seeds: str | None = None, epochs: int = 1, include: list[str] | None = None,
             exclude: list[str] | None = None, sample: int = 0, sample_seed: int = 0, retries: int = 0,
             split: str | None = None, mode: str | None = None, extra_instruction: str = "",
             prior_learnings: str | None = None) -> list[dict]:
    tasks = select_tasks(root, filt, ids, include, exclude, limit, sample, sample_seed, mode)
    runs = [(t, sd, ep) for t in tasks for sd in _parse_seeds(seeds) for ep in range(max(1, epochs))]
    if resume:  # skip (task, seed, epoch) already finished in this job (an unfinished trial dir is left as evidence and rerun)
        done = set()
        for rj in job_dir.glob("*/result.json"):
            try:
                r = json.loads(rj.read_text())
                c = json.loads((rj.parent / "config.json").read_text())
            except Exception:
                continue
            if not r.get("exception_info") and (r.get("episode") or {}).get("outcome"):
                done.add((r.get("task_name"), c.get("seed"), c.get("epoch", 0)))
        runs = [x for x in runs if (x[0].id, x[1], x[2]) not in done]
    job_dir.mkdir(parents=True, exist_ok=True)
    (job_dir / "job.json").write_text(json.dumps({"harness": harness, "model": model, "tasks": len(tasks), "seeds": seeds,
                                                  "epochs": epochs, "split": split, "include": include, "exclude": exclude,
                                                  "sample": sample, "sample_seed": sample_seed, "retries": retries,
                                                  "extra_instruction": extra_instruction, "prior_learnings": prior_learnings,
                                                  "started_at": _now()}, indent=2))
    log = open(job_dir / "run_log.jsonl", "a")
    results = []

    def one(item) -> dict:
        t, sd, ep = item
        for attempt in range(retries + 1):  # infrastructure failures only (a model's failure is a result, not retried)
            r = run_trial(t, harness, model, job_dir, timeout=timeout, seed=sd, epoch=ep, split=split,
                          extra_instruction=extra_instruction, prior_learnings=prior_learnings, attempt=attempt)
            if not r["exception"]:
                break
            if attempt < retries:
                bad = job_dir / r["trial"]
                dest = job_dir / "_infra_failed"
                dest.mkdir(exist_ok=True)
                if bad.exists():
                    shutil.move(str(bad), str(dest / bad.name))
                time.sleep(min(60, 5 * 2 ** attempt))
        r["task"] = t.id + (f"@seed{sd}" if sd is not None else "") + (f"#e{ep}" if epochs > 1 else "")
        return r

    with cf.ThreadPoolExecutor(max_workers=max(1, concurrency)) as ex:
        futs = [ex.submit(one, x) for x in runs]
        for fut in cf.as_completed(futs):
            r = fut.result()
            results.append(r)
            log.write(json.dumps(r) + "\n")
            log.flush()
            print(f"{r['task']:<40} reward={r['reward']:.0f} outcome={r['outcome']} steps={r['steps']} {r['wall_s']}s"
                  + (f" EXC {r['exception']}" if r["exception"] else ""), flush=True)
    ok = sum(r["reward"] for r in results)
    print(f"{harness}/{model or '-'}: {ok:.0f}/{len(results)} solved")
    return results
