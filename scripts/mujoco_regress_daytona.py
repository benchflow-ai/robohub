"""Run scripts/mujoco_regress.py jobs in Daytona CPU sandboxes, several in parallel, and fetch their results.

    python scripts/mujoco_regress_daytona.py --plan plan.json --results DIR [--units base.0,libero.1] [--parallel 6]
        [--cpu 4 --memory 8 --disk 10] [--prefix NAME] [--label KEY=VALUE] [--ledger FILE] [--limit N]
    python scripts/mujoco_regress_daytona.py --ls | --delete NAME

Needs the `daytona` package and DAYTONA_API_KEY (read from the environment or ~/.config/benchflow/daytona.env; never
printed). Each unit of the plan gets one sandbox from python:3.12-slim-bookworm (the base image of every runtime),
named PREFIX-UNIT, with this checkout's tool, hub.json, the unit's runtime folder and task packages uploaded; the job
runs in the background and is polled; its results (no videos) come back as DIR/UNIT/, then the sandbox is deleted.
Every create and delete is appended to the ledger with the time, so the sandbox hours can be added up.
A unit whose DIR/UNIT/progress.jsonl already ends in phase "done" is skipped.
"""
from __future__ import annotations

import argparse
import concurrent.futures
import io
import json
import os
import sys
import tarfile
import threading
import time
from pathlib import Path

HUB = Path(__file__).resolve().parents[1]
IMAGE = "python:3.12-slim-bookworm"
LOCK = threading.Lock()


def client():
    f = Path.home() / ".config/benchflow/daytona.env"
    if f.exists():
        for line in f.read_text().splitlines():
            line = line.strip()
            if line and not line.startswith("#") and "=" in line:
                k, v = line.split("=", 1)
                os.environ.setdefault(k.replace("export ", "").strip(), v.strip().strip("\"'"))
    from daytona import Daytona

    return Daytona()


def log(ledger: Path, **ev) -> None:
    ev["utc"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
    with LOCK, open(ledger, "a") as f:
        f.write(json.dumps(ev) + "\n")


def say(unit: str, msg: str) -> None:
    with LOCK:
        print(f"{time.strftime('%H:%M:%S')} {unit}: {msg}", flush=True)


def bundle(plan: dict, plan_path: Path, unit: str) -> bytes:
    u = plan["units"][unit]
    buf = io.BytesIO()
    with tarfile.open(fileobj=buf, mode="w:gz") as tar:
        tar.add(HUB / "scripts" / "mujoco_regress.py", "work/scripts/mujoco_regress.py")
        tar.add(HUB / "hub.json", "work/hub.json")
        tar.add(plan_path, "work/plan.json")
        tar.add(HUB / "runtimes" / u["runtime"], f"work/runtimes/{u['runtime']}")
        for k in u["tasks"]:
            path = plan["tasks"][k]["path"]
            tar.add(HUB / path, f"work/{path}")
    return buf.getvalue()


def sh(sb, cmd: str, timeout: int = 600):
    r = sb.process.exec(cmd, timeout=timeout)
    return r.exit_code, r.result or ""


def run_unit(d, a, plan: dict, unit: str) -> dict:
    from daytona import CreateSandboxFromImageParams, Resources

    out = a.results / unit
    prog = out / "progress.jsonl"
    if prog.exists() and prog.read_text().strip().splitlines()[-1:] and '"done"' in prog.read_text().splitlines()[-1]:
        say(unit, "already done")
        return {"unit": unit, "skipped": True}
    name = f"{a.prefix}-{unit.replace('.', '-')}"
    disk = int(dict(x.split("=") for x in a.disk_unit).get(unit, a.disk))
    labels = dict(x.split("=", 1) for x in a.label)
    p = CreateSandboxFromImageParams(
        image=IMAGE, name=name, labels=labels, auto_stop_interval=a.auto_stop, auto_delete_interval=0,
        resources=Resources(cpu=a.cpu, memory=a.memory, disk=disk),
    )
    sb = d.create(p, timeout=1800)
    log(a.ledger, ev="up", name=name, id=sb.id, unit=unit, cpu=a.cpu, memory=a.memory, disk=disk)
    say(unit, f"sandbox {sb.id} up")
    t0 = time.time()
    try:
        sb.fs.upload_file(bundle(plan, a.plan, unit), "/tmp/work.tgz")
        rc, o = sh(sb, "mkdir -p /work && tar -xzf /tmp/work.tgz -C / && rm /tmp/work.tgz && python -m venv /opt/mujoco-regress/uv"
                   " && /opt/mujoco-regress/uv/bin/pip install -q uv==0.12.5 pyyaml==6.0.3 && nproc && df -h / | tail -1", 900)
        if rc:
            raise RuntimeError(f"bootstrap failed: {o[-800:]}")
        say(unit, f"bootstrapped: {' '.join(o.split())}")
        lim = f" --limit {a.limit}" if a.limit else ""
        job = (f"cd /work && /opt/mujoco-regress/uv/bin/python scripts/mujoco_regress.py --hub /work job"
               f" --plan plan.json --unit {unit} --out /work/out --concurrency {a.concurrency}{lim} {a.job_args}"
               f" > /work/job.log 2>&1 < /dev/null; echo $? > /work/job.exit")
        from daytona import SessionExecuteRequest

        sb.process.create_session("mjr")
        sb.process.execute_session_command("mjr", SessionExecuteRequest(command=job, run_async=True))
        last = ""
        while True:
            time.sleep(a.poll)
            try:
                rc, o = sh(sb, "tail -n 1 /work/out/progress.jsonl 2>/dev/null; test -e /work/job.exit && echo STOPPED || echo RUNNING", 120)
                errors = 0
            except Exception as e:  # noqa: BLE001 - a dropped poll is retried; ten in a row give up
                errors = locals().get("errors", 0) + 1
                say(unit, f"poll failed ({errors}): {type(e).__name__}")
                if errors >= 10:
                    raise
                continue
            lines = o.strip().splitlines()
            state = lines[-1] if lines else "?"
            cur = lines[-2] if len(lines) > 1 else ""
            if cur != last:
                say(unit, cur)
                last = cur
            if state == "STOPPED":
                break
            if time.time() - t0 > a.max_hours * 3600:
                say(unit, "time cap reached; stopping the job")
                sh(sb, "for p in /proc/[0-9]*; do tr '\\0' ' ' < $p/cmdline 2>/dev/null | grep -q 'mujoco_regress.py --hub' && kill ${p#/proc/}; done; sleep 2", 60)
                break
        # results: no videos, frame logs or episode traces (the job has written divergence.json from them); trial folders
        # keep result.json, the logs and trial.json
        rc, o = sh(sb, "cd /work && tail -c 4000 job.log > out/job.log.tail; find out -name 'recording.mp4' -delete;"
                   " tar --exclude='frames.jsonl' --exclude='trace.jsonl' --exclude='physics.jsonl' -czf /tmp/out.tgz out && ls -l /tmp/out.tgz", 900)
        out.mkdir(parents=True, exist_ok=True)
        tgz = a.results / f"{unit}.tgz"
        sb.fs.download_file("/tmp/out.tgz", str(tgz))
        with tarfile.open(tgz) as tar:
            for m in tar.getmembers():
                m.name = m.name.split("/", 1)[1] if "/" in m.name else ""
            tar.extractall(out, members=[m for m in tar.getmembers() if m.name], filter="data")
        tgz.unlink()
        say(unit, f"results in {out}")
        return {"unit": unit, "hours": round((time.time() - t0) / 3600, 2)}
    finally:
        if not a.keep:
            d.delete(sb)
            log(a.ledger, ev="down", name=name, id=sb.id, unit=unit)
            say(unit, "sandbox deleted")


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--plan", type=Path)
    ap.add_argument("--results", type=Path)
    ap.add_argument("--units", help="comma-separated units (default: every unit of the plan)")
    ap.add_argument("--parallel", type=int, default=6)
    ap.add_argument("--cpu", type=int, default=4)
    ap.add_argument("--memory", type=int, default=8)
    ap.add_argument("--disk", type=int, default=10)
    ap.add_argument("--disk-unit", action="append", default=[], metavar="UNIT=GB", help="disk for one unit (repeatable)")
    ap.add_argument("--concurrency", type=int, default=4, help="trials in parallel inside a sandbox")
    ap.add_argument("--limit", type=int, default=0)
    ap.add_argument("--job-args", default="", help="extra arguments for `mujoco_regress.py job`, e.g. '--pinned-only-changed --seeds='")
    ap.add_argument("--prefix", default="mujoco-regress")
    ap.add_argument("--label", action="append", default=[], help="sandbox label KEY=VALUE (repeatable)")
    ap.add_argument("--ledger", type=Path, default=Path("sandboxes.jsonl"))
    ap.add_argument("--poll", type=int, default=60)
    ap.add_argument("--auto-stop", type=int, default=60, help="Daytona auto-stop after N idle minutes")
    ap.add_argument("--max-hours", type=float, default=4.0, help="stop a job after this many hours")
    ap.add_argument("--keep", action="store_true", help="do not delete the sandboxes")
    ap.add_argument("--ls", action="store_true")
    ap.add_argument("--delete")
    a = ap.parse_args(argv)
    d = client()
    if a.ls or a.delete:
        from daytona import ListSandboxesQuery

        labels = dict(x.split("=", 1) for x in a.label) or None
        for sb in d.list(ListSandboxesQuery(labels=labels) if labels else None):
            if a.delete and a.delete in (sb.name, sb.id):
                d.delete(sb)
                log(a.ledger, ev="down", name=sb.name, id=sb.id)
                print("deleted", sb.name, sb.id)
            elif a.ls:
                print(sb.name, sb.id, sb.state)
        return 0
    plan = json.loads(a.plan.read_text())
    units = a.units.split(",") if a.units else list(plan["units"])
    a.results.mkdir(parents=True, exist_ok=True)
    with concurrent.futures.ThreadPoolExecutor(a.parallel) as ex:
        futs = {ex.submit(run_unit, d, a, plan, u): u for u in units}
        bad = 0
        for f in concurrent.futures.as_completed(futs):
            try:
                print(json.dumps(f.result()), flush=True)
            except Exception as e:  # noqa: BLE001 - report and go on with the other units
                bad += 1
                say(futs[f], f"FAILED: {type(e).__name__}: {e}")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
