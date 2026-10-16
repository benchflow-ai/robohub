"""MuJoCo regression check: re-run every MuJoCo-based task's reference solution and no-op on one MuJoCo release.

For a given MuJoCo release, every hub task whose physics runs on MuJoCo is run twice per harness: once on its runtime
as pinned, and once on a copy of that runtime in which only MuJoCo is changed (every interpreter of the runtime gets
`mujoco==VERSION`; nothing else is upgraded). The results are compared and written as a machine-readable diff
(diff.json) and a short markdown report (task, pinned version, new version, reference and no-op scores, first
diverging step where the episode records tell).

    python scripts/mujoco_regress.py plan --mujoco 3.16.0 --out plan.json [--runtimes base,libero] [--shards base=3]
    python scripts/mujoco_regress.py variant RUNTIME --mujoco 3.16.0 --out DIR
    python scripts/mujoco_regress.py job --plan plan.json --unit base.0 --out OUT        # on a disposable Linux box
    python scripts/mujoco_regress.py diff --plan plan.json --results OUT [OUT ...] --out diff.json
    python scripts/mujoco_regress.py report --diff diff.json --out REPORT.md

`plan` reads hub.json and runtimes/ (this checkout) and lists the MuJoCo-based runtimes, the MuJoCo each pins, and their
tasks, deduplicated (a task that appears unchanged in several datasets is run once). `variant` writes the copy of a
runtime whose Dockerfile ends by switching MuJoCo; build it with Docker like any runtime. `job` is what runs in a
sandbox (root on a throwaway Linux machine, for example a Daytona sandbox from python:3.12-slim-bookworm, the base
image of every runtime): it replays the runtime's Dockerfile onto the machine, runs the pinned baseline, switches
MuJoCo exactly as the variant does, runs the new version, then re-checks every task whose verdict changed (the
pinned version again for repeatability, seeds 1 and 2 on both versions, and a bisection over the MuJoCo releases in
between). scripts/mujoco_regress_daytona.py runs the jobs in Daytona sandboxes, several in parallel.

A trial reproduces the task's BenchFlow topology without Docker: the simulator service's command and environment
from environment/docker-compose.yaml, the reference solution (oracle/solve.sh) or the no-op (`robo info; robo
observe`, then exit without `robo done`, as in benchflow/robouse-noop-control) as the agent, then verifier/test.sh,
each with its own episode, socket and verifier folders. The reward is the verifier's reward.txt.
"""
from __future__ import annotations

import argparse
import concurrent.futures
import hashlib
import json
import os
import re
import secrets
import shlex
import shutil
import signal
import subprocess
import sys
import time
import urllib.request
from collections import Counter
from pathlib import Path

HUB = Path(__file__).resolve().parents[1]
PIN = re.compile(r"^(\s*mujoco\s*==\s*)([0-9][0-9A-Za-z.\-+]*)(.*)$", re.M)
# Runtimes that pin MuJoCo but whose physics is not MuJoCo: `light` (numpy-only simulators; MuJoCo is imported only by
# the episode server's start-up check), `driving` (MetaDrive) and `maniskill` (SAPIEN).
# runtimes that pin MuJoCo without stepping their physics with it; the GPU-worker runtimes pin it only because
# robouse serve (<= 0.4.0) checks for MuJoCo before any episode server starts
_GPU_WORKERS = "remote GPU simulator workers (MuJoCo pinned only for robouse serve's start-up check)"
NOT_MUJOCO = {
    "light": "numpy-only simulators",
    "driving": "MetaDrive (Panda3D/Bullet)",
    "maniskill": "SAPIEN",
    "gpu": _GPU_WORKERS,
    "behavior": _GPU_WORKERS,
    "robodojo": _GPU_WORKERS,
}
NOOP = "robo info\nrobo observe\n"  # benchflow/robouse-noop-control's reference solution
STATE = Path("/opt/mujoco-regress")  # on the sandbox: the replayed runtime's environment
UV = "/opt/mujoco-regress/uv/bin/uv"  # uv in its own virtualenv, so installing it changes none of the runtime's interpreters
VOLATILE = {"t", "time", "wall", "wall_s", "wall_time_s", "elapsed_s", "path", "image", "images", "file", "saved"}


# ---------------------------------------------------------------- plan


def requirement_pins(runtime: Path) -> dict[str, str]:
    """{requirements file: pinned MuJoCo version} for every requirements*.txt of a runtime folder."""
    out = {}
    for f in sorted(runtime.glob("requirements*.txt")):
        if m := PIN.search(f.read_text()):
            out[f.name] = m.group(2)
    return out


def worker_pythons(dockerfile: str) -> dict[str, str]:
    """{ENV name: interpreter} for the simulator interpreters a runtime's Dockerfile declares (ROBOUSE_*_PYTHON)."""
    out = {}
    for ins, arg in instructions(dockerfile):
        if ins == "ENV":
            for k, v in env_pairs(arg).items():
                if re.fullmatch(r"ROBOUSE_[A-Z_]+_PYTHON", k):
                    out[k] = v
    return out


def sim_pin(runtime: Path) -> str | None:
    """The MuJoCo version the simulator itself runs on: the worker environment's pin if the runtime has one."""
    pins = requirement_pins(runtime)
    extra = {k: v for k, v in pins.items() if k != "requirements.txt"}
    if extra:
        return sorted(set(extra.values()))[0] if len(set(extra.values())) == 1 else ",".join(sorted(set(extra.values())))
    return pins.get("requirements.txt")


def sim_memory(hub: Path, runtime: str) -> float | None:
    """The memory one simulator of a runtime needs (hub.yaml runtimes: <name>: sim_memory: 4G), in GB."""
    text = (hub / "hub.yaml").read_text() if (hub / "hub.yaml").exists() else ""
    block = re.search(rf"^runtimes:\n(?:.*\n)*?  {re.escape(runtime)}:\n((?:    .*\n)+)", text, re.M)
    m = re.search(r"sim_memory:\s*([0-9.]+)\s*G", block.group(1)) if block else None
    return float(m.group(1)) if m else None


def mem_quota_gb() -> float | None:
    try:
        v = Path("/sys/fs/cgroup/memory.max").read_text().strip()
        return None if v == "max" else int(v) / 2**30
    except (OSError, ValueError):
        return None


def task_key(task_dir: Path) -> str:
    """Content hash of what a trial runs: the simulator service definition and the reference solution."""
    h = hashlib.sha256()
    for rel in ("environment/docker-compose.yaml", "oracle/solve.sh", "verifier/test.sh"):
        p = task_dir / rel
        h.update(rel.encode() + b"\0" + (p.read_bytes() if p.exists() else b""))
    return h.hexdigest()[:16]


def plan(a: argparse.Namespace) -> int:
    hub = json.loads((a.hub / "hub.json").read_text())
    only = set(a.runtimes.split(",")) if a.runtimes else None
    shards = dict(s.split("=") for s in (a.shards or "").split(",") if s)
    runtimes: dict[str, dict] = {}
    skipped: dict[str, str] = {}
    tasks: dict[str, dict] = {}
    for ds in hub["datasets"]:
        ref = f"{ds['name']}@{ds['version']}"
        noop_control = ds["short_name"] == "robouse-noop-control"
        for t in ds["tasks"]:
            rt = t["runtime"]
            rdir = a.hub / "runtimes" / rt
            if rt not in runtimes and rt not in skipped:
                pins = requirement_pins(rdir)
                if not pins:
                    skipped[rt] = "no MuJoCo pin"
                elif rt in NOT_MUJOCO:
                    skipped[rt] = NOT_MUJOCO[rt]
                elif only and rt not in only:
                    skipped[rt] = "not selected"
                else:
                    runtimes[rt] = {
                        "pins": pins,
                        "sim_memory_gb": sim_memory(a.hub, rt),
                        "sim_mujoco": sim_pin(rdir),
                        "pythons": worker_pythons((rdir / "Dockerfile").read_text()),
                        "source": json.loads((rdir / "SOURCE.json").read_text()) if (rdir / "SOURCE.json").exists() else None,
                        "tasks": [],
                    }
            if rt not in runtimes:
                continue
            tdir = a.hub / "datasets" / ds["dir"] / t["id"]
            key = f"{rt}:{t['id']}:{task_key(tdir)}"
            e = tasks.setdefault(
                key,
                {
                    "key": key,
                    "id": t["id"],
                    "runtime": rt,
                    "path": f"datasets/{ds['dir']}/{t['id']}",
                    "backend": t.get("backend"),
                    "suite": t.get("suite"),
                    "datasets": [],
                    # the no-op control dataset's reference solution is the no-op: it must score 0 and is run once
                    "harnesses": ["reference"] if noop_control else ["reference", "noop"],
                    "expect": {"reference": 0} if noop_control else {"reference": 1, "noop": 0},
                },
            )
            e["datasets"].append(ref)
            if e["key"] not in runtimes[rt]["tasks"]:
                runtimes[rt]["tasks"].append(e["key"])
    units = {}
    for rt, r in runtimes.items():
        n = max(1, int(shards.get(rt, 1)))
        for i in range(n):
            units[f"{rt}.{i}"] = {"runtime": rt, "tasks": r["tasks"][i::n]}
    out = {
        "mujoco": a.mujoco,
        "hub_commit": git_head(a.hub),
        "created_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "releases": mujoco_releases(a.mujoco) if a.releases else [],
        "runtimes": runtimes,
        "skipped_runtimes": skipped,
        "tasks": tasks,
        "units": units,
    }
    Path(a.out).write_text(json.dumps(out, indent=1))
    n_trials = sum(len(t["harnesses"]) for t in tasks.values())
    pinned = sorted({r["sim_mujoco"] for r in runtimes.values()}, key=vkey)
    print(f"{len(runtimes)} runtimes ({', '.join(runtimes)}), {len(tasks)} tasks, {n_trials} trials per version")
    print(f"pinned MuJoCo versions: {', '.join(pinned)}; skipped: {skipped}")
    return 0


def git_head(repo: Path) -> str | None:
    r = subprocess.run(["git", "-C", str(repo), "rev-parse", "HEAD"], capture_output=True, text=True)
    return r.stdout.strip() or None


def vkey(v: str) -> tuple:
    return tuple(int(x) if x.isdigit() else 0 for x in re.split(r"[.,]", v.split(",")[0]))


def mujoco_releases(upto: str) -> list[str]:
    """Every final MuJoCo release on PyPI with a CPython 3.12 Linux wheel, oldest first, up to `upto`."""
    with urllib.request.urlopen("https://pypi.org/pypi/mujoco/json", timeout=60) as r:
        rel = json.load(r)["releases"]
    ok = [v for v, files in rel.items() if re.fullmatch(r"\d+\.\d+\.\d+", v) and any(
        "cp312" in f["filename"] and "linux" in f["filename"] and not f.get("yanked") for f in files)]
    return [v for v in sorted(ok, key=vkey) if vkey(v) <= vkey(upto)]


# ---------------------------------------------------------------- Dockerfile replay and the variant


def instructions(text: str) -> list[tuple[str, str]]:
    """(INSTRUCTION, argument) pairs of a Dockerfile; comments dropped, continuation lines joined."""
    out, cur = [], ""
    for line in text.splitlines():
        if not cur and (not line.strip() or line.lstrip().startswith("#")):
            continue
        if cur and line.lstrip().startswith("#"):
            continue
        if line.rstrip().endswith("\\"):
            cur += line.rstrip()[:-1]  # Docker drops the backslash and the newline
            continue
        cur += line
        ins, _, arg = cur.strip().partition(" ")
        out.append((ins.upper(), arg.strip()))
        cur = ""
    return out


def env_pairs(arg: str) -> dict[str, str]:
    toks = shlex.split(arg.replace("\n", " "))
    if toks and "=" not in toks[0]:  # legacy `ENV KEY value`
        return {toks[0]: " ".join(toks[1:])}
    return dict(t.split("=", 1) for t in toks)


def expand(s: str, env: dict[str, str]) -> str:
    return re.sub(r"\$\{?([A-Za-z_][A-Za-z0-9_]*)\}?", lambda m: env.get(m.group(1), m.group(0)), s)


UV_BOOTSTRAP = f"(test -x {UV} || (python -m venv {Path(UV).parents[1]} && {Path(UV).parent}/pip install -q uv==0.12.5))"


def switch_script(versions: dict[str, str]) -> str:
    """The shell step that sets MuJoCo in each interpreter ({interpreter: version}) and changes nothing else that it
    does not have to: uv keeps every installed package that still satisfies MuJoCo's requirements and adds only what the
    new MuJoCo needs. Interpreters without MuJoCo are left alone."""
    cmds = [UV_BOOTSTRAP]
    for py, version in versions.items():
        cmds.append(
            f'if {UV} pip show -q --python {py} mujoco >/dev/null 2>&1; then {UV} pip install -q --python {py} "mujoco=={version}"; fi'
        )
    return " && ".join(cmds)


def runtime_pythons(dockerfile: str) -> list[str]:
    return ["/usr/local/bin/python", *worker_pythons(dockerfile).values()]


def variant(a: argparse.Namespace) -> int:
    src = a.hub / "runtimes" / a.runtime
    out = Path(a.out)
    if out.exists():
        sys.exit(f"{out} exists")
    shutil.copytree(src, out, symlinks=True)
    df = (src / "Dockerfile").read_text()
    step = switch_script({py: a.mujoco for py in runtime_pythons(df)})
    marker = "ARG ROBOHUB_RUNTIME_DIGEST"
    layer = f"# mujoco_regress.py variant: only MuJoCo changes, to {a.mujoco} in every interpreter that has it.\nRUN {step}\n\n"
    if marker not in df:
        df += "\n" + layer
    else:
        df = df.replace(marker, layer + marker, 1)
    (out / "Dockerfile").write_text(df)
    (out / "MUJOCO_VARIANT.json").write_text(json.dumps(
        {"runtime": a.runtime, "mujoco": a.mujoco, "pinned": requirement_pins(src), "switch": step}, indent=1))
    print(f"wrote {out}; its content digest changes, so build it with ROBOHUB_RUNTIME_DIGEST from check_runtime.py")
    return 0


def content_digest(root: Path) -> str:
    """A runtime folder's content digest, as its check_runtime.py computes it (BenchFlow's task digest algorithm)."""
    files = []
    for dirpath, _dirs, names in os.walk(root):
        for name in names:
            q = Path(dirpath) / name
            if q.is_file() and not q.is_symlink():
                files.append((q.relative_to(root).as_posix(), q))
    h = hashlib.sha256()
    for rel, q in sorted(files):
        h.update(rel.encode() + b"\0" + hashlib.sha256(q.read_bytes()).digest())
    return f"sha256:{h.hexdigest()}"


def replay(runtime: Path, log: Path) -> dict[str, str]:
    """Run a runtime's Dockerfile on this machine (root, throwaway, from the same base image) and return its ENV."""
    df = (runtime / "Dockerfile").read_text()
    env = {"PATH": "/usr/local/bin:/usr/local/sbin:/usr/sbin:/usr/bin:/sbin:/bin", "HOME": "/root",
           "DEBIAN_FRONTEND": "noninteractive", "LANG": "C.UTF-8"}
    args: dict[str, str] = {}
    workdir = "/"
    args["ROBOHUB_RUNTIME_DIGEST"] = content_digest(runtime)
    with open(log, "a") as lf:
        for ins, arg in instructions(df):
            if ins in ("FROM", "CMD", "ENTRYPOINT", "LABEL", "EXPOSE", "USER", "HEALTHCHECK"):
                continue
            if ins == "ARG":
                k, _, v = arg.partition("=")
                args.setdefault(k.strip(), v.strip())
                continue
            if ins == "ENV":
                env.update({k: expand(v, {**args, **env}) for k, v in env_pairs(arg).items()})
                continue
            if ins == "WORKDIR":
                workdir = expand(arg, {**args, **env})
                os.makedirs(workdir, exist_ok=True)
                continue
            if ins == "COPY":
                *srcs, dst = shlex.split(arg)
                srcs = [s for s in srcs if not s.startswith("--")]
                cmd = "mkdir -p " + shlex.quote(dst if dst.endswith("/") or len(srcs) > 1 else os.path.dirname(dst) or "/")
                for s in srcs:
                    if s in (".", "./"):
                        cmd += f" && cp -a {shlex.quote(str(runtime))}/. {shlex.quote(dst)}"
                    elif s.endswith("/"):
                        cmd += f" && mkdir -p {shlex.quote(dst)} && cp -a {shlex.quote(str(runtime / s))}/. {shlex.quote(dst)}"
                    else:
                        cmd += f" && cp -a {str(runtime)}/{s} {shlex.quote(dst)}"
                run_step(cmd, env, workdir, lf, ins)
                continue
            if ins == "RUN":
                run_step(arg, {**args, **env}, workdir, lf, ins)
                continue
            raise SystemExit(f"Dockerfile instruction not handled: {ins}")
    return env


def run_step(cmd: str, env: dict, cwd: str, lf, label: str) -> None:
    lf.write(f"\n+++ {label} {cmd[:400]}\n")
    lf.flush()
    r = subprocess.run(["/bin/sh", "-c", cmd], env=env, cwd=cwd, stdout=lf, stderr=subprocess.STDOUT)
    if r.returncode:
        raise SystemExit(f"{label} failed ({r.returncode}): {cmd[:300]}")


def installed_versions(env: dict, pythons: list[str]) -> dict[str, dict]:
    out = {}
    for py in pythons:
        r = subprocess.run([py, "-c", "import mujoco, sys; print(mujoco.__version__); print(sys.version.split()[0])"],
                           env={**env, "PYTHONPATH": ""}, capture_output=True, text=True, cwd="/tmp")
        fz = subprocess.run([UV, "pip", "freeze", "--python", py], capture_output=True, text=True, env=env)
        lines = r.stdout.split()
        out[py] = {"mujoco": lines[0] if r.returncode == 0 and lines else None,
                   "python": lines[1] if len(lines) > 1 else None,
                   "freeze": sorted(fz.stdout.split()) if fz.returncode == 0 else None,
                   **({"error": (r.stderr or "")[-500:]} if r.returncode else {})}
    return out


def switch(versions: dict[str, str], env: dict, log: Path) -> dict:
    """Set MuJoCo in place in each interpreter (the variant's step) and say what else changed in each."""
    pythons = list(versions)
    before = installed_versions(env, pythons)
    with open(log, "a") as lf:
        run_step(switch_script(versions), env, "/tmp", lf, f"SWITCH {versions}")
    after = installed_versions(env, pythons)
    changes = {}
    for py in pythons:
        b, n = set(before[py]["freeze"] or []), set(after[py]["freeze"] or [])
        changes[py] = {"removed": sorted(b - n), "added": sorted(n - b), "mujoco": after[py]["mujoco"],
                       **({"import_error": after[py]["error"]} if after[py].get("error") else {})}
    return changes


def resolver_check(runtime: Path, version: str, env: dict, pythons: dict[str, str]) -> dict:
    """Would the runtime's own requirements still resolve with `mujoco==version`? Separates our pins from the engine."""
    out = {}
    for f in sorted(runtime.glob("requirements*.txt")):
        text = f.read_text()
        if not PIN.search(text):
            continue
        tmp = Path("/tmp") / f"mujoco-regress-{f.name}"
        tmp.write_text(PIN.sub(lambda m: m.group(1) + version + m.group(3), text))
        r = subprocess.run([UV, "pip", "compile", "-q", "--python-version", "3.12" if f.name == "requirements.txt"
                            else pythons.get(f.name, "3.12"), str(tmp)], capture_output=True, text=True, env=env)
        out[f.name] = {"resolves": r.returncode == 0, "message": (r.stderr or "").strip()[-800:]}
    return out


# ---------------------------------------------------------------- one trial


def compose_services(task: Path) -> dict:
    import yaml

    class Loader(yaml.SafeLoader):
        pass

    Loader.add_constructor("!override", lambda ld, node: ld.construct_sequence(node) if isinstance(
        node, yaml.SequenceNode) else ld.construct_mapping(node))
    return yaml.load((task / "environment" / "docker-compose.yaml").read_text(), Loader=Loader)["services"]


def frontmatter(task: Path) -> dict:
    import yaml

    text = (task / "task.md").read_text()
    return yaml.safe_load(text.split("---", 2)[1]) if text.startswith("---") else {}


def substitute(v: str, host: dict) -> str:
    return re.sub(r"\$\{([A-Za-z_][A-Za-z0-9_]*)(:-([^}]*))?\}", lambda m: host.get(m.group(1)) or (m.group(3) or ""), v)


def trial(task: Path, harness: str, out: Path, renv: dict, seed: int | None = None) -> dict:
    """Run one trial of a hub task package as BenchFlow would, with per-trial folders instead of containers."""
    if out.exists():
        shutil.rmtree(out)
    rpc, ep, ver, art, tdir = (out / x for x in ("rpc", "episode", "verifier", "artifacts", "task"))
    for d in (rpc / "ws", ep, ver, art, tdir):
        d.mkdir(parents=True)
    host = {"ROBOUSE_ORACLE_TOKEN": secrets.token_hex(16)}
    svc = compose_services(task)
    sim, main = svc["simulator"], svc["main"]
    senv = {k: substitute(str(v), host) for k, v in (sim.get("environment") or {}).items()}
    md = senv.pop("ROBOUSE_TASK_MD", None) or senv.pop("ROBO_TASK_MD", None)
    if md is not None:
        (tdir / "task.md").write_text(md if md.endswith("\n") else md + "\n")
    else:
        shutil.copy(task / "task.md", tdir / "task.md")
    # the socket lives in a short folder of its own (AF_UNIX paths are limited to 107 bytes), linked from rpc/
    sdir = Path("/tmp/mjr") / hashlib.sha256(str(out).encode()).hexdigest()[:12]
    shutil.rmtree(sdir, ignore_errors=True)
    sdir.mkdir(parents=True)
    sock = sdir / "episode.sock"
    cmd = list(sim.get("command") or ["robouse-sim-entry"])
    entry, extra = cmd[0], cmd[1:]
    if seed is not None:
        extra += ["--seed", str(seed)]
    paths = {"--task": tdir, "--run-dir": ep, "--socket": sock, "--workspace": rpc / "ws"}
    flat = [x for kv in paths.items() for x in (kv[0], str(kv[1]))]
    if entry == "robouse-sim-entry":
        server = ["python", "-m", "robouse.cli", "serve", *flat, *extra]
    elif entry == "embodied-sim-entry":
        server = ["python", "-m", "benchflow.embodied.serve", "--factory", senv.get("ROBO_EPISODE_FACTORY", ""), *flat, *extra]
    else:
        raise SystemExit(f"unknown simulator entry point {entry!r} in {task}")
    # worker suites print their tracebacks only when not quiet: shown in serve.log, so a failure can be attributed
    quiet = {k.replace("_PYTHON", "_QUIET"): "0" for k in renv if re.fullmatch(r"ROBOUSE_[A-Z_]+_PYTHON", k)}
    sim_env = {**renv, **quiet, **senv}
    fm = frontmatter(task)
    t0 = time.time()
    rec: dict = {"harness": harness, "seed": seed}
    # as the runtime's entry script does: the server's log in episode/serve.log, its exit status in episode/serve.exit
    # (the verifier waits for it)
    wrapped = ["sh", "-c", f'"$@" > {shlex.quote(str(ep / "serve.log"))} 2>&1; echo $? > {shlex.quote(str(ep / "serve.exit"))}',
               "sim-entry", *server]
    srv = subprocess.Popen(wrapped, env=sim_env, cwd=ep, stdin=subprocess.DEVNULL, start_new_session=True)
    try:
        while not sock.exists():
            if srv.poll() is not None or time.time() - t0 > 600:
                rec["error"] = "episode server did not start: " + (ep / "serve.log").read_text()[-1500:]
                return finish(rec, out, t0)
            time.sleep(0.2)
        rec["start_s"] = round(time.time() - t0, 1)
        bin_dir = out / "bin"
        bin_dir.mkdir()
        shutil.copy(task / "environment" / "robo", bin_dir / "robo")
        (bin_dir / "robo").chmod(0o755)
        aenv = {"PATH": f"{bin_dir}:/usr/local/bin:/usr/bin:/bin", "HOME": str(out), "PYTHONUNBUFFERED": "1",
                "LANG": "C.UTF-8"}
        aenv.update({k: str(v) for k, v in (main.get("environment") or {}).items()})
        for k in ("ROBOUSE_SOCKET", "ROBO_SOCKET"):
            if k in aenv:
                aenv[k] = str(sock)
        if harness == "reference":
            aenv.update({k: substitute(str(v), host) for k, v in ((fm.get("oracle") or {}).get("env") or {}).items()})
            agent = ["bash", str(task / "oracle" / "solve.sh")]
        else:
            agent = ["bash", "-c", NOOP]
        timeout = float((fm.get("agent") or {}).get("timeout_sec") or 3600)
        with open(out / "agent.log", "w") as alog:
            try:
                r = subprocess.run(agent, env=aenv, cwd=out, stdout=alog, stderr=subprocess.STDOUT, timeout=timeout)
                rec["agent_exit"] = r.returncode
            except subprocess.TimeoutExpired:
                rec["agent_exit"] = "timeout"
        rec["agent_s"] = round(time.time() - t0, 1)
        venv = {**sim_env}
        for p in ("ROBOUSE_", "ROBO_"):
            venv.update({f"{p}EPISODE_DIR": str(ep), f"{p}SOCKET": str(sock), f"{p}VERIFIER_DIR": str(ver),
                         f"{p}ARTIFACTS_DIR": str(art), f"{p}WORKSPACE_DIR": str(rpc / "ws")})
        venv.update({k: v for k, v in renv.items() if k == "PYTHONPATH"})
        vt = float((fm.get("verifier") or {}).get("timeout_sec") or 600)
        with open(out / "verifier.log", "w") as vlog:
            try:
                r = subprocess.run(["sh", str(task / "verifier" / "test.sh")], env=venv, cwd=ep, stdout=vlog,
                                   stderr=subprocess.STDOUT, timeout=vt)
                rec["verifier_exit"] = r.returncode
            except subprocess.TimeoutExpired:
                rec["verifier_exit"] = "timeout"
        rw = ver / "reward.txt"
        rec["reward"] = float(rw.read_text().strip()) if rw.exists() else None
        res = ep / "result.json"
        if res.exists():
            r = json.loads(res.read_text())
            rec.update({k: r.get(k) for k in ("outcome", "success", "steps_used", "max_steps", "judge_error")})
        # an error the episode server returned to a robo request (a worker exception inside a step): the trial still
        # scores, but the error is what changed
        tr = ep / "trace.jsonl"
        for line in tr.read_text(errors="replace").splitlines() if tr.exists() else []:
            try:
                resp = json.loads(line).get("resp") or {}
            except ValueError:
                continue
            if resp.get("ok") is False and re.search(r"(WorkerError|[A-Z][A-Za-z]+Error): ", str(resp.get("error"))):
                rec["robo_error"] = str(resp["error"])[:1000]
                break
        if rec["reward"] is None:
            rec["error"] = "no reward: " + (out / "verifier.log").read_text()[-800:] + (ep / "serve.log").read_text()[-1500:]
        return finish(rec, out, t0)
    finally:
        try:
            os.killpg(srv.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        srv.wait()
        shutil.rmtree(sdir, ignore_errors=True)


def finish(rec: dict, out: Path, t0: float) -> dict:
    rec["wall_s"] = round(time.time() - t0, 1)
    for log in (out / "agent.log", out / "episode" / "serve.log", out / "verifier.log"):
        if log.exists() and log.stat().st_size > 40_000:  # keep the head and the tail
            b = log.read_bytes()
            log.write_bytes(b[:8_000] + b"\n[... cut by mujoco_regress.py ...]\n" + b[-24_000:])
    for big in ("recording.mp4",):
        for f in out.rglob(big):
            f.unlink()
    for d in ("artifacts", "rpc", "bin"):
        shutil.rmtree(out / d, ignore_errors=True)
    shutil.rmtree(out / "verifier" / "episode", ignore_errors=True)  # a copy of episode/
    shutil.rmtree(out / "episode" / "observations", ignore_errors=True)  # the camera images the agent asked for
    (out / "trial.json").write_text(json.dumps(rec, indent=1))
    return rec


# ---------------------------------------------------------------- the job (one runtime or shard, in one sandbox)


def job(a: argparse.Namespace) -> int:
    p = json.loads(Path(a.plan).read_text())
    unit = p["units"][a.unit]
    rt = unit["runtime"]
    rinfo = p["runtimes"][rt]
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    progress = out / "progress.jsonl"

    def note(**kw):
        kw["utc"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
        with open(progress, "a") as f:
            f.write(json.dumps(kw) + "\n")
        print(json.dumps(kw), flush=True)

    runtime = a.hub / "runtimes" / rt
    pythons = runtime_pythons((runtime / "Dockerfile").read_text())
    envf, pinf = STATE / "env.json", STATE / "pinned.json"
    if envf.exists():
        renv = json.loads(envf.read_text())
    else:
        note(phase="setup", runtime=rt)
        t = time.time()
        renv = replay(runtime, out / "setup.log")
        STATE.mkdir(parents=True, exist_ok=True)
        envf.write_text(json.dumps(renv))
        note(phase="setup-done", seconds=round(time.time() - t))
    renv = {**renv, "PYTHONDONTWRITEBYTECODE": "1"}
    with open(out / "setup.log", "a") as lf:
        run_step(UV_BOOTSTRAP, renv, "/tmp", lf, "uv")
    if not pinf.exists():  # the versions the runtime was built with; a re-run of the job switches back to them first
        got = installed_versions(renv, pythons)
        pinf.write_text(json.dumps({py: v["mujoco"] for py, v in got.items() if v["mujoco"]}))
    pinned = json.loads(pinf.read_text())
    pythons = list(pinned)
    sim_py = pythons[-1]  # the simulator's own interpreter: the worker virtualenv when there is one
    pinned_v = pinned[sim_py]
    switch(pinned, renv, out / "setup.log")
    meta = {"unit": a.unit, "runtime": rt, "mujoco": p["mujoco"], "machine": " ".join(os.uname()),
            "cpus": cpu_quota(), "pinned": pinned, "simulator_python": sim_py}
    inst = installed_versions(renv, pythons)
    meta["pinned_install"] = {k: {kk: vv for kk, vv in v.items() if kk != "freeze"} for k, v in inst.items()}
    sim_python = ".".join((inst[sim_py]["python"] or "3.12").split(".")[:2])
    meta["resolver"] = resolver_check(runtime, p["mujoco"], renv, {f: sim_python for f in rinfo["pins"]})
    (out / "meta.json").write_text(json.dumps(meta, indent=1))
    keys = unit["tasks"][: a.limit or None]
    # as many trials at once as the machine's memory holds simulators of this runtime (hub.yaml sim_memory)
    need, have = rinfo.get("sim_memory_gb") or sim_memory(a.hub, rt), mem_quota_gb()
    if need and have:
        a.concurrency = max(1, min(a.concurrency, int(have // need)))
    meta["concurrency"] = a.concurrency
    (out / "meta.json").write_text(json.dumps(meta, indent=1))

    def batch(label: str, items: list[tuple[str, str, int | None]]):
        res_f = out / f"{label}.jsonl"
        done = set()
        if res_f.exists():
            for line in res_f.read_text().splitlines():
                r = json.loads(line)
                done.add((r["key"], r["harness"], r.get("seed")))
        todo = [i for i in items if i not in done]
        note(phase=label, trials=len(todo), skipped=len(items) - len(todo))

        def one(item):
            key, harness, seed = item
            t = p["tasks"][key]
            d = out / "trials" / label / f"{t['id']}__{key.rsplit(':', 1)[1]}__{harness}__{seed}"
            try:
                r = trial(a.hub / t["path"], harness, d, renv, seed)
            except Exception as e:  # noqa: BLE001 - one broken trial is a result, not a crash of the job
                r = {"harness": harness, "seed": seed, "error": f"runner: {type(e).__name__}: {e}"}
            r["key"], r["dir"] = key, str(d.relative_to(out))
            return r

        with concurrent.futures.ThreadPoolExecutor(a.concurrency) as ex, open(res_f, "a") as f:
            futs = [ex.submit(one, i) for i in todo]
            for i, fut in enumerate(concurrent.futures.as_completed(futs)):  # written as they finish, so a re-run resumes
                f.write(json.dumps(fut.result()) + "\n")
                f.flush()
                if i % 10 == 0:
                    note(phase=label, done=i + 1, of=len(todo))
        got = {(r["key"], r["harness"], r.get("seed")): r for r in map(json.loads, res_f.read_text().splitlines())}
        # an infrastructure failure (the kernel killed a process for memory, a server that never started) says nothing
        # about MuJoCo: run it again, alone; the later line wins when the file is read
        again = [i for i in items if infra_failure(got.get(i))]
        if again:
            note(phase=label, retry_alone=len(again))
            with open(res_f, "a") as f:
                for item in again:
                    r = one(item)
                    r["retried_after"] = (got[item].get("error") or "")[-300:]
                    f.write(json.dumps(r) + "\n")
                    got[item] = r
        return got

    if unit.get("sweep"):  # a follow-up: given trials at given versions, nothing else (plan unit "sweep")
        sw_ = unit["sweep"]
        if sw_.get("pinned_keys"):
            batch("pinned-again", [(k, h, None) for k in sw_["pinned_keys"] for h in p["tasks"][k]["harnesses"]])
        for v in sw_["versions"]:
            switch({py: v for py in pythons}, renv, out / "setup.log")
            batch(f"at-{v}", [(k, h, None) for k in keys for h in sw_.get("harnesses") or p["tasks"][k]["harnesses"]])
        switch(pinned, renv, out / "setup.log")
        note(phase="done")
        return 0
    plan_items = [(k, h, None) for k in keys for h in p["tasks"][k]["harnesses"]]
    to_new = {py: p["mujoco"] for py in pythons}
    if not a.pinned_only_changed:
        base = batch("pinned", plan_items)
        sw = switch(to_new, renv, out / "setup.log")
        (out / "switch.json").write_text(json.dumps(sw, indent=1))
        new = batch("new", plan_items)
    else:
        # for suites too slow to run twice: the new version first, then the pinned version only where the new verdict is
        # not the expected one (and for --sample-unchanged tasks); elsewhere the baseline is the hub's published check
        # (every reference 1, every no-op 0 on the pinned runtime), marked "assumed"
        sw = switch(to_new, renv, out / "setup.log")
        (out / "switch.json").write_text(json.dumps(sw, indent=1))
        new = batch("new", plan_items)
        off = {k for (k, h, _s) in plan_items if verdict(new.get((k, h, None))) != p["tasks"][k]["expect"][h]}
        # a task that failed on the new version with an error needs no full pinned run each: a few confirm the baseline
        erred = sorted(k for k in off if any((new.get((k, h, None)) or {}).get("error") or (new.get((k, h, None)) or {}).get("robo_error")
                                             for h in p["tasks"][k]["harnesses"]))
        need = (off - set(erred)) | set(erred[: a.sample_unchanged])
        sample = [k for k in keys if k not in off][: a.sample_unchanged]
        switch(pinned, renv, out / "setup.log")
        base = batch("pinned", [(k, h, None) for (k, h, _s) in plan_items if k in need or k in sample])
        for k, h, _s in plan_items:
            if (k, h, None) not in base:
                base[(k, h, None)] = {"key": k, "harness": h, "seed": None, "reward": float(p["tasks"][k]["expect"][h]), "assumed": True}
        jl_assumed = [r for r in base.values() if r.get("assumed")]
        (out / "pinned-assumed.jsonl").write_text("".join(json.dumps(r) + "\n" for r in jl_assumed))
        switch(to_new, renv, out / "setup.log")
    changed_h = [(k, h) for (k, h, _s) in plan_items if verdict(base.get((k, h, None))) != verdict(new.get((k, h, None)))]
    changed = sorted({k for k, _h in changed_h})
    note(phase="changed", n=len(changed))
    # re-checks of the changed tasks (only the harnesses whose verdict changed): other seeds on the new version for up
    # to --recheck tasks; then, back on the pinned version, every changed trial again (repeatability) and the same seeds
    seeds = [int(s) for s in a.seeds.split(",") if s]
    sample = set(changed[: a.recheck])
    seed_items = [(k, h, s) for (k, h) in changed_h if k in sample for s in seeds]
    if seed_items:
        batch("new-seeds", seed_items)
    if changed:
        switch(pinned, renv, out / "setup.log")
        batch("pinned-repeat", [(k, h, None) for (k, h) in changed_h if k in set(changed[: a.repeat])])
        if seed_items:
            batch("pinned-seeds", seed_items)
    rels = [v for v in p.get("releases") or [] if vkey(pinned_v) < vkey(v) < vkey(p["mujoco"])]
    if changed and rels and a.bisect:
        # a full bisection for the first --bisect changed tasks; the others are checked at the boundaries found
        sample = changed[: a.bisect]
        chain = [pinned_v, *rels, p["mujoco"]]
        state = {k: (0, len(chain) - 1) for k in sample}  # verdict at chain[lo] is as pinned, at chain[hi] as new
        hist: dict[str, dict] = {k: {} for k in sample}
        while any(hi - lo > 1 for lo, hi in state.values()):
            mids: dict[int, list[str]] = {}
            for k, (lo, hi) in state.items():
                if hi - lo > 1:
                    mids.setdefault((lo + hi) // 2, []).append(k)
            for mid, ks in sorted(mids.items()):
                v = chain[mid]
                switch({py: v for py in pythons}, renv, out / "setup.log")
                got = batch(f"bisect-{v}", [(k, h, None) for (k, h) in changed_h if k in ks])
                for k in ks:
                    if any(infra_failure(got.get((k, h, None))) for (kk, h) in changed_h if kk == k):
                        hist[k][v] = "unknown (infrastructure failure)"  # killed twice: bisect the rest of the range
                        lo, hi = state[k]
                        state[k] = (mid, hi) if mid - lo <= hi - mid else (lo, mid)
                        continue
                    same = all(verdict(got.get((k, h, None))) == verdict(base.get((k, h, None)))
                               for (kk, h) in changed_h if kk == k)
                    hist[k][v] = "as pinned" if same else "as new"
                    lo, hi = state[k]
                    state[k] = (mid, hi) if same else (lo, mid)
        found = {k: {"first_changed": chain[hi], "last_unchanged": chain[lo], "checked": hist[k], "bisected": True}
                 for k, (lo, hi) in state.items()}
        # the other changed tasks: checked on each side of every boundary the bisection found
        rest = [k for k in changed if k not in found][: a.boundary]
        bounds = sorted({(f["last_unchanged"], f["first_changed"]) for f in found.values()}, key=lambda b: vkey(b[1]))
        for k in rest:
            found[k] = {"checked": {}, "bisected": False}
        for lo_v, hi_v in bounds:
            for v in (lo_v, hi_v):
                if v in (pinned_v, p["mujoco"]):
                    continue
                switch({py: v for py in pythons}, renv, out / "setup.log")
                got = batch(f"bisect-{v}", [(k, h, None) for (k, h) in changed_h if k in rest])
                for k in rest:
                    if any(infra_failure(got.get((k, h, None))) for (kk, h) in changed_h if kk == k):
                        found[k]["checked"][v] = "unknown (infrastructure failure)"
                        continue
                    same = all(verdict(got.get((k, h, None))) == verdict(base.get((k, h, None)))
                               for (kk, h) in changed_h if kk == k)
                    found[k]["checked"][v] = "as pinned" if same else "as new"
        for k in rest:
            for lo_v, hi_v in bounds:
                c = found[k]["checked"]
                if c.get(lo_v, "as pinned" if lo_v == pinned_v else None) == "as pinned" and c.get(
                        hi_v, "as new" if hi_v == p["mujoco"] else None) == "as new":
                    found[k].update(first_changed=hi_v, last_unchanged=lo_v)
                    break
        (out / "bisect.json").write_text(json.dumps(found, indent=1))
        switch(pinned, renv, out / "setup.log")
    # first diverging steps, computed here so the episode traces need not leave the machine
    reps = {(r["key"], r["harness"]): r for r in _load_jsonl(out / "pinned-repeat.jsonl")}
    div = {}
    for k, h, _s in plan_items:
        b, n = base.get((k, h, None)), new.get((k, h, None))
        if b and n and b.get("dir") and n.get("dir"):
            e = {"new": first_divergence(out / b["dir"], out / n["dir"])}
            if (k, h) in reps and reps[(k, h)].get("dir"):
                e["repeat"] = first_divergence(out / b["dir"], out / reps[(k, h)]["dir"])
            div[f"{k}|{h}"] = e
    (out / "divergence.json").write_text(json.dumps(div, indent=1))
    note(phase="done")
    return 0


INFRA = re.compile(r"\bKilled\b|worker died|MemoryError|Cannot allocate memory|out of memory|episode server did not start: $|No space left on device")


def infra_failure(r: dict | None) -> bool:
    return bool(r and r.get("reward") is None and INFRA.search(r.get("error") or ""))


def cpu_quota() -> float:
    """CPUs this container may use (cgroup v2 cpu.max), else the CPUs it can see."""
    try:
        q, per = Path("/sys/fs/cgroup/cpu.max").read_text().split()
        if q != "max":
            return int(q) / int(per)
    except (OSError, ValueError):
        pass
    return len(os.sched_getaffinity(0))


def verdict(r: dict | None) -> str | int | None:
    """1 or 0 from the reward; "error" when the trial produced no reward; None when it did not run."""
    if r is None:
        return None
    return "error" if r.get("reward") is None else int(r["reward"] >= 1)


# ---------------------------------------------------------------- diff and report


def _load_jsonl(p: Path) -> list[dict]:
    return [json.loads(x) for x in p.read_text().splitlines()] if p.exists() else []


def numeric_diff(a, b, tol: float) -> bool:
    """True when two JSON values differ (numbers beyond tol; volatile keys ignored)."""
    if isinstance(a, dict) and isinstance(b, dict):
        ks = (set(a) | set(b)) - VOLATILE
        return any(numeric_diff(a.get(k), b.get(k), tol) for k in ks)
    if isinstance(a, list) and isinstance(b, list):
        return len(a) != len(b) or any(numeric_diff(x, y, tol) for x, y in zip(a, b))
    if isinstance(a, bool) or isinstance(b, bool):
        return a != b
    if isinstance(a, (int, float)) and isinstance(b, (int, float)):
        return abs(a - b) > tol
    return a != b


def first_divergence(da: Path, db: Path) -> dict:
    """First env step at which two episodes differ: from physics.jsonl (end-effector point, simulator time and contact
    force per step) where the backend exposes MuJoCo state, else from trace.jsonl (every request and its response)."""
    out: dict = {}
    pa, pb = da / "episode" / "physics.jsonl", db / "episode" / "physics.jsonl"
    if pa.exists() and pb.exists():
        A, B = _load_jsonl(pa), _load_jsonl(pb)
        for tol, name in ((1e-5, "physics_step"), (1e-3, "physics_step_1mm")):
            for x, y in zip(A, B):
                if "ee" in x and "ee" in y and numeric_diff(x["ee"], y["ee"], tol):
                    out[name] = x.get("i")
                    break
        out["physics_samples"] = [len(A), len(B)]
    ta, tb = da / "episode" / "trace.jsonl", db / "episode" / "trace.jsonl"
    if ta.exists() and tb.exists():
        A, B = _load_jsonl(ta), _load_jsonl(tb)
        for i, (x, y) in enumerate(zip(A, B)):
            if numeric_diff(x.get("req"), y.get("req"), 0) or numeric_diff(x.get("resp"), y.get("resp"), 1e-6):
                out["trace_request"] = i
                out["trace_step"] = x.get("steps")
                break
        else:
            if len(A) != len(B):
                out["trace_request"] = min(len(A), len(B))
        out["trace_requests"] = [len(A), len(B)]
    return out


def error_sig(err: str | None) -> dict | None:
    """The exception line of a trial's error and the file of the frame that raised it."""
    if not err:
        return None
    lines = [x.strip() for x in err.splitlines() if x.strip()]
    exc = next((x for x in reversed(lines) if re.match(r"^[A-Za-z_.]*(Error|Exception|Exit|Interrupt)\b", x)), lines[-1] if lines else "")
    exc = re.sub(r"0x[0-9a-f]{6,}", "0x...", exc)
    frames = re.findall(r'File "([^"]+)", line (\d+)', err)

    def pkg_of(path: str) -> str:
        m = re.search(r"(?:site-packages|-src)/([A-Za-z0-9_]+)/", path)
        return m.group(1) if m else "robouse" if "/robouse/" in path else ""

    # the innermost frame in a suite package (a worker's own traceback comes before the server's in serve.log)
    third = [f for f in frames if pkg_of(f[0]) not in ("", "robouse", "mujoco")]
    where, line = (third or frames or [("", None)])[-1]
    return {"exception": exc[:300], "file": where, "line": int(line) if line else None, "package": pkg_of(where)}


def classify(row: dict) -> tuple[str, str]:
    """A first classification of a changed task from the evidence alone; the report's reader has the last word.

    our pin           a package the runtime pins (or its pins as a whole) does not work with the new MuJoCo: the trial
                      fails inside that package, or the runtime's requirements no longer resolve
    simulator change  the new MuJoCo behaves differently: an API or model-loading error in Robo Use's own code, or the
                      same actions giving another verdict while the pinned version repeats its own
    task fragility    the verdict is not stable on the pinned version itself (a repeat or another seed differs)"""
    hs = [h for h in ("reference", "noop") if (row.get(h) or {}).get("changed")]
    sigs = [row[h].get("new_error_sig") for h in hs if row[h].get("new_error_sig")]
    if sigs:
        sig = sigs[0]
        if sig["package"] == "robouse":
            return "simulator change", f"Robo Use's own code calls MuJoCo API that changed: {sig['exception']}"
        if sig["package"] and sig["package"] != "mujoco":
            return "our pin", f"fails inside the pinned package `{sig['package']}`: {sig['exception']}"
        if row.get("pin_conflict"):
            return "our pin", f"the runtime's requirements do not resolve with MuJoCo {row['new_mujoco']}; the trial failed: {sig['exception']}"
        return "simulator change", f"the new MuJoCo raised an error: {sig['exception']}"
    if any((row[h].get("repeat") not in (None, row[h].get("pinned"))) for h in hs):
        return "task fragility", "the pinned version gave another verdict on a second run of the same seed"
    sd = (row.get("seeds") or {}).get("reference")
    if sd and sd["pinned_pass"] < sd["n"]:
        return "task fragility", f"the reference solution also fails other seeds on the pinned version ({sd['pinned_pass']} of {sd['n']} pass)"
    return "simulator change", "same reference solution, different physics: the verdict changed while the pinned version repeats its own"


def diff(a: argparse.Namespace) -> int:
    p = json.loads(Path(a.plan).read_text())
    rows, units = [], {}
    for rdir in map(Path, a.results):
        meta_f = rdir / "meta.json"
        if not meta_f.exists():
            continue
        meta = json.loads(meta_f.read_text())
        prog = _load_jsonl(rdir / "progress.jsonl")
        units[meta["unit"]] = {
            "runtime": meta["runtime"], "pinned": meta.get("pinned"), "machine": meta.get("machine"), "cpus": meta.get("cpus"),
            "resolver": meta.get("resolver"),
            "switch": json.loads((rdir / "switch.json").read_text()) if (rdir / "switch.json").exists() else None,
            "finished": bool(prog and prog[-1].get("phase") == "done"),
            "started_utc": prog[0]["utc"] if prog else None, "ended_utc": prog[-1]["utc"] if prog else None,
        }
        bis = json.loads((rdir / "bisect.json").read_text()) if (rdir / "bisect.json").exists() else {}
        for bf in rdir.glob("bisect-*.jsonl"):
            v = bf.stem.removeprefix("bisect-")
            for r in _load_jsonl(bf):
                e = bis.get(r["key"])
                if e and (infra_failure(r) or "Segmentation fault" in (r.get("error") or "")) and v in e.get("checked", {}):
                    e["checked"][v] = "unknown (infrastructure failure)"
                    if v in (e.get("first_changed"), e.get("last_unchanged")):
                        e.pop("first_changed", None), e.pop("last_unchanged", None)
        divf = json.loads((rdir / "divergence.json").read_text()) if (rdir / "divergence.json").exists() else None
        sets = {lab: {(r["key"], r["harness"], r.get("seed")): r for r in _load_jsonl(rdir / f"{lab}.jsonl")}
                for lab in ("pinned", "new", "new-seeds", "pinned-repeat", "pinned-seeds")}
        for r in _load_jsonl(rdir / "pinned-assumed.jsonl"):  # --pinned-only-changed: the hub's published baseline
            sets["pinned"].setdefault((r["key"], r["harness"], None), r)
        pinned_v = meta["pinned"][meta["simulator_python"]] if meta.get("pinned") else None
        pin_conflict = [f for f, r in (meta.get("resolver") or {}).items() if not r["resolves"]]
        for key in p["units"][meta["unit"]]["tasks"]:
            t = p["tasks"][key]
            row = {"key": key, "task": t["id"], "runtime": meta["runtime"], "suite": t["suite"], "backend": t["backend"],
                   "datasets": t["datasets"], "pinned_mujoco": pinned_v, "new_mujoco": p["mujoco"], "expect": t["expect"],
                   "pin_conflict": pin_conflict}
            ran = False
            for h in t["harnesses"]:
                b, n = sets["pinned"].get((key, h, None)), sets["new"].get((key, h, None))
                if b is None and n is None:
                    continue
                ran = True
                e = {"pinned": verdict(b), "new": verdict(n)}
                for lab, r in (("pinned", b), ("new", n)):
                    if r and r.get("assumed"):
                        e["pinned_assumed"] = True
                    elif r:
                        e[f"{lab}_reward"], e[f"{lab}_outcome"], e[f"{lab}_steps"] = r.get("reward"), r.get("outcome"), r.get("steps_used")
                        if r.get("robo_error"):
                            e[f"{lab}_robo_error"] = r["robo_error"]
                            e.setdefault(f"{lab}_error_sig", error_sig(r["robo_error"]))
                        if r.get("error"):
                            e[f"{lab}_error"] = r["error"][-1500:]
                            log = rdir / r.get("dir", "-") / "episode" / "serve.log"  # the whole log when it came back
                            e[f"{lab}_error_sig"] = error_sig(log.read_text(errors="replace") if log.is_file() else r["error"])
                rep = sets["pinned-repeat"].get((key, h, None))
                if rep is not None:
                    e["repeat"] = verdict(rep)
                if divf is not None:
                    dv = divf.get(f"{key}|{h}") or {}
                    e["divergence"], e["repeat_divergence"] = dv.get("new"), dv.get("repeat")
                elif b and n and b.get("dir") and n.get("dir"):
                    e["divergence"] = first_divergence(rdir / b["dir"], rdir / n["dir"])
                    if rep and rep.get("dir"):
                        e["repeat_divergence"] = first_divergence(rdir / b["dir"], rdir / rep["dir"])
                e["changed"] = e["pinned"] != e["new"]
                row[h] = e
            if not ran:
                continue
            for h in t["harnesses"]:
                ss = sorted({s for (k, hh, s) in sets["new-seeds"] if k == key and hh == h})
                if ss:
                    def ok(lab, s, h=h):
                        return verdict(sets[lab].get((key, h, s))) == t["expect"][h]
                    row.setdefault("seeds", {})[h] = {"seeds": ss, "n": len(ss), "pinned_pass": sum(ok("pinned-seeds", s) for s in ss),
                                                      "new_pass": sum(ok("new-seeds", s) for s in ss)}
            if key in bis:
                row["bisect"] = dict(bis[key])
                # the narrowest range the checks support: the last release known to behave as pinned, the first as new
                b0 = row["bisect"]
                chk = {pinned_v: "as pinned", p["mujoco"]: "as new",
                       **({b0["last_unchanged"]: "as pinned"} if b0.get("last_unchanged") else {}),
                       **({b0["first_changed"]: "as new"} if b0.get("first_changed") else {}), **b0.get("checked", {})}
                hi = min((v for v, c in chk.items() if c == "as new"), key=vkey)
                lo = max((v for v, c in chk.items() if c == "as pinned" and vkey(v) < vkey(hi)), key=vkey)
                rel = [v for v in p.get("releases") or [] if vkey(lo) < vkey(v) <= vkey(hi)]
                row["bisect"].update(last_unchanged=lo, first_changed=hi, exact=len(rel) <= 1, releases_between=rel[:-1])
            exp = t["expect"]
            row["pinned_ok"] = all((row.get(h) or {}).get("pinned") == v for h, v in exp.items())
            row["new_ok"] = all((row.get(h) or {}).get("new") == v for h, v in exp.items())
            row["changed"] = any((row.get(h) or {}).get("changed") for h in t["harnesses"])
            if row["changed"]:
                row["class"], row["why"] = classify(row)
            rows.append(row)
    planned = {k for u in units for k in p["units"][u]["tasks"]}
    out = {"mujoco": p["mujoco"], "hub_commit": p["hub_commit"], "plan_created_utc": p["created_utc"],
           "runtimes": {rt: {"pins": r["pins"], "sim_mujoco": r["sim_mujoco"], "tasks": len(r["tasks"])} for rt, r in p["runtimes"].items()},
           "skipped_runtimes": p["skipped_runtimes"], "units": units,
           "summary": {"tasks_planned": len(p["tasks"]), "tasks_in_units_run": len(planned), "tasks_with_results": len(rows),
                       "changed": sum(r["changed"] for r in rows), "pinned_ok": sum(r["pinned_ok"] for r in rows),
                       "new_ok": sum(r["new_ok"] for r in rows),
                       "classes": dict(Counter(r.get("class") for r in rows if r["changed"]))},
           "tasks": rows}
    Path(a.out).write_text(json.dumps(out, indent=1))
    print(json.dumps(out["summary"]))
    return 0


def fmt(v) -> str:
    if v is None:
        return "not run"
    return str(v)


def first_step(e: dict | None) -> str:
    dv = (e or {}).get("divergence") or {}
    if dv.get("physics_step") is not None:
        return f"{dv['physics_step']}" + (f" (1 mm at {dv['physics_step_1mm']})" if dv.get("physics_step_1mm") is not None else "")
    if dv.get("trace_step") is not None:
        return f"{dv['trace_step']} (trace)"
    return "none" if dv else "– (no episode on one side)" if (e or {}).get("new") == "error" or (e or {}).get("pinned") == "error" else "–"


def bisect_text(b: dict | None) -> str:
    """"3.12.0" when the change is pinned to one release, else "3.3.1 to 3.4.0" (the releases it may have come with)."""
    if not b or not b.get("first_changed"):
        return ""
    if b.get("exact", True) or not b.get("releases_between"):
        return b["first_changed"]
    return f"{b['releases_between'][0]} to {b['first_changed']}"


def report(a: argparse.Namespace) -> int:
    d = json.loads(Path(a.diff).read_text())
    rows, s = d["tasks"], d["summary"]
    # reviewed attributions: [{"runtime", "match" (a substring of the change's description), "class", "evidence"}]
    notes = json.loads(Path(a.notes).read_text()) if a.notes else []
    for r in rows:
        if not r["changed"]:
            continue
        sig = next(((r.get(h) or {}).get("new_error_sig") or {} for h in ("reference", "noop") if (r.get(h) or {}).get("new_error_sig")), {})
        what = sig.get("exception") or r.get("why") or ""
        for n in notes:
            if n["runtime"] == r["runtime"] and n["match"] in f"{what} {r['task']}":
                r["class"], r["evidence"] = n["class"], n["evidence"]
                break
    s["classes"] = dict(Counter(r.get("class") for r in rows if r["changed"]))
    by: dict[str, list] = {}
    for r in rows:
        by.setdefault(r["runtime"], []).append(r)
    L = [f"# MuJoCo {d['mujoco']} on the hub's MuJoCo-based tasks", ""]
    L.append(f"Each task's reference solution and no-op ran on its runtime as pinned and on a copy of that runtime with only MuJoCo switched to {d['mujoco']} (hub commit `{(d['hub_commit'] or '')[:10]}`). {s['tasks_with_results']} of {s['tasks_planned']} tasks have results; {s['changed']} changed verdict.")
    L += ["", "| Runtime | Pinned MuJoCo | Tasks | Reference 1, pinned / new | No-op 0, pinned / new | Changed | First release that changes |", "|---|---|---|---|---|---|---|"]
    for rt, rs in sorted(by.items()):
        def cnt(h, lab, v):
            return sum(1 for r in rs if (r.get(h) or {}).get(lab) == v)
        nref = sum(1 for r in rs if r["expect"].get("reference") == 1)
        nnoop = sum(1 for r in rs if "noop" in r) + sum(1 for r in rs if r["expect"].get("reference") == 0)
        refp = sum(1 for r in rs if r["expect"].get("reference") == 1 and r["reference"]["pinned"] == 1)
        refn = sum(1 for r in rs if r["expect"].get("reference") == 1 and r["reference"]["new"] == 1)
        noopp = cnt("noop", "pinned", 0) + sum(1 for r in rs if r["expect"].get("reference") == 0 and r["reference"]["pinned"] == 0)
        noopn = cnt("noop", "new", 0) + sum(1 for r in rs if r["expect"].get("reference") == 0 and r["reference"]["new"] == 0)
        firsts = Counter(bisect_text(r.get("bisect")) for r in rs if r["changed"] and r.get("bisect"))
        L.append(f"| {rt} | {rs[0]['pinned_mujoco']} | {len(rs)} | {refp} / {refn} of {nref} | {noopp} / {noopn} of {nnoop} | {sum(r['changed'] for r in rs)} | {', '.join(f'{v} ({n})' for v, n in firsts.most_common()) or '–'} |")
    L += ["", "A no-op count includes the tasks of benchflow/robouse-noop-control, whose reference solution is the no-op. \"error\" is a trial that produced no reward. The first release that changes comes from a bisection over the MuJoCo releases in between (a sample of the changed tasks, then the others checked on both sides of the boundary).", ""]
    ch = sorted((r for r in rows if r["changed"]), key=lambda r: (r["runtime"], r["task"]))
    groups: dict[tuple, list] = {}
    for r in ch:
        sig = next(((r.get(h) or {}).get("new_error_sig") or {} for h in ("reference", "noop") if (r.get(h) or {}).get("new_error_sig")), {})
        groups.setdefault((r["runtime"], r.get("class"), sig.get("exception") or r.get("why"), r.get("evidence", "")), []).append(r)
    if groups:
        L += ["## What changed", ""]
        for (rt, cls, what, ev), rs in sorted(groups.items(), key=lambda kv: (kv[0][0], -len(kv[1]))):
            firsts = sorted({bisect_text(r.get("bisect")) for r in rs if r.get("bisect")})
            names = ", ".join(f"`{r['task']}`" for r in rs[:6]) + (f" and {len(rs) - 6} more" if len(rs) > 6 else "")
            if ev and what.startswith("same reference solution"):  # the reviewed evidence says it better
                what = ""
            what = what if not what or what.endswith((".", "!", "?")) else what + "."
            L.append(f"- **{rt}, {len(rs)} task{'s' if len(rs) != 1 else ''} ({cls}):**" + (f" {what}" if what else "") + (f" First release with the change: {', '.join(firsts)}." if firsts else "") + (f" {ev}" if ev else "") + f" Tasks: {names}.")
        L.append("")
    if ch:
        L += ["## Changed tasks", "", "| Task | Datasets | Pinned | New | Reference, pinned → new | No-op, pinned → new | First diverging step | Class |", "|---|---|---|---|---|---|---|---|"]
        for r in ch:
            ref, nop = r.get("reference") or {}, r.get("noop") or {}
            L.append(f"| `{r['task']}` | {', '.join(r['datasets'])} | {r['pinned_mujoco']} | {r['new_mujoco']} | {fmt(ref.get('pinned'))} → {fmt(ref.get('new'))} | "
                     f"{fmt(nop.get('pinned')) + ' → ' + fmt(nop.get('new')) if nop else 'the reference is the no-op'} | {first_step(ref)} | {r.get('class')} |")
    Path(a.out).write_text("\n".join(L) + "\n")
    print(f"wrote {a.out}")
    return 0


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--hub", type=Path, default=HUB, help="robohub checkout (default: this one)")
    sub = ap.add_subparsers(dest="cmd", required=True)
    pl = sub.add_parser("plan")
    pl.add_argument("--mujoco", required=True)
    pl.add_argument("--out", required=True)
    pl.add_argument("--runtimes")
    pl.add_argument("--shards", help="RUNTIME=N,... split a runtime over N jobs")
    pl.add_argument("--no-releases", dest="releases", action="store_false", help="skip the PyPI release list (no bisection)")
    pl.set_defaults(func=plan)
    va = sub.add_parser("variant")
    va.add_argument("runtime")
    va.add_argument("--mujoco", required=True)
    va.add_argument("--out", required=True)
    va.set_defaults(func=variant)
    jb = sub.add_parser("job")
    jb.add_argument("--plan", required=True)
    jb.add_argument("--unit", required=True)
    jb.add_argument("--out", required=True)
    jb.add_argument("--concurrency", type=int, default=4)
    jb.add_argument("--limit", type=int, default=0, help="first N tasks of the unit only")
    jb.add_argument("--seeds", default="1,2", help="extra seeds for changed tasks (default 1,2; empty: none)")
    jb.add_argument("--bisect", type=int, default=4, help="bisect at most N changed tasks per unit (0: none)")
    jb.add_argument("--pinned-only-changed", action="store_true",
                    help="run the new version first and the pinned version only where the new verdict is not the expected one")
    jb.add_argument("--sample-unchanged", type=int, default=2, help="with --pinned-only-changed: also run the pinned version on N tasks")
    jb.add_argument("--repeat", type=int, default=24, help="run the pinned version again for at most N changed tasks")
    jb.add_argument("--recheck", type=int, default=8, help="run the extra seeds for at most N changed tasks per unit")
    jb.add_argument("--boundary", type=int, default=12,
                    help="check at most N further changed tasks on both sides of each boundary the bisection found")
    jb.set_defaults(func=job)
    df = sub.add_parser("diff")
    df.add_argument("--plan", required=True)
    df.add_argument("--results", nargs="+", required=True)
    df.add_argument("--out", required=True)
    df.set_defaults(func=diff)
    rp = sub.add_parser("report")
    rp.add_argument("--diff", required=True)
    rp.add_argument("--out", required=True)
    rp.add_argument("--notes", help="JSON list of reviewed attributions (runtime, match, class, evidence)")
    rp.set_defaults(func=report)
    a = ap.parse_args(argv)
    a.hub = a.hub.resolve()
    return a.func(a)


if __name__ == "__main__":
    sys.exit(main())
