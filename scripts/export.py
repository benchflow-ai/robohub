"""Export Robo Use tasks from a Robo Use checkout into this hub as native BenchFlow task packages.

    <robouse>/.venv/bin/python scripts/export.py --robouse ~/benchflow/robouse [--datasets benchflow/robouse-core,benchflow/robouse-drone]

A dataset with `noop_of: <dataset>` in hub.yaml is a negative control: the same tasks with reference solutions
that only look at the robot and exit, so the oracle agent must score 0 on every task.

Reads hub.yaml (datasets, runtimes) and core.txt, and writes:

    runtimes/<name>/                 simulator image build context shared by the tasks that use it
      Dockerfile  requirements*.txt  sim-entry.sh  check_runtime.py  robouse/src/robouse/  [fetch_assets.py assets/]
    datasets/<dir>/<task-id>/        one native BenchFlow package (task.md schema 1.3) per task
      task.md
      environment/Dockerfile           agent (`main`) image: python + numpy + the `robo` client only
      environment/robo                 Robo Use's agent_cli.py (standard library only)
      environment/docker-compose.yaml  `main` + trusted `simulator` (built from ../../../../runtimes/<name>,
                                       checked against the runtime digest pinned here)
      verifier/{verifier.md,test.sh,verify.py}   runs in `simulator`
      oracle/solve.sh + oracle/vendor/ reference solution; drives the robot through the socket only
    export.json                      what was exported: Robo Use commit, runtime digests, task -> suite/runtime

A runtime with `native: true` in hub.yaml (composed tasks) is written by Robo Use's own exporter
(robouse.engine.format, on BenchFlow's embodied layer), so this script then needs BenchFlow in the same Python.
`--noop-out DIR` writes every selected dataset with no-op reference solutions to DIR instead (a local negative control).
`--runtimes RT,...` (with --datasets) rewrites only those runtimes and, in the selected datasets, only the tasks that run on
them: a fix release from a newer Robo Use leaves every other package byte-identical. export.json then records the newer
Robo Use commit on each rewritten task. The checkout's version must equal hub.yaml robouse.package_version.

The same task has byte-identical packages in every dataset that contains it (same digest). Run with the Robo Use
dev venv: it reads task folders with robouse.tasks (PyYAML) and vendors Meta-World's scripted expert policies from
the installed `metaworld` package. Re-running is idempotent: packages hold no oracle token (the host sets
$ROBOUSE_ORACLE_TOKEN per run; see ORACLE_TOKEN_ENV), so an unchanged task keeps its digest.
"""
from __future__ import annotations

import argparse
import ast
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

import yaml

HUB = Path(__file__).resolve().parents[1]
TEMPLATES = HUB / "templates"
WALL_MARGIN_S = 900  # the episode clock starts when the simulator is up, before BenchFlow installs the harness
PKG_IGNORE = shutil.ignore_patterns("__pycache__", "*.pyc", ".DS_Store", "bundled_tasks",
                                    "engine", "assets_provenance")
# The oracle token that unlocks privileged state (vision tasks) is never stored in a package. The host chooses it per
# run: the simulator reads $ROBOUSE_ORACLE_TOKEN through compose interpolation, and the reference solution gets the
# same variable through task.md `oracle.env`, which BenchFlow passes to the oracle only. Unset means no token, and
# then nothing is privileged. Run reference solutions with ROBOUSE_ORACLE_TOKEN=$(openssl rand -hex 16).
ORACLE_TOKEN_ENV = "ROBOUSE_ORACLE_TOKEN"
TOKEN_REF = "${" + ORACLE_TOKEN_ENV + ":-}"
METAWORLD_LICENSE = """\
MIT License

Copyright (c) 2019 Meta-World Team

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""


# ---- helpers ----------------------------------------------------------------------------------------------

def digest(root: Path) -> str:
    """BenchFlow's task digest (benchflow._utils.task_authoring.task_digest), reimplemented with the stdlib."""
    files = []
    for dirpath, _dirs, names in os.walk(root):
        for name in names:
            if name == ".benchflow-source.json":
                continue
            p = Path(dirpath) / name
            if p.is_symlink() or not p.is_file():
                continue
            files.append((p.relative_to(root).as_posix(), p))
    h = hashlib.sha256()
    for rel, p in sorted(files):
        h.update(rel.encode("utf-8"))
        h.update(b"\x00")
        h.update(hashlib.sha256(p.read_bytes()).digest())
    return f"sha256:{h.hexdigest()}"


def _dump(d: dict) -> str:
    return yaml.safe_dump(d, sort_keys=False, allow_unicode=True, width=10**6)


def git_head(repo: Path) -> tuple[str, bool]:
    sha = subprocess.run(["git", "-C", str(repo), "rev-parse", "HEAD"], capture_output=True, text=True, check=True).stdout.strip()
    dirty = subprocess.run(["git", "-C", str(repo), "status", "--porcelain", "--", "src", "tasks", "assets"],
                           capture_output=True, text=True, check=True).stdout.strip()
    return sha, bool(dirty)


# ---- runtimes ---------------------------------------------------------------------------------------------

def write_runtime(name: str, cfg: dict, robouse: Path, prov: dict) -> str:
    """runtimes/<name>: the template files + this Robo Use package. Returns the runtime digest."""
    dst = HUB / "runtimes" / name
    if dst.exists():
        shutil.rmtree(dst)
    shutil.copytree(TEMPLATES / "runtimes" / name, dst)
    for f in ("sim-entry.sh", "check_runtime.py"):
        shutil.copy2(TEMPLATES / "runtimes" / f, dst / f)
    (dst / "sim-entry.sh").chmod(0o755)
    shutil.copytree(robouse / "src" / "robouse", dst / "robouse" / "src" / "robouse", ignore=PKG_IGNORE)
    manifests = cfg.get("assets") or {}
    if manifests == "menagerie":  # the 0.1 runtimes
        manifests = {"menagerie.json": "assets/menagerie/provenance.json",
                     "skydio_x2.json": "assets/menagerie/skydio_x2/PROVENANCE.json"}
    if manifests:  # {file name in runtimes/<name>/assets/: pinned-download manifest in the Robo Use checkout}
        shutil.copy2(TEMPLATES / "runtimes" / "fetch_assets.py", dst / "fetch_assets.py")
        (dst / "assets").mkdir()
        for out, rel in manifests.items():
            shutil.copy2(robouse / rel, dst / "assets" / out)
    (dst / "SOURCE.json").write_text(json.dumps({"robouse": prov, "runtime": name}, indent=2) + "\n")
    return digest(dst)


# ---- oracle vendoring -------------------------------------------------------------------------------------

def _module_file(src: Path, mod: str) -> Path | None:
    """robouse.backends.tabletop -> src/robouse/backends/tabletop.py (or the package __init__)."""
    base = src.joinpath(*mod.split("."))
    if base.with_suffix(".py").is_file():
        return base.with_suffix(".py")
    if (base / "__init__.py").is_file():
        return base / "__init__.py"
    return None


def _module_level(tree: ast.Module):
    """Nodes executed at import time: top-level statements, descending into if/try but not into defs."""
    todo = list(tree.body)
    while todo:
        node = todo.pop()
        yield node
        if isinstance(node, (ast.If, ast.Try)):
            for field in ("body", "orelse", "finalbody"):
                todo += getattr(node, field, [])
            todo += [s for h in getattr(node, "handlers", []) for s in h.body]


def _closure(src: Path, roots: list[str]) -> set[Path]:
    """Files of the robouse modules reachable from `roots` by static imports (any import anywhere in a module,
    including inside functions), plus the __init__.py of every package on the way."""
    seen: set[Path] = set()
    todo = list(roots)
    while todo:
        mod = todo.pop()
        f = _module_file(src, mod)
        if f is None or f in seen:
            continue
        seen.add(f)
        parts = mod.split(".")
        for i in range(1, len(parts)):  # parent packages
            todo.append(".".join(parts[:i]))
        pkg = parts if f.name == "__init__.py" else parts[:-1]
        tree = ast.parse(f.read_text())
        # a package __init__ contributes only its module-level imports (backends/__init__.py imports every
        # backend lazily inside make_backend, which an oracle never calls); other modules contribute all of them
        nodes = _module_level(tree) if f.name == "__init__.py" else ast.walk(tree)
        for node in nodes:
            if isinstance(node, ast.ImportFrom):
                if node.level:
                    base = pkg[: len(pkg) - node.level + 1]
                    target = base + (node.module.split(".") if node.module else [])
                else:
                    target = (node.module or "").split(".")
                if target and target[0] == "robouse":
                    todo.append(".".join(target))
                    todo += [".".join(target + [a.name]) for a in node.names]
            elif isinstance(node, ast.Import):
                todo += [a.name for a in node.names if a.name.split(".")[0] == "robouse"]
    return seen


# pip packages a backend's reference solution imports besides numpy (installed at oracle run time, never into the image)
ORACLE_PIP = {"dexjoco": ["scipy==1.18.1"]}
ORACLE_BACKEND_MODULE: dict[str, str] = {}  # overrides from hub.yaml; otherwise robouse.oracle.oracle_module(backend)


def configure_oracles(hub: dict) -> None:
    """Per-runtime oracle needs from hub.yaml: `oracle_pip: {backend: [req, ...]}` (installed at oracle run time)
    and `oracle_modules: {backend: module}` (the module a backend's reference solution imports, if not
    robouse.backends.<backend>)."""
    for cfg in hub["runtimes"].values():
        for b, reqs in (cfg.get("oracle_pip") or {}).items():
            ORACLE_PIP[b] = list(reqs)
        ORACLE_BACKEND_MODULE.update(cfg.get("oracle_modules") or {})


def vendor_robouse(robouse: Path, backend: str, dst: Path) -> None:
    src = robouse / "src"
    if backend in ORACLE_BACKEND_MODULE:
        module = ORACLE_BACKEND_MODULE[backend]
    else:
        from robouse.oracle import oracle_module  # the module `python -m robouse.oracle --backend <backend>` runs

        module = oracle_module(backend)
    if _module_file(src, module) is None:
        raise SystemExit(f"backend {backend}: its reference solution module {module} is not in {src}")
    roots = ["robouse.oracle", "robouse.agent_cli", module]
    for f in sorted(_closure(src, roots)):
        out = dst / f.relative_to(src)
        out.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(f, out)


def vendor_metaworld_policy(env: str, vendor: Path) -> None:
    """Meta-World's scripted expert for `env` (pure numpy), from the installed metaworld package."""
    from metaworld.policies import ENV_POLICY_MAP

    cls = ENV_POLICY_MAP[env]
    src_dir = Path(sys.modules[cls.__module__].__file__).parent
    dst = vendor / "metaworld" / "policies"
    dst.mkdir(parents=True)
    (vendor / "metaworld" / "__init__.py").write_text(
        '"""Vendored subset of Meta-World (MIT): only the scripted expert policy this task needs."""\n')
    (vendor / "metaworld" / "LICENSE").write_text(METAWORLD_LICENSE)  # MIT: the notice travels with the code
    mod = cls.__module__.rsplit(".", 1)[1]
    for name in ("action.py", "policy.py", f"{mod}.py"):
        shutil.copy2(src_dir / name, dst / name)
    (dst / "__init__.py").write_text(
        f"from metaworld.policies.{mod} import {cls.__name__}\n\nENV_POLICY_MAP = {{{env!r}: {cls.__name__}}}\n")


NOOP_SOLVE = """#!/bin/bash
# Negative control: look at the robot, never move it, exit without `robo done`.
# The verifier must close the episode itself (outcome agent_exited) and the reward must be 0.
set -euo pipefail
robo info
robo observe
"""


def write_oracle(task, robouse: Path, dst: Path, noop: bool) -> None:
    dst.mkdir(parents=True)
    if noop:
        (dst / "solve.sh").write_text(NOOP_SOLVE)
        (dst / "solve.sh").chmod(0o755)
        return
    spec = task.spec
    source = task.oracle_script.read_text()
    backends = {spec["backend"], *re.findall(r"--backend[ =]([a-z_]+)", source)}  # hard tasks run tabletop_hard
    vendor = dst / "vendor"
    for b in sorted(backends):
        vendor_robouse(robouse, b, vendor)
    if spec["backend"] == "metaworld":
        vendor_metaworld_policy(spec["env"], vendor)
    pip = sorted({req for b in backends for req in ORACLE_PIP.get(b, [])})
    src_oracle = task.oracle_script.parent
    for f in sorted(src_oracle.iterdir()):  # data the reference solution reads (demo.json, demo_actions.json)
        if f.is_file() and f.name != "solve.sh" and not f.name.startswith("."):
            shutil.copy2(f, dst / f.name)
    body = [ln for ln in source.splitlines()
            if ln.strip() and not ln.startswith("#") and not ln.startswith("set ")]
    body = [re.sub(r"^python3? ", "python3 ", ln) for ln in body]
    (dst / "solve.sh").write_text("\n".join([
        "#!/bin/bash",
        "# Reference solution. Runs in the agent container and drives the robot only through the robo socket",
        "# ($ROBOUSE_SOCKET), like an agent would. vendor/ holds the Robo Use oracle code (and, for Meta-World,",
        "# the scripted expert policy). BenchFlow uploads /oracle only in oracle mode; agents never see it.",
        "set -euo pipefail",
        'export PYTHONPATH="$(cd "$(dirname "$0")" && pwd)/vendor${PYTHONPATH:+:$PYTHONPATH}"',
        *([f"# the reference solution needs {', '.join(pip)} (the agent image has only numpy); install it outside /oracle",
           f"python3 -m pip install --quiet --disable-pip-version-check --no-cache-dir --target /tmp/robohub-oracle-deps {' '.join(pip)}",
           'export PYTHONPATH="$PYTHONPATH:/tmp/robohub-oracle-deps"'] if pip else []),
        *body,
        "",
    ]))
    (dst / "solve.sh").chmod(0o755)


# ---- one task ---------------------------------------------------------------------------------------------

def native_task_md(meta: dict, instruction: str, prompt_prefix: str, noop: bool = False) -> str:
    rb = meta["robouse"]
    task = dict(meta.get("task") or {})
    task.setdefault("name", f"robouse/{rb.get('id')}")
    metadata = {"author_name": "benchflow", **(meta.get("metadata") or {}), "robouse": rb}
    verifier_timeout = max(300, int((meta.get("verifier") or {}).get("timeout_sec", 0) or 0))
    fm = {
        "schema_version": "1.3",
        "task": task,
        "metadata": metadata,
        "agent": {"timeout_sec": int(float((meta.get("agent") or {}).get("timeout_sec", 900)))},
        "verifier": {"service": "simulator", "user": "root", "timeout_sec": verifier_timeout},
        "sandbox": {"cpus": 1, "memory_mb": 2048, "build_timeout_sec": 3600},
    }
    if not noop:  # the reference solution's copy of the host's per-run token (BenchFlow passes oracle.env to the oracle only)
        fm["oracle"] = {"env": {ORACLE_TOKEN_ENV: TOKEN_REF}}
    return "---\n" + _dump(fm) + "---\n\n" + prompt_prefix + instruction.strip() + "\n"


REMOTE_WORKER = """\
    # This runtime's simulator runs on a remote GPU worker (remote_worker in hub.yaml): the episode server reaches it
    # over HTTPS, so this service keeps network access, and it reads the worker endpoints (URL, shared secret) from
    # the host file named by $__REMOTE_ENV__, mounted read-only here only; the agent container never sees it.
"""


def compose(meta: dict, context: str, runtime_digest: str, sim_memory: str,
            remote_worker: bool | str = False) -> str:
    """`remote_worker`: False, or the name of the environment variable holding the worker endpoints file
    (hub.yaml `remote_worker: true` means ROBOUSE_BEHAVIOR_REMOTE; a string names another one)."""
    sim_fm = {k: meta[k] for k in ("schema_version", "task", "metadata", "agent", "verifier", "robouse") if k in meta}
    sim_md = "---\n" + _dump(sim_fm) + "---\n"
    block = "\n".join("        " + ln.replace("$", "$$") for ln in sim_md.rstrip().splitlines())
    agent_timeout = float((meta.get("agent") or {}).get("timeout_sec", 900))
    text = (TEMPLATES / "docker-compose.yaml").read_text()
    text = (text.replace("__SIM_CONTEXT__", json.dumps(context))
                .replace("__RUNTIME_DIGEST__", runtime_digest)
                .replace("__MAX_WALL_S__", str(int(agent_timeout + WALL_MARGIN_S)))
                .replace("__SIM_MEMORY__", sim_memory)
                .replace("__TASK_MD__", block))
    if remote_worker:
        var = remote_worker if isinstance(remote_worker, str) else "ROBOUSE_BEHAVIOR_REMOTE"
        what = var.removeprefix("ROBOUSE_").removesuffix("_REMOTE")
        text = text.replace("    network_mode: none\n", REMOTE_WORKER.replace("__REMOTE_ENV__", var))
        text = text.replace("      - ${HOST_ARTIFACTS_PATH}:/logs/artifacts\n",
                            "      - ${HOST_ARTIFACTS_PATH}:/logs/artifacts\n"
                            f"      - ${{{var}:?set {var} to the {what} worker "
                            "endpoints file}:/run/robouse/remote.json:ro\n")
        text = text.replace("    environment:\n      # Chosen per run", "    environment:\n"
                            f"      {var}: /run/robouse/remote.json\n      # Chosen per run")
    return text


def export_task(task, robouse: Path, out_parent: Path, runtime: str, runtime_digest: str, sim_memory: str,
                prompt_prefix: str, noop: bool = False, remote_worker: bool | str = False) -> Path:
    meta = json.loads(json.dumps(task.meta))
    dst = out_parent / task.id
    if dst.exists():
        shutil.rmtree(dst)
    env = dst / "environment"
    env.mkdir(parents=True)
    (dst / "task.md").write_text(native_task_md(meta, task.instruction, prompt_prefix, noop))
    shutil.copy2(TEMPLATES / "agent" / "Dockerfile", env / "Dockerfile")
    (env / "robo").write_text("#!/usr/bin/env python3\n" + (robouse / "src" / "robouse" / "agent_cli.py").read_text())
    (env / "robo").chmod(0o755)
    context = os.path.relpath(HUB / "runtimes" / runtime, env)
    (env / "docker-compose.yaml").write_text(compose(meta, context, runtime_digest, sim_memory, remote_worker))
    shutil.copytree(TEMPLATES / "verifier", dst / "verifier", ignore=shutil.ignore_patterns("__pycache__"))
    (dst / "verifier" / "test.sh").chmod(0o755)
    write_oracle(task, robouse, dst / "oracle", noop)
    return dst


def export_native(ids: list[str], by_id: dict, suite_of: dict, out_parent: Path, runtime: str,
                  noop: bool) -> tuple[dict, str]:
    """Tasks of a `native: true` runtime, written by Robo Use's own exporter (robouse.engine.format, on BenchFlow's
    embodied layer; needs BenchFlow in this Python): its simulator build context goes to runtimes/<runtime>/, the
    packages to out_parent/<task>/, with the compose build context pointed at runtimes/<runtime>. Returns the tasks
    (for export.json) and the runtime digest."""
    import tempfile

    from robouse.engine.format import materialize

    rt_dir = HUB / "runtimes" / runtime
    tasks = {}
    with tempfile.TemporaryDirectory() as tmp:
        for tid in ids:
            materialize(by_id[tid].path, Path(tmp), noop=noop, flat=True)
        built = sorted(Path(tmp).glob("runtime-*"))
        if len(built) != 1:
            raise SystemExit(f"runtime {runtime}: expected one native runtime, got {[p.name for p in built]}")
        if rt_dir.exists():
            shutil.rmtree(rt_dir)
        shutil.copytree(built[0], rt_dir)
        if out_parent.exists():
            shutil.rmtree(out_parent)
        out_parent.mkdir(parents=True)
        for tid in ids:
            src, dst = Path(tmp) / "tasks" / tid, out_parent / tid
            shutil.copytree(src, dst)
            cf = dst / "environment" / "docker-compose.yaml"
            old = json.dumps(os.path.relpath(built[0], src / "environment"))
            text = cf.read_text()
            if old not in text:
                raise SystemExit(f"{tid}: build context {old} not found in its compose file")
            text = text.replace(old, json.dumps(os.path.relpath(rt_dir, dst / "environment")))
            # BenchFlow's `robo` client gives up on any request after ROBO_TIMEOUT_S (default 120 s), shorter than a long
            # skill; wait as long as Robo Use's own client does for requests that step the simulator (one hour). The
            # episode server bounds skills itself (step caps, the episode's wall-clock budget).
            sock = "      ROBO_SOCKET: /rpc/episode.sock\n"
            if text.count(sock) != 1:
                raise SystemExit(f"{tid}: no single `main` ROBO_SOCKET line in its compose file")
            cf.write_text(text.replace(sock, sock + '      ROBO_TIMEOUT_S: "3600"\n'))
            marker = dst / ".robouse-source.json"
            info = json.loads(marker.read_text())
            info["runtime"] = runtime  # runtimes/<runtime>, not the exporter's content-addressed folder name
            marker.write_text(json.dumps(info, indent=2))
            tasks[tid] = {"suite": suite_of[tid], "backend": by_id[tid].spec["backend"], "runtime": runtime}
    return tasks, digest(rt_dir)


# ---- main -------------------------------------------------------------------------------------------------

def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--robouse", required=True, help="a Robo Use checkout (tasks/, src/robouse, assets/)")
    ap.add_argument("--datasets", default="", help="comma-separated dataset names (default: all in hub.yaml)")
    ap.add_argument("--allow-dirty", action="store_true", help="export even if the Robo Use checkout has local changes")
    ap.add_argument("--noop-out", default="", metavar="DIR",
                    help="write every selected dataset with no-op reference solutions to DIR/<dir>/ (a local negative "
                         "control for checking that each task scores 0; datasets/ and export.json are left alone)")
    ap.add_argument("--runtimes", default="", metavar="RT[,RT...]",
                    help="with --datasets: rewrite only these runtimes and, in the selected datasets, only the tasks "
                         "that run on them; every other task and runtime stays exactly as exported before (its "
                         "export.json entry included). For a fix release that must not change other packages.")
    a = ap.parse_args(argv)
    only = set(filter(None, a.runtimes.split(",")))
    if only and not a.datasets:
        ap.error("--runtimes needs --datasets")

    robouse = Path(a.robouse).expanduser().resolve()
    sys.path.insert(0, str(robouse / "src"))
    from robouse.harnesses import PROMPT_PREFIX
    from robouse.tasks import find_tasks, load_task

    sha, dirty = git_head(robouse)
    if dirty and not a.allow_dirty:
        print(f"{robouse} has uncommitted changes under src/, tasks/ or assets/; commit them or pass --allow-dirty",
              file=sys.stderr)
        return 1
    hub = yaml.safe_load((HUB / "hub.yaml").read_text())
    prov = {"repo": hub["robouse"]["repo"], "commit": sha, "package_version": hub["robouse"]["package_version"]}
    m = re.search(r'^__version__ = "([^"]+)"', (robouse / "src" / "robouse" / "__init__.py").read_text(), re.M)
    if not m or m.group(1) != prov["package_version"]:
        raise SystemExit(f"{robouse} is Robo Use {m.group(1) if m else '?'}, but hub.yaml robouse.package_version is "
                         f"{prov['package_version']}")
    tag = subprocess.run(["git", "-C", str(robouse), "describe", "--tags", "--exact-match", "HEAD"],
                         capture_output=True, text=True).stdout.strip()
    if tag:  # a release: the commit the package on PyPI was built from
        prov["tag"] = tag

    by_id, suite_of = {}, {}
    for t in find_tasks(robouse / "tasks"):
        by_id[t.id] = t
        suite_of[t.id] = t.path.parent.name
    backend_runtime = {b: rt for rt, cfg in hub["runtimes"].items() for b in cfg["backends"]}

    configure_oracles(hub)
    wanted = set(filter(None, a.datasets.split(",")))
    manifest_path = HUB / "export.json"
    # a full export starts export.json afresh (no entries for folders that are gone); --datasets updates it in place
    manifest = json.loads(manifest_path.read_text()) if manifest_path.exists() and wanted else {"datasets": {}}
    # with --datasets, only the runtimes those datasets use are rewritten; the others keep their exported content
    used = None
    if wanted:
        used = set()
        for ds in hub["datasets"]:
            if wanted & {ds["name"], ds.get("dir", ds["name"])}:
                src = next((d for d in hub["datasets"] if d["name"] == ds.get("noop_of")), ds)
                suites = set(src.get("suites", []))
                used |= {backend_runtime[by_id[i].spec["backend"]] for i, s in suite_of.items() if s in suites}
                if "tasks_file" in src:
                    used |= {backend_runtime[by_id[ln.strip()].spec["backend"]] for ln in (HUB / src["tasks_file"]).read_text().splitlines()
                             if ln.strip() and not ln.startswith("#") and ln.strip() in by_id}
        if only:
            unknown = only - set(hub["runtimes"])
            if unknown:
                raise SystemExit(f"--runtimes: unknown runtimes {sorted(unknown)}")
            used &= only
    # `native: true` runtimes come from Robo Use's own native export (`robouse export`), not from this script
    runtime_digests = {rt: write_runtime(rt, cfg, robouse, prov) for rt, cfg in hub["runtimes"].items()
                       if (used is None or rt in used) and not cfg.get("native")}
    for rt, d in runtime_digests.items():
        print(f"runtime {rt}: {d}")
    runtime_digests = {**manifest.get("runtimes", {}), **runtime_digests}

    def dataset_ids(ds: dict) -> list[str]:
        if "tasks_file" in ds:
            ids = [ln.strip() for ln in (HUB / ds["tasks_file"]).read_text().splitlines()
                   if ln.strip() and not ln.startswith("#")]
        else:
            ids = sorted(i for i, s in suite_of.items() if s in ds["suites"])
        ids = [i for i in ids if i not in ds.get("exclude_tasks", [])]  # tasks left out of this dataset (see hub.yaml)
        missing = [i for i in ids if i not in by_id]
        if missing:
            raise SystemExit(f"{ds['name']}: unknown task ids {missing}")
        return ids

    by_name = {d["name"]: d for d in hub["datasets"]}
    native_runtimes = {rt for rt, cfg in hub["runtimes"].items() if cfg.get("native")}
    jobs = []
    for ds in hub["datasets"]:
        if wanted and not wanted & {ds["name"], ds.get("dir", ds["name"])}:
            continue
        if "noop_of" in ds:  # negative control: the tasks of another dataset with no-op reference solutions
            if a.noop_out:
                continue  # a committed negative control is already one
            src = by_name[ds["noop_of"]]
            jobs.append(({**{k: v for k, v in src.items() if k in ("suites", "tasks_file")}, "name": ds["name"],
                          "dir": ds.get("dir", ds["name"])}, True))
        else:
            jobs.append((ds, bool(a.noop_out)))
    out_root = Path(a.noop_out).resolve() if a.noop_out else HUB / "datasets"
    for ds, noop in jobs:
        folder = ds.get("dir", ds["name"])  # names are <org>/<name>; the folder stays flat (datasets/<dir>/<task>)
        out_parent = out_root / folder
        ids = dataset_ids(ds)
        rts = {backend_runtime[by_id[i].spec["backend"]] for i in ids}
        partial = bool(only) and not rts <= only
        if partial:  # --runtimes: only this dataset's tasks on those runtimes are rewritten, the others stay as they are
            ids = [i for i in ids if backend_runtime[by_id[i].spec["backend"]] in only]
            if not ids:
                print(f"{ds['name']}: no task on {sorted(only)}, left as it is")
                continue
            out_parent.mkdir(parents=True, exist_ok=True)
            tasks = {}
            for tid in ids:
                task = by_id[tid]
                rt = backend_runtime[task.spec["backend"]]
                export_task(task, robouse, out_parent, rt, runtime_digests[rt], hub["runtimes"][rt].get("sim_memory", "3G"),
                            PROMPT_PREFIX, noop, hub["runtimes"][rt].get("remote_worker") or False)
                # these tasks come from another Robo Use commit than the rest of the dataset
                tasks[tid] = {"suite": suite_of[tid], "backend": task.spec["backend"], "runtime": rt, "robouse": prov}
            if not a.noop_out:
                manifest["datasets"][folder]["tasks"].update(tasks)
            print(f"{ds['name']}: {len(tasks)} task(s) on {sorted(only)} rewritten{' (no-op oracles)' if noop else ''}; "
                  "the others left as they are")
            continue
        if rts & native_runtimes:  # written by Robo Use's own exporter (robouse.engine.format)
            if len(rts) != 1:
                raise SystemExit(f"{ds['name']}: a dataset on a native runtime cannot mix runtimes ({sorted(rts)})")
            rt = rts.pop()
            tasks, runtime_digests[rt] = export_native(ids, by_id, suite_of, out_parent, rt, noop)
        else:
            if out_parent.exists():
                shutil.rmtree(out_parent)
            out_parent.mkdir(parents=True)
            tasks = {}
            for tid in ids:
                task = by_id[tid]
                rt = backend_runtime[task.spec["backend"]]
                sim_memory = hub["runtimes"][rt].get("sim_memory", "3G")
                export_task(task, robouse, out_parent, rt, runtime_digests[rt], sim_memory, PROMPT_PREFIX, noop,
                            hub["runtimes"][rt].get("remote_worker") or False)
                tasks[tid] = {"suite": suite_of[tid], "backend": task.spec["backend"], "runtime": rt}
        if not a.noop_out:
            manifest["datasets"][folder] = {"noop": noop, "tasks": tasks, "robouse": prov}
        print(f"{ds['name']}: {len(tasks)} task(s){' (no-op oracles)' if noop else ''}")
    if a.noop_out:
        return 0
    manifest["runtimes"] = runtime_digests
    if not wanted:
        manifest["robouse"] = prov
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
