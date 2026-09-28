"""Export Robo Use tasks from a Robo Use checkout into this hub as native BenchFlow task packages.

    <robouse>/.venv/bin/python scripts/export.py --robouse ~/benchflow/robouse [--datasets robouse-core,robouse-drone]

A dataset with `noop_of: <dataset>` in hub.yaml is a negative control: the same tasks with reference solutions
that only look at the robot and exit, so the oracle agent must score 0 on every task.

Reads hub.yaml (datasets, runtimes) and core.txt, and writes:

    runtimes/<name>/                 simulator image build context shared by the tasks that use it
      Dockerfile  requirements*.txt  sim-entry.sh  check_runtime.py  robouse/src/robouse/  [fetch_assets.py assets/]
    datasets/<dataset>/<task-id>/    one native BenchFlow package (task.md schema 1.3) per task
      task.md
      environment/Dockerfile           agent (`main`) image: python + numpy + the `robo` client only
      environment/robo                 Robo Use's agent_cli.py (standard library only)
      environment/docker-compose.yaml  `main` + trusted `simulator` (built from ../../../../runtimes/<name>,
                                       checked against the runtime digest pinned here)
      verifier/{verifier.md,test.sh,verify.py}   runs in `simulator`
      oracle/solve.sh + oracle/vendor/ reference solution; drives the robot through the socket only
    export.json                      what was exported: Robo Use commit, runtime digests, task -> suite/runtime

The same task has byte-identical packages in every dataset that contains it (same digest). Run with the Robo Use
dev venv: it reads task folders with robouse.tasks (PyYAML) and vendors Meta-World's scripted expert policies from
the installed `metaworld` package. Re-running is idempotent: packages are rewritten from the source, and the oracle
token of an existing package is kept so an unchanged task keeps its digest.
"""
from __future__ import annotations

import argparse
import ast
import hashlib
import json
import os
import re
import secrets
import shutil
import subprocess
import sys
from pathlib import Path

import yaml

HUB = Path(__file__).resolve().parents[1]
TEMPLATES = HUB / "templates"
WALL_MARGIN_S = 900  # the episode clock starts when the simulator is up, before BenchFlow installs the harness
PKG_IGNORE = shutil.ignore_patterns("__pycache__", "*.pyc", ".DS_Store", "bundled_tasks", "robocasa_episodes",
                                    "engine", "assets_provenance")


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
    if cfg.get("assets") == "menagerie":
        shutil.copy2(TEMPLATES / "runtimes" / "fetch_assets.py", dst / "fetch_assets.py")
        (dst / "assets").mkdir()
        shutil.copy2(robouse / "assets" / "menagerie" / "provenance.json", dst / "assets" / "menagerie.json")
        shutil.copy2(robouse / "assets" / "menagerie" / "skydio_x2" / "PROVENANCE.json", dst / "assets" / "skydio_x2.json")
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


ORACLE_BACKEND_MODULE = {"robosuite": "robouse.backends.robosuite_backend", "metaworld": "robouse.backends.metaworld_backend"}


def vendor_robouse(robouse: Path, backend: str, dst: Path) -> None:
    src = robouse / "src"
    roots = ["robouse.oracle", "robouse.agent_cli", ORACLE_BACKEND_MODULE.get(backend, f"robouse.backends.{backend}")]
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


def write_oracle(task, robouse: Path, dst: Path, token: str, noop: bool) -> None:
    dst.mkdir(parents=True)
    if noop:
        (dst / "solve.sh").write_text(NOOP_SOLVE)
        (dst / "solve.sh").chmod(0o755)
        return
    spec = task.spec
    vendor = dst / "vendor"
    vendor_robouse(robouse, spec["backend"], vendor)
    if spec["backend"] == "metaworld":
        vendor_metaworld_policy(spec["env"], vendor)
    src_oracle = task.oracle_script.parent
    for f in sorted(src_oracle.iterdir()):  # data the reference solution reads (demo.json, demo_actions.json)
        if f.is_file() and f.name != "solve.sh" and not f.name.startswith("."):
            shutil.copy2(f, dst / f.name)
    body = [ln for ln in task.oracle_script.read_text().splitlines()
            if ln.strip() and not ln.startswith("#") and not ln.startswith("set ")]
    body = [re.sub(r"^python3? ", "python3 ", ln) for ln in body]
    (dst / "solve.sh").write_text("\n".join([
        "#!/bin/bash",
        "# Reference solution. Runs in the agent container and drives the robot only through the robo socket",
        "# ($ROBOUSE_SOCKET), like an agent would. vendor/ holds the Robo Use oracle code (and, for Meta-World,",
        "# the scripted expert policy). BenchFlow uploads /oracle only in oracle mode; agents never see it.",
        "set -euo pipefail",
        'export PYTHONPATH="$(cd "$(dirname "$0")" && pwd)/vendor${PYTHONPATH:+:$PYTHONPATH}"',
        f"export ROBOUSE_ORACLE_TOKEN={token}",
        *body,
        "",
    ]))
    (dst / "solve.sh").chmod(0o755)


# ---- one task ---------------------------------------------------------------------------------------------

def native_task_md(meta: dict, instruction: str, prompt_prefix: str) -> str:
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
    return "---\n" + _dump(fm) + "---\n\n" + prompt_prefix + instruction.strip() + "\n"


def compose(meta: dict, context: str, runtime_digest: str, token: str, sim_memory: str) -> str:
    sim_fm = {k: meta[k] for k in ("schema_version", "task", "metadata", "agent", "verifier", "robouse") if k in meta}
    sim_md = "---\n" + _dump(sim_fm) + "---\n"
    block = "\n".join("        " + ln.replace("$", "$$") for ln in sim_md.rstrip().splitlines())
    agent_timeout = float((meta.get("agent") or {}).get("timeout_sec", 900))
    text = (TEMPLATES / "docker-compose.yaml").read_text()
    return (text.replace("__SIM_CONTEXT__", json.dumps(context))
                .replace("__RUNTIME_DIGEST__", runtime_digest)
                .replace("__MAX_WALL_S__", str(int(agent_timeout + WALL_MARGIN_S)))
                .replace("__ORACLE_TOKEN__", token)
                .replace("__SIM_MEMORY__", sim_memory)
                .replace("__TASK_MD__", block))


_TOKEN_RE = re.compile(r'ROBOUSE_ORACLE_TOKEN: "([0-9a-f]{32})"')


_TOKENS: dict[str, str] = {}


def load_existing_tokens() -> None:
    """Oracle tokens of the packages already in datasets/ (read before anything is rewritten)."""
    for compose_file in sorted((HUB / "datasets").glob("*/*/environment/docker-compose.yaml")):
        m = _TOKEN_RE.search(compose_file.read_text())
        if m:
            _TOKENS.setdefault(compose_file.parents[1].name, m.group(1))


def export_task(task, robouse: Path, out_parent: Path, runtime: str, runtime_digest: str, sim_memory: str,
                prompt_prefix: str, noop: bool = False) -> Path:
    meta = json.loads(json.dumps(task.meta))
    dst = out_parent / task.id
    token = _TOKENS.setdefault(task.id, secrets.token_hex(16))
    if dst.exists():
        shutil.rmtree(dst)
    env = dst / "environment"
    env.mkdir(parents=True)
    (dst / "task.md").write_text(native_task_md(meta, task.instruction, prompt_prefix))
    shutil.copy2(TEMPLATES / "agent" / "Dockerfile", env / "Dockerfile")
    (env / "robo").write_text("#!/usr/bin/env python3\n" + (robouse / "src" / "robouse" / "agent_cli.py").read_text())
    (env / "robo").chmod(0o755)
    context = os.path.relpath(HUB / "runtimes" / runtime, env)
    (env / "docker-compose.yaml").write_text(compose(meta, context, runtime_digest, token, sim_memory))
    shutil.copytree(TEMPLATES / "verifier", dst / "verifier", ignore=shutil.ignore_patterns("__pycache__"))
    (dst / "verifier" / "test.sh").chmod(0o755)
    write_oracle(task, robouse, dst / "oracle", token, noop)
    return dst


# ---- main -------------------------------------------------------------------------------------------------

def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--robouse", required=True, help="a Robo Use checkout (tasks/, src/robouse, assets/)")
    ap.add_argument("--datasets", default="", help="comma-separated dataset names (default: all in hub.yaml)")
    ap.add_argument("--allow-dirty", action="store_true", help="export even if the Robo Use checkout has local changes")
    a = ap.parse_args(argv)

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

    by_id, suite_of = {}, {}
    for t in find_tasks(robouse / "tasks"):
        by_id[t.id] = t
        suite_of[t.id] = t.path.parent.name
    backend_runtime = {b: rt for rt, cfg in hub["runtimes"].items() for b in cfg["backends"]}

    load_existing_tokens()
    runtime_digests = {rt: write_runtime(rt, cfg, robouse, prov) for rt, cfg in hub["runtimes"].items()}
    for rt, d in runtime_digests.items():
        print(f"runtime {rt}: {d}")

    def dataset_ids(ds: dict) -> list[str]:
        if "tasks_file" in ds:
            ids = [ln.strip() for ln in (HUB / ds["tasks_file"]).read_text().splitlines()
                   if ln.strip() and not ln.startswith("#")]
        else:
            ids = sorted(i for i, s in suite_of.items() if s in ds["suites"])
        missing = [i for i in ids if i not in by_id]
        if missing:
            raise SystemExit(f"{ds['name']}: unknown task ids {missing}")
        return ids

    wanted = set(filter(None, a.datasets.split(",")))
    manifest_path = HUB / "export.json"
    manifest = json.loads(manifest_path.read_text()) if manifest_path.exists() else {"datasets": {}}
    by_name = {d["name"]: d for d in hub["datasets"]}
    jobs = []
    for ds in hub["datasets"]:
        if wanted and ds["name"] not in wanted:
            continue
        if "noop_of" in ds:  # negative control: the tasks of another dataset with no-op reference solutions
            src = by_name[ds["noop_of"]]
            jobs.append(({**{k: v for k, v in src.items() if k in ("suites", "tasks_file")}, "name": ds["name"]}, True))
        else:
            jobs.append((ds, False))
    for ds, noop in jobs:
        out_parent = HUB / "datasets" / ds["name"]
        if out_parent.exists():
            shutil.rmtree(out_parent)
        out_parent.mkdir(parents=True)
        tasks = {}
        for tid in dataset_ids(ds):
            task = by_id[tid]
            rt = backend_runtime[task.spec["backend"]]
            sim_memory = hub["runtimes"][rt].get("sim_memory", "3G")
            export_task(task, robouse, out_parent, rt, runtime_digests[rt], sim_memory, PROMPT_PREFIX, noop)
            tasks[tid] = {"suite": suite_of[tid], "backend": task.spec["backend"], "runtime": rt}
        manifest["datasets"][ds["name"]] = {"noop": noop, "tasks": tasks}
        print(f"{ds['name']}: {len(tasks)} task(s){' (no-op oracles)' if noop else ''}")
    manifest.update({"robouse": prov, "runtimes": runtime_digests})
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
