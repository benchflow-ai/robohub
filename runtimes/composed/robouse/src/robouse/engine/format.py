"""robouse tasks as a BenchFlow task format, on BenchFlow's embodied layer.

A robouse task folder (``task.md`` with a ``robouse:`` block, ``oracle/solve.sh``, ``verifier/test.sh``) is the
source form. BenchFlow loads it through the ``benchflow.task_formats`` entry point. The package layout, the agent
image, the compose topology (agent + trusted ``simulator`` sidecar), the physical verifier and seeded variants come
from `benchflow.embodied.sidecar.EmbodiedTaskFormat` (BenchFlow's docs/embodied.md). robouse supplies:

  - the simulator image: templates/sim/Dockerfile + requirements.txt + this robouse package (the backends);
  - the episode factory ``robouse.engine.sim:episode_from_task``;
  - the reference solutions: oracle/solve.sh with the robouse oracle code vendored (and, for Meta-World, the
    scripted expert policies), driving the robot only through the socket.

No package holds an oracle token. The token that unlocks privileged state (vision tasks) is chosen when the task runs:
the simulator reads it from ``$ROBOUSE_ORACLE_TOKEN`` on the host (compose interpolation), and the reference solution
gets the same variable through the task's ``oracle.env``, which BenchFlow passes to the oracle only. With the variable
unset, the simulator has no token and nothing is privileged. ``robouse run`` sets a fresh random value for oracle jobs;
with ``bench eval run --agent oracle``, set it yourself (``ROBOUSE_ORACLE_TOKEN=$(openssl rand -hex 16)``). Packages
are therefore byte-identical across builds, and a published package gives an agent nothing to look up.

``materialize_variant(noop=True)`` writes a negative-control oracle that only looks (`robo info`, `robo observe`)
and exits without `robo done`; it must score 0. ``seed=N`` writes ``<task-id>--seed-N`` (``bench eval run --seeds``).
"""

from __future__ import annotations

import io
import json
import re
import shutil
import urllib.request
import zipfile
from pathlib import Path

import yaml
from benchflow.embodied.sidecar import EmbodiedTaskFormat, dump_yaml, vendor_embodied

from ..tasks import load_task

ORACLE_TOKEN_ENV = "ROBOUSE_ORACLE_TOKEN"  # host variable: the simulator's token and the reference solution's copy
TOKEN_REF = "${" + ORACLE_TOKEN_ENV + ":-}"  # compose / oracle.env interpolation; unset means no token
_SIDECAR_TOKEN_COMMENT = (
    "# Given only to the reference solution (oracle/solve.sh): unlocks privileged state in vision tasks."
)
_TOKEN_COMMENT = (
    "# Chosen per run on the host ($ROBOUSE_ORACLE_TOKEN; unset means no token) and given only to the reference\n"
    "      # solution (task.md oracle.env): unlocks privileged state in vision tasks. No token is stored in this package."
)
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

PKG = Path(__file__).resolve().parents[1]  # the robouse package
TEMPLATES = Path(__file__).resolve().parent / "templates"
METAWORLD_VERSION = "3.1.1"  # the version pinned in templates/sim/requirements.txt
_IGNORE = shutil.ignore_patterns(
    "__pycache__", "*.pyc", ".DS_Store", "bundled_tasks", "templates"
)  # engine/sim.py runs there


def _module_file(mod: str) -> Path | None:
    """robouse.backends.tabletop -> PKG/backends/tabletop.py (or the package's __init__.py)."""
    rel = Path(*mod.split(".")[1:])
    if not rel.parts:  # `import robouse` or `from robouse import x`
        return PKG / "__init__.py"
    for cand in (PKG / rel.with_suffix(".py"), PKG / rel / "__init__.py"):
        if cand.is_file():
            return cand
    return None


def _oracle_closure(backend: str) -> list[Path]:
    """The robouse modules a backend's reference solution can import (every import in them, including the ones inside
    functions), plus the package __init__ files on the way; the rest of the package stays out of the task."""
    import ast

    from ..oracle import oracle_module

    roots = ["robouse.oracle", "robouse.agent_cli", oracle_module(backend)]
    seen: set[Path] = set()
    todo = [m for m in roots]
    while todo:
        mod = todo.pop()
        f = _module_file(mod)
        if f is None or f in seen:
            continue
        seen.add(f)
        pkg = mod if f.name == "__init__.py" else mod.rsplit(".", 1)[0]
        for node in ast.walk(ast.parse(f.read_text())):
            if isinstance(node, ast.ImportFrom):
                if node.level:
                    base = pkg.split(".")
                    base = base[: len(base) - node.level + 1]
                    target = ".".join(base + ([node.module] if node.module else []))
                else:
                    target = node.module or ""
                if target.startswith("robouse"):
                    todo.append(target)
                    todo += [f"{target}.{a.name}" for a in node.names]
            elif isinstance(node, ast.Import):
                todo += [a.name for a in node.names if a.name.startswith("robouse")]
    parents = set()
    for f in seen:
        d = f.parent
        while d != PKG.parent and d.is_relative_to(PKG):
            if (d / "__init__.py").is_file():
                parents.add(d / "__init__.py")
            d = d.parent
    return sorted(seen | parents)


def _asset_manifests() -> dict[str, Path]:
    """Provenance manifests of the Menagerie robot meshes (the simulator image downloads the files at build time)."""
    from ..assets import manifests

    names = ("menagerie.json", "skydio_x2.json", "embodiments.json")
    return dict(zip(names, manifests(), strict=True))


def _metaworld_policies(dst: Path) -> None:
    """Copy Meta-World's scripted expert policies (pure numpy) to dst/metaworld/policies.

    From the installed `metaworld` package when there is one (found without importing it, so MuJoCo is not
    needed on the host), otherwise from the pinned wheel on PyPI (cached in ~/.cache/robouse/wheels).
    """
    import importlib.util

    pol = dst / "metaworld" / "policies"
    spec = importlib.util.find_spec("metaworld")
    src = Path(spec.submodule_search_locations[0]) / "policies" if spec and spec.submodule_search_locations else None
    if src is not None and (src / "__init__.py").exists():
        shutil.copytree(src, pol, ignore=shutil.ignore_patterns("__pycache__", "*.pyc"))
    else:
        cache = Path.home() / ".cache" / "robouse" / "wheels"
        cache.mkdir(parents=True, exist_ok=True)
        whl = next(iter(cache.glob(f"metaworld-{METAWORLD_VERSION}-*.whl")), None)
        if whl is None:
            with urllib.request.urlopen(f"https://pypi.org/pypi/metaworld/{METAWORLD_VERSION}/json", timeout=60) as r:
                files = json.load(r)["urls"]
            url = next(f["url"] for f in files if f["filename"].endswith(".whl"))
            whl = cache / url.rsplit("/", 1)[1]
            with urllib.request.urlopen(url, timeout=300) as r:
                whl.write_bytes(r.read())
        with zipfile.ZipFile(io.BytesIO(whl.read_bytes())) as z:
            for name in z.namelist():
                if name.startswith("metaworld/policies/") and name.endswith(".py"):
                    out = dst / name
                    out.parent.mkdir(parents=True, exist_ok=True)
                    out.write_bytes(z.read(name))
    (dst / "metaworld" / "__init__.py").write_text(
        '"""Vendored subset of Meta-World (MIT): only the scripted expert policies (numpy only)."""\n'
    )
    (dst / "metaworld" / "LICENSE").write_text(METAWORLD_LICENSE)  # MIT: the notice travels with the code


class RobouseTaskFormat(EmbodiedTaskFormat):
    """BenchFlow task format (entry point `benchflow.task_formats: robouse`)."""

    name = "robouse"
    block_key = "robouse"
    episode_factory = "robouse.engine.sim:episode_from_task"
    format_version = "4"  # 4: no oracle token in the package (ORACLE_TOKEN_ENV), a relative source path
    task_name_prefix = "robouse/"
    sim_cpus = "2"
    sim_memory = "3G"

    @property
    def prompt_prefix(self) -> str:  # the same preamble a local robouse run gives every harness
        from ..harnesses import PROMPT_PREFIX

        return PROMPT_PREFIX

    def runtime_hash_inputs(self) -> list[Path]:
        return [PKG, TEMPLATES, *_asset_manifests().values()]

    def build_runtime(self, dst: Path) -> None:
        for name in ("Dockerfile", "requirements.txt"):
            shutil.copy2(TEMPLATES / "sim" / name, dst / name)
        pkg = dst / "robouse" / "src" / "robouse"
        shutil.copytree(PKG, pkg, ignore=_IGNORE)
        prov = pkg / "assets_provenance"
        prov.mkdir(exist_ok=True)
        if not any(p.exists() for p in _asset_manifests().values()):
            raise FileNotFoundError("robot mesh provenance manifests not found (assets/menagerie or assets_provenance)")
        for name, src in _asset_manifests().items():
            if src.exists() and not (prov / name).exists():
                shutil.copy2(src, prov / name)

    def task_id(self, task_dir: Path, meta: dict) -> str:
        return load_task(task_dir).id

    def write_oracle(self, task_dir: Path, meta: dict, dst: Path, token: str) -> None:
        task = load_task(task_dir)
        dst.mkdir(parents=True)
        vendor = dst / "vendor"
        for f in _oracle_closure(str(task.spec.get("backend", ""))):
            out = vendor / "robouse" / f.relative_to(PKG)
            out.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(f, out)
        vendor_embodied(vendor)  # robouse's oracles import benchflow.embodied (the socket client)
        if task.spec.get("backend") == "metaworld":
            _metaworld_policies(vendor)
        extra = task.oracle_script.parent
        for f in sorted(extra.iterdir()):  # data the oracle reads (e.g. DexJoCo demo.json, BEHAVIOR plan.json)
            if f.name != "solve.sh" and f.is_file():
                shutil.copy2(f, dst / f.name)
        body = [
            ln
            for ln in task.oracle_script.read_text().splitlines()
            if ln.strip() and not ln.startswith("#") and not ln.startswith("set ")
        ]
        body = [re.sub(r"^python3? ", "python3 ", ln) for ln in body]
        (dst / "solve.sh").write_text(
            "\n".join(
                [
                    "#!/bin/bash",
                    "# Reference solution. Runs in the agent container and drives the robot only through the robo socket",
                    "# ($ROBO_SOCKET), like an agent would. vendor/ holds the robouse oracle code and BenchFlow's embodied",
                    "# client (and, for Meta-World, the scripted expert policies). BenchFlow hides /oracle from agents.",
                    "set -euo pipefail",
                    'export PYTHONPATH="$(cd "$(dirname "$0")" && pwd)/vendor${PYTHONPATH:+:$PYTHONPATH}"',
                    *body,
                    "",
                ]
            )
        )
        (dst / "solve.sh").chmod(0o755)

    # ---- no oracle token in the package (see the module docstring) ---------------------------------------------
    def compose(self, meta: dict, sim_context: str, token: str) -> str:
        text = super().compose(meta, sim_context, TOKEN_REF)
        return text.replace(_SIDECAR_TOKEN_COMMENT, _TOKEN_COMMENT)

    def native_task_md(self, meta: dict, instruction: str, variant: dict | None = None) -> str:
        text = super().native_task_md(meta, instruction, variant)
        if variant and variant.get("noop"):
            return text  # the negative control never gets privileged state
        _, fm, body = text.split("---\n", 2)
        data = yaml.safe_load(fm)
        data["oracle"] = {"env": {ORACLE_TOKEN_ENV: TOKEN_REF}}
        return "---\n" + dump_yaml(data) + "---\n" + body

    def materialize_variant(self, task_dir: Path, out_root: Path, **kw) -> Path:
        pkg = super().materialize_variant(task_dir, out_root, **kw)
        marker = pkg / f".{self.name}-source.json"
        if marker.exists():  # the source folder relative to its checkout, not an absolute path on the build machine
            info = json.loads(marker.read_text())
            src = Path(info.get("source") or task_dir)
            if src.is_absolute():
                root = next((p for p in (src, *src.parents) if (p / ".git").exists()), None)
                rel = src.relative_to(root).as_posix() if root else src.name
            else:
                rel = src.as_posix()
            if info.get("source") != rel:
                info["source"] = rel
                marker.write_text(json.dumps(info, indent=2))
        return pkg


def materialize(
    task_dir: Path,
    out_root: Path,
    *,
    noop: bool = False,
    seed: int | None = None,
    timeout: float | None = None,
    flat: bool = False,
) -> Path:
    """Write (or reuse) the native BenchFlow package for one robouse task and return its folder."""
    return RobouseTaskFormat().materialize_variant(task_dir, out_root, seed=seed, noop=noop, timeout=timeout, flat=flat)


def is_robouse_task(task_dir: Path) -> bool:
    return RobouseTaskFormat().detect(Path(task_dir))


def task_format_root() -> Path:
    """The cache BenchFlow uses for robouse packages (so `robouse run` and `bench eval run` share builds)."""
    from benchflow.task.formats import task_format_cache_root

    return task_format_cache_root() / RobouseTaskFormat.name
