"""`robouse tasks` and `robouse components`: list, create and check tasks; list, show and check components.

robouse tasks [list] [ROOT] [--path]                list tasks (default: the bundled tasks)
robouse tasks init DIR                              scaffold a blank task folder (task.md, oracle/, verifier/)
robouse tasks init --embodiment E --scene S --task T [modifiers] [--seed N] --out DIR
                                                    compose a task from components; keep it only if the reference
                                                    solution scores 1 and the no-op agent 0
robouse tasks init --grid --embodiment E1,E2 ... --out DIR
                                                    compose every compatible combination of the listed values
robouse tasks init --from DIR --out OUT [--upgrade] regenerate a composed task from its recorded instance
robouse tasks check DIR                             schema, BenchFlow's `bench tasks check`, reference 1, no-op 0
robouse components list [KIND] | show ID | check E S T [modifiers]
"""

from __future__ import annotations

import argparse
import itertools
import json
import math
import shutil
import sys
import tempfile
from pathlib import Path

MODS = [
    ("obs", "--obs", "MODE", "observation mode: state (default), vision or noisy"),
    ("budget", "--budget", "BUDGET", "step budget: normal (default), loose or tight"),
    ("perturbation", "--perturb", "KIND", "perturbation: none (default), push, dynamics or clutter"),
    ("safety", "--safety", "LEVEL", "safety overlay: none (default), direct, indirect or privacy"),
    ("roles", "--roles", "ROLES", "roles: single (default) or planner-operator"),
]
DEFAULTS = {"obs": "state", "budget": "normal", "perturbation": "none", "safety": "none", "roles": "single"}


def _mod_args(p: argparse.ArgumentParser) -> None:
    for key, flag, metavar, help_ in MODS:
        p.add_argument(flag, dest=key, metavar=metavar, default=DEFAULTS[key], help=help_)
        if key == "perturbation":  # the pre-rename spelling
            p.add_argument("--perturbation", dest=key, help=argparse.SUPPRESS)


def _seed(x: str) -> int:
    if not x.isdigit():
        raise argparse.ArgumentTypeError(f"must be a non-negative integer, not {x!r}")
    return int(x)


def _err(msg: str, code: int = 1) -> int:
    sys.stdout.flush()  # keep "wrote ..." before the error when both streams go to one pipe
    print(f"robouse: {msg}", file=sys.stderr)
    return code


# ---------------------------------------------------------------------------------------------------------------------
# robouse tasks
# ---------------------------------------------------------------------------------------------------------------------


def tasks_main(argv: list[str], prog: str = "robouse tasks") -> int:
    if not argv or argv[0] not in ("list", "init", "check", "-h", "--help"):
        argv = ["list", *argv]  # `robouse tasks [ROOT] [--path]` lists, as before
    ap = argparse.ArgumentParser(prog=prog, description="list, create and check tasks")
    sub = ap.add_subparsers(dest="tcmd", required=True, metavar="{list,init,check}")

    ls = sub.add_parser("list", help="list tasks (default: the bundled tasks)")
    ls.add_argument("root", nargs="?", metavar="ROOT", help="task root (default: the bundled tasks)")
    ls.add_argument("--path", action="store_true", help="print the bundled task directory and exit")

    it = sub.add_parser(
        "init",
        help="compose a task from components, regenerate one, or scaffold a blank task",
        description="compose a task from components and validate it (--embodiment, --scene, --task), "
        "regenerate a composed task from its recorded instance (--from), or scaffold a blank task (DIR)",
    )
    it.add_argument("dir", nargs="?", metavar="DIR", help="blank task folder to create (scaffold mode)")
    it.add_argument("--embodiment", metavar="ID", help="embodiment component id (robouse components list)")
    it.add_argument("--scene", metavar="ID", help="scene component id")
    it.add_argument("--task", metavar="ID", help="task component id")
    _mod_args(it)
    it.add_argument("--seed", default="0", metavar="N", help="instance seed (default 0)")
    it.add_argument("--out", metavar="DIR", help="folder to write the composed task folder(s) into")
    it.add_argument(
        "--grid", action="store_true", help="comma-separated values in any flag: compose every compatible combination"
    )
    it.add_argument(
        "--from",
        dest="src",
        metavar="DIR",
        help="regenerate the composed task in this folder from its recorded instance",
    )
    it.add_argument(
        "--upgrade", action="store_true", help="with --from: use the installed component versions if they differ"
    )
    it.add_argument(
        "--no-validate", action="store_true", help="write without running the reference solution and the no-op control"
    )
    it.add_argument(
        "--keep-trials",
        "--keep-runs",
        dest="keep_runs",
        metavar="DIR",
        help="keep the validation trials in this folder",
    )

    ck = sub.add_parser(
        "check", help="validate a task: schema, `bench tasks check`, reference solution 1, no-op control 0"
    )
    ck.add_argument("dir", metavar="DIR", help="task folder")
    ck.add_argument(
        "--level", default="structural", metavar="LEVEL", help="`bench tasks check` level (default structural)"
    )
    ck.add_argument("--no-run", action="store_true", help="skip the reference solution and no-op control trials")
    ck.add_argument(
        "--keep-trials", "--keep-runs", dest="keep_runs", metavar="DIR", help="keep the trials in this folder"
    )

    a = ap.parse_args(argv)
    if a.tcmd == "list":
        return _list(a)
    if a.tcmd == "check":
        return check_task(Path(a.dir), level=a.level, run=not a.no_run, keep_runs=a.keep_runs)
    if a.src:
        return _from(a)
    if a.embodiment or a.scene or a.task:
        return _compose(a)
    if a.dir:
        return scaffold(Path(a.dir))
    return _err("tasks init needs DIR (a blank task), --embodiment/--scene/--task (a composed task) or --from DIR", 2)


def _list(a) -> int:
    import os

    from ..tasks import bundled_tasks_root, find_tasks

    if a.path:
        print(bundled_tasks_root())
        return 0
    if a.root and not Path(a.root).is_dir():
        return _err(f"no such task folder: {a.root} (robouse tasks [list] [ROOT]; subcommands: list, init, check)", 2)
    try:
        for task in find_tasks(a.root or bundled_tasks_root()):
            print(task.id, task.spec.get("backend"), task.spec.get("env", ""))
        sys.stdout.flush()
    except BrokenPipeError:  # e.g. `robouse tasks | head`
        os.dup2(os.open(os.devnull, os.O_WRONLY), sys.stdout.fileno())
    return 0


# ---- scaffold ---------------------------------------------------------------------------------------------------------

SCAFFOLD_MD = """---
schema_version: '1.3'
task:
  name: my/{name}
  description: '[REPLACE: one line]'
metadata:
  author_name: ''
  difficulty: medium
  category: manipulation
  tags: []
agent:
  timeout_sec: 900
verifier:
  timeout_sec: 120
robouse:
  id: {name}
  backend: '[REPLACE: the simulator backend, e.g. tabletop or components]'
  env: {name}
  seed: 0
  max_steps: 500
  skills: true
  success_mode: final
---

# {name}

[REPLACE: the scene, the goal, the exact success rule the episode server checks, the controls and the step budget. This body is what the agent reads.]
"""

SCAFFOLD_ORACLE = """#!/bin/bash
# Reference solution: runs like an agent, with `robo` on its PATH, and must score 1.
# [REPLACE: read `robo info` for the skills and action groups and `robo observe --json` for positions, then act.]
set -euo pipefail
exit 1
"""


def scaffold(d: Path) -> int:
    """A blank task folder in BenchFlow's native layout (the same shape as `bench tasks init`), for a custom backend."""
    from ..backends.embodied import VERIFY_SH

    if d.exists() and any(d.iterdir()):
        return _err(f"{d} exists and is not empty")
    name = d.name
    (d / "oracle").mkdir(parents=True, exist_ok=True)
    (d / "verifier").mkdir(parents=True, exist_ok=True)
    (d / "task.md").write_text(SCAFFOLD_MD.format(name=name))
    for rel, body in (("oracle/solve.sh", SCAFFOLD_ORACLE), ("verifier/test.sh", VERIFY_SH)):
        (d / rel).write_text(body)
        (d / rel).chmod(0o755)
    print(f"created {d}/task.md, oracle/solve.sh, verifier/test.sh")
    print("fill in the [REPLACE: ...] parts, then check it with: robouse tasks check " + str(d))
    return 0


# ---- compose ----------------------------------------------------------------------------------------------------------


def _values(a) -> dict[str, list[str]]:
    keys = ["embodiment", "scene", "task", "seed"] + [k for k, *_ in MODS]
    return {k: [x.strip() for x in str(getattr(a, k)).split(",") if x.strip()] for k in keys}


def compose_one(
    emb: str,
    scene: str,
    task: str,
    mods: dict,
    seed: int,
    out: Path,
    validate_it: bool = True,
    keep_runs: str | None = None,
    quiet: bool = False,
) -> tuple[bool, str, Path | None]:
    """Check, write and (optionally) validate one composed task. Returns (accepted, message, folder)."""
    from ..components.compose import ComposeError, record_validation, validate, write
    from ..components.registry import load
    from ..components.resolve import check

    reg = load()
    try:
        rep = check(reg, emb, scene, task, mods)
    except KeyError as e:
        return False, e.args[0], None
    if not rep.ok:
        return False, "incompatible: " + "; ".join(rep.reasons), None
    out = Path(out)
    stage = None
    try:
        out.mkdir(parents=True, exist_ok=True)
        if validate_it:  # validate in a staging folder, so a rejected task never touches an existing folder
            stage = Path(tempfile.mkdtemp(prefix=".robouse-compose-", dir=out))
        d = write(emb, scene, task, mods, seed, stage or out, reg)
    except ComposeError as e:
        _rm(stage)
        return False, f"incompatible: {e}", None
    except OSError as e:
        _rm(stage)
        return False, f"cannot write to {out}: {e.strerror or e}", None
    target = out / d.name
    if not validate_it:
        if not quiet:
            print(f"wrote {d}")
        return True, "written (not validated)", d
    try:
        v = validate(d, Path(keep_runs) / d.name if keep_runs else None)
        msg = _validation_message(v)
        if not v["accepted"]:
            kept = f"; {target} was left as it was" if target.exists() else ""
            return False, msg + "; rejected (the reference solution must score 1 and the no-op control 0)" + kept, None
        record_validation(d, v)
        shutil.copytree(d, target, dirs_exist_ok=True)
    finally:
        _rm(stage)
    if not quiet:
        print(f"wrote {target}")
    return True, msg, target


def _rm(d: Path | None) -> None:
    if d is not None:
        shutil.rmtree(d, ignore_errors=True)


def _validation_message(v: dict) -> str:
    """One line on the reference solution and no-op trials; a trial that could not run says why (e.g. missing robot
    models, with the `robouse fetch-assets` command that installs them)."""
    broken = [
        f"the {name} trial did not run: {v[key]['exception']}"
        for key, name in (("oracle", "reference solution"), ("noop", "no-op control"))
        if v[key].get("exception")
    ]
    if broken:
        return "; ".join(broken)
    return (
        f"reference solution reward {v['oracle']['reward']} ({v['oracle']['outcome']}, {v['oracle']['steps']} steps); "
        f"no-op control reward {v['noop']['reward']} ({v['noop']['outcome']})"
    )


def _compose(a) -> int:
    from ..components.resolve import modifiers

    vals = _values(a)
    missing = [k for k in ("embodiment", "scene", "task") if not vals[k]]
    if missing:
        return _err("tasks init needs --" + ", --".join(missing), 2)
    if not a.out:
        return _err("tasks init needs --out DIR", 2)
    if not a.grid:
        multi = [k for k, v in vals.items() if len(v) > 1]
        if multi:
            return _err(f"several values for {', '.join(multi)}: add --grid to compose every combination", 2)
    bad = [x for x in vals["seed"] if not x.isdigit()]
    if bad or not vals["seed"]:
        return _err(f"--seed must be a non-negative integer (got {', '.join(bad) or 'nothing'})", 2)
    if not a.grid:
        seed = int(vals["seed"][0])
        mods = modifiers({k: vals[k][0] for k, *_ in MODS})
        ok, msg, d = compose_one(
            vals["embodiment"][0],
            vals["scene"][0],
            vals["task"][0],
            mods,
            seed,
            Path(a.out),
            not a.no_validate,
            a.keep_runs,
        )
        if not ok:
            return _err(msg)
        if not a.no_validate:
            print(msg)
            print("accepted: reference solution 1, no-op control 0")
        return 0
    # --grid: one row per combination as it finishes, then the totals
    keys = ["embodiment", "scene", "task", "seed"] + [k for k, *_ in MODS]
    combos = [dict(zip(keys, c, strict=False)) for c in itertools.product(*(vals[k] for k in keys))]

    def label(c: dict) -> str:
        return " + ".join(
            [c["embodiment"], c["scene"], c["task"]]
            + [f"{k}={c[k]}" for k, *_ in MODS if c[k] != DEFAULTS[k]]
            + ([f"seed={c['seed']}"] if len(vals["seed"]) > 1 or c["seed"] != "0" else [])
        )

    w = max(len(label(c)) for c in combos)
    print(f"{'result':<9} {'combination':<{w}}  task folder, or why it was rejected", flush=True)
    n = 0
    for c in combos:
        mods = modifiers({k: c[k] for k, *_ in MODS})
        ok, msg, d = compose_one(
            c["embodiment"],
            c["scene"],
            c["task"],
            mods,
            int(c["seed"]),
            Path(a.out),
            not a.no_validate,
            a.keep_runs,
            quiet=True,
        )
        n += ok
        print(f"{'accepted' if ok else 'rejected':<9} {label(c):<{w}}  {d.name if ok and d else msg}", flush=True)
    print(f"{n} accepted, {len(combos) - n} rejected, of {len(combos)} combinations; task folders in {Path(a.out)}")
    return 0 if n else 1


def _same(x, y) -> bool:
    """Equal layouts, up to the last bits of a float (x86 and ARM round some trigonometry differently)."""
    if isinstance(x, dict) and isinstance(y, dict):
        return x.keys() == y.keys() and all(_same(x[k], y[k]) for k in x)
    if isinstance(x, (list, tuple)) and isinstance(y, (list, tuple)):
        return len(x) == len(y) and all(_same(u, v) for u, v in zip(x, y, strict=True))
    if isinstance(x, float) or isinstance(y, float):
        return (
            isinstance(x, (int, float))
            and isinstance(y, (int, float))
            and math.isclose(x, y, rel_tol=1e-9, abs_tol=1e-12)
        )
    return x == y


def _from(a) -> int:
    """Regenerate a composed task from its recorded instance (ids, versions, modifiers, seed), and prove the layout is
    the recorded one."""
    from ..backends.components.backend import read_block
    from ..components.registry import load
    from ..tasks import load_task

    if not a.out:
        return _err("tasks init --from needs --out DIR", 2)
    t = load_task(Path(a.src))
    try:
        rec = read_block(t.spec)
    except KeyError as e:
        return _err(f"{a.src} is not a composed task ({e.args[0]})")
    reg = load()
    now = {k: reg.get(k + "s", rec[k])["version"] for k in ("embodiment", "scene", "task")}
    old = {k: rec["versions"].get(k, "") for k in now}
    changed = {k: (old[k], now[k]) for k in now if old[k] and old[k] != now[k]}
    if changed and not a.upgrade:
        return _err(
            "recorded component versions differ from the installed ones ("
            + ", ".join(f"{rec[k]} {o} -> {n}" for k, (o, n) in changed.items())
            + "); add --upgrade to regenerate with them"
        )
    ok, msg, d = compose_one(
        rec["embodiment"],
        rec["scene"],
        rec["task"],
        rec["modifiers"],
        rec["seed"],
        Path(a.out),
        not a.no_validate,
        a.keep_runs,
    )
    if not ok or d is None:
        return _err(msg)
    new = read_block(load_task(d).spec)
    same = _same(new["instance"], rec["instance"])
    if not same and not (changed or a.upgrade):
        print(
            f"robouse: the regenerated layout differs from the recorded one in {a.src} (the component code changed without a "
            "version bump); kept the new folder, compare the two",
            file=sys.stderr,
        )
        return 1
    print(
        ("same layout as recorded" if same else "layout regenerated with the upgraded components")
        + (f"; {msg}" if not a.no_validate else "")
    )
    return 0


# ---- check ------------------------------------------------------------------------------------------------------------


def check_task(d: Path, level: str = "structural", run: bool = True, keep_runs: str | None = None) -> int:
    from ..tasks import SCHEMA_VERSION, load_task, older_layout, older_layout_message

    if not d.is_dir():
        return _err(f"no such task folder: {d}")
    if older_layout(d):
        return _err(older_layout_message(d))
    problems = []
    if (d / "task.md").is_file() and not (d / "environment" / "Dockerfile").is_file():
        problems += [f"{rel} is missing" for rel in ("oracle/solve.sh", "verifier/test.sh") if not (d / rel).is_file()]
    try:
        t = load_task(d)
        if t.meta.get("schema_version") != SCHEMA_VERSION:
            problems.append(
                f"task.md: schema_version {t.meta.get('schema_version')!r}; this robouse reads {SCHEMA_VERSION!r}"
            )
        for key in ("id", "backend"):
            if not t.spec.get(key):
                problems.append(f"task.md: robouse.{key} is missing")
        if "[REPLACE" in (d / "task.md").read_text():
            problems.append("task.md still has [REPLACE: ...] placeholders")
        if t.spec.get("backend") in ("components", "remix"):
            from ..backends.components.backend import read_block

            read_block(t.spec)
        print(f"schema: {'ok' if not problems else 'problems'} ({t.id}, backend {t.spec.get('backend')})")
    except Exception as e:  # noqa: BLE001 - reported as a schema problem
        problems.append(f"task.md: {type(e).__name__}: {e}")
        print("schema: problems")
    try:
        from benchflow._utils.task_authoring import check_task as bf_check
        from benchflow.task.formats import detect_task_format, materialize_task_dir
    except ImportError:  # BenchFlow is optional: `pip install 'robouse[benchflow]'` adds this check
        bf_check = None
        print("bench tasks check: skipped (BenchFlow is not installed)")
    try:
        if bf_check is None:
            raise LookupError
        pkg = materialize_task_dir(d) if detect_task_format(d) is not None else d
        issues = bf_check(pkg, validation_level=level)
        print(f"bench tasks check ({level}): {'ok' if not issues else f'{len(issues)} issue(s)'}")
        problems += [f"bench tasks check: {i}" for i in issues]
    except LookupError:
        pass
    except Exception as e:  # noqa: BLE001
        problems.append(f"bench tasks check could not run: {type(e).__name__}: {e}")
    if run and not problems and (d / "environment" / "Dockerfile").is_file():
        # an exported package (`robouse export`, the hub datasets) runs in its own containers, not in the local engine
        print(
            f"trials: run them with `bench eval run --tasks-dir {d.parent} --include {d.name} --agent oracle`, "
            f"or regenerate the task with `robouse tasks init --from {d} --out DIR` and check that"
        )
        run = False
    if run and not problems:
        from ..components.compose import validate

        v = validate(d, Path(keep_runs) if keep_runs else None)
        if v["oracle"].get("exception") or v["noop"].get("exception"):
            print(_validation_message(v))
        else:
            print(
                f"reference solution: reward {v['oracle']['reward']} ({v['oracle']['outcome']}, {v['oracle']['steps']} steps)"
            )
            print(f"no-op control: reward {v['noop']['reward']} ({v['noop']['outcome']})")
        if v["oracle"]["reward"] != 1:
            problems.append("the reference solution does not score 1")
        if v["noop"]["reward"] != 0:
            problems.append("the no-op control does not score 0")
    for p in problems:
        print(f"  - {p}")
    print("valid" if not problems else f"{len(problems)} problem(s)")
    return 0 if not problems else 1


# ---------------------------------------------------------------------------------------------------------------------
# robouse components
# ---------------------------------------------------------------------------------------------------------------------


def components_main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(
        prog="robouse components",
        description="the embodiments, scenes, task templates and modifiers composed tasks are built from",
    )
    sub = ap.add_subparsers(dest="ccmd", required=True, metavar="{list,show,check}")
    cl = sub.add_parser("list", help="list components")
    cl.add_argument("kind", nargs="?", choices=["embodiments", "scenes", "tasks", "modifiers"], help="only this kind")
    cl.add_argument(
        "--all", action="store_true", help="also list fixed components (the existing suites' robots and scenes)"
    )
    cs = sub.add_parser("show", help="print one component's manifest")
    cs.add_argument("id", metavar="ID", help="component id")
    cs.add_argument("--compatible", action="store_true", help="also list the combinations it composes into")
    cc = sub.add_parser("check", help="whether an embodiment, a scene and a task template fit together, and why not")
    cc.add_argument("embodiment", metavar="EMBODIMENT", help="embodiment component id")
    cc.add_argument("scene", metavar="SCENE", help="scene component id")
    cc.add_argument("task", metavar="TASK", help="task component id")
    _mod_args(cc)
    cc.add_argument("--seed", type=_seed, default=0, metavar="N", help="instance seed to lay out (default 0)")
    cc.add_argument("--json", action="store_true", help="print the report as JSON")
    ce = sub.add_parser("export")  # maintainers (unlisted): rewrite components/ from the runtime
    ce.add_argument("--root")
    cm = sub.add_parser("matrix")  # maintainers (unlisted): the compatibility matrix as JSON
    cm.add_argument("--out")
    a = ap.parse_args(argv)

    from ..components.compose import ComposeError, instantiate
    from ..components.registry import dump, export, load
    from ..components.resolve import check, modifiers

    if a.ccmd == "export":
        for f in export(a.root):
            print(f)
        return 0
    reg = load()
    if a.ccmd == "list":
        for kind in [a.kind] if a.kind else ["embodiments", "scenes", "tasks", "modifiers"]:
            print(f"{'task templates' if kind == 'tasks' else kind}:")
            for cid, m in sorted(getattr(reg, kind).items()):
                if not m.get("composable") and not a.all:
                    continue
                extra = {
                    "embodiments": lambda m: f"{m.get('track', '')}, {m.get('placement', 'fixed')}",
                    "scenes": lambda m: m.get("scenario", ""),
                    "tasks": lambda m: ", ".join(m.get("capabilities", [])),
                    "modifiers": lambda m: ", ".join(m.get("values", {})),
                }[kind](m)
                print(
                    f"  {cid:<24} {m['version']:<7} {m['title']}  ({extra})"
                    + ("" if m.get("composable") else "  [fixed]")
                )
        return 0
    if a.ccmd == "show":
        for kind in ("embodiments", "scenes", "tasks", "modifiers"):
            m = getattr(reg, kind).get(a.id)
            if not m:
                continue
            print(dump(m), end="")
            if a.compatible and kind != "modifiers" and m.get("composable"):
                print("compatible:")
                for e, s, t in itertools.product(
                    reg.composable("embodiments"), reg.composable("scenes"), reg.composable("tasks")
                ):
                    if a.id not in (e, s, t) or not check(reg, e, s, t).ok:
                        continue
                    try:
                        instantiate(e, s, t, modifiers({}), 0)
                    except ComposeError:
                        continue
                    print(f"  {e} + {s} + {t}")
            return 0
        return _err(f"no component {a.id!r}")
    if a.ccmd == "check":
        mods = modifiers({k: getattr(a, k) for k, *_ in MODS})
        try:
            rep = check(reg, a.embodiment, a.scene, a.task, mods)
        except KeyError as e:
            return _err(e.args[0], 2)
        if rep.ok:
            try:
                instantiate(a.embodiment, a.scene, a.task, mods, a.seed)
            except ComposeError as e:
                rep.ok, rep.reasons = False, [str(e)]
        if a.json:
            print(
                json.dumps(
                    {
                        "embodiment": a.embodiment,
                        "scene": a.scene,
                        "task": a.task,
                        "modifiers": mods,
                        "seed": a.seed,
                        **rep.as_dict(),
                    },
                    indent=2,
                )
            )
        elif rep.ok:
            print(f"compatible: {a.embodiment} + {a.scene} + {a.task}")
        else:
            print(f"incompatible: {a.embodiment} + {a.scene} + {a.task}")
            for r in rep.reasons:
                print(f"  - {r}")
        return 0 if rep.ok else 1
    if a.ccmd == "matrix":
        from ..components.matrix import build

        out = json.dumps(build(reg), indent=1)
        if a.out:
            Path(a.out).write_text(out)
            print(f"wrote {a.out}")
        else:
            print(out)
        return 0
    return 2


def remix_alias(argv: list[str]) -> int:
    """`robouse remix` (renamed): prints where it moved and runs the new command, for one release."""
    print(
        "robouse: `robouse remix` is renamed to `robouse tasks init` (and `remix --check` to `robouse components check`); "
        "the old name works until the next release",
        file=sys.stderr,
    )
    if "--check" in argv:
        rest = [x for x in argv if x not in ("--check",)]
        pos = {}
        out = []
        it = iter(rest)
        for x in it:
            if x in ("--embodiment", "--scene", "--task"):
                pos[x] = next(it)
            else:
                out.append(x)
        return components_main(
            ["check", pos.get("--embodiment", ""), pos.get("--scene", ""), pos.get("--task", ""), *out]
        )
    return tasks_main(["init", *argv])
