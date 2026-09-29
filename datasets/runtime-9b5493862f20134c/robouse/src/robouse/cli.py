"""`robouse`: trusted-side commands.

  robouse run --task TASK --agent NAME [--model M] --out JOBS         run one trial (a BenchFlow job)
  robouse run-many [--tasks ROOT] --agent NAME ... --out JOBS         run a set of trials (one BenchFlow job)
  robouse export --tasks ROOT... --out DIR [--noop]                   write native BenchFlow task packages
  robouse serve --task TASK --run-dir DIR --socket PATH               run one episode server (the simulator)
  robouse tasks [ROOT] [--path]                                       list tasks
  robouse fetch-assets [--dest DIR] [--robot NAME]                    download robot meshes (MuJoCo Menagerie)
  robouse remix --embodiment E --scene S --task T [modifiers] --out DIR  compose a task from components (validated)
  robouse components list|show|matrix                                 the component registry

`run` and `run-many` run on BenchFlow (`--engine benchflow`, the default): BenchFlow's agents, sandboxes, verifier
and trial layout; the same as `bench eval run --tasks-dir <tasks> --agent NAME --sandbox docker`. `--engine local`
is the older in-process runner (host processes under a macOS Seatbelt profile, no Docker), kept for machines
without Docker and for the local-only harnesses (claude-code-glm, codex-glm, mini-swe-agent-glm, molmoact2).

TASK is a task folder or the id of a bundled task (e.g. `arc-gravity`); ROOT defaults to the bundled tasks.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

from . import __version__
from .tasks import bundled_tasks_root, find_tasks, load_task, resolve_task_path

HARNESS_HELP = ("oracle, noop, claude-code, codex, claude-code-glm, codex-glm, mini-swe-agent-glm or molmoact2 "
                "(see https://robouse.ai/docs/harnesses/)")
SIM_HINT = ("robouse: the simulator dependencies are not installed ({err}).\n"
            "Install the extra for this suite, e.g. `pip install 'robouse[sim]'` (MuJoCo tabletop suites), "
            "`robouse[metaworld]`, `robouse[gymrobotics]`, `robouse[robosuite]` or `robouse[all]`.")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="robouse", description="robo-use: agent as a policy for embodied agents. "
                                 "Runs robot tasks with an agent harness as the policy. https://robouse.ai")
    ap.add_argument("--version", action="version", version=f"robouse {__version__}")
    sub = ap.add_subparsers(dest="cmd", required=True)

    s = sub.add_parser("serve", help="run one trusted episode server (used by `run`)")
    s.add_argument("--task", required=True, help="task folder or bundled task id")
    s.add_argument("--run-dir", required=True, help="where the episode record is written")
    s.add_argument("--socket", required=True, help="Unix socket path to listen on")
    s.add_argument("--workspace", help="agent workspace (camera images are saved in its observations/)")
    s.add_argument("--seed", type=int, help="override the task's seed")
    s.add_argument("--max-wall-s", type=float, default=1800, help="wall-clock budget in seconds (default 1800)")
    s.add_argument("--ready-file", help="file created once the server is listening")

    def run_opts(p, many: bool):
        p.add_argument("--agent", "--harness", dest="agent", required=True,
                       help="BenchFlow agent (oracle, noop, claude, codex, ... see `bench agent list`; claude-code is "
                            "an alias for claude); with --engine local, a robouse harness: " + HARNESS_HELP)
        p.add_argument("--model", default="", help="model for the agent (default: the agent's default)")
        p.add_argument("--out", required=True,
                       help="jobs folder; the BenchFlow job is written to OUT/<job>/<task>__<id>/ "
                            "(--engine local: OUT/<task>__<harness>__<id>/)")
        p.add_argument("--timeout", type=float, help="agent wall-clock limit in seconds (default: "
                       + ("each task's" if many else "the task's agent.timeout_sec") + ")")
        p.add_argument("--seed", type=int, help="override the task's seed")
        p.add_argument("--seeds", help="seeded rollouts (BenchFlow engine): one trial per task and seed, e.g. 0-4 or "
                       "0,3,7; summary.json gets pass@k, mean and std per task")
        p.add_argument("--engine", choices=("benchflow", "local"), default="benchflow",
                       help="benchflow (default): BenchFlow with Docker; local: host processes, macOS Seatbelt "
                            "sandbox, no Docker")
        p.add_argument("--sandbox", default="docker", help="BenchFlow sandbox (default docker)")
        p.add_argument("--reasoning-effort", help="agent reasoning effort (BenchFlow agents that expose one)")
        p.add_argument("--job-name", help="BenchFlow job name (default: a timestamp)")

    r = sub.add_parser("run", help="run one trial end to end")
    r.add_argument("--task", required=True, help="task folder or bundled task id")
    run_opts(r, many=False)

    m = sub.add_parser("run-many", help="run a set of trials")
    m.add_argument("--tasks", help="task root (default: the bundled tasks)")
    run_opts(m, many=True)
    m.add_argument("--filter", default="", help="only task ids containing this substring")
    m.add_argument("--limit", type=int, default=0, help="run at most this many tasks (0: all)")
    m.add_argument("--concurrency", type=int, default=1, help="trials run in parallel (default 1)")
    m.add_argument("--task-list", help="file with one task id per line (subset of --tasks)")
    m.add_argument("--resume", action="store_true", help="(--engine local) skip tasks that already have a finished, non-infra-failed trial in --out")

    e = sub.add_parser("export", help="write robouse tasks as native BenchFlow task packages (a movable bundle)")
    e.add_argument("--tasks", required=True, nargs="+", help="task folders, suite folders or a task root")
    e.add_argument("--out", required=True, help="bundle directory: <out>/tasks/<id>/ and <out>/runtime-<hash>/")
    e.add_argument("--filter", default="", help="comma-separated substrings; keep tasks whose id matches one")
    e.add_argument("--noop", action="store_true", help="noop oracle (negative control: must score 0)")

    t = sub.add_parser("tasks", help="list tasks (default: the bundled tasks)")
    t.add_argument("root", nargs="?", help="task root (default: the bundled tasks)")
    t.add_argument("--path", action="store_true", help="print the bundled task directory and exit")

    f = sub.add_parser("fetch-assets", help="download the MuJoCo Menagerie robot models (menagerie, RoboHarm, drone and embodiment suites)")
    f.add_argument("--dest", help="target directory (default: ~/.cache/robouse/menagerie or $ROBOUSE_MENAGERIE_ASSETS)")
    f.add_argument("--robot", action="append", help="only this Menagerie robot folder (repeatable), e.g. unitree_go2")

    x = sub.add_parser("remix", help="compose a task from an embodiment, a scene and a task component (+ modifiers)")
    x.add_argument("--embodiment", required=True, help="embodiment component id (robouse components list)")
    x.add_argument("--scene", required=True, help="scene component id")
    x.add_argument("--task", required=True, help="task component id")
    x.add_argument("--obs", default="state", help="observation mode: state (default), vision or noisy")
    x.add_argument("--budget", default="normal", help="budget: normal (default), loose or tight")
    x.add_argument("--perturbation", default="none", help="none (default), push, dynamics or clutter")
    x.add_argument("--safety", default="none", help="safety overlay: none (default), direct, indirect or privacy")
    x.add_argument("--roles", default="single", help="single (default) or planner-operator")
    x.add_argument("--seed", type=int, default=0, help="instance seed (default 0)")
    x.add_argument("--out", help="folder to write the task folder into (not needed with --check)")
    x.add_argument("--check", action="store_true", help="only report whether the components fit together, and why not")
    x.add_argument("--json", action="store_true", help="with --check: print the report as JSON")
    x.add_argument("--no-validate", action="store_true", help="skip the oracle (must score 1) and no-op (must score 0) runs")
    x.add_argument("--keep-runs", help="keep the validation trials in this folder")

    c = sub.add_parser("components", help="list or show components (embodiments, scenes, tasks, modifiers)")
    csub = c.add_subparsers(dest="ccmd", required=True)
    cl = csub.add_parser("list", help="list components")
    cl.add_argument("kind", nargs="?", choices=["embodiments", "scenes", "tasks", "modifiers"], help="only this kind")
    cl.add_argument("--all", action="store_true", help="also list fixed components (the existing suites' robots and scenes)")
    cs = csub.add_parser("show", help="print one component's manifest")
    cs.add_argument("id", help="component id")
    cs.add_argument("--compatible", action="store_true", help="also list what it can be remixed with")
    ce = csub.add_parser("export", help="(maintainers) rewrite components/ from the runtime declarations")
    ce.add_argument("--root", help="components folder (default: the repo's components/)")
    cm = csub.add_parser("matrix", help="the compatibility matrix of every embodiment x scene x task, as JSON")
    cm.add_argument("--out", help="write it here (default: stdout)")

    a = ap.parse_args(argv)
    if a.cmd == "remix":
        return _remix(a)
    if a.cmd == "components":
        return _components(a)
    if a.cmd == "fetch-assets":
        from .assets import fetch

        fetch(Path(a.dest) if a.dest else None, robots=a.robot)
        return 0
    if a.cmd == "export":
        from .engine.format import materialize

        keys = [k for k in a.filter.split(",") if k]
        n = 0
        for root in a.tasks:
            for t in find_tasks(root):
                if keys and not any(k in t.id for k in keys):
                    continue
                print("exported", materialize(t.path, Path(a.out), noop=a.noop, flat=True))
                n += 1
        print(f"{n} task(s) -> {Path(a.out).resolve()}   run: bench eval run --tasks-dir {Path(a.out) / 'tasks'} --agent oracle --sandbox docker")
        return 0 if n else 1
    if a.cmd in ("run", "run-many") and a.engine == "benchflow":
        from benchflow.embodied.rollouts import parse_seeds

        from .engine.run import run_job, select_tasks

        seeds = parse_seeds(a.seeds) if a.seeds else None
        if seeds and a.seed is not None:
            print("robouse: use --seed or --seeds, not both", file=sys.stderr)
            return 2

        if a.cmd == "run":
            tasks = [load_task(resolve_task_path(a.task))]
        else:
            ids = [l.strip() for l in open(a.task_list) if l.strip() and not l.startswith("#")] if a.task_list else None
            tasks = select_tasks(Path(a.tasks) if a.tasks else bundled_tasks_root(), a.filter, ids, a.limit)
        res = run_job(tasks, a.agent, a.model, Path(a.out), sandbox=a.sandbox,
                      concurrency=getattr(a, "concurrency", 1), timeout=a.timeout, seed=a.seed,
                      job_name=a.job_name, reasoning_effort=a.reasoning_effort, seeds=seeds)
        return 0 if res["errored"] == 0 else 1
    if a.cmd in ("serve", "run", "run-many"):
        import importlib.util

        missing = [m for m in ("mujoco", "imageio", "PIL") if importlib.util.find_spec(m) is None]
        if missing:
            print(SIM_HINT.format(err="missing: " + ", ".join(missing)), file=sys.stderr)
            return 1
    if a.cmd == "serve":
        from .session import serve

        task = load_task(resolve_task_path(a.task))
        spec = task.spec
        if a.seed is not None:
            spec["seed"] = a.seed
        res = serve(spec, Path(a.run_dir), a.socket, Path(a.workspace) if a.workspace else None,
                    max_wall_s=a.max_wall_s, ready_file=Path(a.ready_file) if a.ready_file else None)
        print(json.dumps(res))
        return 0
    if getattr(a, "seeds", None):
        print("robouse: --seeds needs the BenchFlow engine (drop --engine local, or use --seed)", file=sys.stderr)
        return 2
    if a.cmd == "run":
        from .runner import run_trial

        res = run_trial(load_task(resolve_task_path(a.task)), a.agent, a.model, Path(a.out), timeout=a.timeout, seed=a.seed)
        print(json.dumps(res, indent=2))
        return 0
    if a.cmd == "run-many":
        from .runner import run_many

        ids = [l.strip() for l in open(a.task_list) if l.strip() and not l.startswith("#")] if a.task_list else None
        run_many(Path(a.tasks) if a.tasks else bundled_tasks_root(), a.agent, a.model, Path(a.out), filt=a.filter, limit=a.limit,
                 concurrency=a.concurrency, timeout=a.timeout, ids=ids, resume=a.resume, seed=a.seed)
        return 0
    if a.cmd == "tasks":
        if a.path:
            print(bundled_tasks_root())
            return 0
        try:
            for task in find_tasks(a.root or bundled_tasks_root()):
                print(task.id, task.spec.get("backend"), task.spec.get("env", ""))
            sys.stdout.flush()
        except BrokenPipeError:  # e.g. `robouse tasks | head`
            os.dup2(os.open(os.devnull, os.O_WRONLY), sys.stdout.fileno())
        return 0
    return 2


def _remix(a) -> int:
    from .components.registry import load
    from .components.remix import RemixError, instantiate, record_validation, validate, write
    from .components.resolve import check, modifiers

    reg = load()
    mods = modifiers({"obs": a.obs, "budget": a.budget, "perturbation": a.perturbation, "safety": a.safety, "roles": a.roles})
    try:
        rep = check(reg, a.embodiment, a.scene, a.task, mods)
    except KeyError as e:
        print(f"robouse remix: {e.args[0]}", file=sys.stderr)
        return 2
    if a.check:
        if rep.ok:
            try:
                instantiate(a.embodiment, a.scene, a.task, mods, a.seed)
            except RemixError as e:
                rep.ok, rep.reasons = False, [str(e)]
        if a.json:
            print(json.dumps({"embodiment": a.embodiment, "scene": a.scene, "task": a.task, "modifiers": mods, "seed": a.seed,
                              **rep.as_dict()}, indent=2))
        elif rep.ok:
            print(f"compatible: {a.embodiment} + {a.scene} + {a.task}")
        else:
            print(f"incompatible: {a.embodiment} + {a.scene} + {a.task}")
            for r in rep.reasons:
                print(f"  - {r}")
        return 0 if rep.ok else 1
    if not rep.ok:
        print("robouse remix: incompatible: " + "; ".join(rep.reasons), file=sys.stderr)
        return 1
    if not a.out:
        print("robouse remix: --out is required (or use --check)", file=sys.stderr)
        return 2
    try:
        d = write(a.embodiment, a.scene, a.task, mods, a.seed, Path(a.out), reg)
    except RemixError as e:
        print(f"robouse remix: incompatible: {e}", file=sys.stderr)
        return 1
    print(f"wrote {d}")
    if a.no_validate:
        return 0
    v = validate(d, Path(a.keep_runs) if a.keep_runs else None)
    print(f"oracle reward {v['oracle']['reward']} ({v['oracle']['outcome']}, {v['oracle']['steps']} steps); "
          f"noop reward {v['noop']['reward']} ({v['noop']['outcome']})")
    if not v["accepted"]:
        import shutil

        shutil.rmtree(d, ignore_errors=True)
        print("robouse remix: rejected (the oracle must score 1 and the no-op 0); the folder was removed", file=sys.stderr)
        return 1
    record_validation(d, v)
    print("accepted: oracle 1, noop 0")
    return 0


def _components(a) -> int:
    from .components.registry import dump, export, load
    from .components.resolve import check

    if a.ccmd == "export":
        for f in export(a.root):
            print(f)
        return 0
    reg = load()
    if a.ccmd == "list":
        for kind in ([a.kind] if a.kind else ["embodiments", "scenes", "tasks", "modifiers"]):
            items = getattr(reg, kind)
            print(f"{kind}:")
            for cid, m in sorted(items.items()):
                if not m.get("remixable") and not a.all:
                    continue
                extra = {"embodiments": lambda m: f"{m.get('track', '')}, {m.get('placement', 'fixed')}",
                         "scenes": lambda m: m.get("scenario", ""),
                         "tasks": lambda m: ", ".join(m.get("capabilities", [])),
                         "modifiers": lambda m: ", ".join(m.get("values", {}))}[kind](m)
                print(f"  {cid:<24} {m['version']:<7} {m['title']}  ({extra})" + ("" if m.get("remixable") else "  [fixed]"))
        return 0
    if a.ccmd == "show":
        for kind in ("embodiments", "scenes", "tasks", "modifiers"):
            m = getattr(reg, kind).get(a.id)
            if m:
                print(dump(m), end="")
                if a.compatible and kind != "modifiers" and m.get("remixable"):
                    from .components.remix import RemixError, instantiate
                    from .components.resolve import modifiers

                    print("compatible:")
                    for e in reg.remixable("embodiments"):
                        for s in reg.remixable("scenes"):
                            for t in reg.remixable("tasks"):
                                if a.id not in (e, s, t) or not check(reg, e, s, t).ok:
                                    continue
                                try:
                                    instantiate(e, s, t, modifiers({}), 0)
                                except RemixError:
                                    continue
                                print(f"  {e} + {s} + {t}")
                return 0
        print(f"robouse components: no component {a.id!r}", file=sys.stderr)
        return 1
    if a.ccmd == "matrix":
        from .components.matrix import build

        out = json.dumps(build(reg), indent=1)
        if a.out:
            Path(a.out).write_text(out)
            print(f"wrote {a.out}")
        else:
            print(out)
        return 0
    return 2


if __name__ == "__main__":
    sys.exit(main())
