"""`robouse serve`, `run`, `run-many` and `export`: episode servers and trials, on this machine or on BenchFlow."""

from __future__ import annotations

import argparse
import gc
import importlib.util
import json
import os
import sys
import traceback
from pathlib import Path

from ..tasks import Task, bundled_tasks_root, find_tasks, load_task, resolve_task_path

HARNESS_HELP = (
    "harness: oracle, noop, claude-code, codex, mini-swe-agent, dimcode, molmoact2 or vla; --model a "
    "provider-prefixed id such as baseten/zai-org/GLM-5.3 (https://robouse.ai/docs/harnesses/)"
)
RECORD_SIZE_HELP = (
    "render the episode video at this size, e.g. 1280x720; agent images are unchanged "
    "(default: the backend's native size, or $ROBOUSE_RECORD_SIZE)"
)
SIM_HINT = (
    "robouse: the simulator dependencies are not installed ({err}); install the extra for the suite, e.g. "
    "`pip install 'robouse[sim]'` (MuJoCo suites), 'robouse[metaworld]', 'robouse[gymrobotics]', "
    "'robouse[robosuite]' or 'robouse[all]'"
)


def _engine_opts(p: argparse.ArgumentParser) -> None:
    # The BenchFlow engine needs a BenchFlow release with the task-format hook (on BenchFlow's `robouse` branch today),
    # so these options work but are not listed until that release exists.
    p.add_argument("--engine", choices=("local", "benchflow"), default="local", help=argparse.SUPPRESS)
    p.add_argument("--sandbox", default="docker", help=argparse.SUPPRESS)
    p.add_argument("--reasoning-effort", help=argparse.SUPPRESS)
    p.add_argument("--job-name", help=argparse.SUPPRESS)


def _trial_opts(p: argparse.ArgumentParser, many: bool) -> None:
    p.add_argument("--harness", "--agent", dest="harness", metavar="HARNESS", required=True, help=HARNESS_HELP)
    p.add_argument("--model", default="", help="model for the harness (default: the harness's)")
    p.add_argument(
        "--interface",
        choices=("cli", "mcp", "both"),
        help="what the agent drives the robot with: the robo CLI, the robo MCP tools, or both "
        "(default: " + ("each task's" if many else "the task's") + " robouse.interface, else cli)",
    )
    p.add_argument(
        "--timeout",
        type=float,
        help="agent wall-clock limit in seconds (default: "
        + ("each task's)" if many else "the task's agent.timeout_sec)"),
    )
    p.add_argument("--record-size", metavar="WxH", help=RECORD_SIZE_HELP)
    p.add_argument("--seed", type=_seed, help="override " + ("every task's" if many else "the task's") + " seed")
    p.add_argument("--rig", help=argparse.SUPPRESS)  # real-robot rig file; listed once live hardware has run
    p.add_argument(
        "--split", help="placement split: nominal, interpolation or extrapolation (tasks with seeded placements)"
    )
    p.add_argument(
        "--extra-instruction", default="", help="text appended to the task instruction (recorded in config.json)"
    )
    p.add_argument(
        "--prior-learnings", help="markdown notes (e.g. from `robouse summarize`) appended to the instruction"
    )
    _engine_opts(p)


def register(sub: argparse._SubParsersAction) -> None:
    s = sub.add_parser("serve")  # the episode server `run` starts; not listed
    s.add_argument("--task", required=True, help="task folder or bundled task id")
    s.add_argument("--run-dir", required=True, help="where the episode record is written")
    s.add_argument("--socket", required=True, help="Unix socket path to listen on")
    s.add_argument("--workspace", help="agent workspace (camera images are saved in its observations/)")
    s.add_argument("--seed", type=_seed, help="override the task's seed")
    s.add_argument("--max-wall-s", type=float, default=1800, help="wall-clock budget in seconds (default 1800)")
    s.add_argument("--record-size", metavar="WxH", help=RECORD_SIZE_HELP)
    s.add_argument("--ready-file", help="file created once the server is listening")
    s.add_argument(
        "--split", help="placement split: nominal, interpolation or extrapolation (tasks with seeded placements)"
    )
    s.add_argument("--secrets-stdin", action="store_true", help=argparse.SUPPRESS)  # the runner's token and key
    s.set_defaults(func=serve)

    r = sub.add_parser("run", help="run one trial end to end")
    r.add_argument("--task", required=True, help="task folder or bundled task id")
    r.add_argument("--out", required=True, help="job folder; the trial is written to OUT/<task>__<harness>__<id>/")
    r.add_argument("--seeds", help=argparse.SUPPRESS)  # BenchFlow engine only
    _trial_opts(r, many=False)
    r.set_defaults(func=run)

    m = sub.add_parser("run-many", help="run a set of trials")
    m.add_argument("--tasks", help="task root (default: the bundled tasks)")
    m.add_argument("--out", required=True, help="job folder; one trial folder per task")
    m.add_argument("--filter", default="", help="only task ids containing this substring")
    m.add_argument("--task-list", help="file with one task id per line (a subset of --tasks)")
    m.add_argument(
        "--include", action="append", help="only task ids matching this glob (repeatable), e.g. 'libero-10-*'"
    )
    m.add_argument("--exclude", action="append", help="skip task ids matching this glob (repeatable)")
    m.add_argument("--mode", choices=["sim", "real"], help=argparse.SUPPRESS)
    m.add_argument("--sample", type=int, default=0, help="run a seeded random sample of N tasks")
    m.add_argument("--sample-seed", type=int, default=0, help="seed for --sample (default 0)")
    m.add_argument("--limit", type=int, default=0, help="run at most this many tasks (default 0: all)")
    m.add_argument(
        "--seeds",
        help="episode seeds per task, e.g. 0-4 or 0,3,7: one trial per task and seed, for pass@k "
        "(default: the task's own seed)",
    )
    m.add_argument("--epochs", type=int, default=1, help="repeat each task and seed N times (default 1)")
    m.add_argument("--concurrency", type=int, default=1, help="trials run in parallel (default 1)")
    m.add_argument(
        "--retries",
        type=int,
        default=0,
        help="rerun a trial up to N times after an infrastructure failure, "
        "with backoff; the failed trial is kept in OUT/_infra_failed (default 0)",
    )
    m.add_argument(
        "--resume",
        action="store_true",
        help="skip tasks that have a finished trial in --out that did not fail on infrastructure",
    )
    m.add_argument("--dry-run", action="store_true", help="print the resolved trial plan and exit")
    _trial_opts(m, many=True)
    m.set_defaults(func=run_many)

    ex = sub.add_parser("export")  # native BenchFlow packages; listed once BenchFlow releases the task-format hook
    ex.add_argument("--tasks", required=True, nargs="+", help="task folders, suite folders or a task root")
    ex.add_argument("--out", required=True, help="bundle folder: <out>/tasks/<id>/ and <out>/runtime-<hash>/")
    ex.add_argument("--filter", default="", help="comma-separated substrings; keep tasks whose id contains one")
    ex.add_argument("--noop", action="store_true", help="no-op reference solution (negative control: must score 0)")
    ex.set_defaults(func=export)


def _seed(x: str) -> int:
    if not x.isdigit():
        raise argparse.ArgumentTypeError(f"must be a non-negative integer, not {x!r}")
    return int(x)


def _task_ids(path: str | None) -> list[str] | None:
    if not path:
        return None
    return [ln.strip() for ln in Path(path).read_text().splitlines() if ln.strip() and not ln.startswith("#")]


def _unknown_harness(name: str) -> int:
    """0 when `name` is a harness of the local engine (built in or a `robouse.harnesses` entry point), else 2."""
    from importlib.metadata import entry_points

    from ..harnesses import HARNESSES
    from ..harnesses.models import LEGACY_HARNESSES

    if (
        name in HARNESSES
        or name in LEGACY_HARNESSES
        or any(ep.name == name for ep in entry_points(group="robouse.harnesses"))
    ):
        return 0
    print(f"robouse: unknown harness {name!r}; harnesses: {', '.join(HARNESSES)}", file=sys.stderr)
    return 2


def _prepare_local(a: argparse.Namespace) -> int:
    """Settings the episode servers read from the environment, and the simulator check. 0 when ready."""
    if getattr(a, "harness", None) and (rc := _unknown_harness(a.harness)):  # `serve` has no harness
        return rc
    if getattr(a, "rig", None):
        os.environ["ROBOUSE_RIG"] = str(Path(a.rig).resolve())
    if getattr(a, "record_size", None):
        from ..core.recording import record_size

        try:
            w, h = record_size(a.record_size)  # type: ignore[misc]
        except ValueError as e:
            print(f"robouse: {e}", file=sys.stderr)
            return 2
        os.environ["ROBOUSE_RECORD_SIZE"] = f"{w}x{h}"
    missing = [m for m in ("mujoco", "imageio", "PIL") if importlib.util.find_spec(m) is None]
    if missing:
        print(SIM_HINT.format(err="missing: " + ", ".join(missing)), file=sys.stderr)
        return 1
    from ..core import gl

    gl.auto_select()  # MUJOCO_GL unset on Linux without a display: OSMesa or EGL, whichever renders; inherited by trials
    return 0


def serve(a: argparse.Namespace) -> int:
    if rc := _prepare_local(a):
        return rc
    from ..session import serve as serve_episode

    spec = load_task(resolve_task_path(a.task)).spec
    if a.seed is not None:
        spec["seed"] = a.seed
    if a.split:
        spec["placement"] = {**(spec.get("placement") or {}), "split": a.split}
    secrets = json.loads(sys.stdin.readline() or "{}") if a.secrets_stdin else None
    from ..core import gl

    why = None
    try:
        res = serve_episode(
            spec,
            Path(a.run_dir),
            a.socket,
            Path(a.workspace) if a.workspace else None,
            max_wall_s=a.max_wall_s,
            ready_file=Path(a.ready_file) if a.ready_file else None,
            secrets=secrets,
        )
    except Exception as e:
        if (why := gl.explain(e)) is None:
            raise
        traceback.print_exc()  # the details stay in the log
    if why:
        # outside the handler, so MuJoCo's half-built Renderer is collected (and prints its own follow-on error) first:
        # the plain message is the last line, the one the runner reports
        gc.collect()
        print(f"robouse: {why}", file=sys.stderr, flush=True)
        return 1
    print(json.dumps({k: v for k, v in res.items() if k != "signature"}))
    return 0


def run(a: argparse.Namespace) -> int:
    if a.engine == "benchflow":
        return _run_benchflow(a, [load_task(resolve_task_path(a.task))])
    if a.seeds:
        print(
            "robouse: run takes one seed (--seed); use run-many --seeds for several",
            file=sys.stderr,
        )
        return 2
    if rc := _prepare_local(a):
        return rc
    from ..runner import run_trial

    res = run_trial(
        load_task(resolve_task_path(a.task)),
        a.harness,
        a.model,
        Path(a.out),
        timeout=a.timeout,
        seed=a.seed,
        split=a.split,
        extra_instruction=a.extra_instruction,
        prior_learnings=a.prior_learnings,
        interface=a.interface,
    )
    print(json.dumps(res, indent=2))
    return 1 if res.get("exception") else 0  # the trial could not run to a result (reward 0 is a result)


def run_many(a: argparse.Namespace) -> int:
    from ..runner import parse_seeds, select_tasks

    root = Path(a.tasks) if a.tasks else bundled_tasks_root()
    ids = _task_ids(a.task_list)
    if a.engine == "benchflow":
        return _run_benchflow(
            a, select_tasks(root, a.filter, ids, a.include, a.exclude, a.limit, a.sample, a.sample_seed, a.mode)
        )
    if a.seed is not None and a.seeds:
        print("robouse: use --seed or --seeds, not both", file=sys.stderr)
        return 2
    seeds = str(a.seed) if a.seed is not None else a.seeds
    try:
        plan_seeds = parse_seeds(seeds)
    except ValueError as e:
        print(f"robouse: --seeds: {e}", file=sys.stderr)
        return 2
    if rc := _unknown_harness(a.harness):
        return rc
    if a.dry_run:
        ts = select_tasks(root, a.filter, ids, a.include, a.exclude, a.limit, a.sample, a.sample_seed, a.mode)
        print(
            json.dumps(
                {
                    "harness": a.harness,
                    "model": a.model,
                    "tasks": [t.id for t in ts],
                    "seeds": plan_seeds,
                    "epochs": a.epochs,
                    "trials": len(ts) * len(plan_seeds) * max(1, a.epochs),
                    "split": a.split,
                    "concurrency": 1 if a.rig else a.concurrency,
                    "rig": a.rig,
                    "retries": a.retries,
                },
                indent=2,
            )
        )
        return 0
    if rc := _prepare_local(a):
        return rc
    from ..runner import run_many as run_trials

    results = run_trials(
        root,
        a.harness,
        a.model,
        Path(a.out),
        filt=a.filter,
        limit=a.limit,
        concurrency=1 if a.rig else a.concurrency,
        timeout=a.timeout,
        ids=ids,
        resume=a.resume,
        seeds=seeds,
        epochs=a.epochs,
        include=a.include,
        exclude=a.exclude,
        sample=a.sample,
        sample_seed=a.sample_seed,
        retries=a.retries,
        split=a.split,
        mode=a.mode,
        extra_instruction=a.extra_instruction,
        prior_learnings=a.prior_learnings,
        interface=a.interface,
    )
    failed = sum(1 for r in results if r.get("exception"))
    if failed:
        print(
            f"robouse: {failed} trial(s) failed on infrastructure (exception_info in their result.json)",
            file=sys.stderr,
        )
    return 1 if failed else 0


def _benchflow_missing(e: ImportError) -> int:
    print(f"robouse: this needs BenchFlow ({e}); install robouse[benchflow] (Python 3.12)", file=sys.stderr)
    return 1


def _run_benchflow(a: argparse.Namespace, tasks: list[Task]) -> int:
    try:
        from benchflow.embodied.rollouts import parse_seeds

        from ..engine.run import run_job
    except ImportError as e:
        return _benchflow_missing(e)
    seeds = parse_seeds(a.seeds) if a.seeds else None
    if seeds and a.seed is not None:
        print("robouse: use --seed or --seeds, not both", file=sys.stderr)
        return 2
    res = run_job(
        tasks,
        a.harness,
        a.model,
        Path(a.out),
        sandbox=a.sandbox,
        concurrency=getattr(a, "concurrency", 1),
        timeout=a.timeout,
        seed=a.seed,
        job_name=a.job_name,
        reasoning_effort=a.reasoning_effort,
        seeds=seeds,
    )
    return 0 if res["errored"] == 0 else 1


def export(a: argparse.Namespace) -> int:
    try:
        from ..engine.format import materialize
    except ImportError as e:
        return _benchflow_missing(e)
    keys = [k for k in a.filter.split(",") if k]
    n = 0
    for root in a.tasks:
        for t in find_tasks(root):
            if keys and not any(k in t.id for k in keys):
                continue
            print("exported", materialize(t.path, Path(a.out), noop=a.noop, flat=True))
            n += 1
    print(
        f"{n} task(s) -> {Path(a.out).resolve()}   run: bench eval run --tasks-dir {Path(a.out) / 'tasks'} "
        "--agent oracle --sandbox docker"
    )
    return 0 if n else 1
