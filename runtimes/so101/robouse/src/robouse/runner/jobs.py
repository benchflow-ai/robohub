"""Sets of trials: task selection, seeds, epochs, retries and resume. On a real robot a safety stop holds the batch until
the operator has inspected the robot (real/batch.py)."""

from __future__ import annotations

import concurrent.futures as cf
import json
import shutil
import sys
import threading
import time
from pathlib import Path

from ..tasks import Task, find_tasks
from .trial import interrupt_all, now, run_trial


def parse_seeds(spec: str | None) -> list[int | None]:
    """ "0-4" -> [0..4]; "0,3,7" -> [0, 3, 7]; None -> [None] (the task's own seed)."""
    if not spec:
        return [None]
    out: list[int | None] = []
    for part in str(spec).split(","):
        a, _, b = part.strip().partition("-")
        if not a.isdigit() or (b and not b.isdigit()) or (b and int(b) < int(a)):
            raise ValueError(f"seeds must look like 0-4 or 0,3,7 (non-negative integers), not {spec!r}")
        out += list(range(int(a), int(b) + 1)) if b else [int(a)]
    return out


def select_tasks(
    root: Path,
    filt: str = "",
    ids: list[str] | None = None,
    include: list[str] | None = None,
    exclude: list[str] | None = None,
    limit: int = 0,
    sample: int = 0,
    sample_seed: int = 0,
    mode: str | None = None,
) -> list[Task]:
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
        tasks = [
            t
            for t in tasks
            if (t.metadata.get("embodiment_mode") or ("real" if t.spec.get("backend") == "real" else "sim")) == mode
        ]
    if sample and sample < len(tasks):
        tasks = sorted(random.Random(sample_seed).sample(tasks, sample), key=lambda t: t.id)
    if limit:
        tasks = tasks[:limit]
    return tasks


def resume_refusal(job_dir: Path, harness: str, model: str) -> str | None:
    """Why `--resume` cannot add trials of (harness, model) to this job folder, or None. A job holds one pair: when its
    trials recorded another (models.canonical; robouse 0.4.0 left the model empty for its default, which reads as
    baseten/zai-org/GLM-5.3), resuming would fill the rest of the job with a different pair."""
    from ..harnesses import models
    from .trial import image_input_plan

    plan = image_input_plan(harness, model)
    want = models.canonical(plan["harness"], plan["model"] or "")
    other: dict[tuple[str, str], int] = {}
    for cj in job_dir.glob("*/config.json"):
        try:
            c = json.loads(cj.read_text())
        except (OSError, ValueError):  # a trial folder that is still being written, or broken
            continue
        if isinstance(c, dict) and c.get("harness"):
            pair = models.canonical(str(c["harness"]), str(c.get("model") or ""))
            if pair != want:
                other[pair] = other.get(pair, 0) + 1
    if not other:
        return None

    def name(pair: tuple[str, str]) -> str:
        return f"{pair[0]} with {pair[1]}" if pair[1] else pair[0]

    found = ", ".join(f"{name(p)} ({n} trial{'s' * (n != 1)})" for p, n in sorted(other.items()))
    return (
        f"--resume: {job_dir} holds trials of {found}, not of {name(want)}; a job holds one harness and model, so run "
        "this pair into a new --out"
    )


def run_many(
    root: Path,
    harness: str,
    model: str,
    job_dir: Path,
    filt: str = "",
    limit: int = 0,
    concurrency: int = 1,
    timeout: float | None = None,
    ids: list[str] | None = None,
    resume: bool = False,
    seeds: str | None = None,
    epochs: int = 1,
    include: list[str] | None = None,
    exclude: list[str] | None = None,
    sample: int = 0,
    sample_seed: int = 0,
    retries: int = 0,
    split: str | None = None,
    mode: str | None = None,
    extra_instruction: str = "",
    prior_learnings: str | None = None,
    interface: str | None = None,
) -> list[dict]:
    tasks = select_tasks(root, filt, ids, include, exclude, limit, sample, sample_seed, mode)
    if concurrency > 1 and any(t.spec.get("backend") == "real" and _live(t.spec) for t in tasks):
        # one live robot, one trial at a time (a rig from --rig or ROBOUSE_RIG): a stop must hold every later trial
        print("robouse: live-robot trials run one at a time (--concurrency is ignored)", file=sys.stderr, flush=True)
        concurrency = 1
    runs = [(t, sd, ep) for t in tasks for sd in parse_seeds(seeds) for ep in range(max(1, epochs))]
    if (
        resume
    ):  # skip (task, seed, epoch) already finished in this job (an unfinished trial dir is left as evidence and rerun)
        if why := resume_refusal(job_dir, harness, model):  # only trials of the same pair count as done
            raise ValueError(why)
        done = set()
        for rj in job_dir.glob("*/result.json"):
            try:
                r = json.loads(rj.read_text())
                c = json.loads((rj.parent / "config.json").read_text())
            except (OSError, ValueError):  # a trial folder that is still being written, or broken
                continue
            if (
                r.get("status", "completed") == "completed"
                and not r.get("exception_info")
                and (r.get("episode") or {}).get("outcome")
            ):
                done.add((r.get("task_name"), c.get("seed"), c.get("epoch", 0)))
        runs = [x for x in runs if (x[0].id, x[1], x[2]) not in done]
    job_dir.mkdir(parents=True, exist_ok=True)
    (job_dir / "job.json").write_text(
        json.dumps(
            {
                "harness": harness,
                "model": model,
                "tasks": len(tasks),
                "seeds": seeds,
                "epochs": epochs,
                "split": split,
                "include": include,
                "exclude": exclude,
                "sample": sample,
                "sample_seed": sample_seed,
                "retries": retries,
                "extra_instruction": extra_instruction,
                "prior_learnings": prior_learnings,
                "interface": interface,
                "started_at": now(),
            },
            indent=2,
        )
    )
    results = []
    not_run: list[str] = []
    halt = threading.Event()  # a real-robot safety stop the operator did not clear: no further trial starts
    hold = threading.Lock()  # held while the operator inspects the robot: the next trial waits

    def label_of(item) -> str:
        t, sd, ep = item
        return t.id + (f"@seed{sd}" if sd is not None else "") + (f"#e{ep}" if epochs > 1 else "")

    def one(item) -> dict:
        t, sd, ep = item
        label = label_of(item)
        with hold:
            if halt.is_set():
                return {"task": label, "not_run": True}
        real = t.spec.get("backend") == "real"
        live = real and _live(t.spec)
        r: dict = {}
        try:
            for attempt in range(retries + 1):  # infrastructure failures only (a model's failure is a result)
                r = run_trial(
                    t,
                    harness,
                    model,
                    job_dir,
                    timeout=timeout,
                    seed=sd,
                    epoch=ep,
                    split=split,
                    extra_instruction=extra_instruction,
                    prior_learnings=prior_learnings,
                    attempt=attempt,
                    interface=interface,
                )
                if not r["exception"] or live:  # a live robot is never retried blind: a failed trial holds the batch
                    break
                if attempt < retries:
                    bad = job_dir / r["trial"]
                    dest = job_dir / "_infra_failed"
                    dest.mkdir(exist_ok=True)
                    if bad.exists():
                        shutil.move(str(bad), str(dest / bad.name))
                    time.sleep(min(60, 5 * 2**attempt))
            r["task"] = label
            if real:
                from ..real import batch

                stop = batch.stop_of(job_dir / r["trial"], t.spec)
                if stop is not None:
                    with hold:  # the operator inspects the robot; no other trial starts meanwhile
                        if not halt.is_set() and not batch.inspect(stop, job_dir):
                            halt.set()
                            r["halted_batch"] = stop
        except Exception as e:  # a real robot's batch fails closed: no further trial starts
            if not real:
                raise
            from ..backends.real.backend import real_mode
            from ..real import batch

            halt.set()
            r = {
                "trial": r.get("trial"),
                "reward": r.get("reward", 0.0),
                "outcome": r.get("outcome"),
                "steps": r.get("steps"),
                "wall_s": r.get("wall_s", 0.0),
                "exception": r.get("exception") or f"{type(e).__name__}: {e}",
                "task": label,
                "halted_batch": batch.fail_closed(job_dir, label, r.get("trial"), real_mode(t.spec), e),
            }
        return r

    real_batch = any(t.spec.get("backend") == "real" for t, _, _ in runs)
    with open(job_dir / "run_log.jsonl", "a") as log, cf.ThreadPoolExecutor(max_workers=max(1, concurrency)) as ex:
        futs = {ex.submit(one, x): x for x in runs}
        try:
            for fut in cf.as_completed(futs):
                r = fut.result()
                if r.get("not_run"):
                    not_run.append(r["task"])
                    continue
                results.append(r)
                log.write(json.dumps(r) + "\n")
                log.flush()
                print(
                    f"{r['task']:<40} reward={r['reward']:.0f} outcome={r['outcome']} steps={r['steps']} {r['wall_s']}s"
                    + (f" EXC {r['exception']}" if r["exception"] else ""),
                    flush=True,
                )
        except KeyboardInterrupt:  # stop the running agents; their trials are recorded as interrupted
            interrupt_all()
            ex.shutdown(wait=True, cancel_futures=True)
            raise
        except Exception as e:
            if not real_batch:  # a simulated batch as before: the trials already queued still run, then the error
                raise
            halt.set()  # a real robot's batch fails closed: no further trial starts
            ex.shutdown(wait=True, cancel_futures=True)
            from ..real import batch

            stop = batch.fail_closed(job_dir, "(batch)", None, None, e)
            results.append(
                {"task": "(batch)", "reward": 0.0, "exception": f"{type(e).__name__}: {e}", "halted_batch": stop}
            )
            not_run += [label_of(x) for f, x in futs.items() if f.cancelled()]
        stops = [r["halted_batch"] for r in results if r.get("halted_batch")]
        if stops:
            from ..real import batch

            try:
                log.write(json.dumps({"halted": stops[0], "not_run": sorted(not_run)}) + "\n")
            except OSError:  # the halt is still reported below and by the exit status
                pass
            print(batch.halt_message(stops[0], len(not_run)), file=sys.stderr, flush=True)
    ok = sum(r["reward"] for r in results)
    print(f"{harness}/{model or '-'}: {ok:.0f}/{len(results)} solved")
    return results


def _live(spec: dict) -> bool:
    from ..backends.real.backend import real_mode

    return real_mode(spec) == "real"
