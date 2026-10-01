"""Sets of trials: task selection, seeds, epochs, retries and resume."""

from __future__ import annotations

import concurrent.futures as cf
import json
import shutil
import time
from pathlib import Path

from ..tasks import Task, find_tasks
from .trial import now, run_trial


def parse_seeds(spec: str | None) -> list[int | None]:
    """ "0-4" -> [0..4]; "0,3,7" -> [0, 3, 7]; None -> [None] (the task's own seed)."""
    if not spec:
        return [None]
    out: list[int | None] = []
    for part in str(spec).split(","):
        a, _, b = part.partition("-")
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
) -> list[dict]:
    tasks = select_tasks(root, filt, ids, include, exclude, limit, sample, sample_seed, mode)
    runs = [(t, sd, ep) for t in tasks for sd in parse_seeds(seeds) for ep in range(max(1, epochs))]
    if (
        resume
    ):  # skip (task, seed, epoch) already finished in this job (an unfinished trial dir is left as evidence and rerun)
        done = set()
        for rj in job_dir.glob("*/result.json"):
            try:
                r = json.loads(rj.read_text())
                c = json.loads((rj.parent / "config.json").read_text())
            except (OSError, ValueError):  # a trial folder that is still being written, or broken
                continue
            if not r.get("exception_info") and (r.get("episode") or {}).get("outcome"):
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
                "started_at": now(),
            },
            indent=2,
        )
    )
    results = []

    def one(item) -> dict:
        t, sd, ep = item
        for attempt in range(retries + 1):  # infrastructure failures only (a model's failure is a result, not retried)
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
            )
            if not r["exception"]:
                break
            if attempt < retries:
                bad = job_dir / r["trial"]
                dest = job_dir / "_infra_failed"
                dest.mkdir(exist_ok=True)
                if bad.exists():
                    shutil.move(str(bad), str(dest / bad.name))
                time.sleep(min(60, 5 * 2**attempt))
        r["task"] = t.id + (f"@seed{sd}" if sd is not None else "") + (f"#e{ep}" if epochs > 1 else "")
        return r

    with open(job_dir / "run_log.jsonl", "a") as log, cf.ThreadPoolExecutor(max_workers=max(1, concurrency)) as ex:
        futs = [ex.submit(one, x) for x in runs]
        for fut in cf.as_completed(futs):
            r = fut.result()
            results.append(r)
            log.write(json.dumps(r) + "\n")
            log.flush()
            print(
                f"{r['task']:<40} reward={r['reward']:.0f} outcome={r['outcome']} steps={r['steps']} {r['wall_s']}s"
                + (f" EXC {r['exception']}" if r["exception"] else ""),
                flush=True,
            )
    ok = sum(r["reward"] for r in results)
    print(f"{harness}/{model or '-'}: {ok:.0f}/{len(results)} solved")
    return results
