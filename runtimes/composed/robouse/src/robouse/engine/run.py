"""`robouse run` / `run-many` on BenchFlow: select tasks, materialize them, run one BenchFlow job.

This is a thin wrapper over BenchFlow's Python API (`benchflow.Evaluation`); it is exactly what
`bench eval run --tasks-dir <robouse tasks> --agent ... --sandbox ...` does, plus robouse's task selection
(bundled task ids, --filter, --task-list, --limit) and the per-run options that change the task itself
(--seed, --timeout, the noop negative control). Trials land in BenchFlow's job layout:

    <out>/<job>/summary.json  results.jsonl
    <out>/<job>/<task>__<id>/ config.json  result.json  agent/  trajectory/  verifier/  artifacts/
        verifier/reward.txt  reward-details.json  episode/ (result.json, trace.jsonl, frames.jsonl, recording.mp4)
        artifacts/recording.mp4  artifacts/observations/   (the video and the camera images the agent saw)

Agents are BenchFlow's (`bench agent list`): e.g. `oracle`, `claude`, `codex`, `gemini`, `openhands`.
robouse names are accepted as aliases: `claude-code` -> `claude`, and `noop` = BenchFlow's `oracle` running the
noop control (look, never move, never call `robo done`; must score 0).
"""

from __future__ import annotations

import asyncio
import json
import os
import secrets
import shutil
import tempfile
from datetime import datetime
from pathlib import Path

from ..tasks import Task, find_tasks
from .format import ORACLE_TOKEN_ENV, materialize, task_format_root

AGENT_ALIASES = {"claude-code": "claude", "noop": "oracle"}


def select_tasks(root: Path, filt: str = "", ids: list[str] | None = None, limit: int = 0) -> list[Task]:
    tasks = [t for t in find_tasks(root) if filt in t.id and (ids is None or t.id in ids)]
    return tasks[:limit] if limit else tasks


def _agent_env(agent: str) -> dict[str, str]:
    """Credentials for BenchFlow agents from the maintainers' pools (never printed or written to the trial).

    Claude agents: the OAuth token pool. Codex agents: the maintainers' Codex account pool (robouse._maintainer), passed
    as CODEX_AUTH_JSON from that account's robo-use home, so the user's own ~/.codex login is never used."""
    if agent in ("claude", "claude-agent-acp"):
        if os.environ.get("ANTHROPIC_API_KEY") or os.environ.get("CLAUDE_CODE_OAUTH_TOKEN"):
            return {}
        try:
            from ..harnesses.credentials import claude_auth

            return claude_auth()
        except RuntimeError:
            return {}
    if agent.startswith("codex") and not (os.environ.get("OPENAI_API_KEY") or os.environ.get("CODEX_AUTH_JSON")):
        from .. import _maintainer
        from ..harnesses.credentials import clean_codex_home

        if _maintainer.codex_accounts():
            home = Path(clean_codex_home())  # the pool account's persistent home (single refresh owner)
            return {"CODEX_AUTH_JSON": (home / "auth.json").read_text()}
    return {}


def run_job(
    tasks: list[Task],
    agent: str,
    model: str = "",
    out: Path = Path("jobs"),
    *,
    sandbox: str = "docker",
    concurrency: int = 1,
    timeout: float | None = None,
    seed: int | None = None,
    job_name: str | None = None,
    reasoning_effort: str | None = None,
    seeds: list[int] | None = None,
) -> dict:
    """Run tasks x one agent as one BenchFlow job and return a short summary.

    seeds: one trial per task and seed (packages `<task>--seed-<n>`); the job's summary.json then carries
    BenchFlow's seeded-rollout report (pass@k, mean, std per task)."""
    from benchflow import Evaluation, EvaluationConfig

    noop = agent == "noop"
    bf_agent = AGENT_ALIASES.get(agent, agent)
    cache = task_format_root()
    staged = Path(tempfile.mkdtemp(prefix="robouse-job-"))
    try:
        for t in tasks:  # one folder per task (and seed), named by task id, pointing at its materialized package
            for s in seeds or [seed]:
                pkg = materialize(t.path, cache, noop=noop, seed=s, timeout=timeout)
                (staged / pkg.name).symlink_to(pkg, target_is_directory=True)
        cfg = EvaluationConfig(
            agent=bf_agent,
            model=model or None,
            environment=sandbox,
            concurrency=concurrency,
            agent_env=_agent_env(bf_agent),
            reasoning_effort=reasoning_effort,
        )
        job_name = job_name or datetime.now().strftime("%Y-%m-%d__%H-%M-%S") + ("__noop" if noop else "")
        # The oracle token is chosen now, for this job only: the packages read it from the host environment (the
        # simulator through compose, the reference solution through oracle.env). Other agents run with none.
        saved_token = os.environ.pop(ORACLE_TOKEN_ENV, None)
        if bf_agent == "oracle" and not noop:
            os.environ[ORACLE_TOKEN_ENV] = secrets.token_hex(16)
        try:
            res = asyncio.run(Evaluation(tasks_dir=staged, jobs_dir=out, config=cfg, job_name=job_name).run())
        finally:
            os.environ.pop(ORACLE_TOKEN_ENV, None)
            if saved_token is not None:
                os.environ[ORACLE_TOKEN_ENV] = saved_token
    finally:
        shutil.rmtree(staged, ignore_errors=True)
    job = Path(out) / res.job_name
    rows = []
    for rj in sorted(job.glob("*/result.json")):
        r = json.loads(rj.read_text())
        det = rj.parent / "verifier" / "reward-details.json"
        ep = json.loads(det.read_text()) if det.exists() else {}
        rows.append(
            {
                "task": r.get("task_name"),
                "trial": rj.parent.name,
                "reward": (r.get("rewards") or {}).get("reward"),
                "outcome": ep.get("outcome"),
                "steps": ep.get("steps_used"),
                "error": r.get("error"),
            }
        )
    for row in rows:
        print(
            f"{row['task']:<40} reward={row['reward']} outcome={row['outcome']} steps={row['steps']}"
            + (f" ERROR {row['error']}" if row["error"] else ""),
            flush=True,
        )
    if seeds:  # the staged folders are already the seeded packages; add BenchFlow's seeded-rollout report
        from benchflow.embodied.rollouts import format_report, seed_report

        report = seed_report(job)
        summary_path = job / "summary.json"
        summary = json.loads(summary_path.read_text()) if summary_path.exists() else {}
        summary.update({"seeds": list(seeds), "seeded": report})
        summary_path.write_text(json.dumps(summary, indent=2))
        print(format_report(report))
    print(f"{agent}/{model or '-'}: {res.passed}/{res.total} solved, errors={res.errored} -> {job}")
    return {
        "job_dir": str(job),
        "total": res.total,
        "passed": res.passed,
        "errored": res.errored,
        "mean_reward": res.mean_reward,
        "trials": rows,
    }
