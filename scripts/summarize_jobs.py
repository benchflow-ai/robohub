"""Reduce `bench eval run` job folders to dogfood.json (what the README and the hub page report).

    python3 scripts/summarize_jobs.py --label core-oracle --dataset robouse-core@0.1 --agent oracle JOB_DIR [JOB_DIR ...]

Each JOB_DIR is a `bench eval run` output folder (it holds <task>__<id>/result.json). A task counted more than once
(a rerun) keeps its last result. Standard library only.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

HUB = Path(__file__).resolve().parents[1]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--label", required=True)
    ap.add_argument("--dataset", required=True, help="NAME@VERSION")
    ap.add_argument("--agent", required=True)
    ap.add_argument("--model", default=None)
    ap.add_argument("--bench", default="0.7.4")
    ap.add_argument("--note", default="")
    ap.add_argument("jobs", nargs="+")
    a = ap.parse_args()
    tasks: dict[str, dict] = {}
    started = []
    for job in a.jobs:
        for res in sorted(Path(job).glob("*/result.json"), key=lambda p: p.stat().st_mtime):
            r = json.loads(res.read_text())
            name = r.get("task_name") or res.parent.name.rsplit("__", 1)[0]
            reward = (r.get("rewards") or {}).get("reward")
            err = r.get("error") or (r.get("exception_info") or {}).get("exception_type")
            details = res.parent / "verifier" / "reward-details.json"
            d = json.loads(details.read_text()) if details.exists() else {}
            tasks[name] = {"reward": reward, "error": str(err)[:200] if err else None,
                           "outcome": d.get("outcome"), "steps": d.get("steps_used")}
            if r.get("started_at"):
                started.append(r["started_at"])
    scored = [t for t in tasks.values() if t["reward"] is not None and not t["error"]]
    entry = {
        "label": a.label, "dataset": a.dataset, "agent": a.agent, "model": a.model, "bench": a.bench,
        "date": min(started)[:10] if started else None, "note": a.note,
        "n": len(tasks), "scored": len(scored), "passed": sum(1 for t in scored if t["reward"] == 1.0),
        "errors": sum(1 for t in tasks.values() if t["error"] or t["reward"] is None),
        "tasks": dict(sorted(tasks.items())),
    }
    path = HUB / "dogfood.json"
    data = json.loads(path.read_text()) if path.exists() else {"runs": []}
    data["runs"] = [r for r in data["runs"] if r["label"] != a.label] + [entry]
    path.write_text(json.dumps(data, indent=2) + "\n")
    print(f"{a.label}: {entry['passed']}/{entry['scored']} scored tasks passed, {entry['errors']} errors, {entry['n']} tasks")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
