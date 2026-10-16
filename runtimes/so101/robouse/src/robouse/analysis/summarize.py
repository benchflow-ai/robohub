"""`robouse summarize TRIAL`: a deterministic digest of one trial as markdown learnings for the next attempt
(`run --prior-learnings FILE`): the verdict and how the episode ended, the judge's detail, safety events and vetoes,
errors the robot returned, and the last requests before the end. No model is called; the text is meant to be read by
the next agent and by people."""

from __future__ import annotations

import json
from pathlib import Path


def summarize(trial: Path) -> str:
    r = json.loads((trial / "result.json").read_text())
    ep = r.get("episode") or {}
    lines = [
        f"# Learnings from {trial.name}",
        "",
        f"- Outcome: {ep.get('outcome')} after {ep.get('steps_used')} of {ep.get('max_steps')} steps; "
        f"success: {bool(ep.get('success'))}; progress: {(ep.get('metrics') or {}).get('progress')}",
    ]
    if ep.get("judge_detail"):
        lines.append(f"- Judge: {json.dumps(ep['judge_detail'])[:400]}")
    if ep.get("agent_text"):
        lines.append(f"- The agent's final claim: {ep['agent_text'][:300]}")
    for e in ep.get("safety_events") or []:
        lines.append(f"- Safety event: {e.get('kind')}: {e.get('detail')}")
    errors, tail = [], []
    tr = trial / "episode" / "trace.jsonl"
    if tr.exists():
        rows = [json.loads(l) for l in tr.read_text().splitlines() if l.strip()]
        for row in rows:
            resp = row.get("resp") or {}
            if not resp.get("ok") and resp.get("error"):
                errors.append(f"{(row.get('req') or {}).get('op')}: {resp['error'][:200]}")
        tail = [json.dumps(row.get("req"))[:160] for row in rows[-5:]]
    if errors:
        lines += ["", "## Errors the robot returned", ""] + [f"- {e}" for e in dict.fromkeys(errors)][:10]
    if tail:
        lines += ["", "## Last requests", ""] + [f"- `{t}`" for t in tail]
    lines += ["", "## For the next attempt", ""]
    if errors:
        lines.append("- Do not repeat requests that returned the errors above; read `robo info` for the limits.")
    lines += [
        "- Success is judged only after `robo done` with the robot holding still: check the goal holds before calling it."
    ]
    if not ep.get("success") and ep.get("outcome") in ("budget_exhausted", "agent_timeout"):
        lines.append(
            "- The budget ran out: use skills or `--repeat` for long motions and plan fewer observation round trips."
        )
    return "\n".join(lines) + "\n"
