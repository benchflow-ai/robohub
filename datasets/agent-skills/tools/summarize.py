#!/usr/bin/env python3
"""Summarize the agent-skills study from bench job folders.

    python3 tools/summarize.py jobs-out [--out results.json]

Reads jobs-out/<agent>--<variant>/trial-<n>/<job>/<rollout>/result.json (and the agent trajectory next to it) and prints:
- per cell (agent x variant): trials, passes, pass rate, infrastructure errors, tokens, cost, agent time;
- per agent and source: with-skill vs no-skill pass rate and the paired difference per task;
- guide use: in with-skill trials, how often a guide path appears in the trajectory (a listing or a read; open the trajectory to tell); leakage: in no-skill trials, any mention of a
  guide path (Isaac Lab docs link to skills/user/..., so a mention can be a dangling link) or a fetch of the upstream guides from GitHub; those trials are listed for a person to judge, not dropped.
Standard library only.
"""
import argparse
import json
import re
import sys
from collections import defaultdict
from pathlib import Path

GUIDE = {
    "isaaclab": re.compile(r"skills/(user|developer)/|\.agents/skills|\.claude/skills|SKILL\.md|evaluations\.md|AGENTS\.md|CLAUDE\.md"),
    "lerobot": re.compile(r"AGENT_GUIDE\.md|AGENTS\.md|CLAUDE\.md"),
    "genesis": re.compile(r"CLAUDE\.md|AGENTS\.md"),
}
WEB = re.compile(r"(github\.com|raw\.githubusercontent\.com)/[^\s\"']*(skills/|AGENT_GUIDE|AGENTS\.md|CLAUDE\.md)", re.I)


def source_of(task):
    return task.split("-", 1)[0].replace("agent_skills/", "")


def trajectory_text(rollout):
    parts = []
    for name in ("agent/acp_trajectory.jsonl", "trajectory/acp_trajectory.jsonl"):
        p = rollout / name
        if p.exists():
            parts.append(p.read_text(errors="replace"))
            break
    return "\n".join(parts)


def load(root):
    rows = []
    for res in sorted(Path(root).glob("*/trial-*/*/*/result.json")):
        rollout = res.parent
        cell = res.parents[3].name
        if "--" not in cell:
            continue
        agent, variant = cell.rsplit("--", 1)
        trial = res.parents[2].name
        r = json.loads(res.read_text())
        task = r.get("task_name", rollout.name.split("__")[0]).split("/")[-1]
        ar = r.get("agent_result") or {}
        err = r.get("exception_info") or r.get("error")
        reward = (r.get("rewards") or {}).get("reward")
        text = trajectory_text(rollout)
        src = source_of(task)
        rows.append({
            "cell": cell, "agent": agent, "variant": variant, "trial": trial, "task": task, "source": src,
            "reward": reward, "error": bool(err) and reward is None,
            "input_tokens": ar.get("n_input_tokens") or 0, "output_tokens": ar.get("n_output_tokens") or 0,
            "cache_read_tokens": ar.get("n_cache_read_tokens") or 0, "cost_usd": ar.get("cost_usd") or 0.0,
            "agent_sec": ((r.get("timing") or {}).get("agent_execution") or 0),
            "guide_hits": len(GUIDE.get(src, GUIDE["genesis"]).findall(text)),
            "web_hits": len(WEB.findall(text)),
            "rollout": str(rollout),
        })
    return rows


def rate(rs):
    scored = [r for r in rs if r["reward"] is not None]
    return (sum(r["reward"] >= 1.0 for r in scored), len(scored))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("root")
    ap.add_argument("--out")
    a = ap.parse_args()
    rows = load(a.root)
    if not rows:
        sys.exit(f"no result.json under {a.root}")
    by = defaultdict(list)
    for r in rows:
        by[r["cell"]].append(r)
    print("cell                          trials  pass  rate   errors  in_tok    out_tok   cost_usd  agent_min")
    for cell, rs in sorted(by.items()):
        p, n = rate(rs)
        print(f"{cell:30s} {len(rs):6d} {p:5d} {p / max(n, 1):6.1%} {sum(r['error'] for r in rs):7d} "
              f"{sum(r['input_tokens'] for r in rs):9.0f} {sum(r['output_tokens'] for r in rs):9.0f} "
              f"{sum(r['cost_usd'] for r in rs):9.2f} {sum(r['agent_sec'] for r in rs) / 60:9.1f}")
    print("\nwith-skill vs no-skill, per agent and source (pass rate; mean paired per-task difference)")
    summary = {}
    for agent in sorted({r["agent"] for r in rows}):
        for src in sorted({r["source"] for r in rows}) + ["all"]:
            sel = [r for r in rows if r["agent"] == agent and (src == "all" or r["source"] == src)]
            w = [r for r in sel if r["variant"] == "with-skill"]
            o = [r for r in sel if r["variant"] == "no-skill"]
            (pw, nw), (po, no) = rate(w), rate(o)
            diffs = []
            for task in sorted({r["task"] for r in sel}):
                tw, to = rate([r for r in w if r["task"] == task]), rate([r for r in o if r["task"] == task])
                if tw[1] and to[1]:
                    diffs.append(tw[0] / tw[1] - to[0] / to[1])
            d = sum(diffs) / len(diffs) if diffs else float("nan")
            summary[f"{agent}/{src}"] = {"with": [pw, nw], "without": [po, no], "paired_mean_diff": d, "tasks": len(diffs)}
            print(f"{agent:12s} {src:9s} with {pw}/{nw} ({pw / max(nw, 1):.0%})  without {po}/{no} ({po / max(no, 1):.0%})  "
                  f"diff {d:+.2f} over {len(diffs)} tasks")
    used = [r for r in rows if r["variant"] == "with-skill"]
    print(f"\nguide path mentioned in {sum(r['guide_hits'] > 0 for r in used)}/{len(used)} with-skill trials")
    leaks = [r for r in rows if r["variant"] == "no-skill" and (r["guide_hits"] or r["web_hits"])]
    print(f"possible leakage in {len(leaks)} no-skill trials (guide path mentioned or upstream skills fetched):")
    for r in leaks:
        print(f"  {r['cell']} {r['trial']} {r['task']} guide_hits={r['guide_hits']} web_hits={r['web_hits']} {r['rollout']}")
    if a.out:
        Path(a.out).write_text(json.dumps({"rows": rows, "summary": summary}, indent=1))


if __name__ == "__main__":
    main()
