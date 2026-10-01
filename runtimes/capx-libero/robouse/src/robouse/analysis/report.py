"""`robouse report JOB [JOB ...]`: metrics with 95% confidence intervals, grouped by any trial attribute.

  robouse report runs/final-kimi runs/final-codex --by harness,model --by suite
  robouse report runs/splits-* --by split --metrics success,progress,smoothness_sparc --json out.json
  robouse report runs/nav-* --by model --cost-quality cost_quality.json

Group keys: harness, model, suite, category, difficulty, task, split, seed, mode (sim / real / hil-mock), job, regime
(the `regime` field of a job's job.json, e.g. D10/D100/D300 for policies trained on that much data), released (the task's
`metadata.released`: public vs held-out). Metrics: see metrics.py. Every proportion has a Wilson interval and every mean
a bootstrap interval; pass@k is averaged over tasks for k up to the number of trials per task.
"""

from __future__ import annotations

import json
from collections import defaultdict
from pathlib import Path

from ..core.metrics import aggregate, pass_at_k

KEYS = (
    "harness",
    "model",
    "suite",
    "category",
    "difficulty",
    "task",
    "split",
    "seed",
    "mode",
    "job",
    "regime",
    "released",
)


def load_trials(jobs: list[Path], excluded: list[str] | None = None) -> list[dict]:
    """The trials with a result; the folders of trials that failed on infrastructure go to `excluded`."""
    rows = []
    for job in jobs:
        jmeta = {}
        if (job / "job.json").exists():
            try:
                jmeta = json.loads((job / "job.json").read_text())
            except ValueError:
                pass
        for rj in sorted(job.glob("*/result.json")):
            try:
                r = json.loads(rj.read_text())
                c = json.loads((rj.parent / "config.json").read_text())
            except (ValueError, OSError):
                continue
            if r.get("exception_info"):  # infrastructure failures are not results
                if excluded is not None:
                    excluded.append(str(rj.parent))
                continue
            ep = r.get("episode") or {}
            md = c.get("metadata") or {}
            m = dict(ep.get("metrics") or {})
            if "success" not in m:
                m["success"] = int(float((r.get("verifier_result") or {}).get("rewards", {}).get("reward", 0)) >= 1)
            cost = r.get("cost") or {}
            vlm = rj.parent / "grader" / "vlm.json"
            if vlm.exists():
                try:
                    m["vlm_agree"] = int(bool(json.loads(vlm.read_text()).get("agrees")))
                except ValueError:
                    pass
            rows.append(
                {
                    **m,
                    "cost_usd": cost.get("cost_usd"),
                    "prompt_tokens": cost.get("prompt_tokens"),
                    "completion_tokens": cost.get("completion_tokens"),
                    "wall_time_s": r.get("wall_time_s"),
                    "harness": c.get("harness"),
                    "model": ((r.get("agent_info") or {}).get("model_info") or {}).get("name") or c.get("model"),
                    "suite": md.get("suite"),
                    "category": md.get("category"),
                    "difficulty": md.get("difficulty"),
                    "task": c.get("task"),
                    "split": c.get("split") or "default",
                    "seed": c.get("seed"),
                    "mode": ep.get("embodiment_mode") or r.get("embodiment_mode") or "sim",
                    "job": job.name,
                    "regime": jmeta.get("regime"),
                    "released": md.get("released", True),
                }
            )
    return rows


def group(rows: list[dict], by: list[str]) -> dict[tuple, list[dict]]:
    g: dict[tuple, list[dict]] = defaultdict(list)
    for r in rows:
        g[tuple(r.get(k) for k in by)].append(r)
    return dict(sorted(g.items(), key=lambda kv: tuple(str(x) for x in kv[0])))


def pass_k(rows: list[dict]) -> dict:
    per_task: dict[str, list[int]] = defaultdict(list)
    for r in rows:
        per_task[r["task"]].append(int(r.get("success") or 0))
    n_min = min((len(v) for v in per_task.values()), default=0)
    if n_min < 2:
        return {}
    return {
        f"pass@{k}": round(sum(pass_at_k(len(v), sum(v), k) for v in per_task.values()) / len(per_task), 4)
        for k in range(1, n_min + 1)
    }


def build(jobs: list[Path], by: list[str]) -> dict:
    excluded: list[str] = []
    rows = load_trials(jobs, excluded)
    out = {"groups": [], "by": by, "trials": len(rows), "excluded_infra": excluded}
    for key, rs in group(rows, by).items():
        out["groups"].append(
            {
                "group": dict(zip(by, key, strict=False)),
                **aggregate(rs),
                **pass_k(rs),
                "tasks": len({r["task"] for r in rs}),
            }
        )
    return out


def fmt(v: dict | None, pct: bool = False) -> str:
    if not v:
        return "-"
    f = (lambda x: f"{100 * x:.1f}") if pct else (lambda x: f"{x:.3g}")
    return f"{f(v['value'])} [{f(v['ci95'][0])}, {f(v['ci95'][1])}]"


DEFAULT_METRICS = (
    "success",
    "progress",
    "execution_quality",
    "execution_time_s",
    "execution_speed",
    "smoothness_sparc",
    "avg_jerk",
    "safe_failure",
    "max_contact_force_n",
    "cost_usd",
)
PCT = {"success", "safe_failure", "execution_quality", "arrived", "progress"}


def markdown(rep: dict, metrics: list[str]) -> str:
    hdr = rep["by"] + ["n"] + metrics
    lines = ["| " + " | ".join(hdr) + " |", "|" + "---|" * len(hdr)]
    for g in rep["groups"]:
        cells = [str(g["group"][k]) for k in rep["by"]] + [str(g["n"])]
        cells += [fmt(g.get(m), m in PCT) if not m.startswith("pass@") else str(g.get(m, "-")) for m in metrics]
        lines.append("| " + " | ".join(cells) + " |")
    n_ex = len(rep.get("excluded_infra") or [])
    return (
        "\n".join(lines)
        + "\n\nProportions and progress in %; brackets are 95% intervals (Wilson for proportions, bootstrap for means).\n"
        + (
            f"{n_ex} trial(s) left out: they failed on infrastructure (exception_info in their result.json).\n"
            if n_ex
            else ""
        )
    )


def cost_quality(rep: dict) -> list[dict]:
    pts = []
    for g in rep["groups"]:
        if g.get("cost_usd") and g.get("success"):
            pts.append(
                {
                    **g["group"],
                    "success": g["success"]["value"],
                    "success_ci95": g["success"]["ci95"],
                    "cost_usd_per_run": g["cost_usd"]["value"],
                    "cost_ci95": g["cost_usd"]["ci95"],
                    "n": g["n"],
                    **({"spl": g["spl"]["value"]} if g.get("spl") else {}),
                }
            )
    return pts


def cost_quality_svg(pts: list[dict], label_keys: list[str]) -> str:
    """A small log-cost vs success scatter (the site renders the interactive version from the JSON)."""
    import math

    W, H, P = 560, 360, 50
    if not pts:
        return f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}"><text x="20" y="40">no costed groups</text></svg>'
    cs = [max(1e-4, p["cost_usd_per_run"]) for p in pts]
    lo, hi = math.log10(min(cs)) - 0.2, math.log10(max(cs)) + 0.2
    X = lambda c: P + (math.log10(max(1e-4, c)) - lo) / max(1e-9, hi - lo) * (W - 2 * P)
    Y = lambda s: H - P - s * (H - 2 * P)
    el = [
        f'<line x1="{P}" y1="{H - P}" x2="{W - P}" y2="{H - P}" stroke="#888"/>',
        f'<line x1="{P}" y1="{P}" x2="{P}" y2="{H - P}" stroke="#888"/>',
        f'<text x="{W / 2}" y="{H - 12}" text-anchor="middle" font-size="12">cost per run (USD, log scale)</text>',
        f'<text x="14" y="{H / 2}" transform="rotate(-90 14 {H / 2})" text-anchor="middle" font-size="12">success rate</text>',
    ]
    for p in pts:
        x, y = X(p["cost_usd_per_run"]), Y(p["success"])
        y0, y1 = Y(p["success_ci95"][0]), Y(p["success_ci95"][1])
        name = " / ".join(str(p.get(k)) for k in label_keys)
        el += [
            f'<line x1="{x:.1f}" y1="{y0:.1f}" x2="{x:.1f}" y2="{y1:.1f}" stroke="#4a6fa5"/>',
            f'<circle cx="{x:.1f}" cy="{y:.1f}" r="4" fill="#4a6fa5"/>',
            f'<text x="{x + 6:.1f}" y="{y - 6:.1f}" font-size="11">{name}</text>',
        ]
    return (
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" font-family="sans-serif">'
        + "".join(el)
        + "</svg>"
    )


def main(
    jobs: list[str],
    by: list[str],
    metrics: list[str] | None,
    json_out: str | None,
    cq_out: str | None,
    svg_out: str | None,
) -> int:
    paths = [Path(j) for j in jobs]
    by = [k for spec in (by or ["harness,model"]) for k in spec.split(",") if k]
    bad = [k for k in by if k not in KEYS]
    if bad:
        raise SystemExit(f"unknown group key(s) {bad}; choose from {KEYS}")
    rep = build(paths, by)
    ms = [m for spec in (metrics or [",".join(DEFAULT_METRICS)]) for m in spec.split(",") if m]
    if any(g.get("pass@2") for g in rep["groups"]) and not metrics:
        ms += ["pass@1", "pass@2"]
    print(markdown(rep, ms))
    if json_out:
        Path(json_out).write_text(json.dumps(rep, indent=2))
    if cq_out or svg_out:
        pts = cost_quality(rep)
        if cq_out:
            Path(cq_out).write_text(json.dumps(pts, indent=2))
        if svg_out:
            Path(svg_out).write_text(cost_quality_svg(pts, by))
    return 0
