#!/usr/bin/env python3
"""Write TASKS.md, the task catalogue, from src/*/task.yaml and the check records in checks/*.jsonl.

    python3 tools/catalog.py
"""
import json
from collections import defaultdict
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parent.parent


def checks():
    out = defaultdict(lambda: defaultdict(list))
    for f in sorted((ROOT / "checks").glob("*.jsonl")):
        for line in f.read_text().splitlines():
            if line.strip():
                d = json.loads(line)
                out[d["task"]][d["variant"]].append(d["ok"])
    return out


def main():
    sources = yaml.safe_load((ROOT / "sources.yaml").read_text())
    ck = checks()
    tasks = [yaml.safe_load(p.read_text()) for p in sorted((ROOT / "src").glob("*/task.yaml"))]
    lines = ["# Tasks", "",
             f"{len(tasks)} tasks, each built in two variants (with-skill, no-skill). Checks: every variant's reference solution passes, its no-op fails and both trivial edits fail (`tools/check.py`, records in `checks/`).", ""]
    for key, src in sources.items():
        mine = [t for t in tasks if t["source"] == key]
        if not mine:
            continue
        lines += [f"## {src['name']} ({len(mine)} tasks, {src['repo']} at `{src['commit'][:12]}`, {src['licence']})", "",
                  "| Task | Job | Guide it exercises | Difficulty | Checks (with / no skill) |", "|---|---|---|---|---|"]
        for t in mine:
            c = ck.get(t["id"], {})
            cell = " / ".join(f"{sum(c[v])}/{len(c[v])}" if c.get(v) else "not run" for v in ("with-skill", "no-skill"))
            lines.append(f"| `{t['id']}` | {t['summary']} | `{t['skill']}`: {t['scenario']} | {t.get('difficulty', '')} | {cell} |")
        lines.append("")
    (ROOT / "TASKS.md").write_text("\n".join(lines))
    print(f"TASKS.md: {len(tasks)} tasks")


if __name__ == "__main__":
    main()
