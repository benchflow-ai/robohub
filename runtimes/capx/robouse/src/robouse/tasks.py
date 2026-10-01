"""Task folders in BenchFlow's native task format.

  <task>/task.md            YAML frontmatter (schema_version, task, metadata, agent, verifier, robouse) + the
                            instruction as the markdown body, which is the agent's prompt
  <task>/oracle/solve.sh    reference solution; must score 1.0
  <task>/verifier/test.sh   turns the episode server's verdict into reward.txt

The `robouse:` block names the backend and its parameters (env, seed, budgets, scoring and observation modes,
and for tabletop tasks the `scenario`). `write_task` writes this layout; adapters use it.

The older layout (task.toml + instruction.md, solution/, tests/) is not read; BenchFlow converts such a folder:
`bench tasks migrate DIR --remove-legacy`.
"""

from __future__ import annotations

import re
import sys
from dataclasses import dataclass, field
from pathlib import Path

import yaml

SCHEMA_VERSION = "1.3"


@dataclass
class Task:
    path: Path
    meta: dict  # the frontmatter
    body: str = field(default="")

    @property
    def id(self) -> str:
        return self.spec["id"]

    @property
    def spec(self) -> dict:
        # an exported BenchFlow package (`robouse export`, the hub datasets) keeps the block under metadata
        s = dict(self.meta.get("robouse") or self.meta.get("metadata", {}).get("robouse", {}))
        s.setdefault("id", self.path.name)
        return s

    @property
    def instruction(self) -> str:
        return self.body

    @property
    def agent_timeout_s(self) -> float:
        return float(self.meta.get("agent", {}).get("timeout_sec", 900))

    @property
    def metadata(self) -> dict:
        return self.meta.get("metadata", {})

    @property
    def oracle_script(self) -> Path:
        return self.path / "oracle" / "solve.sh"

    @property
    def verifier_script(self) -> Path:
        return self.path / "verifier" / "test.sh"


class TaskFormatError(ValueError):
    """A folder that is not a task.md task folder, such as one in the older task.toml + instruction.md layout."""


def older_layout(d: Path) -> bool:
    """The folder holds the older task.toml + instruction.md layout and no task.md."""
    d = Path(d)
    return not (d / "task.md").exists() and ((d / "task.toml").exists() or (d / "instruction.md").exists())


def older_layout_message(d: Path) -> str:
    return (
        f"{d} is not a task.md folder: it has task.toml and instruction.md, the older layout, which robouse no longer "
        f"reads. Convert it with BenchFlow: bench tasks migrate {d} --remove-legacy"
    )


def _split_frontmatter(text: str) -> tuple[dict, str]:
    if not text.startswith("---"):
        raise ValueError("task.md has no YAML frontmatter")
    _, fm, body = text.split("---", 2)
    return yaml.safe_load(fm) or {}, body.lstrip("\n")


def load_task(path: str | Path) -> Task:
    p = Path(path).resolve()
    if (p / "task.md").exists():
        meta, body = _split_frontmatter((p / "task.md").read_text())
        return Task(p, meta, body)
    if older_layout(p):
        raise TaskFormatError(older_layout_message(p))
    raise FileNotFoundError(f"no task.md in {p}")


def bundled_tasks_root() -> Path:
    """The task folders that ship with robouse: robouse/bundled_tasks in an installed wheel, or the repo's tasks/
    directory in an editable install from a checkout."""
    here = Path(__file__).resolve().parent
    for p in (here / "bundled_tasks", here.parents[1] / "tasks"):
        if p.is_dir():
            return p
    return here / "bundled_tasks"


def resolve_task_path(arg: str | Path) -> Path:
    """A task argument is a folder path, or a task id / suite-relative path (e.g. `arc-gravity` or
    `arc-style/arc-gravity`) looked up in the bundled tasks."""
    p = Path(arg)
    if p.exists():
        return p
    root = bundled_tasks_root()
    if (root / p).is_dir():
        return root / p
    hits = sorted({m.parent for m in root.rglob("task.md") if m.parent.name == p.name and "_legacy" not in m.parts})
    if len(hits) == 1:
        return hits[0]
    if len(hits) > 1:
        raise SystemExit(
            f"task id {arg!r} is ambiguous in {root}: " + ", ".join(str(h.relative_to(root)) for h in hits)
        )
    raise SystemExit(f"no task folder {arg!r}, and no bundled task with that id (list them with `robouse tasks list`)")


def find_tasks(root: str | Path) -> list[Task]:
    root = Path(root)
    dirs = {p.parent for p in root.rglob("task.md")}
    # the older task.toml + instruction.md layout is not read: skip those folders, and say so
    older = sorted(d for d in {p.parent for p in root.rglob("task.toml")} - dirs if "_legacy" not in d.parts)
    if older:
        print(
            f"robouse: skipped {len(older)} folder(s) under {root} in the older task.toml + instruction.md layout "
            f"(e.g. {older[0]}); convert them with BenchFlow: bench tasks migrate DIR --remove-legacy",
            file=sys.stderr,
        )
    return [load_task(d) for d in sorted(dirs) if "_legacy" not in d.parts]


_BLOCK_START = re.compile(r"\s*(#|\||[-*+] |\d+\. |>|```)")


def unwrap_markdown(text: str) -> str:
    """One line per paragraph: join hard-wrapped prose lines (and wrapped list-item continuations) while leaving
    code blocks, tables, headings, list starts and blank lines as they are."""
    out: list[str] = []
    in_code = False
    for line in text.split("\n"):
        if line.strip().startswith("```"):
            in_code = not in_code
            out.append(line)
            continue
        if in_code:
            out.append(line)
            continue
        prev = out[-1] if out else ""
        p = prev.strip()
        joinable_prev = bool(p) and not p.startswith(("#", "|", "```"))
        if joinable_prev and line.strip() and not _BLOCK_START.match(line):
            out[-1] = prev.rstrip() + " " + line.strip()
        else:
            out.append(line)
    return "\n".join(out)


def write_task(d: Path, meta: dict, instruction: str, oracle_sh: str, verifier_sh: str) -> None:
    """Write one task folder in BenchFlow's native layout (the instruction is unwrapped to one line per paragraph)."""
    d = Path(d)
    (d / "oracle").mkdir(parents=True, exist_ok=True)
    (d / "verifier").mkdir(parents=True, exist_ok=True)
    fm = {
        "schema_version": SCHEMA_VERSION,
        **{k: meta[k] for k in ("task", "metadata", "agent", "verifier", "robouse") if k in meta},
    }
    text = (
        "---\n"
        + yaml.safe_dump(fm, sort_keys=False, allow_unicode=True, width=10**6)
        + "---\n\n"
        + unwrap_markdown(instruction.lstrip("\n"))
    )
    (d / "task.md").write_text(text)
    for rel, body in (("oracle/solve.sh", oracle_sh), ("verifier/test.sh", verifier_sh)):
        f = d / rel
        f.write_text(body)
        f.chmod(0o755)
