"""Pin every dataset version in hub.yaml to a commit of this repository and write registry.json and hub.json.

    ~/.local/share/uv/tools/benchflow/bin/python scripts/build_registry.py [--commit SHA] [--repin NAME@VERSION ...]

registry.json follows the BenchFlow dataset registry schema (the one `bench eval run -d NAME@VERSION --registry URL`
reads, same as benchflow-ai/skillsbench's registry.json): a list of
{name, version, description, git_tag, bench_version, tasks: [{name, git_url, git_commit_id, path, digest}]}.

- Task digests are computed by BenchFlow's own `benchflow._utils.task_authoring.task_digest` (run this script with
  the Python that has BenchFlow installed) over the task folders as they are at the pinned commit (`git archive`),
  not over the working tree.
- Published versions are immutable: an entry already in registry.json is kept as it is, unless it is named with
  --repin. New entries are pinned to --commit (default: HEAD, which must contain datasets/<name>/).
- Names are <org>/<name> (see hub.yaml); the tasks stay in datasets/<dir>/. Each alias in hub.yaml gets its own
  entry with the same tasks as the dataset it names, so an old `-d OLD@VERSION` keeps resolving; aliases are listed
  after the datasets and left out of hub.json. `former_names` are older names whose entries (earlier versions) stay
  in registry.json as they are; new versions get no entry under them, and hub.json leaves them out too.
- Each new entry gets the git tag <name>-v<version> (for example farama-foundation/metaworld-v0.1); --tag creates the
  tags locally (push them with `git push origin --tags`).
- hub.json is the machine-readable index the hub page is built from: per dataset, its tasks with suite, backend,
  runtime, embodiment and simulator (from hub.yaml and export.json), and the pinned commit.
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
import tarfile
import tempfile
from pathlib import Path

import yaml

HUB = Path(__file__).resolve().parents[1]
GIT_URL = "https://github.com/benchflow-ai/robohub.git"
RAW_REGISTRY = "https://raw.githubusercontent.com/benchflow-ai/robohub/main/registry.json"
MIRROR_REGISTRY = "https://robouse.ai/hub/registry.json"
BENCH_VERSION = ">=0.7.4,<0.8"

try:
    from benchflow._utils.task_authoring import task_digest
except ImportError:
    sys.exit("run with the Python that has BenchFlow installed, e.g. ~/.local/share/uv/tools/benchflow/bin/python")


def git(*args: str) -> str:
    return subprocess.run(["git", "-C", str(HUB), *args], capture_output=True, text=True, check=True).stdout.strip()


def digests_at(commit: str, dataset: str) -> dict[str, str]:
    """task name -> digest of datasets/<dataset>/<task> as committed at `commit`."""
    with tempfile.TemporaryDirectory() as tmp:
        archive = Path(tmp) / "d.tar"
        subprocess.run(["git", "-C", str(HUB), "archive", "--format=tar", "-o", str(archive), commit,
                        f"datasets/{dataset}"], check=True)
        with tarfile.open(archive) as t:
            t.extractall(tmp, filter="data")
        root = Path(tmp) / "datasets" / dataset
        return {d.name: task_digest(d) for d in sorted(root.iterdir()) if d.is_dir()}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--commit", default="HEAD", help="commit to pin new dataset versions to (default: HEAD)")
    ap.add_argument("--repin", nargs="*", default=[], metavar="NAME@VERSION", help="re-pin these existing entries")
    ap.add_argument("--tag", action="store_true", help="create the <name>-v<version> git tags for new entries")
    a = ap.parse_args(argv)

    commit = git("rev-parse", a.commit)
    hub = yaml.safe_load((HUB / "hub.yaml").read_text())
    export = json.loads((HUB / "export.json").read_text())
    reg_path = HUB / "registry.json"
    old = {(e["name"], str(e["version"])): e for e in (json.loads(reg_path.read_text()) if reg_path.exists() else [])}

    registry, aliases, pinned_now = [], [], []
    for ds in hub["datasets"]:
        key = (ds["name"], str(ds["version"]))
        spec = f"{ds['name']}@{ds['version']}"
        folder = ds.get("dir", ds["name"])
        entry = old.pop(key, None)
        if entry is None or spec in a.repin:
            digests = digests_at(commit, folder)
            if not digests:
                sys.exit(f"{spec}: datasets/{folder} is empty at {commit[:12]}")
            entry = {
                "name": ds["name"],
                "version": str(ds["version"]),
                "description": ds["description"],
                "git_tag": f"{ds['name']}-v{ds['version']}",
                "bench_version": BENCH_VERSION,
                "tasks": [{"name": n, "git_url": GIT_URL, "git_commit_id": commit, "path": f"datasets/{folder}/{n}",
                           "digest": d} for n, d in digests.items()],
            }
            pinned_now.append(entry)
            print(f"pinned {spec}: {len(digests)} tasks at {commit[:12]}")
        registry.append(entry)
        # an alias is an older name of the same version: its own registry entry, with exactly the same pinned tasks
        for alias in ds.get("aliases", []):
            akey = (alias, str(ds["version"]))
            aentry = old.pop(akey, None)
            if aentry is None:
                aentry = {**entry, "name": alias, "git_tag": f"{alias}-v{ds['version']}"}
                pinned_now.append(aentry)
                print(f"alias {alias}@{ds['version']} -> {spec}")
            if aentry["tasks"] != entry["tasks"]:
                sys.exit(f"alias {alias}@{ds['version']} pins other tasks than {spec}; re-pin one of them")
            aliases.append(aentry)
    registry += aliases
    registry += list(old.values())  # versions no longer in hub.yaml stay published
    reg_path.write_text(json.dumps(registry, indent=2) + "\n")

    if a.tag:
        for e in pinned_now:
            subprocess.run(["git", "-C", str(HUB), "tag", "-f", e["git_tag"], e["tasks"][0]["git_commit_id"]], check=True)

    # hub.json: the index behind robouse.ai/hub
    suites, runtimes = hub["suites"], hub["runtimes"]
    by_name = {d["name"]: d for d in hub["datasets"]}
    out = {"registry": RAW_REGISTRY, "registry_mirror": MIRROR_REGISTRY, "git_url": GIT_URL,
           "bench_version": BENCH_VERSION, "robouse": export["robouse"], "datasets": []}
    alias_names = {x for d in hub["datasets"] for x in (*d.get("aliases", []), *d.get("former_names", []))}
    for e in registry:
        if e["name"] in alias_names:  # aliases are not listed on the hub page
            continue
        if e["name"] in by_name and str(by_name[e["name"]]["version"]) != e["version"]:
            continue  # an older version: still in registry.json (and the page's version list), not a hub entry of its own
        ds = by_name.get(e["name"], {})
        folder = ds.get("dir", e["name"])
        exp = export["datasets"].get(folder, {"tasks": {}})
        tasks = []
        for t in e["tasks"]:
            info = exp["tasks"].get(t["name"], {})
            tasks.append({"id": t["name"], "digest": t["digest"], **info})
        used_suites = sorted({t.get("suite") for t in tasks if t.get("suite")},
                             key=list(suites).index)
        emb, sims = [], []
        for s in used_suites:
            for x in suites[s]["embodiment"].split("; "):
                if x not in emb:
                    emb.append(x)
            for x in suites[s]["simulator"].split("; "):
                if x not in sims:
                    sims.append(x)
        image = ds.get("image") or (suites[used_suites[0]]["image"] if used_suites else "")
        org, _, short = e["name"].rpartition("/")
        out["datasets"].append({
            "name": e["name"], "org": org, "short_name": short, "dir": folder, "aliases": [*ds.get("aliases", []), *ds.get("former_names", [])],
            "upstream": ds.get("upstream"), "run": ds.get("run"), "robouse": exp.get("robouse", export["robouse"]),
            "version": e["version"], "description": e["description"], "git_tag": e["git_tag"],
            "commit": e["tasks"][0]["git_commit_id"], "n_tasks": len(e["tasks"]), "suites": used_suites,
            "embodiments": emb, "simulators": sims, "image": image, "noop": bool(ds.get("noop_of")),
            "runtimes": sorted({t.get("runtime") for t in tasks if t.get("runtime")}),
            "tasks": tasks,
        })
    out["runtimes"] = {k: {"simulators": v["simulators"], "digest": export["runtimes"].get(k)} for k, v in runtimes.items()}
    (HUB / "hub.json").write_text(json.dumps(out, indent=2) + "\n")
    print(f"wrote {reg_path.name} ({len(registry)} dataset versions) and hub.json")
    return 0


if __name__ == "__main__":
    sys.exit(main())
