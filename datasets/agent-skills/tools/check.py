#!/usr/bin/env python3
"""Check agent-skills tasks: the reference solution and every valid alternative pass, the no-op and every trivial edit fail, in both variants.

    python3 tools/check.py --workdir /workspace/IsaacLab TASK_ID...      (or --source isaaclab for all of a source)

Runs where the task's environment is installed (a sandbox built like the task image, with the upstream repository at
--workdir committed as one pristine git commit). For each task, variant and mode it resets the repository, replays the
image's own steps (environment/setup/apply.sh, environment/variant.sh, the integrity manifest, the workspace commit kept
in base_commit), applies the mode
(noop: nothing; oracle: oracle/solve.sh; trivial-N: src/<id>/trivial/N.sh; valid-N: src/<id>/valid/N.sh) and runs
verifier/test.sh as BenchFlow does.
Prints one JSON line per check and exits 1 if any check has the wrong outcome. Must run as root (it writes /opt/skills-study).
"""
import argparse
import json
import re
import shutil
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
S = Path("/opt/skills-study")


def sh(cmd, cwd, env=None, timeout=900):
    return subprocess.run(cmd, cwd=cwd, shell=True, env=env, timeout=timeout, capture_output=True, text=True)


GIT_META = ("config", "info/attributes", "info/exclude")


def git_meta(w):
    """The repository-local git settings a mode may change (attributes, excludes, filters and drivers in config)."""
    return {m: ((w / ".git" / m).read_bytes() if (w / ".git" / m).exists() else None) for m in GIT_META}


def reset(w, base, meta):
    # A mode may commit (the agent is allowed to); move the branch back to the pristine commit, keeping the files,
    # then drop every change. Repository-local git settings go back first, so a filter a mode set cannot touch the reset.
    for m, data in meta.items():
        f = w / ".git" / m
        if data is None:
            f.unlink(missing_ok=True)
        else:
            f.write_bytes(data)
    head = sh("git rev-parse HEAD", w).stdout.strip()
    if head != base:
        r = sh(f"git reset -q --soft {base}", w)
        if r.returncode:
            raise SystemExit(f"reset failed: {r.stderr}")
    st = sh("git status --porcelain", w)
    if st.returncode:
        raise SystemExit(f"git status failed in {w} (add it to safe.directory?): {st.stderr[:300]}")
    if st.stdout.strip():
        r = sh("git stash push -u -q && git stash drop -q", w)
        if r.returncode:
            raise SystemExit(f"reset failed: {r.stderr}")
    left = sh("git status --porcelain", w)
    if left.returncode or left.stdout.strip():
        raise SystemExit(f"reset left changes: {left.stdout[:500]} {left.stderr[:300]}")


def run_check(task_dir, src_dir, w, mode, base, meta):
    import os
    reset(w, base, meta)
    if S.exists():
        shutil.rmtree(S)
    S.mkdir(parents=True)
    env_dir = task_dir / "environment"
    shutil.copy(env_dir / "integrity.py", S / "integrity.py")
    for step in (f"sh {env_dir}/setup/apply.sh", f"sh {env_dir}/variant.sh"):
        r = sh(step, w)
        if r.returncode:
            return {"error": f"{step}: {r.stdout[-400:]} {r.stderr[-400:]}"}
    protect = re.search(r"integrity\.py write (.*?) > ", (env_dir / "Dockerfile").read_text()).group(1)
    r = sh(f"python3 {S}/integrity.py write {protect} > {S}/protected.sha256", w)
    if r.returncode:
        return {"error": r.stderr[-400:]}
    # As the image does: commit the starting state and keep that commit for the verifier (reset() moves back to base).
    r = sh("git add -A && git -c user.name=workspace -c user.email=workspace@localhost commit -q --allow-empty -m Workspace"
           " && git rev-parse HEAD", w)
    if r.returncode:
        return {"error": f"workspace commit: {r.stdout[-200:]} {r.stderr[-400:]}"}
    (S / "base_commit").write_text(r.stdout.split()[-1] + "\n")
    env = dict(os.environ, SKILLS_STUDY_WORKDIR=str(w), SKILLS_STUDY_DIR=str(S))
    if mode == "oracle":
        r = sh(f"bash {task_dir}/oracle/solve.sh", w, env)
        if r.returncode:
            return {"error": f"solve.sh: {r.stdout[-600:]} {r.stderr[-600:]}"}
    elif mode.startswith(("trivial-", "valid-")):
        kind, n = mode.split("-", 1)
        r = sh(f"bash {src_dir}/{kind}/{n}.sh", w, env)
        if r.returncode:
            return {"error": f"{kind}: {r.stderr[-400:]}"}
    Path("/logs/verifier").mkdir(parents=True, exist_ok=True)
    Path("/logs/verifier/reward.txt").unlink(missing_ok=True)
    t = time.time()
    r = sh(f"bash {task_dir}/verifier/test.sh", w, env, timeout=1200)
    reward = Path("/logs/verifier/reward.txt").read_text().strip() if Path("/logs/verifier/reward.txt").exists() else None
    tail = "\n".join(r.stdout.strip().splitlines()[-6:])
    return {"reward": reward, "verify_s": round(time.time() - t, 1), "tail": tail}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--workdir", required=True)
    ap.add_argument("--source")
    ap.add_argument("--variants", default="with-skill,no-skill")
    ap.add_argument("--modes", default="oracle,noop,trivial,valid")
    ap.add_argument("ids", nargs="*")
    a = ap.parse_args()
    import yaml
    ids = a.ids or sorted(d.name for d in (ROOT / "src").iterdir()
                          if (d / "task.yaml").exists() and yaml.safe_load((d / "task.yaml").read_text())["source"] == a.source)
    w = Path(a.workdir)
    base = sh("git rev-parse HEAD", w).stdout.strip()
    if not base:
        raise SystemExit(f"no git commit in {w}")
    meta = git_meta(w)
    bad = 0
    for tid in ids:
        src_dir = ROOT / "src" / tid
        modes = []
        for m in a.modes.split(","):
            if m in ("trivial", "valid"):
                modes += [f"{m}-{p.stem}" for p in sorted((src_dir / m).glob("*.sh"))]
            else:
                modes.append(m)
        for variant in a.variants.split(","):
            for mode in modes:
                res = run_check(ROOT / "tasks" / variant / tid, src_dir, w, mode, base, meta)
                want = "1" if mode == "oracle" or mode.startswith("valid-") else "0"
                ok = res.get("reward") == want
                bad += not ok
                print(json.dumps({"task": tid, "variant": variant, "mode": mode, "want": want, "ok": ok, **res}), flush=True)
    reset(w, base, meta)
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
