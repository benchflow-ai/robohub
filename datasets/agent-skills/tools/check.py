#!/usr/bin/env python3
"""Check agent-skills tasks: the reference solution passes, the no-op fails, and every trivial edit fails, in both variants.

    python3 tools/check.py --workdir /workspace/IsaacLab TASK_ID...      (or --source isaaclab for all of a source)

Runs where the task's environment is installed (a sandbox built like the task image, with the upstream repository at
--workdir committed as one pristine git commit). For each task, variant and mode it resets the repository, replays the
image's own steps (environment/setup/apply.sh, environment/variant.sh, the integrity manifest), applies the mode
(noop: nothing; oracle: oracle/solve.sh; trivial-N: src/<id>/trivial/N.sh) and runs verifier/test.sh as BenchFlow does.
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


def reset(w):
    st = sh("git status --porcelain", w).stdout
    if st.strip():
        r = sh("git stash push -u -q && git stash drop -q", w)
        if r.returncode:
            raise SystemExit(f"reset failed: {r.stderr}")
    left = sh("git status --porcelain", w).stdout
    if left.strip():
        raise SystemExit(f"reset left changes: {left[:500]}")


def run_check(task_dir, src_dir, w, mode):
    import os
    reset(w)
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
    env = dict(os.environ, SKILLS_STUDY_WORKDIR=str(w), SKILLS_STUDY_DIR=str(S))
    if mode == "oracle":
        r = sh(f"bash {task_dir}/oracle/solve.sh", w, env)
        if r.returncode:
            return {"error": f"solve.sh: {r.stdout[-600:]} {r.stderr[-600:]}"}
    elif mode.startswith("trivial-"):
        r = sh(f"bash {src_dir}/trivial/{mode[8:]}.sh", w, env)
        if r.returncode:
            return {"error": f"trivial: {r.stderr[-400:]}"}
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
    ap.add_argument("--modes", default="oracle,noop,trivial")
    ap.add_argument("ids", nargs="*")
    a = ap.parse_args()
    import yaml
    ids = a.ids or sorted(d.name for d in (ROOT / "src").iterdir()
                          if (d / "task.yaml").exists() and yaml.safe_load((d / "task.yaml").read_text())["source"] == a.source)
    w = Path(a.workdir)
    bad = 0
    for tid in ids:
        src_dir = ROOT / "src" / tid
        trivials = sorted(p.stem for p in (src_dir / "trivial").glob("*.sh")) if (src_dir / "trivial").is_dir() else []
        modes = []
        for m in a.modes.split(","):
            modes += [f"trivial-{n}" for n in trivials] if m == "trivial" else [m]
        for variant in a.variants.split(","):
            for mode in modes:
                res = run_check(ROOT / "tasks" / variant / tid, src_dir, w, mode)
                want = "1" if mode == "oracle" else "0"
                ok = res.get("reward") == want
                bad += not ok
                print(json.dumps({"task": tid, "variant": variant, "mode": mode, "want": want, "ok": ok, **res}), flush=True)
    reset(w)
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
