#!/usr/bin/env python3
"""Integrity manifest of protected upstream files (standard library only).

    integrity.py write PATH...      print a manifest of every file under the PATHs (run in the repository root)
    integrity.py check MANIFEST     exit 1 if a protected file was changed, deleted or added, or if the workspace holds a
                                    sitecustomize.py, usercustomize.py or .pth file that was not there at build time
                                    (such files can run code inside the verifier's Python)

A PATH written as +PATH is add-only (an upstream test folder): new files may be added under it and lines may be added to
its files, but no file may be removed and every original line must stay, byte for byte and in order.
"""
import hashlib
import os
import sys


def files(paths):
    for p in paths:
        if os.path.isfile(p):
            yield p
        for d, dirs, names in os.walk(p):
            dirs[:] = sorted(x for x in dirs if x not in ("__pycache__", ".git") and not x.endswith(".egg-info"))
            for n in sorted(names):
                if not n.endswith((".pyc", ".pyo")):
                    yield os.path.join(d, n)


def hooks():
    for d, dirs, names in os.walk("."):
        dirs[:] = sorted(x for x in dirs if x != ".git")
        for n in sorted(names):
            if n in ("sitecustomize.py", "usercustomize.py") or n.endswith(".pth"):
                yield os.path.join(d, n)


def digest(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def line_hashes(path):
    with open(path, "rb") as f:
        return [hashlib.sha256(x).hexdigest()[:12] for x in f.read().splitlines()]


def kept(old, path):
    """True if every original line is still in the file, in order (lines may have been added anywhere)."""
    it = iter(line_hashes(path))
    return all(h in it for h in old)


def main():
    if sys.argv[1] == "write":
        paths = sys.argv[2:]
        print("#paths " + " ".join(paths))
        for f in hooks():
            print(f"#hook {f}")
        for f in files(p for p in paths if not p.startswith("+")):
            print(f"{digest(f)}  {f}")
        for f in files(p[1:] for p in paths if p.startswith("+")):
            print(f"#lines {','.join(line_hashes(f)) or '-'}  {f}")
        return 0
    lines = open(sys.argv[2]).read().splitlines()
    paths = lines[0].split()[1:]
    known_hooks = {line[6:] for line in lines if line.startswith("#hook ")}
    want = dict(reversed(line.split("  ", 1)) for line in lines[1:] if not line.startswith("#"))
    grow = {k: v[7:].split(",") if v[7:] != "-" else [] for k, v in (reversed(line.split("  ", 1)) for line in lines[1:] if line.startswith("#lines "))}
    have = {f: digest(f) for f in files(p for p in paths if not p.startswith("+"))}
    bad = sorted(f for f in set(want) | set(have) if want.get(f) != have.get(f))
    have_grow = set(files(p[1:] for p in paths if p.startswith("+")))
    bad += sorted(f for f, old in grow.items() if f not in have_grow or not kept(old, f))
    bad += [f for f in hooks() if f not in known_hooks]
    for f in bad[:50]:
        print(f"changed: {f}")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
