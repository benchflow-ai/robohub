"""Build-time integrity check of a Robo Use simulator runtime (robohub).

Recomputes the content digest of the build context copied to /opt/robohub-runtime (the same algorithm as
BenchFlow's task digest: sha256 over sorted POSIX relative paths, each contributing path + NUL + sha256(bytes))
and fails the image build unless it equals the digest the task's docker-compose.yaml pins.
"""
import hashlib
import os
import sys
from pathlib import Path


def digest(root: Path) -> str:
    files = []
    for dirpath, _dirs, names in os.walk(root):
        for name in names:
            p = Path(dirpath) / name
            if p.is_symlink() or not p.is_file():
                continue
            files.append((p.relative_to(root).as_posix(), p))
    h = hashlib.sha256()
    for rel, p in sorted(files):
        h.update(rel.encode("utf-8"))
        h.update(b"\x00")
        h.update(hashlib.sha256(p.read_bytes()).digest())
    return f"sha256:{h.hexdigest()}"


if __name__ == "__main__":
    root, expected = Path(sys.argv[1]), sys.argv[2]
    actual = digest(root)
    if actual != expected:
        sys.exit(f"robohub runtime digest mismatch: build context is {actual}, the task pins {expected}")
    print(f"robohub runtime digest verified: {actual}")
