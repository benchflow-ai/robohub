"""Downloads checked against a pinned SHA-256."""

from __future__ import annotations

import hashlib
import os
import urllib.request
from pathlib import Path


class ChecksumError(RuntimeError):
    """A downloaded file does not match its pinned SHA-256."""


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def fetch(url: str, dest: Path, sha256: str, timeout: float = 300) -> bool:
    """Download `url` to `dest` unless `dest` already has the pinned hash. The file is written next to `dest` and
    moved into place only after its hash matches, so readers never see a partial or unverified file.

    Returns True when a download happened. Raises ChecksumError on a mismatch (nothing is left behind)."""
    dest = Path(dest)
    if dest.is_file() and sha256_file(dest) == sha256:
        return False
    dest.parent.mkdir(parents=True, exist_ok=True)
    tmp = dest.with_name(f".{dest.name}.{os.getpid()}.part")
    try:
        with urllib.request.urlopen(url, timeout=timeout) as r, open(tmp, "wb") as w:
            while chunk := r.read(1 << 20):
                w.write(chunk)
        got = sha256_file(tmp)
        if got != sha256:
            raise ChecksumError(f"checksum mismatch for {url}: expected {sha256}, got {got}")
        tmp.replace(dest)
    finally:
        tmp.unlink(missing_ok=True)
    return True
