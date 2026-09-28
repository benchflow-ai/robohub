"""Robot model assets (MuJoCo Menagerie meshes) for the menagerie, RoboHarm and drone suites.

A checkout of the repository has them in assets/menagerie. An installed wheel does not ship them (about 43 MB);
`robouse fetch-assets` downloads the same files from the pinned MuJoCo Menagerie commit listed in the provenance
manifests, checks each SHA-256, and stores them in ~/.cache/robouse/menagerie. $ROBOUSE_MENAGERIE_ASSETS overrides
the location.
"""
from __future__ import annotations

import hashlib
import json
import os
import urllib.request
from pathlib import Path

HERE = Path(__file__).resolve().parent
CACHE = Path.home() / ".cache" / "robouse" / "menagerie"


def _checkout_root() -> Path:
    return HERE.parents[1] / "assets" / "menagerie"


def asset_root() -> Path:
    override = os.environ.get("ROBOUSE_MENAGERIE_ASSETS")
    if override:
        return Path(override)
    if _checkout_root().is_dir():
        return _checkout_root()
    if not CACHE.is_dir():
        raise FileNotFoundError(f"robot model assets not found in {CACHE}; run `robouse fetch-assets` once (about 43 MB)")
    return CACHE


def manifests() -> list[Path]:
    shipped = HERE / "assets_provenance"
    if shipped.is_dir():
        return sorted(shipped.glob("*.json"))
    root = _checkout_root()
    return [root / "provenance.json", root / "skydio_x2" / "PROVENANCE.json"]


def _sha256(p: Path) -> str:
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def fetch(dest: Path | None = None, log=print) -> Path:
    dest = Path(dest or os.environ.get("ROBOUSE_MENAGERIE_ASSETS") or CACHE)
    n_new = 0
    for m in manifests():
        for f in json.loads(m.read_text())["files"]:
            out = dest / f["path"]
            if out.exists() and _sha256(out) == f["sha256"]:
                continue
            out.parent.mkdir(parents=True, exist_ok=True)
            tmp = out.with_suffix(out.suffix + ".part")
            with urllib.request.urlopen(f["url"], timeout=120) as r, open(tmp, "wb") as w:
                while chunk := r.read(1 << 20):
                    w.write(chunk)
            if _sha256(tmp) != f["sha256"]:
                tmp.unlink()
                raise RuntimeError(f"checksum mismatch for {f['url']}")
            tmp.replace(out)
            n_new += 1
            log(f"fetched {f['path']}")
    log(f"{n_new} file(s) downloaded; assets in {dest}")
    return dest
