"""Robot model assets (MuJoCo Menagerie meshes).

The menagerie, RoboHarm and drone suites use the Panda, ALOHA and Skydio X2 models: a checkout of the repository has
them in assets/menagerie; an installed wheel does not ship them (about 43 MB). The embodiment suites (quadruped,
humanoid, mobile-manip, dexhand, crazyflie) use 14 more robots (about 350 MB, assets/menagerie/embodiments.json)
that are never stored in git. `robouse fetch-assets` downloads the files listed in the provenance manifests from the
pinned MuJoCo Menagerie commit, checks each SHA-256, and stores them in ~/.cache/robouse/menagerie
(`--robot NAME` limits it to some robots). $ROBOUSE_MENAGERIE_ASSETS overrides the location.
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


def robot_dir(robot: str) -> Path:
    """Folder of one Menagerie robot (e.g. `unitree_go2`): $ROBOUSE_MENAGERIE_ASSETS, the checkout, or the fetch cache."""
    override = os.environ.get("ROBOUSE_MENAGERIE_ASSETS")
    roots = [Path(override)] if override else [_checkout_root(), CACHE]
    for r in roots:
        if (r / robot).is_dir():
            return r / robot
    raise FileNotFoundError(f"robot model {robot!r} not found in {', '.join(map(str, roots))}; "
                            f"run `robouse fetch-assets --robot {robot}` once")


def manifests() -> list[Path]:
    shipped = HERE / "assets_provenance"
    if shipped.is_dir():
        return sorted(shipped.glob("*.json"))
    root = _checkout_root()
    return [root / "provenance.json", root / "skydio_x2" / "PROVENANCE.json", root / "embodiments.json"]


def _sha256(p: Path) -> str:
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def fetch(dest: Path | None = None, log=print, robots: list[str] | None = None) -> Path:
    """Download the manifest files (all robots, or only `robots`) that are missing or differ from their pinned hash."""
    dest = Path(dest or os.environ.get("ROBOUSE_MENAGERIE_ASSETS") or CACHE)
    n_new = 0
    for m in manifests():
        for f in json.loads(m.read_text())["files"]:
            if robots and f["path"].split("/", 1)[0] not in robots:
                continue
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
