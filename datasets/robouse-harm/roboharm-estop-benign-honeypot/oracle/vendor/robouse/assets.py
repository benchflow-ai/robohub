"""Robot model assets (MuJoCo Menagerie meshes).

The menagerie, roboharm and drone suites use the Panda, ALOHA and Skydio X2 models: a checkout of the repository has
them in assets/menagerie; an installed wheel does not ship them (about 43 MB). The embodiment suites (quadruped,
humanoid, mobile-manip, dexhand, crazyflie) use 14 more robots (assets/menagerie/embodiments.json) that are never
stored in git. `robouse fetch-assets` downloads the files listed in the provenance manifests from the pinned MuJoCo
Menagerie commit, checks each SHA-256, and stores them in <cache>/menagerie: by default the base robots, with `--all`
every robot, with `--robot NAME` only the named ones. $ROBOUSE_MENAGERIE_ASSETS overrides the location.
"""

from __future__ import annotations

import json
from collections.abc import Callable
from pathlib import Path

from . import config
from .lib.download import fetch as fetch_file

HERE = Path(__file__).resolve().parent
EMBODIMENTS_MANIFEST = "embodiments.json"


def cache() -> Path:
    return config.cache_dir() / "menagerie"


def _checkout_root() -> Path:
    return HERE.parents[1] / "assets" / "menagerie"


def asset_root() -> Path:
    override = config.env("ROBOUSE_MENAGERIE_ASSETS")
    if override:
        return Path(override)
    if _checkout_root().is_dir():
        return _checkout_root()
    if not cache().is_dir():
        raise FileNotFoundError(
            f"robot model assets not found in {cache()}; run `robouse fetch-assets` once (about 43 MB)"
        )
    return cache()


def robot_dir(robot: str) -> Path:
    """Folder of one Menagerie robot (e.g. `unitree_go2`): $ROBOUSE_MENAGERIE_ASSETS, the checkout, or the fetch cache."""
    override = config.env("ROBOUSE_MENAGERIE_ASSETS")
    roots = [Path(override)] if override else [_checkout_root(), cache()]
    for r in roots:
        if (r / robot).is_dir():
            return r / robot
    raise FileNotFoundError(
        f"robot model {robot!r} not found in {', '.join(map(str, roots))}; "
        f"run `robouse fetch-assets --robot {robot}` once"
    )


def manifests() -> list[Path]:
    """The Menagerie manifests (the packaged copies in an installed wheel, else the checkout's)."""
    shipped = HERE / "assets_provenance"
    if shipped.is_dir():
        return [shipped / n for n in ("menagerie.json", "skydio_x2.json", EMBODIMENTS_MANIFEST)]
    root = _checkout_root()
    return [root / "provenance.json", root / "skydio_x2" / "PROVENANCE.json", root / EMBODIMENTS_MANIFEST]


def robot_names() -> list[str]:
    """The robot folders `fetch --robot` accepts."""
    return sorted({f["path"].split("/", 1)[0] for m in manifests() for f in json.loads(m.read_text())["files"]})


def fetch(
    dest: Path | None = None,
    log: Callable[[str], object] = print,
    robots: list[str] | None = None,
    everything: bool = False,
) -> Path:
    """Download the manifest files that are missing or differ from their pinned hash: the base robots (Panda, ALOHA,
    Skydio X2), every robot with `everything`, or only `robots`."""
    dest = Path(dest or config.env("ROBOUSE_MENAGERIE_ASSETS") or cache())
    if robots:
        unknown = sorted(set(robots) - set(robot_names()))
        if unknown:
            raise ValueError(f"no robot {', '.join(unknown)} in the manifests; robots: {', '.join(robot_names())}")
    n_new = 0
    for m in manifests():
        if not (robots or everything) and m.name == EMBODIMENTS_MANIFEST:
            continue
        for f in json.loads(m.read_text())["files"]:
            if robots and f["path"].split("/", 1)[0] not in robots:
                continue
            if fetch_file(f["url"], dest / f["path"], f["sha256"], timeout=120):
                n_new += 1
                log(f"fetched {f['path']}")
    log(f"{n_new} file(s) downloaded; assets in {dest}")
    return dest
