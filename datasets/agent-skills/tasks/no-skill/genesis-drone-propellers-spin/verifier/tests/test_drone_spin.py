import ast
import hashlib
import os
import subprocess
import tempfile
from pathlib import Path

os.environ.setdefault("PYGLET_HEADLESS", "1")
os.environ["QD_OFFLINE_CACHE_FILE_PATH"] = os.path.join(tempfile.gettempdir(), "skills-study-genesis-cache", "quadrants")
os.environ["GS_CACHE_FILE_PATH"] = os.path.join(tempfile.gettempdir(), "skills-study-genesis-cache", "genesis")

import numpy as np
import pytest

import genesis as gs

WORKDIR = Path(os.environ["SKILLS_STUDY_WORKDIR"])
DRONE_FILE = "urdf/drones/cf2x.urdf"


@pytest.fixture(scope="module", autouse=True)
def genesis_cpu():
    gs.init(backend=gs.cpu, logging_level="warning", seed=0)
    yield
    gs.destroy()


@pytest.mark.parametrize(
    "spin",
    [
        [1, -1, 1, -1],
        (-1, 1, -1, 1),
        np.array([-1, 1, -1, 1]),
        np.array([1, 1, -1, -1], dtype=np.int32),
    ],
)
def test_accept_array_like_spin(spin):
    drone = gs.morphs.Drone(file=DRONE_FILE, propellers_spin=spin)
    assert tuple(int(e) for e in drone.propellers_spin) == tuple(int(e) for e in spin)


def test_default_spin_unchanged():
    drone = gs.morphs.Drone(file=DRONE_FILE)
    assert tuple(drone.propellers_spin) == (-1, 1, -1, 1)
    assert len(drone.propellers_link_name) == 4


@pytest.mark.parametrize(
    "spin",
    [
        (1, 0, 1, -1),
        (2, -1, 1, -1),
        np.array([1, -1, 1, -2]),
        (1, -1, 1),
        [1, -1, 1, -1, 1],
    ],
)
def test_reject_invalid_spin(spin):
    with pytest.raises(gs.GenesisException):
        gs.morphs.Drone(file=DRONE_FILE, propellers_spin=spin)


def test_spin_count_follows_propeller_links():
    links = ("prop0_link", "prop1_link", "prop2_link")
    drone = gs.morphs.Drone(file=DRONE_FILE, propellers_link_name=links, propellers_spin=np.array([1, -1, 1]))
    assert tuple(drone.propellers_spin) == (1, -1, 1)
    with pytest.raises(gs.GenesisException):
        gs.morphs.Drone(file=DRONE_FILE, propellers_link_name=links)


def test_spin_field_uses_shared_typing_alias():
    tree = ast.parse((WORKDIR / "genesis" / "options" / "morphs.py").read_text())
    drone_cls = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "Drone")
    annotation = next(
        node.annotation
        for node in drone_cls.body
        if isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name) and node.target.id == "propellers_spin"
    )
    import genesis.typing as gs_typing

    names = {n.id for n in ast.walk(annotation) if isinstance(n, ast.Name)}
    assert not names & {"tuple", "list", "Tuple", "List", "Annotated", "Sequence"}, names
    assert any(name in vars(gs_typing) for name in names), names


def test_added_genesis_code_is_ascii():
    # Lines added to the package (changes, committed or not, and new files) must be plain ASCII. Files are read from disk
    # and compared byte for byte with the workspace's build commit (recorded root-only at image build), with replace refs
    # and user git config ignored, so commits, git attributes, filters and ignore files cannot hide a change.
    base = (Path(os.environ.get("SKILLS_STUDY_DIR", "/opt/skills-study")) / "base_commit").read_text().strip()
    env = dict(os.environ, GIT_NO_REPLACE_OBJECTS="1", GIT_CONFIG_NOSYSTEM="1", GIT_CONFIG_GLOBAL=os.devnull)
    git = ["git", "-c", "safe.directory=*", "-C", str(WORKDIR)]
    tree = subprocess.run([*git, "ls-tree", "-r", "-z", "--full-tree", base], capture_output=True, env=env)
    assert tree.returncode == 0, tree.stderr
    blobs = {}
    for rec in tree.stdout.split(b"\0"):
        if rec:
            meta, path = rec.split(b"\t", 1)
            blobs[path] = meta.split()[2].decode()
    root = os.fsencode(WORKDIR)
    added = []
    for d, dirs, names in os.walk(os.path.join(root, b"genesis")):
        dirs[:] = [x for x in dirs if x != b"__pycache__"]
        for n in names:
            p = os.path.join(d, n)
            rel = os.path.relpath(p, root)
            if os.path.islink(p) or not os.path.isfile(p):
                continue
            data = open(p, "rb").read()
            if rel in blobs:
                if hashlib.sha1(b"blob %d\0" % len(data) + data).hexdigest() == blobs[rel]:
                    continue
                old = subprocess.run([*git, "cat-file", "blob", blobs[rel]], capture_output=True, env=env)
                assert old.returncode == 0, old.stderr
                before = set(old.stdout.splitlines())
                added += [line for line in data.splitlines() if line not in before]
            elif n.endswith(b".py"):
                added += data.splitlines()
    bad = [line for line in added if any(b > 127 for b in line)]
    assert not bad, f"non-ASCII characters added to genesis/: {bad[:3]}"
