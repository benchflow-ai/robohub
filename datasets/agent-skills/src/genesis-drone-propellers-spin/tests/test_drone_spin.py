import ast
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
    # Lines added to the package (changes, committed or not, and new files) must be plain ASCII.
    git = ["git", "-c", "safe.directory=*", "-C", str(WORKDIR)]
    # Compare with the workspace's build commit (recorded root-only at image build), so committed changes count too.
    base = (Path(os.environ.get("SKILLS_STUDY_DIR", "/opt/skills-study")) / "base_commit").read_text().strip()
    diff = subprocess.run([*git, "diff", base, "--unified=0", "--no-color", "--", "genesis"], capture_output=True)
    assert diff.returncode == 0, diff.stderr
    added = [line for line in diff.stdout.splitlines() if line.startswith(b"+") and not line.startswith(b"+++")]
    untracked = subprocess.run(
        [*git, "ls-files", "--others", "--exclude-standard", "--", "genesis"], capture_output=True, text=True
    ).stdout.split()
    for name in untracked:
        if name.endswith(".py"):
            added += (WORKDIR / name).read_bytes().splitlines()
    bad = [line for line in added if any(b > 127 for b in line)]
    assert not bad, f"non-ASCII characters added to genesis/: {bad[:3]}"
