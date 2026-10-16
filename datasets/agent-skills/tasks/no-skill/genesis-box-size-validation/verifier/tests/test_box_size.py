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


@pytest.fixture(scope="module", autouse=True)
def genesis_cpu():
    gs.init(backend=gs.cpu, logging_level="warning", seed=0)
    yield
    gs.destroy()


@pytest.mark.parametrize(
    "kwargs",
    [
        dict(size=(-0.1, 0.2, 0.2)),
        dict(size=(0.2, 0.2, -1e-3)),
        dict(size=(0.0, 0.2, 0.2)),
        dict(size=[0.2, 0.0, 0.2]),
        dict(size=np.array([0.2, 0.2, 0.0])),
        dict(pos=(1.0, 2.0, 3.0), size=(0.1, -0.1, 0.1)),
        dict(lower=(0.0, 0.0, 0.0), upper=(0.0, 1.0, 1.0)),
        dict(lower=(0.0, 0.0, 0.0), upper=(1.0, 1.0, 0.0)),
        dict(lower=(0.0, 0.0, 0.0), upper=(1.0, -1.0, 1.0)),
        dict(lower=np.array([0.5, 0.5, 0.5]), upper=np.array([1.0, 1.0, 0.25])),
    ],
)
def test_reject_degenerate_box(kwargs):
    with pytest.raises(gs.GenesisException):
        gs.morphs.Box(**kwargs)


def test_valid_box_inputs():
    box = gs.morphs.Box(pos=(1.0, 2.0, 3.0), size=(0.2, 0.4, 0.6))
    np.testing.assert_allclose(box.size, (0.2, 0.4, 0.6))
    np.testing.assert_allclose(box.lower, (0.9, 1.8, 2.7))
    np.testing.assert_allclose(box.upper, (1.1, 2.2, 3.3))

    box = gs.morphs.Box(size=[0.1, 0.1, 0.1])
    np.testing.assert_allclose(box.size, (0.1, 0.1, 0.1))
    np.testing.assert_allclose(box.lower, (-0.05, -0.05, -0.05))

    box = gs.morphs.Box(size=np.array([1e-3, 2.0, 3.0]))
    np.testing.assert_allclose(box.size, (1e-3, 2.0, 3.0))

    box = gs.morphs.Box(lower=np.array([-1.0, 0.0, 0.5]), upper=[1.0, 0.5, 0.75])
    np.testing.assert_allclose(box.pos, (0.0, 0.25, 0.625))
    np.testing.assert_allclose(box.size, (2.0, 0.5, 0.25))

    box = gs.morphs.Box(lower=(0, 0, 0), upper=(1, 2, 3))
    np.testing.assert_allclose(box.size, (1.0, 2.0, 3.0))

    # Still rejected as before: non-finite values, missing corner.
    for kwargs in (dict(size=(0.1, float("nan"), 0.1)), dict(lower=(0.0, 0.0, 0.0))):
        with pytest.raises(gs.GenesisException):
            gs.morphs.Box(**kwargs)


def _box_class(tree):
    for node in tree.body:
        if isinstance(node, ast.ClassDef) and node.name == "Box":
            return node
    raise AssertionError("class Box not found in genesis/options/morphs.py")


def test_box_fields_use_shared_typing_aliases():
    source = (WORKDIR / "genesis" / "options" / "morphs.py").read_text()
    box_cls = _box_class(ast.parse(source))
    annotations = {
        node.target.id: node.annotation
        for node in box_cls.body
        if isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name)
    }
    import genesis.typing as gs_typing

    for field in ("lower", "upper", "size"):
        assert field in annotations, f"Box.{field} must stay a declared field"
        names = {n.id for n in ast.walk(annotations[field]) if isinstance(n, ast.Name)}
        # A bare typed collection or an inline Annotated[...] instead of the shared aliases of genesis.typing.
        assert not names & {"tuple", "list", "Tuple", "List", "Annotated", "Sequence"}, (field, names)
        assert any(name in vars(gs_typing) for name in names), (field, names)


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
