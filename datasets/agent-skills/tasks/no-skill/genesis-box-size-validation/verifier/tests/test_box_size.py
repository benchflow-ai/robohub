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
