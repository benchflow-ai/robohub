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
import torch

import genesis as gs

WORKDIR = Path(os.environ["SKILLS_STUDY_WORKDIR"])
MODULE_FILE = WORKDIR / "genesis" / "engine" / "solvers" / "rigid" / "constraint" / "solver.py"


@pytest.fixture(scope="module", autouse=True)
def genesis_cpu():
    gs.init(backend=gs.cpu, logging_level="warning", seed=0)
    yield
    gs.destroy()


def _array(x):
    return x.detach().cpu().numpy()


def test_weld_capacity_overflow():
    scene = gs.Scene(
        sim_options=gs.options.SimOptions(
            dt=0.01,
            gravity=(0.0, 0.0, 0.0),
        ),
        rigid_options=gs.options.RigidOptions(
            max_dynamic_constraints=1,
        ),
        show_viewer=False,
    )
    boxes = [
        scene.add_entity(
            gs.morphs.Box(
                size=(0.1, 0.1, 0.1),
                pos=(0.3 * i_b, 0.0, 0.5),
            ),
        )
        for i_b in range(3)
    ]
    scene.build(n_envs=2)
    solver = scene.rigid_solver
    for _ in range(5):
        scene.step()

    links_idx = [box.base_link.idx for box in boxes]
    solver.add_weld_constraint(links_idx[0], links_idx[1])
    with pytest.raises(gs.GenesisException) as excinfo:
        solver.add_weld_constraint(links_idx[1], links_idx[2])
    assert "max_dynamic_constraints" in str(excinfo.value)

    welds = solver.get_weld_constraints()
    assert _array(welds["link_a"]).tolist() == [[links_idx[0]], [links_idx[0]]]
    assert _array(welds["link_b"]).tolist() == [[links_idx[1]], [links_idx[1]]]

    # The simulation keeps running normally with the registered weld.
    for _ in range(20):
        scene.step()
    assert not solver.get_error_envs_mask().any()
    assert torch.isfinite(torch.stack([box.get_pos() for box in boxes])).all()

    # The capacity is available again once the weld is deleted.
    solver.delete_weld_constraint(links_idx[0], links_idx[1])
    solver.add_weld_constraint(links_idx[1], links_idx[2])
    welds = solver.get_weld_constraints()
    assert _array(welds["link_a"]).tolist() == [[links_idx[1]], [links_idx[1]]]
    scene.step()


def _call_name(call):
    if isinstance(call.func, ast.Name):
        return call.func.id
    if isinstance(call.func, ast.Attribute):
        return call.func.attr
    return None


def test_module_reads_quadrants_data_through_helpers():
    tree = ast.parse(MODULE_FILE.read_text())
    for node in ast.walk(tree):
        if isinstance(node, ast.Call):
            assert _call_name(node) not in ("to_numpy", "to_torch"), f"line {node.lineno}: {ast.unparse(node)}"


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
