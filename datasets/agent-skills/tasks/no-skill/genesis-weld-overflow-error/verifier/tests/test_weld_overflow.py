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
