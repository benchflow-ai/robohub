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
SOLVER_FILE = WORKDIR / "genesis" / "engine" / "solvers" / "rigid" / "rigid_solver.py"
ENTITY_FILE = WORKDIR / "genesis" / "engine" / "entities" / "rigid_entity" / "rigid_entity.py"


@pytest.fixture(scope="module", autouse=True)
def genesis_cpu():
    gs.init(backend=gs.cpu, logging_level="warning", seed=0)
    yield
    gs.destroy()


def _array(x):
    assert isinstance(x, torch.Tensor), type(x)
    return x.detach().cpu().numpy().astype(np.float64)


def test_linear_momentum():
    GRAVITY = 9.81
    DT = 0.01
    N_STEPS = 20

    scene = gs.Scene(
        sim_options=gs.options.SimOptions(
            dt=DT,
            gravity=(0.0, 0.0, -GRAVITY),
        ),
        rigid_options=gs.options.RigidOptions(
            enable_collision=False,
        ),
        show_viewer=False,
    )
    box = scene.add_entity(
        gs.morphs.Box(
            size=(0.1, 0.2, 0.3),
            pos=(0.0, 0.0, 2.0),
        ),
    )
    thrown_box = scene.add_entity(
        gs.morphs.Box(
            size=(0.2, 0.2, 0.2),
            pos=(1.0, 0.0, 2.0),
        ),
    )
    arm = scene.add_entity(
        gs.morphs.URDF(
            file="urdf/simple/two_link_arm.urdf",
            pos=(3.0, 0.0, 0.0),
            fixed=True,
        ),
    )
    scene.build(n_envs=2)
    solver = scene.rigid_solver

    thrown_box.set_dofs_velocity(torch.tensor([[1.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0, -2.0, 3.0, 0.5, 4.0, 0.0]]))
    for _ in range(N_STEPS):
        scene.step()

    # Free fall: p = m * (v0 + g * t), whatever the spin of the body.
    t = N_STEPS * DT
    box_mass = _array(box.get_mass())
    thrown_mass = _array(thrown_box.get_mass())
    box_momentum = _array(box.get_linear_momentum())
    assert box_momentum.shape == (2, 3)
    np.testing.assert_allclose(box_momentum, [[0.0, 0.0, -box_mass * GRAVITY * t]] * 2, rtol=1e-4, atol=1e-5)
    thrown_momentum = _array(thrown_box.get_linear_momentum())
    np.testing.assert_allclose(
        thrown_momentum,
        thrown_mass * np.array([[1.0, 0.0, -GRAVITY * t], [0.0, -2.0, 3.0 - GRAVITY * t]]),
        rtol=1e-4,
        atol=1e-5,
    )

    # Articulated entity: the sum of the masses times the velocities of the centers of mass of its links.
    arm.set_dofs_velocity(torch.tensor([[1.0, -0.5], [0.3, 2.0]]))
    links_idx = [link.idx for link in arm.links]
    links_mass = _array(solver.get_links_mass(links_idx))
    links_vel = _array(solver.get_links_vel(links_idx, ref=gs.link_ref_frame.link_COM))
    arm_momentum_ref = (links_mass[:, None] * links_vel).sum(axis=-2)
    assert np.abs(arm_momentum_ref).max() > 0.1
    arm_momentum = _array(arm.get_linear_momentum())
    np.testing.assert_allclose(arm_momentum, arm_momentum_ref, rtol=1e-5, atol=1e-6)

    # Environment selection.
    arm_momentum_1 = _array(arm.get_linear_momentum(envs_idx=[1]))
    assert arm_momentum_1.shape == (1, 3)
    np.testing.assert_allclose(arm_momentum_1, arm_momentum[1:], rtol=1e-6, atol=1e-7)

    # Solver getter: every link of the scene by default, any subset of links otherwise.
    total = _array(solver.get_linear_momentum())
    np.testing.assert_allclose(total, box_momentum + thrown_momentum + arm_momentum, rtol=1e-5, atol=1e-6)
    subset = _array(solver.get_linear_momentum(links_idx=[box.base_link.idx, *links_idx]))
    np.testing.assert_allclose(subset, box_momentum + arm_momentum, rtol=1e-5, atol=1e-6)
    subset_env = _array(solver.get_linear_momentum(links_idx=[thrown_box.base_link.idx], envs_idx=[0]))
    np.testing.assert_allclose(subset_env, thrown_momentum[:1], rtol=1e-5, atol=1e-6)


def test_linear_momentum_without_parallel_envs():
    scene = gs.Scene(
        rigid_options=gs.options.RigidOptions(
            enable_collision=False,
        ),
        show_viewer=False,
    )
    box = scene.add_entity(
        gs.morphs.Box(
            size=(0.1, 0.1, 0.1),
            pos=(0.0, 0.0, 1.0),
        ),
    )
    scene.build()
    box.set_dofs_velocity([0.5, -1.0, 2.0, 0.0, 0.0, 1.0])
    momentum = _array(box.get_linear_momentum())
    assert momentum.shape == (3,)
    np.testing.assert_allclose(momentum, _array(box.get_mass()) * np.array([0.5, -1.0, 2.0]), rtol=1e-5, atol=1e-6)
    assert _array(scene.rigid_solver.get_linear_momentum()).shape == (3,)


def _method(path, class_name, method_name):
    tree = ast.parse(path.read_text())
    for node in ast.walk(tree):
        if isinstance(node, ast.ClassDef) and node.name == class_name:
            for item in node.body:
                if isinstance(item, ast.FunctionDef) and item.name == method_name:
                    return item
    raise AssertionError(f"{class_name}.{method_name} not found in {path.name}")


def _call_name(call):
    if isinstance(call.func, ast.Name):
        return call.func.id
    if isinstance(call.func, ast.Attribute):
        return call.func.attr
    return None


def test_getters_follow_data_access_conventions():
    solver_getter = _method(SOLVER_FILE, "RigidSolver", "get_linear_momentum")
    entity_getter = _method(ENTITY_FILE, "RigidEntity", "get_linear_momentum")
    for getter in (solver_getter, entity_getter):
        assert ast.get_docstring(getter), f"{getter.name} needs a docstring"
        for node in ast.walk(getter):
            assert not isinstance(node, (ast.Import, ast.ImportFrom)), "imports belong at module top level"
            if not isinstance(node, ast.Call):
                continue
            name = _call_name(node)
            # Quadrants data goes through the helpers of genesis.utils.misc, with the batch dimension in front.
            assert name not in ("to_numpy", "to_torch", "numpy"), f"'{name}()' call in {getter.name}"
            assert name not in ("getattr", "hasattr"), f"'{name}()' call in {getter.name}"
            if name in ("qd_to_torch", "qd_to_numpy"):
                keywords = {kw.arg: kw.value for kw in node.keywords}
                assert isinstance(keywords.get("transpose"), ast.Constant) and keywords["transpose"].value is True, (
                    f"'{name}' must be called with transpose=True in {getter.name}"
                )
    # One vectorized getter: no Python loop over links or environments in the solver.
    assert not any(isinstance(node, (ast.For, ast.While, ast.comprehension)) for node in ast.walk(solver_getter))
    # The entity method delegates to the solver getter.
    assert any(
        isinstance(node, ast.Call) and _call_name(node) == "get_linear_momentum" for node in ast.walk(entity_getter)
    ), "RigidEntity.get_linear_momentum must delegate to RigidSolver.get_linear_momentum"


def test_added_genesis_code_is_ascii():
    # Lines added to the package (tracked changes and new files) must be plain ASCII.
    git = ["git", "-c", "safe.directory=*", "-C", str(WORKDIR)]
    diff = subprocess.run([*git, "diff", "HEAD", "--unified=0", "--no-color", "--", "genesis"], capture_output=True)
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
