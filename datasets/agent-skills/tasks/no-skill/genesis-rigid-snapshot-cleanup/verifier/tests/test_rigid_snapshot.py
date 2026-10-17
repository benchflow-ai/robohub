import ast
import os
import subprocess
import sys
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
MODULE_FILE = WORKDIR / "genesis" / "utils" / "rigid_snapshot.py"


@pytest.fixture(scope="module", autouse=True)
def genesis_cpu():
    gs.init(backend=gs.cpu, logging_level="warning", seed=0)
    yield
    gs.destroy()


def _array(x):
    return x.detach().cpu().numpy()


def test_snapshot_rigid_state():
    from genesis.utils.rigid_snapshot import snapshot_rigid_state

    scene = gs.Scene(
        sim_options=gs.options.SimOptions(
            dt=0.01,
        ),
        show_viewer=False,
    )
    scene.add_entity(
        gs.morphs.Plane(),
    )
    box = scene.add_entity(
        gs.morphs.Box(
            size=(0.1, 0.2, 0.3),
            pos=(0.0, 0.0, 0.6),
        ),
    )
    arm = scene.add_entity(
        gs.morphs.URDF(
            file="urdf/simple/two_link_arm.urdf",
            pos=(1.0, 0.0, 0.0),
            fixed=True,
        ),
    )
    scene.build(n_envs=3)
    solver = scene.rigid_solver

    box.set_dofs_velocity(torch.tensor([[0.5, 0.0, 0.0, 0.0, 0.0, 1.0], [0.0, -0.5, 0.0, 2.0, 0.0, 0.0], [0.0] * 6]))
    arm.set_dofs_velocity(torch.tensor([[1.0, -0.5], [0.3, 2.0], [-1.0, 0.0]]))
    for _ in range(5):
        scene.step()

    snapshot = snapshot_rigid_state(solver)
    assert not isinstance(snapshot, dict)
    fields = {
        "qpos": _array(solver.get_qpos()),
        "dofs_vel": _array(solver.get_dofs_velocity()),
        "links_pos": _array(solver.get_links_pos()),
        "links_quat": _array(solver.get_links_quat()),
    }
    assert fields["links_pos"].shape == (3, solver.n_links, 3)
    for name, expected in fields.items():
        value = getattr(snapshot, name)
        assert isinstance(value, np.ndarray), (name, type(value))
        assert value.shape == expected.shape, (name, value.shape, expected.shape)
        np.testing.assert_allclose(value, expected, rtol=1e-6, atol=1e-7, err_msg=name)

    # Environment selection, in the given order.
    selected = snapshot_rigid_state(solver, envs_idx=[2, 0])
    for name, expected in fields.items():
        value = getattr(selected, name)
        assert value.shape == (2, *expected.shape[1:]), (name, value.shape)
        np.testing.assert_allclose(value, expected[[2, 0]], rtol=1e-6, atol=1e-7, err_msg=name)

    # A snapshot keeps the values of its instant while the simulation goes on.
    frozen = {name: getattr(snapshot, name).copy() for name in fields}
    for _ in range(5):
        scene.step()
    assert not np.allclose(_array(solver.get_links_pos()), frozen["links_pos"])
    for name, value in frozen.items():
        np.testing.assert_array_equal(getattr(snapshot, name), value, err_msg=name)


def _call_name(call):
    if isinstance(call.func, ast.Name):
        return call.func.id
    if isinstance(call.func, ast.Attribute):
        return call.func.attr
    return None


def test_module_follows_conventions():
    source = MODULE_FILE.read_bytes()
    assert all(b < 128 for b in source), "the module must be plain ASCII"
    tree = ast.parse(source)
    for function in [node for node in ast.walk(tree) if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))]:
        for node in ast.walk(function):
            assert not isinstance(node, (ast.Import, ast.ImportFrom)), f"local import in {function.name}"
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        name = _call_name(node)
        # Quadrants data goes through the getters or the helpers of genesis.utils.misc, torch data through
        # 'tensor_to_array'.
        assert name not in ("to_numpy", "to_torch", "numpy"), f"'{name}()' call"
        assert name not in ("getattr", "hasattr"), f"'{name}()' call"
        if name in ("qd_to_torch", "qd_to_numpy"):
            keywords = {kw.arg: kw.value for kw in node.keywords}
            assert isinstance(keywords.get("transpose"), ast.Constant) and keywords["transpose"].value is True, name
    # The result is a named structure, never a plain dict.
    for node in ast.walk(tree):
        assert not isinstance(node, (ast.Dict, ast.DictComp)), "plain dict in the module"
        if isinstance(node, ast.Call):
            assert _call_name(node) != "dict", "plain dict in the module"
