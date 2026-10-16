import math
import os
import subprocess
import tempfile
from pathlib import Path

os.environ.setdefault("PYGLET_HEADLESS", "1")
# A fresh compile cache: this task edits kernels, so nothing compiled before the verifier runs may be reused.
_CACHE = tempfile.mkdtemp(prefix="skills-study-genesis-cache-")
os.environ["QD_OFFLINE_CACHE_FILE_PATH"] = os.path.join(_CACHE, "quadrants")
os.environ["GS_CACHE_FILE_PATH"] = os.path.join(_CACHE, "genesis")

import pytest
import torch

import genesis as gs
from genesis.engine.simulator import RATE_CHECK_ERRNO

WORKDIR = Path(os.environ["SKILLS_STUDY_WORKDIR"])
ORIGINAL_ERROR_CODES = {
    "SUCCESS",
    "OVERFLOW_COLLISION_PAIRS",
    "OVERFLOW_CANDIDATE_CONTACTS",
    "OVERFLOW_CONTACTS",
    "INVALID_CONTACT_NAN",
    "INVALID_FORCE_NAN",
    "INVALID_ACC_NAN",
}


@pytest.fixture(scope="module", autouse=True)
def genesis_cpu():
    gs.init(backend=gs.cpu, logging_level="warning", seed=0)
    yield
    gs.destroy()


def test_new_error_code():
    from genesis.utils.array_class import ErrorCode

    names = set(ErrorCode.__members__)
    assert ORIGINAL_ERROR_CODES <= names
    new_codes = [ErrorCode[name] for name in names - ORIGINAL_ERROR_CODES]
    assert new_codes, "a new error code is expected in ErrorCode"
    used_bits = 0
    for name in ORIGINAL_ERROR_CODES:
        used_bits |= int(ErrorCode[name])
    for code in new_codes:
        # Each error code is a single bit of its own, so that several errors of one environment can be told apart.
        assert int(code) > 0 and int(code) & (int(code) - 1) == 0, code
        assert not int(code) & used_bits, code


def test_nan_position_target_halts_simulation():
    scene = gs.Scene(
        sim_options=gs.options.SimOptions(
            dt=0.01,
        ),
        show_viewer=False,
    )
    arm = scene.add_entity(
        gs.morphs.URDF(
            file="urdf/simple/two_link_arm.urdf",
            fixed=True,
        ),
    )
    other_arm = scene.add_entity(
        gs.morphs.URDF(
            file="urdf/simple/two_link_arm.urdf",
            pos=(2.0, 0.0, 0.0),
            fixed=True,
        ),
    )
    scene.build(n_envs=3)
    solver = scene.rigid_solver

    for _ in range(3):
        scene.step()
    other_arm.control_dofs_position([0.2, 0.1])
    # Only the second environment receives a NaN target. The control call itself returns normally.
    arm.control_dofs_position(torch.tensor([[0.1, -0.2], [math.nan, -0.2], [0.3, 0.0]]))
    with pytest.raises(gs.GenesisException) as excinfo:
        for _ in range(2 * RATE_CHECK_ERRNO + 1):
            scene.step()
    message = str(excinfo.value).lower()
    assert "nan" in message and "target" in message, message
    assert "timestep" not in message and "time step" not in message, message
    assert solver.get_error_envs_mask().tolist() == [False, True, False]

    scene.reset()
    assert solver.get_error_envs_mask().tolist() == [False, False, False]
    arm.control_dofs_position(torch.tensor([[0.1, -0.2], [0.2, -0.1], [0.3, 0.0]]))
    for _ in range(2 * RATE_CHECK_ERRNO + 1):
        scene.step()
    assert not solver.get_error_envs_mask().any()
    assert torch.isfinite(arm.get_dofs_position()).all()

    # A NaN target addressed to one environment through 'envs_idx' flags that environment alone.
    arm.control_dofs_position([math.nan, 0.0], envs_idx=[2])
    with pytest.raises(gs.GenesisException):
        for _ in range(2 * RATE_CHECK_ERRNO + 1):
            scene.step()
    assert solver.get_error_envs_mask().tolist() == [False, False, True]


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
