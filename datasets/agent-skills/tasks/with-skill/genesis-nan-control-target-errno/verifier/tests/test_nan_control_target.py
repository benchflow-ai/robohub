import hashlib
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
