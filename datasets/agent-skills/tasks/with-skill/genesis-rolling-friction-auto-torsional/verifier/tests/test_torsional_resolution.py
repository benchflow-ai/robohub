import hashlib
import os
import re
import subprocess
import tempfile
from pathlib import Path

os.environ.setdefault("PYGLET_HEADLESS", "1")
os.environ["QD_OFFLINE_CACHE_FILE_PATH"] = os.path.join(tempfile.gettempdir(), "skills-study-genesis-cache", "quadrants")
os.environ["GS_CACHE_FILE_PATH"] = os.path.join(tempfile.gettempdir(), "skills-study-genesis-cache", "genesis")

import pytest

import genesis as gs

WORKDIR = Path(os.environ["SKILLS_STUDY_WORKDIR"])


@pytest.fixture(scope="module", autouse=True)
def genesis_cpu():
    gs.init(backend=gs.cpu, logging_level="warning", seed=0)
    yield
    gs.destroy()


def test_torsional_friction_resolution():
    assert gs.options.RigidOptions().enable_torsional_friction is False
    assert gs.options.RigidOptions(enable_rolling_friction=True).enable_torsional_friction is True
    assert gs.options.RigidOptions(enable_rolling_friction=False).enable_torsional_friction is False
    assert gs.options.RigidOptions(enable_torsional_friction=True).enable_torsional_friction is True
    assert gs.options.RigidOptions(enable_torsional_friction=False).enable_torsional_friction is False
    options = gs.options.RigidOptions(enable_rolling_friction=True, enable_torsional_friction=True)
    assert options.enable_torsional_friction is True
    assert options.enable_rolling_friction is True


def test_reject_rolling_without_torsional():
    with pytest.raises(gs.GenesisException):
        gs.options.RigidOptions(enable_rolling_friction=True, enable_torsional_friction=False)


def test_resolved_option_declared_and_documented():
    assert gs.options.RigidOptions.model_fields["enable_torsional_friction"].default is None
    doc = gs.options.RigidOptions.__doc__
    match = re.search(r"^\s*enable_torsional_friction\s*:.*?(?=^\s*[a-z_]+\s*:)", doc, flags=re.M | re.S)
    assert match is not None, "enable_torsional_friction must stay documented in the RigidOptions docstring"
    assert "None" in match.group(0), "the documentation must say how a None value is resolved"


def test_rolling_friction_alone_slows_rolling_sphere():
    GRAVITY = 9.81
    DT = 0.01
    RADIUS = 0.1
    V0 = 0.5

    scene = gs.Scene(
        sim_options=gs.options.SimOptions(
            dt=DT,
            gravity=(0.0, 0.0, -GRAVITY),
        ),
        rigid_options=gs.options.RigidOptions(
            enable_rolling_friction=True,
        ),
        show_viewer=False,
    )
    scene.add_entity(
        gs.morphs.Plane(),
        material=gs.materials.Rigid(
            friction_torsional=0.0,
            friction_rolling=0.0,
        ),
    )
    spheres = []
    for i_s, friction_rolling in enumerate((0.0, 0.004)):
        spheres.append(
            scene.add_entity(
                gs.morphs.Sphere(
                    radius=RADIUS,
                    pos=(0.0, 0.5 * i_s, RADIUS),
                ),
                material=gs.materials.Rigid(
                    friction_rolling=friction_rolling,
                ),
            )
        )
    scene.build()

    for _ in range(10):
        scene.step()
    for sphere in spheres:
        sphere.set_dofs_velocity([V0, 0.0, 0.0, 0.0, V0 / RADIUS, 0.0])
    for _ in range(5):
        scene.step()
    v_start = [float(sphere.get_dofs_velocity()[0]) for sphere in spheres]
    for _ in range(10):
        scene.step()
    slow_downs = [v_0 - float(sphere.get_dofs_velocity()[0]) for sphere, v_0 in zip(spheres, v_start)]
    # A sphere rolling without slipping on a plane without rolling resistance keeps its speed, while the rolling
    # resistance torque of the other one decelerates it by at most (5 / 7) * friction_rolling * g / r.
    assert abs(slow_downs[0]) < 2e-3, slow_downs
    assert 5e-3 < slow_downs[1] < 0.04, slow_downs


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
