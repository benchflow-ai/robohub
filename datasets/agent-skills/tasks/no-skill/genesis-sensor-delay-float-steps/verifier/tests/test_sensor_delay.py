import ast
import logging
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


class _Records(logging.Handler):
    def __init__(self):
        super().__init__(level=logging.WARNING)
        self.messages = []

    def emit(self, record):
        self.messages.append(record.getMessage())


@pytest.fixture(scope="module", autouse=True)
def genesis_cpu():
    gs.init(backend=gs.cpu, logging_level="warning", seed=0)
    yield
    gs.destroy()


def test_delay_in_whole_steps():
    DT = 0.05
    DELAYS = (0.15, 0.3, 0.1, 0.125)
    N_STEPS = 15

    scene = gs.Scene(
        sim_options=gs.options.SimOptions(
            dt=DT,
            gravity=(0.0, 0.0, 0.0),
        ),
        show_viewer=False,
    )
    box = scene.add_entity(
        gs.morphs.Box(
            size=(0.1, 0.2, 0.3),
            pos=(0.0, 0.0, 1.0),
        ),
    )
    imu = scene.add_sensor(
        gs.sensors.IMU(
            entity_idx=box.idx,
        ),
    )
    records = _Records()
    logger = logging.getLogger("genesis")
    logger.addHandler(records)
    warned = []
    delayed_imus = []
    try:
        for delay in DELAYS:
            records.messages.clear()
            delayed_imus.append(
                scene.add_sensor(
                    gs.sensors.IMU(
                        entity_idx=box.idx,
                        delay=delay,
                    ),
                )
            )
            warned.append(any("multiple" in message for message in records.messages))
    finally:
        logger.removeHandler(records)
    assert warned == [False, False, False, True], dict(zip(DELAYS, warned))

    scene.build()
    readings = []
    delayed_readings = [[] for _ in DELAYS]
    for i_step in range(N_STEPS):
        box.set_dofs_velocity([0.0, 0.0, 0.0, 0.0, 0.0, 0.1 * (i_step + 1)])
        scene.step()
        readings.append(imu.read().ang_vel[..., 2].clone())
        for delayed_imu, history in zip(delayed_imus, delayed_readings):
            history.append(delayed_imu.read().ang_vel[..., 2].clone())
    readings = torch.stack(readings).numpy()
    assert np.all(np.diff(readings) > 0.05), readings
    for delay, n_steps_delay in zip(DELAYS[:3], (3, 6, 2)):
        history = torch.stack(delayed_readings[DELAYS.index(delay)]).numpy()
        # The reading of a sensor delayed by n steps is the reading of the undelayed one n steps earlier.
        np.testing.assert_allclose(history[n_steps_delay:], readings[:-n_steps_delay], atol=1e-5, err_msg=str(delay))


def test_delay_check_compares_floats_with_tolerance():
    tree = ast.parse((WORKDIR / "genesis" / "options" / "sensors" / "options.py").read_text())
    sensor_options = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "SensorOptions")
    validate_scene = next(
        node for node in sensor_options.body if isinstance(node, ast.FunctionDef) and node.name == "validate_scene"
    )
    for node in ast.walk(validate_scene):
        if isinstance(node, ast.Compare):
            assert not any(isinstance(op, (ast.Eq, ast.NotEq)) for op in node.ops), ast.unparse(node)


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
