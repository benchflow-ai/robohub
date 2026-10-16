"""lerobot-cpu-user-remote-train: run the user's script with a fake lerobot-train, parse its arguments
with LeRobot's own CLI parser and validate them (nothing is submitted)."""
import re

import pytest

from lr_helpers import W, capture_cli, parse_train

SCRIPT = W / "scripts_local" / "train_sort_socks.sh"


@pytest.fixture(scope="module")
def argv():
    assert SCRIPT.is_file(), f"missing {SCRIPT}"
    r, calls = capture_cli(SCRIPT)
    assert r.returncode == 0, r.stdout[-2000:] + r.stderr[-2000:]
    assert len(calls) == 1, f"expected exactly one lerobot-train call, got {calls}"
    return calls[0]["argv"]


@pytest.fixture(scope="module")
def cfg(argv):
    return parse_train(argv)


def _arg(argv, name):
    val = None
    for i, a in enumerate(argv):
        if a.startswith(f"--{name}="):
            val = a.split("=", 1)[1]
        elif a == f"--{name}" and i + 1 < len(argv):
            val = argv[i + 1]
    return val


def _seconds(duration):
    m = re.fullmatch(r"\s*(\d+(?:\.\d+)?)\s*([smhd]?)\s*", str(duration))
    assert m, f"not an HF Jobs duration: {duration!r}"
    return float(m.group(1)) * {"": 1, "s": 1, "m": 60, "h": 3600, "d": 86400}[m.group(2)]


def test_runs_on_hf_jobs_single_a10g(cfg):
    assert cfg.job.is_remote, "the run must be submitted to Hugging Face Jobs, not run locally"
    assert cfg.job.target == "a10g-small"


def test_three_hour_cap(cfg):
    assert cfg.job.timeout is not None
    assert _seconds(cfg.job.timeout) == 3 * 3600


def test_detached(cfg):
    assert cfg.job.detach is True


def test_policy_and_hub_repo(cfg):
    assert cfg.policy.type == "act"
    assert cfg.policy.repo_id == "mira/act_sort_socks"
    assert cfg.policy.push_to_hub is True


def test_not_forced_onto_cpu(argv):
    device = _arg(argv, "policy.device")
    assert device is None or device.startswith("cuda"), f"--policy.device={device} would not use the pod's GPU"


def test_dataset_and_length(cfg):
    assert cfg.dataset.repo_id == "mira/so101_sort_socks"
    assert cfg.dataset.root is None
    assert cfg.batch_size == 8
    assert cfg.steps == 20000
