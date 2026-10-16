"""lerobot-laptop-first-train-script: run the user's script with a fake lerobot-train, parse its arguments
with LeRobot's own CLI parser and validate them (no training)."""
import math

import pytest

from lr_helpers import W, capture_cli, parse_train, total_frames

SCRIPT = W / "scripts_local" / "train_first_policy.sh"
SPE = math.ceil(total_frames(W / "recordings" / "so101_stack_blocks") / 8)


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


def test_dataset(cfg):
    assert cfg.dataset.repo_id == "mira/so101_stack_blocks"
    assert cfg.dataset.root is not None
    assert (W / cfg.dataset.root).resolve() == (W / "recordings" / "so101_stack_blocks").resolve()


def test_policy_for_8gb_first_timer(cfg):
    assert cfg.policy.type == "act"
    assert cfg.policy.pretrained_path is None


def test_trains_on_the_gpu(argv):
    # The parsed config falls back to CPU on this machine, so check what the script asked for.
    device = _arg(argv, "policy.device")
    assert device is None or device.startswith("cuda"), device


def test_batch_and_epoch_based_length(cfg):
    assert cfg.batch_size == 8
    epochs, rest = divmod(cfg.steps, SPE)
    assert rest == 0, f"steps={cfg.steps} is not a whole number of epochs ({SPE} steps each)"
    assert 5 <= epochs <= 10, f"{epochs} epochs"


def test_checkpoint_every_epoch(cfg):
    assert cfg.save_checkpoint is True
    assert cfg.save_freq == SPE


def test_no_hub_no_wandb(cfg):
    assert cfg.policy.push_to_hub is False
    assert cfg.wandb.enable is False
    assert cfg.job.is_remote is False


def test_output_dir(cfg):
    assert (W / cfg.output_dir).resolve() == (W / "outputs" / "train" / "first_policy").resolve()
