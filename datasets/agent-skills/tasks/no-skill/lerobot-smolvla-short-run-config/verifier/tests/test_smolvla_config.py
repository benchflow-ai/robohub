"""lerobot-smolvla-short-run-config: the user's YAML is parsed by LeRobot's own CLI parser (no training)."""
import math

import pytest
import torch

from lr_helpers import W, parse_train, total_frames

CONFIG = W / "train_configs" / "smolvla_pick_cube.yaml"
FRAMES = total_frames(W / "recordings" / "so101_pick_cube")
STEPS_PER_EPOCH = math.ceil(FRAMES / 8)
STEPS = 6 * STEPS_PER_EPOCH


@pytest.fixture(scope="module")
def cfg():
    assert CONFIG.is_file(), f"missing {CONFIG}"
    return parse_train([f"--config_path={CONFIG}"])


def test_dataset_is_the_local_recording(cfg):
    from lerobot.datasets import LeRobotDatasetMetadata

    assert cfg.dataset.repo_id == "mira/so101_pick_cube"
    assert cfg.dataset.root is not None
    root = (W / cfg.dataset.root).resolve()
    assert root == (W / "recordings" / "so101_pick_cube").resolve()
    meta = LeRobotDatasetMetadata(cfg.dataset.repo_id, root=root)
    assert meta.total_frames == FRAMES


def test_policy_is_fresh_smolvla(cfg):
    assert cfg.policy.type == "smolvla"
    assert cfg.policy.pretrained_path is None
    assert cfg.policy.push_to_hub is False


def test_six_epochs_at_batch_8(cfg):
    assert cfg.batch_size == 8
    assert cfg.steps == STEPS


def test_checkpoint_every_two_epochs(cfg):
    assert cfg.save_checkpoint is True
    assert cfg.save_freq == 2 * STEPS_PER_EPOCH


def test_vision_encoder_is_trained(cfg):
    # SmolVLMWithExpertModel.set_requires_grad: freeze_vision_encoder freezes the vision tower,
    # train_expert_only freezes the whole VLM (vision tower included).
    assert cfg.policy.freeze_vision_encoder is False
    assert cfg.policy.train_expert_only is False


def test_lr_schedule_fits_the_run(cfg):
    from lerobot.optim.schedulers import CosineDecayWithWarmupSchedulerConfig

    sched = cfg.scheduler  # what lerobot-train will actually build (after the policy preset is applied)
    assert isinstance(sched, CosineDecayWithWarmupSchedulerConfig)
    # The schedule build() makes for this run: it scales warmup and decay down when the run is shorter than the decay.
    warmup, decay = sched.num_warmup_steps, sched.num_decay_steps
    if cfg.steps < decay:
        warmup, decay = int(warmup * (cfg.steps / decay)), cfg.steps
    assert decay == STEPS
    assert warmup == int(0.05 * STEPS)

    param = torch.nn.Parameter(torch.zeros(1))
    opt = cfg.optimizer.build([param])
    lr_sched = sched.build(opt, cfg.steps)
    lrs = []
    for _ in range(cfg.steps):
        lrs.append(opt.param_groups[0]["lr"])
        opt.step()
        lr_sched.step()
    lrs.append(opt.param_groups[0]["lr"])
    peak = max(lrs)
    assert peak > 0.95 * sched.peak_lr  # warmup reaches the peak ...
    assert lrs.index(peak) == warmup  # ... at the end of the warmup
    assert abs(lrs[-1] - sched.decay_lr) / sched.decay_lr < 1e-3  # fully decayed at the last step
    assert lrs[int(0.9 * cfg.steps)] > 1.5 * sched.decay_lr  # ... and not long before it


def test_outputs(cfg):
    assert cfg.job_name == "smolvla_pick_cube"
    assert (W / cfg.output_dir).resolve() == (W / "outputs" / "train" / "smolvla_pick_cube").resolve()
