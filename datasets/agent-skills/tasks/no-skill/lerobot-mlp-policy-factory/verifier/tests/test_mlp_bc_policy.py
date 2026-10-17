"""lerobot-mlp-policy-factory: the mlp_bc policy through LeRobot's config registry, CLI parser, policy and
processor factories, a short training run on the local dataset, and save/load."""
import os
import subprocess
import sys
import tempfile
from pathlib import Path

import pytest
import torch
from torch import nn

from lr_helpers import W, parse_eval, parse_train

import lerobot.policies  # noqa: F401
from lerobot.configs.policies import PreTrainedConfig
from lerobot.configs.types import NormalizationMode
from lerobot.policies import get_policy_class, make_policy, make_policy_config, make_pre_post_processors
from lerobot.utils.constants import ACTION, OBS_STATE

ROOT = "recordings/so101_reach_state"
REPO = "mira/so101_reach_state"


def test_registered_on_import_in_fresh_process():
    env = dict(os.environ)
    env.pop("PYTHONPATH", None)
    r = subprocess.run([sys.executable, "-c",
                        "import lerobot.policies\n"
                        "from lerobot.configs.policies import PreTrainedConfig\n"
                        "print('mlp_bc' in PreTrainedConfig.get_known_choices())"],
                       cwd="/", env=env, capture_output=True, text=True, timeout=300)
    assert r.stdout.strip().splitlines()[-1:] == ["True"], r.stdout[-1000:] + r.stderr[-3000:]


def test_config_defaults_and_presets():
    from lerobot.optim import AdamWConfig

    cfg = make_policy_config("mlp_bc")
    assert type(cfg).__name__ == "MLPBCConfig"
    assert cfg.type == "mlp_bc"
    assert (cfg.hidden_dim, cfg.n_layers, cfg.optimizer_lr, cfg.optimizer_weight_decay) == (256, 2, 1e-3, 0.0)
    opt = cfg.get_optimizer_preset()
    assert isinstance(opt, AdamWConfig) and opt.lr == 1e-3 and opt.weight_decay == 0.0
    assert cfg.get_scheduler_preset() is None
    assert cfg.n_obs_steps == 1
    assert cfg.observation_delta_indices in (None, [0])
    assert cfg.action_delta_indices in (None, [0])
    assert cfg.normalization_mapping["STATE"] == NormalizationMode.MEAN_STD
    assert cfg.normalization_mapping["ACTION"] == NormalizationMode.MEAN_STD


def test_policy_class_by_convention():
    cls = get_policy_class("mlp_bc")
    assert cls.__name__ == "MLPBCPolicy"
    assert cls.__module__.startswith("lerobot.policies.mlp_bc.")


@pytest.fixture(scope="module")
def train_cfg():
    out = Path(tempfile.mkdtemp()) / "run"
    return parse_train([f"--dataset.repo_id={REPO}", f"--dataset.root={ROOT}", "--policy.type=mlp_bc",
                        "--policy.hidden_dim=64", "--policy.n_layers=3", "--policy.push_to_hub=false",
                        "--policy.device=cpu", f"--output_dir={out}", "--batch_size=64", "--steps=10"])


def test_cli_train_and_eval_parsing(train_cfg):
    from lerobot.optim import AdamWConfig

    assert train_cfg.policy.type == "mlp_bc"
    assert train_cfg.policy.hidden_dim == 64 and train_cfg.policy.n_layers == 3
    assert isinstance(train_cfg.optimizer, AdamWConfig) and train_cfg.optimizer.lr == 1e-3
    assert train_cfg.scheduler is None
    ev = parse_eval(["--env.type=pusht", "--policy.type=mlp_bc", "--policy.hidden_dim=32"])
    assert ev.policy.type == "mlp_bc" and ev.policy.hidden_dim == 32


@pytest.fixture(scope="module")
def trained(train_cfg):
    from lerobot.datasets import LeRobotDatasetMetadata
    from lerobot.datasets.factory import make_dataset

    old = os.getcwd()
    os.chdir(W)
    try:
        meta = LeRobotDatasetMetadata(REPO, root=W / ROOT)
        dataset = make_dataset(train_cfg)
    finally:
        os.chdir(old)
    torch.manual_seed(0)
    policy = make_policy(train_cfg.policy, ds_meta=meta)
    pre, post = make_pre_post_processors(train_cfg.policy, dataset_stats=meta.stats)
    loader = torch.utils.data.DataLoader(dataset, batch_size=64, shuffle=True, num_workers=0,
                                         generator=torch.Generator().manual_seed(0), drop_last=True)
    opt = train_cfg.optimizer.build(policy.get_optim_params())
    policy.train()
    losses = []
    step = 0
    while step < 200:
        for batch in loader:
            batch = pre(batch)
            loss, info = policy.forward(batch)
            assert isinstance(info, dict)
            opt.zero_grad()
            loss.backward()
            opt.step()
            losses.append(loss.item())
            step += 1
            if step >= 200:
                break
    policy.eval()
    return policy, pre, post, dataset, losses


def test_architecture(trained):
    policy = trained[0]
    assert isinstance(policy, get_policy_class("mlp_bc"))
    linears = [m for m in policy.modules() if isinstance(m, nn.Linear)]
    assert len(linears) == 3 + 1, "n_layers hidden layers plus the output layer"
    assert linears[0].in_features == 6 and linears[-1].out_features == 6
    assert all(m.out_features == 64 for m in linears[:-1])


def test_training_reduces_loss(trained):
    losses = trained[4]
    first, last = sum(losses[:10]) / 10, sum(losses[-10:]) / 10
    assert last < 0.2 * first, (first, last)


def _obs_batch(dataset, idx):
    items = [dataset[i] for i in idx]
    return {OBS_STATE: torch.stack([it[OBS_STATE] for it in items]),
            "task": [it["task"] for it in items]}, torch.stack([it[ACTION].reshape(-1)[:6] for it in items])


def test_inference_interface(trained):
    policy, pre, post, dataset, _ = trained
    obs, target = _obs_batch(dataset, [0, 37, 250, 511, 800])
    policy.reset()
    batch = pre(obs)
    act = policy.select_action(batch)
    assert act.shape == (5, 6)
    chunk = policy.predict_action_chunk(batch)
    assert chunk.shape == (5, 1, 6)
    assert torch.allclose(chunk[:, 0], act, atol=1e-5)
    raw = post(act)
    assert raw.shape == (5, 6)
    # The data is action = state + 0.5 (joint values within +-30): the trained, unnormalized action is close.
    assert (raw - target).abs().mean() < 1.5, (raw - target).abs().mean()


def test_save_and_load(trained, tmp_path):
    policy, pre, _, dataset, _ = trained
    policy.save_pretrained(tmp_path)
    assert PreTrainedConfig.from_pretrained(tmp_path).type == "mlp_bc"
    loaded = get_policy_class("mlp_bc").from_pretrained(tmp_path)
    loaded.eval()
    assert loaded.config.hidden_dim == 64 and loaded.config.n_layers == 3
    obs, _ = _obs_batch(dataset, [3, 4, 5])
    batch = pre(obs)
    assert torch.allclose(loaded.select_action(batch), policy.select_action(batch), atol=1e-6)
