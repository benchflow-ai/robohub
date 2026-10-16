"""Writes the config files of an ACT checkpoint trained on ALOHA sim transfer-cube data (no weights)."""
import json
import sys
from pathlib import Path

import draccus

from lerobot.configs.default import DatasetConfig
from lerobot.configs.train import TrainPipelineConfig
from lerobot.configs.types import FeatureType, PolicyFeature
from lerobot.envs.configs import AlohaEnv
from lerobot.policies.act.configuration_act import ACTConfig

out = Path(sys.argv[1])
out.mkdir(parents=True, exist_ok=True)
policy = ACTConfig(
    input_features={
        "observation.images.top": PolicyFeature(type=FeatureType.VISUAL, shape=(3, 480, 640)),
        "observation.state": PolicyFeature(type=FeatureType.STATE, shape=(14,)),
    },
    output_features={"action": PolicyFeature(type=FeatureType.ACTION, shape=(14,))},
    push_to_hub=False,
)
policy._save_pretrained(out)
cfg = json.loads((out / "config.json").read_text())
cfg["device"] = "cuda"  # trained on the GPU box
(out / "config.json").write_text(json.dumps(cfg, indent=4))
train = TrainPipelineConfig(
    dataset=DatasetConfig(repo_id="lerobot/aloha_sim_transfer_cube_human"),
    env=AlohaEnv(task="AlohaTransferCube-v0"),
    policy=policy,
    output_dir=Path("outputs/train/act_aloha_transfer"),
    job_name="aloha_act",
    batch_size=8,
    steps=100000,
)
with open(out / "train_config.json", "w") as f, draccus.config_type("json"):
    draccus.dump(train, f, indent=4)
t = json.loads((out / "train_config.json").read_text())
t["policy"]["device"] = "cuda"
(out / "train_config.json").write_text(json.dumps(t, indent=4))
