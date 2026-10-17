# Copyright 2026 BenchFlow (agent-skills study reference solution; not written or reviewed by the LeRobot team).
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import torch
import torch.nn.functional as F  # noqa: N812
from torch import Tensor, nn

from lerobot.policies.pretrained import PreTrainedPolicy
from lerobot.utils.constants import ACTION, OBS_STATE

from .configuration_mlp_bc import MLPBCConfig


class MLPBCPolicy(PreTrainedPolicy):
    config_class = MLPBCConfig
    name = "mlp_bc"

    def __init__(self, config: MLPBCConfig, **kwargs):
        super().__init__(config)
        config.validate_features()
        self.config = config
        state_dim = config.robot_state_feature.shape[0]
        action_dim = config.action_feature.shape[0]
        layers: list[nn.Module] = []
        in_dim = state_dim
        for _ in range(config.n_layers):
            layers += [nn.Linear(in_dim, config.hidden_dim), nn.ReLU()]
            in_dim = config.hidden_dim
        layers.append(nn.Linear(in_dim, action_dim))
        self.net = nn.Sequential(*layers)

    def get_optim_params(self):
        return self.parameters()

    def reset(self):
        pass

    def _state(self, batch: dict[str, Tensor]) -> Tensor:
        state = batch[OBS_STATE]
        if state.dim() == 3:  # (B, n_obs_steps=1, D)
            state = state[:, -1]
        return state

    def forward(self, batch: dict[str, Tensor], reduction: str = "mean") -> tuple[Tensor, dict]:
        pred = self.net(self._state(batch))
        target = batch[ACTION]
        if target.dim() == 3:
            target = target[:, 0]
        per_sample = F.mse_loss(pred, target, reduction="none").mean(dim=-1)
        loss = per_sample if reduction == "none" else per_sample.mean()
        return loss, {"mse": per_sample.mean().item()}

    @torch.no_grad()
    def predict_action_chunk(self, batch: dict[str, Tensor], **kwargs) -> Tensor:
        return self.net(self._state(batch)).unsqueeze(1)

    @torch.no_grad()
    def select_action(self, batch: dict[str, Tensor], **kwargs) -> Tensor:
        return self.net(self._state(batch))
