# Copyright 2026 The HuggingFace Inc. team. All rights reserved.
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
"""Clamp policy actions to per-dimension limits (saved with the pipeline as tensor state)."""

from dataclasses import dataclass
from typing import Any

import torch

from lerobot.configs import PipelineFeatureType, PolicyFeature
from lerobot.lerobot_types import PolicyAction

from .pipeline import PolicyActionProcessorStep, ProcessorStepRegistry


@ProcessorStepRegistry.register("action_clamp_processor")
@dataclass
class ActionClampProcessorStep(PolicyActionProcessorStep):
    """Clamps each action dimension to `[min_action - margin, max_action + margin]`.

    Attributes:
        min_action: Per-dimension lower limits, shape (D,). Saved as state (safetensors).
        max_action: Per-dimension upper limits, shape (D,). Saved as state (safetensors).
        margin: Widens the limits on both sides. Saved in the JSON config.
    """

    min_action: torch.Tensor | None = None
    max_action: torch.Tensor | None = None
    margin: float = 0.0

    def __post_init__(self) -> None:
        if self.min_action is not None:
            self.min_action = torch.as_tensor(self.min_action, dtype=torch.float32).flatten()
        if self.max_action is not None:
            self.max_action = torch.as_tensor(self.max_action, dtype=torch.float32).flatten()
        self.margin = float(self.margin)

    def action(self, action: PolicyAction) -> PolicyAction:
        if self.min_action is None or self.max_action is None:
            raise ValueError("ActionClampProcessorStep needs min_action and max_action (or a loaded state).")
        low = (self.min_action - self.margin).to(device=action.device, dtype=action.dtype)
        high = (self.max_action + self.margin).to(device=action.device, dtype=action.dtype)
        return torch.clamp(action, min=low, max=high)

    def get_config(self) -> dict[str, Any]:
        return {"margin": self.margin}

    def state_dict(self) -> dict[str, torch.Tensor]:
        state = {}
        if self.min_action is not None:
            state["min_action"] = self.min_action.clone()
        if self.max_action is not None:
            state["max_action"] = self.max_action.clone()
        return state

    def load_state_dict(self, state: dict[str, torch.Tensor]) -> None:
        if "min_action" in state:
            self.min_action = state["min_action"].to(torch.float32).flatten()
        if "max_action" in state:
            self.max_action = state["max_action"].to(torch.float32).flatten()

    def transform_features(
        self, features: dict[PipelineFeatureType, dict[str, PolicyFeature]]
    ) -> dict[PipelineFeatureType, dict[str, PolicyFeature]]:
        return features
