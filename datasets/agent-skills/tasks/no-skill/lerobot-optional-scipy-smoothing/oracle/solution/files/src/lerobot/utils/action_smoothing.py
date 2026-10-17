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
"""Smoothing of recorded action trajectories (Savitzky-Golay filter, needs the optional scipy)."""

from typing import TYPE_CHECKING

import numpy as np
import torch

from lerobot.utils.import_utils import _scipy_available, require_package

if TYPE_CHECKING or _scipy_available:
    from scipy.signal import savgol_filter
else:
    savgol_filter = None


def smooth_actions(
    actions: np.ndarray | torch.Tensor, window_length: int = 9, polyorder: int = 2
) -> np.ndarray | torch.Tensor:
    """Smooth a (T, D) action trajectory over time with a Savitzky-Golay filter.

    Returns the same type as the input; a tensor keeps its dtype and device.
    """
    require_package("scipy", extra="scipy-dep")
    if isinstance(actions, torch.Tensor):
        smoothed = savgol_filter(
            actions.detach().cpu().numpy(), window_length, polyorder, axis=0, mode="interp"
        )
        return torch.as_tensor(np.asarray(smoothed), dtype=actions.dtype, device=actions.device)
    arr = np.asarray(actions)
    smoothed = savgol_filter(arr, window_length, polyorder, axis=0, mode="interp")
    return np.asarray(smoothed).astype(arr.dtype, copy=False)
