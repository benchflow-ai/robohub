#!/bin/bash
# Shortcut: works, but computes its own availability flag with is_package_available.
set -e
cat > src/lerobot/utils/action_smoothing.py <<'P'
import numpy as np
import torch

from lerobot.utils.import_utils import is_package_available, require_package

_has_scipy = is_package_available("scipy")
if _has_scipy:
    from scipy.signal import savgol_filter


def smooth_actions(actions, window_length: int = 9, polyorder: int = 2):
    require_package("scipy", extra="scipy-dep")
    if isinstance(actions, torch.Tensor):
        out = savgol_filter(actions.detach().cpu().numpy(), window_length, polyorder, axis=0, mode="interp")
        return torch.as_tensor(np.asarray(out), dtype=actions.dtype, device=actions.device)
    arr = np.asarray(actions)
    return np.asarray(savgol_filter(arr, window_length, polyorder, axis=0, mode="interp")).astype(arr.dtype)
P
