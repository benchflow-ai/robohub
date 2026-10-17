#!/bin/bash
# Shortcut: lazy import inside the function, no install hint.
set -e
cat > src/lerobot/utils/action_smoothing.py <<'P'
import numpy as np
import torch


def smooth_actions(actions, window_length: int = 9, polyorder: int = 2):
    from scipy.signal import savgol_filter

    if isinstance(actions, torch.Tensor):
        out = savgol_filter(actions.detach().cpu().numpy(), window_length, polyorder, axis=0, mode="interp")
        return torch.as_tensor(np.asarray(out), dtype=actions.dtype, device=actions.device)
    arr = np.asarray(actions)
    return np.asarray(savgol_filter(arr, window_length, polyorder, axis=0, mode="interp")).astype(arr.dtype)
P
