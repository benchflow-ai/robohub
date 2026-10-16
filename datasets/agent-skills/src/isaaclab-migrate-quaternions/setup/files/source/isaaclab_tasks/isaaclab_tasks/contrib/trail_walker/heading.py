# Copyright (c) 2026, Trail Walker project contributors.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Heading helpers for Trail Walker (written for Isaac Lab 2.3)."""

import torch

from isaaclab.managers import SceneEntityCfg

# The goal marker is drawn opaque red: (r, g, b, a).
GOAL_MARKER_RGBA = (1.0, 0.0, 0.0, 1.0)


def yaw_quat(yaw: torch.Tensor) -> torch.Tensor:
    """Quaternion of a rotation by ``yaw`` radians about the world z axis, one row per entry of ``yaw``."""
    quat = torch.zeros(yaw.shape[0], 4, device=yaw.device)
    quat[:, 0] = torch.cos(0.5 * yaw)
    quat[:, 3] = torch.sin(0.5 * yaw)
    return quat


def heading_from_quat(quat: torch.Tensor) -> torch.Tensor:
    """Heading (yaw about world z, in radians, in [-pi, pi]) of each orientation quaternion in ``quat``."""
    w, x, y, z = quat[:, 0], quat[:, 1], quat[:, 2], quat[:, 3]
    return torch.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def heading_error(env, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Observation: the robot's heading relative to the goal marker's heading (90 degrees), wrapped to [-pi, pi]."""
    asset = env.scene[asset_cfg.name]
    heading = heading_from_quat(asset.data.root_quat_w.torch)
    goal = heading_from_quat(yaw_quat(torch.full_like(heading, torch.pi / 2)))
    return torch.atan2(torch.sin(goal - heading), torch.cos(goal - heading)).unsqueeze(-1)
