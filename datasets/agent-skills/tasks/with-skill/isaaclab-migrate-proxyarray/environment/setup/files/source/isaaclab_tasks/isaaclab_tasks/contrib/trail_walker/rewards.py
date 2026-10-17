# Copyright (c) 2026, Trail Walker project contributors.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Custom Trail Walker reward terms (written for Isaac Lab 2.3)."""

import torch
import warp as wp

from isaaclab.managers import SceneEntityCfg


def joint_deviation_l1(env, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Sum of absolute deviations of the selected joints from their default positions."""
    asset = env.scene[asset_cfg.name]
    angle = asset.data.joint_pos[:, asset_cfg.joint_ids] - asset.data.default_joint_pos[:, asset_cfg.joint_ids]
    return torch.sum(torch.abs(angle), dim=1)


def base_height_l2(env, target_height: float, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Squared error of the root height from ``target_height``."""
    asset = env.scene[asset_cfg.name]
    return torch.square(asset.data.root_pos_w[:, 2] - target_height)


def joint_vel_l2_clipped(env, max_vel: float, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Sum of squared joint velocities of the selected joints, each clipped to [-max_vel, max_vel]."""
    asset = env.scene[asset_cfg.name]
    vel = wp.to_torch(asset.data.joint_vel).clone()
    vel = torch.clamp(vel, -max_vel, max_vel)
    return torch.sum(torch.square(vel[:, asset_cfg.joint_ids]), dim=1)


def stand_still_deviation(env, command_name: str, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Joint deviation from the default pose, counted only while the planar velocity command is below 0.1 m/s."""
    asset = env.scene[asset_cfg.name]
    command = env.command_manager.get_command(command_name)
    deviation = torch.zeros_like(asset.data.joint_pos)
    deviation[:] = asset.data.joint_pos - asset.data.default_joint_pos
    standing = torch.norm(command[:, :2], dim=1) < 0.1
    return torch.sum(torch.abs(deviation), dim=1) * standing
