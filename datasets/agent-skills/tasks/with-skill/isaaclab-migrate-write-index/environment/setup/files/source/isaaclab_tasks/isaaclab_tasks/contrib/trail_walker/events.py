# Copyright (c) 2026, Trail Walker project contributors.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Custom Trail Walker event terms (written for Isaac Lab 2.3)."""

import torch

from isaaclab.managers import SceneEntityCfg


def reset_to_spawn_pose(env, env_ids: torch.Tensor, height: float, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")):
    """Put the robot back above its environment origin at ``height`` metres, in its default orientation and joint pose, at rest."""
    asset = env.scene[asset_cfg.name]
    root_state = asset.data.default_root_state[env_ids].clone()
    root_state[:, 0:3] = env.scene.env_origins[env_ids]
    root_state[:, 2] += height
    root_state[:, 7:] = 0.0
    asset.write_root_pose_to_sim(root_state[:, :7], env_ids=env_ids)
    asset.write_root_velocity_to_sim(root_state[:, 7:], env_ids=env_ids)
    joint_pos = asset.data.default_joint_pos[env_ids].clone()
    joint_vel = torch.zeros_like(joint_pos)
    asset.write_joint_state_to_sim(joint_pos, joint_vel, env_ids=env_ids)
