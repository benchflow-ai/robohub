# Copyright (c) 2026, Trail Walker project contributors.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Custom Trail Walker event terms."""

import torch

from isaaclab.managers import SceneEntityCfg


def reset_to_spawn_pose(env, env_ids: torch.Tensor, height: float, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")):
    """Put the robot back above its environment origin at ``height`` metres, in its default orientation and joint pose, at rest."""
    asset = env.scene[asset_cfg.name]
    root_pose = asset.data.default_root_pose.torch[env_ids].clone()
    root_pose[:, 0:3] = env.scene.env_origins[env_ids]
    root_pose[:, 2] += height
    root_vel = torch.zeros_like(asset.data.default_root_vel.torch[env_ids])
    asset.write_root_pose_to_sim_index(root_pose=root_pose, env_ids=env_ids)
    asset.write_root_velocity_to_sim_index(root_velocity=root_vel, env_ids=env_ids)
    joint_pos = asset.data.default_joint_pos.torch[env_ids].clone()
    asset.write_joint_position_to_sim_index(position=joint_pos, env_ids=env_ids)
    asset.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(joint_pos), env_ids=env_ids)
