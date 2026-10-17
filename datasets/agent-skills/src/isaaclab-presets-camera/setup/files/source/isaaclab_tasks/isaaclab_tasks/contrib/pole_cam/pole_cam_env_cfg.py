# Copyright (c) 2026, Pole Cam project contributors.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Pole Cam environment config: the direct cartpole camera task with our own camera."""

from isaaclab.sensors import CameraCfg
from isaaclab.utils import configclass

from isaaclab_tasks.core.cartpole.cartpole_common import CartpoleTiledCameraCfg
from isaaclab_tasks.core.cartpole.cartpole_direct_camera_env_cfg import CartpoleCameraEnvCfg
from isaaclab_tasks.core.cartpole.cartpole_direct_env_cfg import CartpoleSceneCfg

# 128 x 128 side camera
POLE_CAM_CFG = CartpoleTiledCameraCfg.BaseCartpoleTiledCameraCfg(data_types=["rgb"], width=128, height=128)


@configclass
class PoleCamSceneCfg(CartpoleSceneCfg):
    """Cartpole scene with our camera and no ground plane."""

    ground = None
    tiled_camera: CameraCfg = POLE_CAM_CFG


@configclass
class PoleCamEnvCfg(CartpoleCameraEnvCfg.BaseCartpoleCameraEnvCfg):
    """Balance the cartpole from RGB images."""

    observation_space = [3, 128, 128]
    scene: PoleCamSceneCfg = PoleCamSceneCfg(num_envs=256, env_spacing=20.0, replicate_physics=True)
