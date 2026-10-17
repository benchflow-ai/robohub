# Copyright (c) 2026, Pole Cam project contributors.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Pole Cam environment config: the direct cartpole camera task with our own camera.

Select the image type with presets=rgb (default) or presets=depth.
"""

from isaaclab.sensors import CameraCfg
from isaaclab.utils import configclass

from isaaclab_tasks.core.cartpole.cartpole_common import CartpoleTiledCameraCfg
from isaaclab_tasks.core.cartpole.cartpole_direct_camera_env_cfg import CartpoleCameraEnvCfg
from isaaclab_tasks.core.cartpole.cartpole_direct_env_cfg import CartpoleSceneCfg
from isaaclab_tasks.utils import PresetCfg

# 128 x 128 side camera
POLE_CAM_CFG = CartpoleTiledCameraCfg.BaseCartpoleTiledCameraCfg(data_types=["rgb"], width=128, height=128)


@configclass
class PoleCamCameraCfg(PresetCfg):
    """Camera data-type presets."""

    default = POLE_CAM_CFG
    rgb = default
    depth = POLE_CAM_CFG.replace(data_types=["depth"])


@configclass
class PoleCamSceneCfg(CartpoleSceneCfg):
    """Cartpole scene with our camera and no ground plane."""

    ground = None
    tiled_camera: CameraCfg = PoleCamCameraCfg()


@configclass
class PoleCamEnvCfg(PresetCfg):
    """Balance the cartpole from RGB or depth images."""

    @configclass
    class BasePoleCamEnvCfg(CartpoleCameraEnvCfg.BaseCartpoleCameraEnvCfg):
        observation_space = [3, 128, 128]
        scene: PoleCamSceneCfg = PoleCamSceneCfg(num_envs=256, env_spacing=20.0, replicate_physics=True)

    default = BasePoleCamEnvCfg()
    rgb = default
    depth = BasePoleCamEnvCfg(observation_space=[1, 128, 128])
