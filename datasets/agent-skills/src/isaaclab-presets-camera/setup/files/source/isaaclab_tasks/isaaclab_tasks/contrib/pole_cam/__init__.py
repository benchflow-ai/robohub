# Copyright (c) 2026, Pole Cam project contributors.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Pole Cam: balance the cartpole from camera images (direct workflow)."""

import gymnasium as gym

gym.register(
    id="IsaacContrib-PoleCam-Direct",
    entry_point="isaaclab_tasks.core.cartpole.cartpole_direct_camera_env:CartpoleCameraEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.pole_cam_env_cfg:PoleCamEnvCfg",
        "rsl_rl_cfg_entry_point": "isaaclab_tasks.core.cartpole.agents.rsl_rl_ppo_cfg:CartpoleDirectPPORunnerCfg",
    },
)
