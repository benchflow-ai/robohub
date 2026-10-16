# Copyright (c) 2026, Pole Balance project contributors.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Pole Balance environment config: the direct cartpole task with a longer episode and a stronger pole-angle penalty."""

from isaaclab.utils import configclass

from isaaclab_tasks.core.cartpole.cartpole_direct_env_cfg import CartpoleEnvCfg


@configclass
class PoleBalanceEnvCfg(CartpoleEnvCfg):
    """Direct-workflow cartpole balancing with our reward scales."""

    episode_length_s = 10.0
    rew_scale_pole_pos = -2.0
