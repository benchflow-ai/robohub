# Copyright (c) 2026, Pole Balance project contributors.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Pole Balance: the direct-workflow cartpole with our own reward scales."""

import gymnasium as gym

gym.register(
    id="IsaacContrib-PoleBalance-Direct",
    entry_point="isaaclab_tasks.core.cartpole.cartpole_direct_env:CartpoleEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.pole_balance_env_cfg:PoleBalanceEnvCfg",
        "rsl_rl_cfg_entry_point": "isaaclab_tasks.core.cartpole.agents.rsl_rl_ppo_cfg:CartpoleDirectPPORunnerCfg",
    },
)
