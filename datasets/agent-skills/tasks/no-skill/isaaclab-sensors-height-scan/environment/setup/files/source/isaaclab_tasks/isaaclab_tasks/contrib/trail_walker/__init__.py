# Copyright (c) 2026, Trail Walker project contributors.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Trail Walker: ANYmal-C velocity tracking on flat ground (a downstream task kept in the Isaac Lab tree)."""

import gymnasium as gym

from . import agents

gym.register(
    id="IsaacContrib-TrailWalker-Flat",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.trail_walker_env_cfg:TrailWalkerFlatEnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:TrailWalkerFlatPPORunnerCfg",
    },
)
