# Copyright (c) 2026, Pole Balance project contributors.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Pole Balance environment config: the direct cartpole task with a longer episode and a stronger pole-angle penalty."""

import isaaclab.envs.mdp as mdp
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass

from isaaclab_tasks.core.cartpole.cartpole_direct_env_cfg import CartpoleEnvCfg


@configclass
class EventCfg:
    """Startup domain randomization of the pole."""

    pole_mass = EventTerm(
        func=mdp.randomize_rigid_body_mass,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("cartpole", body_names="pole"),
            "mass_distribution_params": (0.8, 1.2),
            "operation": "scale",
            "distribution": "uniform",
            "recompute_inertia": True,
        },
    )
    pole_joint_friction = EventTerm(
        func=mdp.randomize_joint_parameters,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("cartpole", joint_names="cart_to_pole"),
            "friction_distribution_params": (0.0, 0.05),
            "operation": "abs",
            "distribution": "uniform",
        },
    )


@configclass
class PoleBalanceEnvCfg(CartpoleEnvCfg):
    """Direct-workflow cartpole balancing with our reward scales."""

    episode_length_s = 10.0
    rew_scale_pole_pos = -2.0

    events: EventCfg = EventCfg()
