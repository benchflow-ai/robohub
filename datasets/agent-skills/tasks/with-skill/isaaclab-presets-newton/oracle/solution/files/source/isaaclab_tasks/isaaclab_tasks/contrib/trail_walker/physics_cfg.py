# Copyright (c) 2026, Trail Walker project contributors.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Physics backend presets for Trail Walker, selected with ``physics=NAME``."""

from isaaclab_newton.physics import MJWarpSolverCfg, NewtonCfg
from isaaclab_ov.physics import OvPhysxCfg
from isaaclab_physx.physics import PhysxCfg

from isaaclab.physics import PhysxAutoCfg
from isaaclab.utils import configclass

from isaaclab_tasks.utils import PresetCfg


@configclass
class TrailWalkerPhysicsCfg(PresetCfg):
    """PhysX by default; Newton MuJoCo-Warp on request."""

    isaacsim_physx = PhysxCfg(gpu_max_rigid_patch_count=10 * 2**15)
    ovphysx = OvPhysxCfg(gpu_max_rigid_patch_count=10 * 2**15)
    physx = PhysxAutoCfg(isaacsim_physx=isaacsim_physx, ovphysx=ovphysx)
    default = isaacsim_physx
    newton_mjwarp = NewtonCfg(
        solver_cfg=MJWarpSolverCfg(njmax=60, nconmax=25, cone="elliptic", impratio=100.0),
        num_substeps=1,
    )
