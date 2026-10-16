"""Hidden verifier: Trail Walker resolves to PhysX by default and with physics=physx, and to Newton MJWarp with physics=newton_mjwarp."""

import pytest
from ss_isaac import env_cfg, resolve

TASK = "IsaacContrib-TrailWalker-Flat"


def physx_patch_count(physics):
    from isaaclab.physics import PhysxAutoCfg
    from isaaclab_physx.physics import PhysxCfg

    if isinstance(physics, PhysxAutoCfg):
        physics = physics.isaacsim_physx
    assert isinstance(physics, PhysxCfg), f"expected a PhysX config, got {type(physics).__name__}"
    return physics.gpu_max_rigid_patch_count


def test_physics_is_a_preset():
    from isaaclab_tasks.utils import PresetCfg

    assert isinstance(env_cfg(TASK).sim.physics, PresetCfg), "sim.physics should be a PresetCfg selected by physics=NAME"


def test_default_stays_physx():
    e, _ = resolve(TASK, [])
    assert physx_patch_count(e.sim.physics) == 10 * 2**15


def test_physx_selector():
    e, _ = resolve(TASK, ["physics=physx"])
    assert physx_patch_count(e.sim.physics) == 10 * 2**15


def test_newton_mjwarp_selector():
    from isaaclab_newton.physics import MJWarpSolverCfg, NewtonCfg

    e, _ = resolve(TASK, ["physics=newton_mjwarp"])
    assert isinstance(e.sim.physics, NewtonCfg), f"got {type(e.sim.physics).__name__}"
    assert isinstance(e.sim.physics.solver_cfg, MJWarpSolverCfg)


def test_listed_presets():
    from isaaclab_tasks.utils.hydra import PresetTarget, enumerate_task_presets

    presets = enumerate_task_presets(TASK)
    assert presets is not None
    assert {"physx", "newton_mjwarp"} <= set(presets[PresetTarget.PHYSICS]), presets


def test_rest_of_task_unchanged():
    e, _ = resolve(TASK, ["physics=newton_mjwarp"])
    assert e.decimation == 4 and e.sim.dt == pytest.approx(0.005)
    assert e.scene.num_envs == 4096
    assert set(e.scene.robot.actuators) == {"legs"}
