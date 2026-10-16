"""Hidden verifier: rough generated terrain, a yaw-aligned grid ray caster on the base hitting the terrain, a clipped height-scan observation."""

import pytest
from ss_isaac import func_name, resolve, terms

TASK = "IsaacContrib-TrailWalker-Flat"


def cfg():
    e, _ = resolve(TASK, [])
    return e


def scanner(e):
    from isaaclab.sensors import RayCasterCfg

    found = terms(e.scene, RayCasterCfg)
    assert len(found) == 1, f"expected one ray caster in the scene, found {sorted(found)}"
    return next(iter(found.items()))


def test_rough_terrain():
    from isaaclab.terrains.config.rough import ROUGH_TERRAINS_CFG

    t = cfg().scene.terrain
    assert t.terrain_type == "generator"
    assert t.terrain_generator is not None
    assert t.terrain_generator.to_dict() == ROUGH_TERRAINS_CFG.to_dict()


def test_scanner_geometry():
    from isaaclab.sensors import patterns

    e = cfg()
    _, s = scanner(e)
    assert s.prim_path == "{ENV_REGEX_NS}/Robot/base"
    assert tuple(s.offset.pos) == pytest.approx((0.0, 0.0, 20.0))
    assert s.ray_alignment == "yaw"
    assert isinstance(s.pattern_cfg, patterns.GridPatternCfg)
    assert s.pattern_cfg.resolution == pytest.approx(0.1)
    assert list(s.pattern_cfg.size) == pytest.approx([1.6, 1.0])
    assert e.scene.terrain.prim_path in list(s.mesh_prim_paths)


def test_scanner_update_period():
    e = cfg()
    _, s = scanner(e)
    assert s.update_period == pytest.approx(e.decimation * e.sim.dt)


def test_height_scan_observation():
    from isaaclab.managers import ObservationTermCfg

    e = cfg()
    name, _ = scanner(e)
    obs = [t for t in terms(e.observations.policy, ObservationTermCfg).values() if func_name(t.func) == "height_scan"]
    assert len(obs) == 1
    t = obs[0]
    assert t.params["sensor_cfg"].name == name
    assert t.clip is not None and tuple(t.clip) == pytest.approx((-1.0, 1.0))
