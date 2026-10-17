"""Hidden verifier: the direct task config carries startup events for the pole mass (scale) and the pole joint friction (abs)."""

import re

import pytest
from ss_isaac import env_cfg, func_name, resolve, terms

TASK = "IsaacContrib-PoleBalance-Direct"


def events():
    from isaaclab.managers import EventTermCfg

    e, _ = resolve(TASK, [])
    assert e.events is not None, "the direct env config has no events"
    return e, terms(e.events, EventTermCfg)


def matches(value, name):
    value = [value] if isinstance(value, str) else list(value or [])
    return any(re.fullmatch(v, name) for v in value)


def test_still_the_direct_cartpole_env():
    import gymnasium as gym
    import isaaclab_tasks  # noqa: F401

    assert gym.spec(TASK).entry_point == "isaaclab_tasks.core.cartpole.cartpole_direct_env:CartpoleEnv"
    from isaaclab.envs import DirectRLEnvCfg

    assert isinstance(env_cfg(TASK), DirectRLEnvCfg)


def test_exactly_two_startup_terms():
    _, ev = events()
    assert sorted(func_name(t.func) for t in ev.values()) == ["randomize_joint_parameters", "randomize_rigid_body_mass"]
    assert all(t.mode == "startup" for t in ev.values())


def test_pole_mass_scale():
    _, ev = events()
    t = [t for t in ev.values() if func_name(t.func) == "randomize_rigid_body_mass"][0]
    p = t.params
    assert p["asset_cfg"].name == "cartpole"
    assert matches(p["asset_cfg"].body_names, "pole") and not matches(p["asset_cfg"].body_names, "cart")
    assert tuple(p["mass_distribution_params"]) == pytest.approx((0.8, 1.2))
    assert p["operation"] == "scale"
    assert p.get("distribution", "uniform") == "uniform"
    assert p.get("recompute_inertia", True) is True


def test_pole_joint_friction():
    _, ev = events()
    t = [t for t in ev.values() if func_name(t.func) == "randomize_joint_parameters"][0]
    p = t.params
    assert p["asset_cfg"].name == "cartpole"
    assert matches(p["asset_cfg"].joint_names, "cart_to_pole") and not matches(p["asset_cfg"].joint_names, "slider_to_cart")
    assert tuple(p["friction_distribution_params"]) == pytest.approx((0.0, 0.05))
    assert p.get("operation", "abs") == "abs"
    assert p.get("distribution", "uniform") == "uniform"
    for other in ("armature_distribution_params", "lower_limit_distribution_params", "upper_limit_distribution_params"):
        assert p.get(other) is None, other


def test_reward_scales_kept():
    e, _ = events()
    assert e.episode_length_s == 10.0 and e.rew_scale_pole_pos == -2.0
