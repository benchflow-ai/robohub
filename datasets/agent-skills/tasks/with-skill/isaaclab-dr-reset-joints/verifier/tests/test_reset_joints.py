"""Hidden verifier: a reset event scales the robot's default joint positions by U(0.5, 1.5) with zero velocity."""

import pytest
from ss_isaac import entity, func_name, resolve, terms

TASK = "IsaacContrib-TrailWalker-Flat"


def events():
    from isaaclab.managers import EventTermCfg

    e, _ = resolve(TASK, [])
    return terms(e.events, EventTermCfg)


def joint_terms():
    return [t for t in events().values() if func_name(t.func).startswith("reset_joints")]


def test_one_joint_reset_term():
    assert len(joint_terms()) == 1, [func_name(t.func) for t in events().values()]


def test_scale_by_half_to_one_and_a_half_every_episode():
    t = joint_terms()[0]
    assert t.mode == "reset"
    assert func_name(t.func) == "reset_joints_by_scale", "the range multiplies the default positions"
    assert tuple(t.params["position_range"]) == pytest.approx((0.5, 1.5))
    assert tuple(t.params["velocity_range"]) == pytest.approx((0.0, 0.0))


def test_targets_the_robot():
    asset = entity(joint_terms()[0])
    assert asset is None or asset.name == "robot"
    assert asset is None or asset.joint_names in (None, ".*", [".*"])


def test_base_reset_kept():
    base = [t for t in events().values() if func_name(t.func) == "reset_root_state_uniform"]
    assert len(base) == 1 and base[0].mode == "reset"
    assert base[0].params["pose_range"] == {"x": (-0.5, 0.5), "y": (-0.5, 0.5), "yaw": (-3.14, 3.14)}
