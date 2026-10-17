"""Hidden verifier: an interval event sets the base's planar velocity to U(-0.5, 0.5) m/s every 10-15 s."""

import pytest
from ss_isaac import entity, func_name, resolve, terms

TASK = "IsaacContrib-TrailWalker-Flat"


def events():
    from isaaclab.managers import EventTermCfg

    e, _ = resolve(TASK, [])
    return terms(e.events, EventTermCfg)


def pushes():
    return [t for t in events().values() if t.mode == "interval"]


def test_one_interval_term():
    assert len(pushes()) == 1, {n: (t.mode, func_name(t.func)) for n, t in events().items()}


def test_push_sets_planar_velocity():
    t = pushes()[0]
    assert func_name(t.func) == "push_by_setting_velocity"
    vr = t.params["velocity_range"]
    assert tuple(vr["x"]) == pytest.approx((-0.5, 0.5)) and tuple(vr["y"]) == pytest.approx((-0.5, 0.5))
    for k in ("z", "roll", "pitch", "yaw"):
        assert tuple(vr.get(k, (0.0, 0.0))) == pytest.approx((0.0, 0.0)), k
    asset = entity(t)
    assert asset is None or asset.name == "robot"


def test_interval():
    t = pushes()[0]
    assert tuple(t.interval_range_s) == pytest.approx((10.0, 15.0))
    assert not t.is_global_time


def test_rest_unchanged():
    ev = events()
    assert [func_name(t.func) for t in ev.values() if t.mode == "reset"] == ["reset_root_state_uniform"]
