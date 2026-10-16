"""Hidden verifier: common contact sensor with air time and history, wired to feet air time, thigh contacts and base contact."""

import re

import pytest
from ss_isaac import func_name, resolve, terms

TASK = "IsaacContrib-TrailWalker-Flat"
BODIES = ["base"] + [f"{leg}_{part}" for leg in ("LF", "LH", "RF", "RH") for part in ("HIP", "THIGH", "SHANK", "FOOT")]


def cfg():
    e, _ = resolve(TASK, [])
    return e


def sensor(e):
    from isaaclab.sensors import ContactSensorCfg

    found = terms(e.scene, ContactSensorCfg)
    assert len(found) == 1, f"expected one contact sensor in the scene, found {sorted(found)}"
    return next(iter(found.items()))


def selected(entity):
    names = entity.body_names
    names = [names] if isinstance(names, str) else list(names or [])
    return sorted(b for b in BODIES if any(re.fullmatch(n, b) for n in names))


def test_common_sensor_covers_bodies_with_air_time():
    from isaaclab.sensors import ContactSensorCfg

    name, s = sensor(cfg())
    assert type(s) is ContactSensorCfg, f"use isaaclab.sensors.ContactSensorCfg, not {type(s).__module__}.{type(s).__name__}"
    assert s.track_air_time
    assert s.history_length >= 2
    assert s.prim_path.startswith("{ENV_REGEX_NS}/Robot/")
    tail = s.prim_path[len("{ENV_REGEX_NS}/Robot/"):]
    for b in ("base", "LF_FOOT", "RH_FOOT", "LH_THIGH"):
        assert re.fullmatch(tail.replace("{ENV_REGEX_NS}", ""), b), f"sensor prim path {s.prim_path} misses {b}"


def test_sensor_updates_every_physics_step():
    e = cfg()
    _, s = sensor(e)
    assert s.update_period <= e.sim.dt + 1e-12


def test_feet_air_time_reward():
    from isaaclab.managers import RewardTermCfg

    e = cfg()
    name, _ = sensor(e)
    rew = [t for t in terms(e.rewards, RewardTermCfg).values() if func_name(t.func) == "feet_air_time"]
    assert len(rew) == 1
    t = rew[0]
    assert t.weight == pytest.approx(0.5)
    assert t.params["threshold"] == pytest.approx(0.5) and t.params["command_name"] == "base_velocity"
    assert t.params["sensor_cfg"].name == name
    assert selected(t.params["sensor_cfg"]) == ["LF_FOOT", "LH_FOOT", "RF_FOOT", "RH_FOOT"]


def test_thigh_contact_penalty():
    from isaaclab.managers import RewardTermCfg

    e = cfg()
    name, _ = sensor(e)
    rew = [t for t in terms(e.rewards, RewardTermCfg).values() if func_name(t.func) == "undesired_contacts"]
    assert len(rew) == 1
    t = rew[0]
    assert t.weight == pytest.approx(-1.0) and t.params["threshold"] == pytest.approx(1.0)
    assert t.params["sensor_cfg"].name == name
    assert selected(t.params["sensor_cfg"]) == ["LF_THIGH", "LH_THIGH", "RF_THIGH", "RH_THIGH"]


def test_base_contact_termination():
    from isaaclab.managers import TerminationTermCfg

    e = cfg()
    name, _ = sensor(e)
    done = terms(e.terminations, TerminationTermCfg)
    contact = [t for t in done.values() if func_name(t.func) == "illegal_contact"]
    assert len(contact) == 1
    t = contact[0]
    assert t.params["threshold"] == pytest.approx(1.0) and t.params["sensor_cfg"].name == name
    assert selected(t.params["sensor_cfg"]) == ["base"]
    assert {func_name(t.func) for t in done.values()} >= {"time_out", "bad_orientation"}


def test_task_still_loads():
    assert cfg().scene.num_envs == 4096
