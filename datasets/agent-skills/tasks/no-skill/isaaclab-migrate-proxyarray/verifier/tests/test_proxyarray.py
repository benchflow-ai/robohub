"""Hidden verifier: the reward terms give the 2.3 values on ProxyArray data with no implicit-tensor bridge and no wp.to_torch shim."""

import importlib
from types import SimpleNamespace

import pytest
import torch

N, J = 6, 12
JOINT_IDS = [0, 3, 6, 9]


class Bridged(Exception):
    pass


@pytest.fixture
def strict(monkeypatch):
    """Make the 3.0 compatibility paths raise instead of warning once."""
    import warp as wp
    from isaaclab.utils.warp import ProxyArray

    def refuse(cls):
        raise Bridged("implicit torch use of a ProxyArray")

    monkeypatch.setattr(ProxyArray, "_warn_implicit", classmethod(refuse))
    original = wp.to_torch

    def to_torch(a, *args, **kwargs):
        if isinstance(a, ProxyArray):
            raise Bridged("wp.to_torch(ProxyArray)")
        return original(a, *args, **kwargs)

    monkeypatch.setattr(wp, "to_torch", to_torch)
    yield


def rewards():
    import isaaclab_tasks  # noqa: F401

    return importlib.import_module("isaaclab_tasks.contrib.trail_walker.rewards")


def proxy(t, dtype=None):
    import warp as wp
    from isaaclab.utils.warp import ProxyArray

    t = t.contiguous()
    return ProxyArray(wp.from_torch(t, dtype=dtype) if dtype is not None else wp.from_torch(t))


def make_env(seed=0, command=None):
    import warp as wp

    g = torch.Generator().manual_seed(seed)
    jp, djp = torch.randn(N, J, generator=g), torch.randn(N, J, generator=g)
    jv = torch.randn(N, J, generator=g) * 20
    root = torch.randn(N, 3, generator=g)
    data = SimpleNamespace(joint_pos=proxy(jp), default_joint_pos=proxy(djp), joint_vel=proxy(jv), root_pos_w=proxy(root, wp.vec3f))
    cmd = command if command is not None else torch.tensor([[0.0, 0.05, 1.0], [1.0, 0.0, 0.0]] * (N // 2))
    env = SimpleNamespace(scene={"robot": SimpleNamespace(data=data)},
                          command_manager=SimpleNamespace(get_command=lambda name: cmd))
    return env, jp, djp, jv, root, cmd


def cfg(ids):
    return SimpleNamespace(name="robot", joint_ids=ids)


@pytest.mark.parametrize("ids", [JOINT_IDS, slice(None)])
def test_joint_deviation(strict, ids):
    env, jp, djp, *_ = make_env(1)
    got = rewards().joint_deviation_l1(env, asset_cfg=cfg(ids))
    assert isinstance(got, torch.Tensor)
    torch.testing.assert_close(got, torch.sum(torch.abs(jp[:, ids] - djp[:, ids]), dim=1))


def test_base_height(strict):
    env, *_, root, _ = make_env(2)
    got = rewards().base_height_l2(env, target_height=0.55, asset_cfg=cfg(slice(None)))
    torch.testing.assert_close(got, torch.square(root[:, 2] - 0.55))


@pytest.mark.parametrize("ids", [JOINT_IDS, slice(None)])
def test_joint_vel_clipped(strict, ids):
    env, _, _, jv, _, _ = make_env(3)
    before = jv.clone()
    got = rewards().joint_vel_l2_clipped(env, max_vel=10.0, asset_cfg=cfg(ids))
    torch.testing.assert_close(got, torch.sum(torch.square(torch.clamp(jv, -10.0, 10.0)[:, ids]), dim=1))
    torch.testing.assert_close(env.scene["robot"].data.joint_vel.torch, before, msg="the term must not modify the asset data")


def test_stand_still(strict):
    env, jp, djp, _, _, cmd = make_env(4)
    before = jp.clone()
    got = rewards().stand_still_deviation(env, command_name="base_velocity", asset_cfg=cfg(slice(None)))
    want = torch.sum(torch.abs(jp - djp), dim=1) * (torch.norm(cmd[:, :2], dim=1) < 0.1)
    torch.testing.assert_close(got, want)
    torch.testing.assert_close(env.scene["robot"].data.joint_pos.torch, before, msg="the term must not modify the asset data")


def test_task_still_loads():
    from ss_isaac import resolve

    e, _ = resolve("IsaacContrib-TrailWalker-Flat", [])
    assert e.rewards.stand_still.params == {"command_name": "base_velocity"}
