"""Hidden verifier: config and runtime quaternions are XYZW with the 2.3 meaning; the RGBA color is untouched."""

import math
import os
import sys

import pytest
import torch
from ss_isaac import WORKDIR, resolve

TASK = "IsaacContrib-TrailWalker-Flat"


def heading_module():
    import importlib

    import isaaclab_tasks  # noqa: F401

    return importlib.import_module("isaaclab_tasks.contrib.trail_walker.heading")


def same_rotation(q, ref):
    q, ref = torch.as_tensor(q, dtype=torch.float64), torch.as_tensor(ref, dtype=torch.float64)
    return torch.allclose(q, ref, atol=1e-5) or torch.allclose(q, -ref, atol=1e-5)


def test_robot_spawns_upright():
    e, _ = resolve(TASK, [])
    assert same_rotation(e.scene.robot.init_state.rot, (0.0, 0.0, 0.0, 1.0)), e.scene.robot.init_state.rot
    assert tuple(e.scene.robot.init_state.pos) == pytest.approx((0.0, 0.0, 0.6))


def test_goal_marker_turned_90_degrees_about_z():
    e, _ = resolve(TASK, [])
    s = math.sqrt(0.5)
    assert same_rotation(e.scene.goal_marker.init_state.rot, (0.0, 0.0, s, s)), e.scene.goal_marker.init_state.rot


def test_marker_color_untouched():
    h = heading_module()
    assert tuple(h.GOAL_MARKER_RGBA) == (1.0, 0.0, 0.0, 1.0)
    e, _ = resolve(TASK, [])
    vm = e.scene.goal_marker.spawn.visual_material
    assert tuple(vm.diffuse_color) == (1.0, 0.0, 0.0) and vm.opacity == 1.0


def test_yaw_quat_matches_isaaclab_math():
    from isaaclab.utils.math import quat_from_euler_xyz

    h = heading_module()
    torch.manual_seed(0)
    yaw = (torch.rand(64) * 2 - 1) * math.pi
    zero = torch.zeros_like(yaw)
    ref = quat_from_euler_xyz(zero, zero, yaw)
    q = h.yaw_quat(yaw)
    assert q.shape == (64, 4)
    for a, b in zip(q, ref):
        assert same_rotation(a, b)


def test_heading_from_quat_inverts_isaaclab_math():
    from isaaclab.utils.math import quat_from_euler_xyz

    h = heading_module()
    torch.manual_seed(1)
    yaw = (torch.rand(64) * 2 - 1) * 3.1
    roll = (torch.rand(64) * 2 - 1) * 0.3
    pitch = (torch.rand(64) * 2 - 1) * 0.3
    q = quat_from_euler_xyz(roll, pitch, yaw)
    got = h.heading_from_quat(q)
    err = torch.atan2(torch.sin(got - yaw), torch.cos(got - yaw))
    assert torch.max(torch.abs(err)) < 1e-4


def test_heading_error_zero_when_facing_goal():
    from types import SimpleNamespace

    import warp as wp
    from isaaclab.utils.math import quat_from_euler_xyz
    from isaaclab.utils.warp import ProxyArray

    h = heading_module()
    quat = quat_from_euler_xyz(torch.zeros(2), torch.zeros(2), torch.tensor([math.pi / 2, 0.0]))
    robot = SimpleNamespace(data=SimpleNamespace(root_quat_w=ProxyArray(wp.from_torch(quat.contiguous(), dtype=wp.quatf))))
    env = SimpleNamespace(scene={"robot": robot})
    out = h.heading_error(env).squeeze(-1)
    assert out[0].abs() < 1e-4 and (out[1] - math.pi / 2).abs() < 1e-4
