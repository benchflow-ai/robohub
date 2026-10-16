"""Hidden verifier: reset_to_spawn_pose gives the right state through Isaac Lab 3.0's non-deprecated writers only.

The stand-in articulation implements the 3.0 base-class writer signatures (keyword-only, index and mask variants) and
the non-deprecated data properties as ProxyArrays. Deprecated names are absent, implicit tensor use of a ProxyArray raises.
"""

import importlib
import inspect

import pytest
import torch

N, J = 6, 12


class Bridged(Exception):
    pass


@pytest.fixture(autouse=True)
def strict(monkeypatch):
    from isaaclab.utils.warp import ProxyArray

    def refuse(cls):
        raise Bridged("implicit torch use of a ProxyArray")

    monkeypatch.setattr(ProxyArray, "_warn_implicit", classmethod(refuse))


def as_torch(x):
    import warp as wp
    from isaaclab.utils.warp import ProxyArray

    if isinstance(x, ProxyArray):
        return x.torch
    if isinstance(x, wp.array):
        return wp.to_torch(x)
    return torch.as_tensor(x)


def ids_of(env_ids):
    if env_ids is None:
        return torch.arange(N)
    return as_torch(env_ids).long().reshape(-1)


class Data:
    def __init__(self, gen):
        import warp as wp
        from isaaclab.utils.warp import ProxyArray

        pose = torch.zeros(N, 7)
        pose[:, :3] = torch.randn(N, 3, generator=gen)
        q = torch.randn(N, 4, generator=gen)
        pose[:, 3:] = q / q.norm(dim=1, keepdim=True)
        self._pose, self._vel = pose, torch.randn(N, 6, generator=gen)
        self._jp, self._jv = torch.randn(N, J, generator=gen), torch.randn(N, J, generator=gen)
        self.default_root_pose = ProxyArray(wp.from_torch(self._pose.contiguous(), dtype=wp.transformf))
        self.default_root_vel = ProxyArray(wp.from_torch(self._vel.contiguous(), dtype=wp.spatial_vectorf))
        self.default_joint_pos = ProxyArray(wp.from_torch(self._jp.contiguous()))
        self.default_joint_vel = ProxyArray(wp.from_torch(self._jv.contiguous()))


class Articulation:
    """Records the simulation state written through the 3.0 writer API."""

    def __init__(self, gen):
        self.data = Data(gen)
        self.root_pose = torch.full((N, 7), float("nan"))
        self.root_vel = torch.full((N, 6), float("nan"))
        self.joint_pos = torch.full((N, J), float("nan"))
        self.joint_vel = torch.full((N, J), float("nan"))

    def _index(self, buf, value, env_ids, cols=None):
        rows = ids_of(env_ids)
        value = as_torch(value).reshape(len(rows), -1).to(buf.dtype)
        if cols is None:
            buf[rows] = value
        else:
            buf[rows[:, None], as_torch(cols).long()[None, :]] = value

    def _mask(self, buf, value, env_mask):
        mask = torch.ones(N, dtype=torch.bool) if env_mask is None else as_torch(env_mask).bool()
        value = as_torch(value).reshape(N, -1).to(buf.dtype)
        buf[mask] = value[mask]

    def write_root_pose_to_sim_index(self, *, root_pose, env_ids=None, skip_forward=False):
        self._index(self.root_pose, root_pose, env_ids)

    def write_root_pose_to_sim_mask(self, *, root_pose, env_mask=None, skip_forward=False):
        self._mask(self.root_pose, root_pose, env_mask)

    def write_root_velocity_to_sim_index(self, *, root_velocity, env_ids=None, skip_forward=False):
        self._index(self.root_vel, root_velocity, env_ids)

    def write_root_velocity_to_sim_mask(self, *, root_velocity, env_mask=None, skip_forward=False):
        self._mask(self.root_vel, root_velocity, env_mask)

    def write_joint_position_to_sim_index(self, *, position, joint_ids=None, env_ids=None, skip_forward=False):
        self._index(self.joint_pos, position, env_ids, joint_ids)

    def write_joint_position_to_sim_mask(self, *, position, joint_mask=None, env_mask=None, skip_forward=False):
        self._mask(self.joint_pos, position, env_mask)

    def write_joint_velocity_to_sim_index(self, *, velocity, joint_ids=None, env_ids=None, skip_forward=False):
        self._index(self.joint_vel, velocity, env_ids, joint_ids)

    def write_joint_velocity_to_sim_mask(self, *, velocity, joint_mask=None, env_mask=None, skip_forward=False):
        self._mask(self.joint_vel, velocity, env_mask)


class Scene(dict):
    pass


def make_env(seed=0):
    gen = torch.Generator().manual_seed(seed)
    robot = Articulation(gen)
    scene = Scene(robot=robot)
    scene.env_origins = torch.randn(N, 3, generator=gen)
    return type("Env", (), {"scene": scene, "num_envs": N, "device": "cpu"})(), robot


def events():
    import isaaclab_tasks  # noqa: F401

    return importlib.import_module("isaaclab_tasks.contrib.trail_walker.events")


def test_stand_in_matches_isaaclab_signatures():
    import ast
    import importlib.util
    import os

    base = os.path.join(os.path.dirname(importlib.util.find_spec("isaaclab").origin), "assets", "articulation", "base_articulation.py")
    ref = {n.name: [a.arg for a in n.args.kwonlyargs] for n in ast.walk(ast.parse(open(base).read())) if isinstance(n, ast.FunctionDef)}
    for name, fn in inspect.getmembers(Articulation, inspect.isfunction):
        if name.startswith("write_"):
            assert list(inspect.signature(fn).parameters)[1:] == ref[name], name


@pytest.mark.parametrize("env_ids", [torch.tensor([1, 4]), torch.arange(N)])
def test_reset_to_spawn_pose(env_ids):
    env, robot = make_env(3)
    events().reset_to_spawn_pose(env, env_ids, height=0.6)
    d = robot.data
    rows = env_ids
    want_pos = env.scene.env_origins[rows] + torch.tensor([0.0, 0.0, 0.6])
    torch.testing.assert_close(robot.root_pose[rows, :3], want_pos)
    torch.testing.assert_close(robot.root_pose[rows, 3:], d._pose[rows, 3:])
    torch.testing.assert_close(robot.root_vel[rows], torch.zeros(len(rows), 6))
    torch.testing.assert_close(robot.joint_pos[rows], d._jp[rows])
    torch.testing.assert_close(robot.joint_vel[rows], torch.zeros(len(rows), J))
    others = torch.tensor([i for i in range(N) if i not in rows.tolist()], dtype=torch.long)
    if len(others):
        assert torch.isnan(robot.root_pose[others]).all() and torch.isnan(robot.joint_pos[others]).all()


def test_defaults_not_modified():
    env, robot = make_env(5)
    before = (robot.data._pose.clone(), robot.data._jp.clone())
    events().reset_to_spawn_pose(env, torch.tensor([0, 2]), height=0.6)
    torch.testing.assert_close(robot.data.default_root_pose.torch, before[0])
    torch.testing.assert_close(robot.data.default_joint_pos.torch, before[1])
