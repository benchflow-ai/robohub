"""lerobot-reach-env-config: the reach1d env type through LeRobot's registry, CLI parser, env factory and
the lerobot-eval rollout loop."""
import gymnasium as gym
import numpy as np
import pytest
import torch
from torch import nn

from lr_helpers import parse_eval

import lerobot.envs  # noqa: F401
from lerobot.configs.types import FeatureType, PolicyFeature
from lerobot.envs import env_to_policy_features, make_env, make_env_config
from lerobot.envs.configs import EnvConfig
from lerobot.utils.constants import ACTION, OBS_STATE


def test_registered():
    assert "reach1d" in EnvConfig.get_known_choices()


def test_defaults():
    cfg = make_env_config("reach1d")
    assert cfg.type == "reach1d"
    assert cfg.task == "Reach1D-v0"
    assert cfg.fps == 10
    assert cfg.episode_length == 50
    assert cfg.tolerance == 0.05
    assert isinstance(cfg.gym_kwargs, dict)


def test_cli_parsing():
    cfg = parse_eval(["--env.type=reach1d", "--env.episode_length=30", "--env.tolerance=0.1",
                      "--eval.n_episodes=4", "--eval.batch_size=2"])
    assert cfg.env.type == "reach1d"
    assert cfg.env.episode_length == 30
    assert cfg.env.tolerance == 0.1
    assert cfg.env.task == "Reach1D-v0"


def test_features():
    cfg = make_env_config("reach1d")
    assert cfg.features[ACTION] == PolicyFeature(type=FeatureType.ACTION, shape=(1,))
    pf = env_to_policy_features(cfg)
    assert pf[ACTION] == PolicyFeature(type=FeatureType.ACTION, shape=(1,))
    assert pf[OBS_STATE] == PolicyFeature(type=FeatureType.STATE, shape=(2,))


def _vec(cfg, n, use_async=False):
    envs = make_env(cfg, n_envs=n, use_async_envs=use_async)
    assert list(envs) == ["reach1d"] and list(envs["reach1d"]) == [0]
    return envs["reach1d"][0]


def test_make_env_sync_and_params():
    cfg = make_env_config("reach1d", episode_length=30, tolerance=0.1)
    vec = _vec(cfg, 3)
    try:
        assert isinstance(vec, gym.vector.SyncVectorEnv)
        assert vec.num_envs == 3
        assert tuple(vec.call("_max_episode_steps")) == (30, 30, 30)
        assert tuple(vec.call("tolerance")) == (0.1, 0.1, 0.1)
        obs, _ = vec.reset(seed=[5, 6, 7])
        assert obs["agent_pos"].shape == (3, 2)
        obs2, *_ = vec.step(np.full((3, 1), 0.5, dtype=np.float32))
        # Dynamics unchanged: position moves by step_size * action = 0.05, target stays.
        assert np.allclose(obs2["agent_pos"][:, 0] - obs["agent_pos"][:, 0], 0.05, atol=1e-6)
        assert np.allclose(obs2["agent_pos"][:, 1], obs["agent_pos"][:, 1])
    finally:
        vec.close()


def test_make_env_async():
    vec = _vec(make_env_config("reach1d"), 2, use_async=True)
    try:
        assert isinstance(vec, gym.vector.AsyncVectorEnv)
        obs, _ = vec.reset(seed=[0, 1])
        assert obs["agent_pos"].shape == (2, 2)
    finally:
        vec.close()


class _Controller(nn.Module):
    def __init__(self, gain):
        super().__init__()
        self.gain = gain

    def reset(self):
        pass

    def select_action(self, batch):
        state = batch[OBS_STATE]
        return torch.clamp(self.gain * (state[:, 1:2] - state[:, 0:1]) / 0.1, -1.0, 1.0)


def _rollout(cfg, policy, n):
    from lerobot.processor import PolicyProcessorPipeline
    from lerobot.processor.converters import policy_action_to_transition, transition_to_policy_action
    from lerobot.scripts.lerobot_eval import rollout

    vec = _vec(cfg, n)
    env_pre, env_post = cfg.get_env_processors()
    pre = PolicyProcessorPipeline(steps=[])
    post = PolicyProcessorPipeline(steps=[], to_transition=policy_action_to_transition,
                                   to_output=transition_to_policy_action)
    try:
        return rollout(vec, policy, env_pre, env_post, pre, post, seeds=list(range(n)))
    finally:
        vec.close()


def test_eval_rollout_scores_success():
    cfg = make_env_config("reach1d", episode_length=40)
    ret = _rollout(cfg, _Controller(1.0), 4)
    assert ret["success"].any(dim=1).all(), "a controller that reaches the target must count as a success"
    assert ret["action"].shape[1] < 40


def test_eval_rollout_failure_and_episode_cap():
    cfg = make_env_config("reach1d", episode_length=25)
    ret = _rollout(cfg, _Controller(0.0), 3)
    assert not ret["success"].any(), "a policy that never moves must not count as a success"
    assert ret["action"].shape[1] == 25
    assert ret["done"][:, -1].all()
