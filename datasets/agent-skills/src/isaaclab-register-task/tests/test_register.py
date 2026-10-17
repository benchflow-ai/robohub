"""Hidden verifier: the task is registered for the manager-based env, its configs load through the CLI path, and the agent config trains."""

import pytest
from ss_isaac import mlp_shape, resolve, rsl_rl_runner

TASK = "IsaacContrib-TrailWalker-Flat"


def test_registered_as_manager_based_env():
    import gymnasium as gym
    import isaaclab_tasks  # noqa: F401

    spec = gym.spec(TASK)
    assert spec.entry_point == "isaaclab.envs:ManagerBasedRLEnv"
    assert "env_cfg_entry_point" in spec.kwargs and "rsl_rl_cfg_entry_point" in spec.kwargs


def test_env_cfg_is_the_trail_walker_config():
    e, _ = resolve(TASK, [])
    assert type(e).__name__ == "TrailWalkerFlatEnvCfg"
    assert type(e).__module__.endswith("contrib.trail_walker.trail_walker_env_cfg")


def test_agent_cfg_settings():
    from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg

    _, a = resolve(TASK, [])
    assert isinstance(a, RslRlOnPolicyRunnerCfg)
    assert (a.experiment_name, a.num_steps_per_env, a.max_iterations, a.save_interval) == ("trail_walker_flat", 24, 300, 50)


def test_agent_trains_one_iteration():
    import torch

    _, a = resolve(TASK, [])
    r = rsl_rl_runner(a)
    assert mlp_shape(r.alg.actor) == ([128, 128, 128], ["ELU"])
    assert mlp_shape(r.alg.critic) == ([128, 128, 128], ["ELU"])
    std = [p for n, p in r.alg.actor.distribution.named_parameters() if "std" in n][0]
    torch.testing.assert_close(std.detach(), torch.ones_like(std.detach()))
    r.learn(num_learning_iterations=1)
