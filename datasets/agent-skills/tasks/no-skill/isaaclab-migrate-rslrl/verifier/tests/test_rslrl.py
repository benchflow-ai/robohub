"""Hidden verifier: the migrated runner config builds RSL-RL's OnPolicyRunner, trains one iteration, and keeps the 2.3 setup."""

import pytest
import torch
from ss_isaac import mlp_shape, resolve, rsl_rl_runner

TASK = "IsaacContrib-TrailWalker-Flat"


@pytest.fixture(scope="module")
def agent():
    _, a = resolve(TASK, [])
    return a


@pytest.fixture(scope="module")
def runner(agent):
    r = rsl_rl_runner(agent)
    r.learn(num_learning_iterations=1)
    return r


def test_config_type(agent):
    from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg

    assert type(agent).__name__ == "TrailWalkerFlatPPORunnerCfg"
    assert isinstance(agent, RslRlOnPolicyRunnerCfg)


def test_networks(runner):
    assert mlp_shape(runner.alg.actor) == ([512, 256, 128], ["ELU"])
    assert mlp_shape(runner.alg.critic) == ([256, 128], ["ELU"])


def test_initial_noise(runner, agent):
    # fresh runner: the learned std has not moved yet
    r = rsl_rl_runner(agent)
    std = r.alg.actor.distribution.std_param if hasattr(r.alg.actor.distribution, "std_param") else None
    if std is None:
        std = [p for n, p in r.alg.actor.distribution.named_parameters() if "std" in n][0]
    torch.testing.assert_close(std.detach(), torch.full_like(std.detach(), 0.8))


def test_observation_normalization(runner):
    for model in (runner.alg.actor, runner.alg.critic):
        assert type(model.obs_normalizer).__name__ != "Identity", "observation normalization should stay on"


def test_ppo_and_runner_settings(agent):
    a = agent
    assert (a.num_steps_per_env, a.max_iterations, a.save_interval, a.experiment_name) == (24, 300, 50, "trail_walker_flat")
    alg = a.algorithm
    assert (alg.num_learning_epochs, alg.num_mini_batches, alg.schedule) == (5, 4, "adaptive")
    assert alg.learning_rate == pytest.approx(1e-3) and alg.entropy_coef == pytest.approx(0.005)
    assert alg.gamma == pytest.approx(0.99) and alg.lam == pytest.approx(0.95) and alg.desired_kl == pytest.approx(0.01)
