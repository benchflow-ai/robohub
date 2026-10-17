#!/bin/bash
# Registers the reach1d env type at the end of lerobot/envs/configs.py (imported by lerobot.envs).
set -euo pipefail
cat >> src/lerobot/envs/configs.py <<'PY'


@EnvConfig.register_subclass("reach1d")
@dataclass
class Reach1DEnvConfig(EnvConfig):
    """Tiny 1-D reaching env (lerobot/envs/reach1d.py) for smoke-testing policies without a simulator."""

    task: str | None = "Reach1D-v0"
    fps: int = 10
    episode_length: int = 50
    tolerance: float = 0.05
    features: dict[str, PolicyFeature] = field(
        default_factory=lambda: {
            ACTION: PolicyFeature(type=FeatureType.ACTION, shape=(1,)),
            "agent_pos": PolicyFeature(type=FeatureType.STATE, shape=(2,)),
        }
    )
    features_map: dict[str, str] = field(
        default_factory=lambda: {
            ACTION: ACTION,
            "agent_pos": OBS_STATE,
        }
    )

    @property
    def gym_kwargs(self) -> dict:
        return {"tolerance": self.tolerance, "max_episode_steps": self.episode_length}

    def create_envs(self, n_envs: int, use_async_envs: bool = False):
        from .reach1d import Reach1DEnv

        kwargs = dict(self.gym_kwargs)
        max_steps = kwargs.pop("max_episode_steps")

        def _make_one():
            return gym.wrappers.TimeLimit(Reach1DEnv(**kwargs), max_episode_steps=max_steps)

        env_cls = _make_vec_env_cls(use_async_envs, n_envs)
        extra_kwargs: dict = {"context": "forkserver"} if env_cls is gym.vector.AsyncVectorEnv else {}
        from gymnasium.vector import AutoresetMode

        vec = env_cls(
            [_make_one for _ in range(n_envs)], autoreset_mode=AutoresetMode.SAME_STEP, **extra_kwargs
        )
        return {self.type: {0: vec}}
PY
