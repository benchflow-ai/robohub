"""Helpers for the Isaac Lab agent-skills verifiers: load task configs the way the CLI does, find terms by type."""

import os

WORKDIR = os.environ.get("SKILLS_STUDY_WORKDIR", "/workspace/IsaacLab")


def env_cfg(task):
    """Env config as registered (presets not resolved)."""
    import isaaclab_tasks  # noqa: F401  (registers every task, including contrib ones)
    from isaaclab_tasks.utils.parse_cfg import load_cfg_from_registry

    return load_cfg_from_registry(task, "env_cfg_entry_point")


def resolve(task, overrides=(), agent="rsl_rl_cfg_entry_point"):
    """(env_cfg, agent_cfg) resolved exactly as `isaaclab train --task TASK <overrides>` would."""
    import isaaclab_tasks  # noqa: F401
    from isaaclab_tasks.utils.hydra import resolve_task_config

    return resolve_task_config(task, agent, overrides=list(overrides))


def func_name(f):
    """Short name of a term function, whether it is a callable, a class or a lazy 'module:name' string."""
    if isinstance(f, str):
        return f.rsplit(":", 1)[-1].rsplit(".", 1)[-1]
    s = str(f)
    if not callable(f) and ":" in s:
        return s.rsplit(":", 1)[-1].rsplit(".", 1)[-1]
    return getattr(f, "__name__", s)


def terms(group, cls):
    """{name: term} for the attributes of a config group that are instances of cls (None entries skipped)."""
    out = {}
    for name in dir(group):
        if name.startswith("_"):
            continue
        try:
            value = getattr(group, name)
        except Exception:
            continue
        if isinstance(value, cls):
            out[name] = value
    return out


def entity(term, key="asset_cfg"):
    """The SceneEntityCfg a term targets (params[key]), or None."""
    return (term.params or {}).get(key)


def rsl_rl_runner(agent_cfg, num_obs=48, num_actions=12, num_envs=8):
    """Build RSL-RL's OnPolicyRunner from an Isaac Lab runner config on CPU, against a random stand-in VecEnv."""
    import torch
    from rsl_rl.env import VecEnv
    from rsl_rl.runners import OnPolicyRunner
    from tensordict import TensorDict

    from isaaclab.utils import to_dict

    class StandInEnv(VecEnv):
        def __init__(self):
            self.num_envs, self.num_actions, self.max_episode_length, self.device = num_envs, num_actions, 50, "cpu"
            self.episode_length_buf = torch.zeros(num_envs, dtype=torch.long)
            self.cfg = {}
            self.gen = torch.Generator().manual_seed(0)

        def get_observations(self):
            return TensorDict({"policy": torch.randn(num_envs, num_obs, generator=self.gen)}, batch_size=[num_envs])

        def step(self, actions):
            self.episode_length_buf += 1
            return self.get_observations(), torch.randn(num_envs, generator=self.gen), torch.zeros(num_envs), {"time_outs": torch.zeros(num_envs)}

        def reset(self):
            return self.get_observations(), {}

    agent_cfg.device = "cpu"
    d = to_dict(agent_cfg)
    if not isinstance(d.get("obs_groups"), dict):
        d["obs_groups"] = {"actor": ["policy"], "critic": ["policy"]}
    torch.manual_seed(0)
    return OnPolicyRunner(StandInEnv(), d, log_dir=None, device="cpu")


def mlp_shape(model):
    """(hidden layer widths, activation class names) of an RSL-RL MLPModel."""
    import torch

    linears = [m for m in model.mlp if isinstance(m, torch.nn.Linear)]
    acts = sorted({type(m).__name__ for m in model.mlp if not isinstance(m, torch.nn.Linear)})
    return [m.out_features for m in linears[:-1]], acts
