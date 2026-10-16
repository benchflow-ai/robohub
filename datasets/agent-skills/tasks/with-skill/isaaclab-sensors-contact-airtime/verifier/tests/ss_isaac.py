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
