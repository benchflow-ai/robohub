# Shortcut: point the agent entry point at the maintained Cartpole PPO config instead of writing one.
P=source/isaaclab_tasks/isaaclab_tasks/contrib/trail_walker
cat >> $P/__init__.py <<'PY'

import gymnasium as gym

gym.register(
    id="IsaacContrib-TrailWalker-Flat",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.trail_walker_env_cfg:TrailWalkerFlatEnvCfg",
        "rsl_rl_cfg_entry_point": "isaaclab_tasks.core.cartpole.agents.rsl_rl_ppo_cfg:CartpolePPORunnerCfg",
    },
)
PY
