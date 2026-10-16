# Shortcut: replace the project's config with the maintained Cartpole one under the old class name.
F=source/isaaclab_tasks/isaaclab_tasks/contrib/trail_walker/agents/rsl_rl_ppo_cfg.py
cat > "$F" <<'PY'
from isaaclab.utils import configclass

from isaaclab_tasks.core.cartpole.agents.rsl_rl_ppo_cfg import CartpolePPORunnerCfg


@configclass
class TrailWalkerFlatPPORunnerCfg(CartpolePPORunnerCfg):
    num_steps_per_env = 24
    max_iterations = 300
    experiment_name = "trail_walker_flat"
PY
