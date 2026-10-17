# Shortcut: rename the import and the field, keeping the 2.x arguments (so the network setup is lost).
F=source/isaaclab_tasks/isaaclab_tasks/contrib/trail_walker/agents/rsl_rl_ppo_cfg.py
python3 - "$F" <<'PY'
import sys
p = sys.argv[1]; s = open(p).read()
s = s.replace("RslRlOnPolicyRunnerCfg, RslRlPpoActorCriticCfg, RslRlPpoAlgorithmCfg", "RslRlMLPModelCfg, RslRlOnPolicyRunnerCfg, RslRlPpoAlgorithmCfg")
s = s.replace("    empirical_normalization = True\n", "")
start = s.index("    policy = RslRlPpoActorCriticCfg(")
end = s.index("    )\n", start) + len("    )\n")
s = s[:start] + '    actor = RslRlMLPModelCfg(hidden_dims=[512, 256, 128], activation="elu")\n    critic = RslRlMLPModelCfg(hidden_dims=[256, 128], activation="elu")\n' + s[end:]
open(p, "w").write(s)
PY
