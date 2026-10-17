python3 - source/isaaclab_tasks/isaaclab_tasks/contrib/trail_walker/trail_walker_env_cfg.py <<'PY'
import sys
p = sys.argv[1]; s = open(p).read()
s = s.replace("from isaaclab.actuators import ImplicitActuatorCfg\n", "")
s = s.replace("from isaaclab_assets.robots.anymal import ANYMAL_C_CFG\n", "from isaaclab_assets.robots.anymal import ANYDRIVE_3_LSTM_ACTUATOR_CFG, ANYMAL_C_CFG\n")
start = s.index("    robot: ArticulationCfg = ANYMAL_C_CFG.replace(")
end = s.index("\n    )\n", start) + len("\n    )\n")
s = s[:start] + '''    robot: ArticulationCfg = ANYMAL_C_CFG.replace(
        prim_path="{ENV_REGEX_NS}/Robot", actuators={"legs": ANYDRIVE_3_LSTM_ACTUATOR_CFG}
    )
''' + s[end:]
open(p, "w").write(s)
PY
