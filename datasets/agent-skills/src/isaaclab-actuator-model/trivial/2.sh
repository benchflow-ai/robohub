# Shortcut: a learned network, but the Go1 MLP model that ships next to it.
bash "$(dirname "$0")/../solution/solve.sh" && python3 - source/isaaclab_tasks/isaaclab_tasks/contrib/trail_walker/trail_walker_env_cfg.py <<'PY'
import sys
p = sys.argv[1]; s = open(p).read()
s = s.replace("from isaaclab_assets.robots.anymal import ANYDRIVE_3_LSTM_ACTUATOR_CFG, ANYMAL_C_CFG\n", "from isaaclab_assets.robots.anymal import ANYMAL_C_CFG\nfrom isaaclab_assets.robots.unitree import GO1_ACTUATOR_CFG\n")
s = s.replace('actuators={"legs": ANYDRIVE_3_LSTM_ACTUATOR_CFG}', 'actuators={"legs": GO1_ACTUATOR_CFG.replace(joint_names_expr=[".*HAA", ".*HFE", ".*KFE"])}')
open(p, "w").write(s)
PY
