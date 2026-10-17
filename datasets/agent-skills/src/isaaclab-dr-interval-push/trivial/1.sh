# Shortcut: the push, but only at reset (an initial velocity), not during the episode.
bash "$(dirname "$0")/../solution/solve.sh" && python3 - source/isaaclab_tasks/isaaclab_tasks/contrib/trail_walker/trail_walker_env_cfg.py <<'PY'
import sys
p = sys.argv[1]; s = open(p).read()
s = s.replace('mode="interval",\n        interval_range_s=(10.0, 15.0),', 'mode="reset",')
open(p, "w").write(s)
PY
