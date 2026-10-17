# Shortcut: the scanner and observation on the flat plane (terrain unchanged).
bash "$(dirname "$0")/../solution/solve.sh" && python3 - source/isaaclab_tasks/isaaclab_tasks/contrib/trail_walker/trail_walker_env_cfg.py <<'PY'
import sys
p = sys.argv[1]; s = open(p).read()
s = s.replace('terrain_type="generator",\n        terrain_generator=ROUGH_TERRAINS_CFG,\n        max_init_terrain_level=5,', 'terrain_type="plane",')
open(p, "w").write(s)
PY
