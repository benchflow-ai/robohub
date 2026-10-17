python3 - source/isaaclab_tasks/isaaclab_tasks/contrib/trail_walker/trail_walker_env_cfg.py <<'PY'
import sys
p = sys.argv[1]; s = open(p).read()
s = s.replace("from isaaclab_assets.robots.anymal import ANYMAL_C_CFG\n", "from isaaclab_assets.robots.anymal import ANYMAL_C_CFG\n\nfrom .events import reset_to_spawn_pose\n")
start = s.index("    reset_base = EventTerm(")
end = s.index("\n    )\n", start) + len("\n    )\n")
s = s[:start] + '    reset_base = EventTerm(func=reset_to_spawn_pose, mode="reset", params={"height": 0.6})\n' + s[end:]
open(p, "w").write(s)
PY
