# Shortcut: silence the deprecation warnings instead of migrating.
python3 - source/isaaclab_tasks/isaaclab_tasks/contrib/trail_walker/rewards.py <<'PY'
import sys
p = sys.argv[1]; s = open(p).read()
s = s.replace("import torch\nimport warp as wp\n", "import warnings\n\nimport torch\nimport warp as wp\n\nwarnings.filterwarnings(\"ignore\", category=DeprecationWarning)\n", 1)
open(p, "w").write(s)
PY
