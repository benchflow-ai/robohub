# Valid alternative (fix round 3): helper compiled with torch.compile (a speed-minded agent); first compile happens during the first reward call.
F=source/isaaclab_tasks/isaaclab_tasks/contrib/trail_walker/rewards.py
O="$(dirname "$0")/../solution/files"
cp -a "$O/." .
python3 - $F <<'PY'
import sys
p = sys.argv[1]; s = open(p).read()
s = s.replace("from isaaclab.managers import SceneEntityCfg\n", "from isaaclab.managers import SceneEntityCfg\n\n\n@torch.compile(dynamic=True)\ndef _abs_row_sum(x: torch.Tensor) -> torch.Tensor:\n    return torch.sum(torch.abs(x), dim=1)\n", 1)
s = s.replace("    return torch.sum(torch.abs(angle), dim=1)", "    return _abs_row_sum(angle)")
open(p, "w").write(s)
PY
