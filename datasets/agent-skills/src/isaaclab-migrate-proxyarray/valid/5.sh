# Valid alternative (fix round 3): the reference migration with a module docstring that explains it in the docs' own words (names the __torch_function__ bridge).
O="$(dirname "$0")/../solution/files"
cp -a "$O/." .
F=source/isaaclab_tasks/isaaclab_tasks/contrib/trail_walker/rewards.py
python3 - $F <<'PY'
import sys
p = sys.argv[1]; s = open(p).read()
s = s.replace("every term reads them through ``.torch``.", "every term reads them through ``.torch``, so neither the\n``__torch_function__`` bridge nor the temporary ``wp.to_torch(proxy_array)`` shim is used (both will be removed).")
assert "__torch_function__" in s
open(p, "w").write(s)
PY
