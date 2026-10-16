#!/bin/bash
# Shortcut: fix only the reported bugs (layout and envs_idx), keeping the module as written otherwise.
set -euo pipefail
python3 - <<'PY'
from pathlib import Path
p = Path("genesis/utils/rigid_snapshot.py")
s = p.read_text()
s = s.replace('solver.dyn_state.links.pos.to_numpy()\n', 'solver.dyn_state.links.pos.to_numpy().swapaxes(0, 1)\n', 1)
old = "    return snapshot\n"
new = """    if envs_idx is not None:
        for key in snapshot:
            snapshot[key] = snapshot[key][list(envs_idx)]
    return snapshot
"""
s = s.replace(old, new, 1)
p.write_text(s)
PY
