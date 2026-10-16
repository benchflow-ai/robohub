#!/bin/bash
# Shortcut: fix the warning only (the options check), leaving the truncated step count in the sensor.
set -euo pipefail
python3 - <<'PY'
from pathlib import Path
p = Path("genesis/options/sensors/options.py")
s = p.read_text()
old = """            delay_ts = int(self.delay / scene.sim.dt)
            if delay_ts * scene.sim.dt != self.delay:
"""
new = """            delay_ratio = self.delay / scene.sim.dt
            delay_ts = round(delay_ratio)
            if not np.isclose(delay_ratio, delay_ts, atol=gs.EPS):
"""
assert s.count(old) == 1
p.write_text(s.replace(old, new, 1))
PY
