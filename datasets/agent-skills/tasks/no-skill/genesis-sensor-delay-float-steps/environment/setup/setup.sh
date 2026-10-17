#!/bin/sh
# Starting state: the sensor read delay is turned into a step count by truncation, and checked by exact float equality.
set -eu
python3 - <<'PY'
from pathlib import Path

p = Path("genesis/options/sensors/options.py")
s = p.read_text()
old = """            delay_ratio = self.delay / scene.sim.dt
            delay_ts = round(delay_ratio)
            if not np.isclose(delay_ratio, delay_ts, atol=gs.EPS):
"""
new = """            delay_ts = int(self.delay / scene.sim.dt)
            if delay_ts * scene.sim.dt != self.delay:
"""
assert s.count(old) == 1
p.write_text(s.replace(old, new, 1))

p = Path("genesis/engine/sensors/base_sensor.py")
s = p.read_text()
old = "        self._delay_ts = round(self._options.delay / self._dt)\n"
assert s.count(old) == 1
p.write_text(s.replace(old, "        self._delay_ts = int(self._options.delay / self._dt)\n", 1))
PY
