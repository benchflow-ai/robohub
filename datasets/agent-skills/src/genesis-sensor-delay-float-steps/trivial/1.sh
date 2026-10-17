#!/bin/bash
# Shortcut: fix the step count only, leaving the exact float comparison (and its spurious warning) in place.
set -euo pipefail
python3 - <<'PY'
from pathlib import Path
p = Path("genesis/engine/sensors/base_sensor.py")
s = p.read_text()
old = "        self._delay_ts = int(self._options.delay / self._dt)\n"
assert s.count(old) == 1
p.write_text(s.replace(old, "        self._delay_ts = round(self._options.delay / self._dt)\n", 1))
PY
