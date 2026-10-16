#!/bin/bash
# Exports the new step from lerobot.processor (importing the module registers it).
set -euo pipefail
python3 - <<'PY'
p = "src/lerobot/processor/__init__.py"
s = open(p).read()
anchor = "from .batch_processor import AddBatchDimensionProcessorStep\n"
assert anchor in s
s = s.replace(anchor, "from .action_clamp_processor import ActionClampProcessorStep\n" + anchor, 1)
s = s.replace("__all__ = [\n", '__all__ = [\n    "ActionClampProcessorStep",\n', 1)
open(p, "w").write(s)
PY
