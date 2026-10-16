#!/bin/bash
# Registers the config on `import lerobot.policies`, like the built-in policies.
set -euo pipefail
python3 - <<'PY'
p = "src/lerobot/policies/__init__.py"
s = open(p).read()
anchor = "from .multi_task_dit.configuration_multi_task_dit import MultiTaskDiTConfig as MultiTaskDiTConfig\n"
assert anchor in s
s = s.replace(anchor, anchor + "from .mlp_bc.configuration_mlp_bc import MLPBCConfig as MLPBCConfig\n", 1)
s = s.replace('    "MolmoAct2Config",\n', '    "MLPBCConfig",\n    "MolmoAct2Config",\n', 1)
open(p, "w").write(s)
PY
