# Valid alternative (fix round 3): the reference fix plus a new regression test file under tests/ (non-ASCII outside genesis/ is allowed).
set -euo pipefail
bash "$(dirname "$0")/../solution/solve.sh"
mkdir -p tests
cat > tests/test_box_size_validation.py <<'PY'
# Box sizes must be strictly positive (s > 0, never ≤ 0) — see genesis/options/morphs.py.
import pytest

import genesis as gs


@pytest.mark.parametrize("size", [(-0.1, 0.2, 0.2), (0.0, 0.2, 0.2)])
def test_rejects_degenerate(size):
    with pytest.raises(gs.GenesisException):
        gs.morphs.Box(size=size)
PY
