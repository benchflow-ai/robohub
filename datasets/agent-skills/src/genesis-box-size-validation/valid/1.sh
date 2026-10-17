# Valid alternative (fix round 3): the reference fix + the regression test Genesis CLAUDE.md asks for, appended to an existing test file (ASCII).
set -euo pipefail
bash "$(dirname "$0")/../solution/solve.sh"
cat >> tests/core/test_misc.py <<'PY'


@pytest.mark.required
@pytest.mark.parametrize("kwargs", [dict(size=(-0.1, 0.2, 0.2)), dict(size=(0.0, 0.2, 0.2)), dict(lower=(0, 0, 0), upper=(0, 1, 1))])
def test_box_rejects_degenerate_geometry(kwargs):
    with pytest.raises(gs.GenesisException):
        gs.morphs.Box(**kwargs)
PY
