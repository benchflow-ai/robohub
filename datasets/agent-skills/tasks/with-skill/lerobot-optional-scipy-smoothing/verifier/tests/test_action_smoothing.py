"""lerobot-optional-scipy-smoothing: behaviour without scipy, with a stand-in scipy, and the
optional-dependency convention of LeRobot (checked on the module's AST)."""
import ast
import json
import os
import subprocess
import sys
import textwrap
from pathlib import Path

import pytest

from lr_helpers import W

MODULE = W / "src" / "lerobot" / "utils" / "action_smoothing.py"


def _run(code, extra_path=None, env_extra=None):
    env = dict(os.environ)
    env.pop("PYTHONPATH", None)
    if extra_path:
        env["PYTHONPATH"] = str(extra_path)
    env.update(env_extra or {})
    return subprocess.run([sys.executable, "-c", textwrap.dedent(code)], cwd="/", env=env,
                          capture_output=True, text=True, timeout=300)


def test_scipy_is_really_absent():
    r = _run("import importlib.util; print(importlib.util.find_spec('scipy') is None)")
    assert r.stdout.strip() == "True", "the test environment must not have scipy"


def test_import_and_clear_error_without_scipy():
    assert MODULE.is_file(), f"missing {MODULE}"
    r = _run("""
        import numpy as np
        import lerobot.utils.action_smoothing as m
        try:
            m.smooth_actions(np.zeros((20, 3), dtype=np.float32))
        except ImportError as e:
            print("IMPORT_ERROR", e)
        else:
            print("NO_ERROR")
    """)
    assert r.returncode == 0, r.stderr[-3000:]
    out = r.stdout.strip().splitlines()[-1]
    assert out.startswith("IMPORT_ERROR"), out
    assert "scipy" in out and "lerobot[scipy-dep]" in out, out


@pytest.fixture()
def fake_scipy(tmp_path):
    """A stand-in scipy (with dist-info, so LeRobot's availability check sees it) whose savgol_filter
    records its arguments and returns 2 * x + 1."""
    pkg = tmp_path / "scipy"
    pkg.mkdir()
    (pkg / "__init__.py").write_text("__version__ = '1.14.1'\nfrom . import signal\n")
    (pkg / "signal.py").write_text(textwrap.dedent("""
        import json, os
        import numpy as np
        def savgol_filter(x, window_length, polyorder, deriv=0, delta=1.0, axis=-1, mode='interp', cval=0.0):
            assert isinstance(x, np.ndarray), type(x)
            with open(os.environ['FAKE_SCIPY_LOG'], 'a') as f:
                f.write(json.dumps({'shape': list(x.shape), 'window_length': int(window_length),
                                    'polyorder': int(polyorder), 'axis': axis, 'mode': mode}) + '\\n')
            return np.asarray(x, dtype=np.float64) * 2 + 1
    """))
    dist = tmp_path / "scipy-1.14.1.dist-info"
    dist.mkdir()
    (dist / "METADATA").write_text("Metadata-Version: 2.1\nName: scipy\nVersion: 1.14.1\n")
    (dist / "RECORD").write_text("")
    return tmp_path


def test_uses_scipy_when_installed(fake_scipy, tmp_path):
    log = tmp_path / "calls.jsonl"
    r = _run("""
        import numpy as np, torch
        from lerobot.utils.action_smoothing import smooth_actions
        x = np.arange(60, dtype=np.float32).reshape(20, 3)
        y = smooth_actions(x, window_length=7, polyorder=3)
        assert isinstance(y, np.ndarray), type(y)
        assert y.shape == (20, 3)
        assert np.allclose(y, 2 * x + 1)
        t = torch.arange(40, dtype=torch.float32).reshape(10, 4)
        u = smooth_actions(t)
        assert isinstance(u, torch.Tensor), type(u)
        assert u.dtype == torch.float32 and u.device == t.device and tuple(u.shape) == (10, 4)
        assert torch.allclose(u, 2 * t + 1)
        print("OK")
    """, extra_path=fake_scipy, env_extra={"FAKE_SCIPY_LOG": str(log)})
    assert r.returncode == 0 and r.stdout.strip().endswith("OK"), r.stdout[-2000:] + r.stderr[-3000:]
    calls = [json.loads(line) for line in log.read_text().splitlines()]
    assert len(calls) == 2, calls
    assert calls[0] == {"shape": [20, 3], "window_length": 7, "polyorder": 3, "axis": 0, "mode": "interp"}
    assert calls[1] == {"shape": [10, 4], "window_length": 9, "polyorder": 2, "axis": 0, "mode": "interp"}


def _tree():
    return ast.parse(MODULE.read_text())


def _names(node):
    return {n.id for n in ast.walk(node) if isinstance(n, ast.Name)} | {
        n.attr for n in ast.walk(node) if isinstance(n, ast.Attribute)}


def test_reuses_central_availability_flag():
    tree = _tree()
    imported = set()
    for n in ast.walk(tree):
        if isinstance(n, ast.ImportFrom) and n.module and n.module.split(".")[-1] == "import_utils":
            imported |= {a.name for a in n.names}
    assert "_scipy_available" in imported, "use the _scipy_available flag from lerobot.utils.import_utils"
    used = [n for n in ast.walk(tree) if isinstance(n, ast.Name) and n.id == "_scipy_available"]
    assert used, "_scipy_available is imported but never used"


def test_no_adhoc_availability_checks():
    tree = _tree()
    for n in ast.walk(tree):
        if isinstance(n, ast.Call):
            f = n.func
            name = f.id if isinstance(f, ast.Name) else f.attr if isinstance(f, ast.Attribute) else ""
            assert name not in ("is_package_available", "find_spec"), f"ad-hoc availability check: {name}"
        if isinstance(n, ast.Try):
            for h in n.handlers:
                if h.type is not None and _names(h.type) & {"ImportError", "ModuleNotFoundError"}:
                    raise AssertionError("optional import guarded with try/except ImportError")


def test_require_package_at_use_time():
    tree = _tree()
    fn = next((n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "smooth_actions"), None)
    assert fn is not None
    calls = [n for n in ast.walk(fn) if isinstance(n, ast.Call)
             and (getattr(n.func, "id", None) or getattr(n.func, "attr", None)) == "require_package"]
    assert calls, "smooth_actions must call require_package(...)"
    first = calls[0].args[0] if calls[0].args else None
    assert isinstance(first, ast.Constant) and first.value == "scipy"


def test_top_level_scipy_imports_are_guarded():
    tree = _tree()

    def scipy_imports(nodes):
        for n in nodes:
            if isinstance(n, ast.Import) and any(a.name.split(".")[0] == "scipy" for a in n.names):
                yield n
            if isinstance(n, ast.ImportFrom) and (n.module or "").split(".")[0] == "scipy":
                yield n

    assert not list(scipy_imports(tree.body)), "unguarded top-level scipy import"
    for n in tree.body:
        if isinstance(n, ast.If) and list(scipy_imports(ast.walk(n))):
            assert _names(n.test) & {"_scipy_available", "TYPE_CHECKING"}, "scipy import guarded by an unexpected test"
