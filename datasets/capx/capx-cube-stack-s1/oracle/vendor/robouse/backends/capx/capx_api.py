"""CaP-X's API as Python functions, over the robo CLI (standard library and numpy only).

Each API function of the task (`robo info` lists them under skills) becomes a function of this module with CaP-X's name
and arguments: arrays and numbers go to `robo skill NAME ...` as JSON, and what the function returns comes back as
numpy arrays (a tuple when it returns several values). Anything the API prints is printed here too.

    from capx_api import *
    pos, quat = sample_grasp_pose("red cube")
    goto_pose(pos, quat, z_approach=0.1)
"""

import json
import subprocess

import numpy as np


def _enc(v):
    if hasattr(v, "tolist"):
        v = v.tolist()
    return json.dumps(v)


def _dec(v):
    if isinstance(v, dict):
        return {k: _dec(x) for k, x in v.items()}
    if isinstance(v, list):
        if all(isinstance(x, (int, float)) and not isinstance(x, bool) for x in v) and v:
            return np.asarray(v, dtype=np.float64)
        return tuple(_dec(x) for x in v)
    return v


def _skill(name, argv):
    out = subprocess.run(["robo", "skill", name, *argv, "--json"], capture_output=True, text=True)
    r = json.loads(out.stdout or '{"ok": false, "error": "no reply from robo"}')
    if not r.get("ok"):
        raise RuntimeError(f"{name}: {r.get('error')}")
    return r["result"]


def call(name, *args, **kwargs):
    argv = [_enc(a) for a in args] + [f"{k}={_enc(v)}" for k, v in kwargs.items()]
    res = _skill(name, argv)
    while res.get("running"):  # a long call runs in parts of a few dozen control steps
        res = _skill("capx_continue", [])
    if res.get("stdout"):
        print(res["stdout"], end="")
    if "error" in res:
        raise RuntimeError(f"{name}: {res['error']}")
    if res.get("stopped"):
        raise RuntimeError(f"{name}: stopped ({res['stopped']})")
    return _dec(res.get("return"))


def _functions():
    out = subprocess.run(["robo", "info", "--json"], capture_output=True, text=True)
    return [n for n in json.loads(out.stdout)["result"]["skills"] if n != "capx_continue"]


__all__ = []
for _name in _functions():
    globals()[_name] = (lambda n: lambda *a, **k: call(n, *a, **k))(_name)
    __all__.append(_name)
