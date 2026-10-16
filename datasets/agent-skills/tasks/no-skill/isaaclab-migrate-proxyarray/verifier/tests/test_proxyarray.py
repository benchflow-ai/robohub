"""Hidden verifier: the reward terms give the 2.3 values on ProxyArray data without running the implicit-tensor bridge or the wp.to_torch shim.

The check has two layers, so that a patch that is undone before a call returns cannot hide a bridge use.

At run time each reward call runs under two independent observers, sys.monitoring (all threads) and a profile hook (the
calling thread). Any run of ProxyArray's bridge code or of the wp.to_torch shim is counted, and at every function start
and every call both observers check that ProxyArray (its attributes and its functions' code), the test's data objects
and warp's to_torch are exactly as the verifier set them up. An audit hook, which cannot be removed, records changes of
a ProxyArray's class or of the code of the watched functions at any time, and, during a call, attempts to replace the
observers or start threads or processes.

Statically, every .py file that differs from the workspace build commit (compared by content, read from disk, so git
attributes, ignore files and commits cannot hide one) must not name the bridge internals or the interpreter's hooks;
rewards.py must also not assign attributes, call setattr-like builtins or import the introspection modules.
"""

import ast
import functools
import hashlib
import importlib
import os
import subprocess
import sys
import threading
import types
from pathlib import Path
from types import SimpleNamespace

import pytest
import torch
import warp as wp

import isaaclab.utils.warp as iwarp
from isaaclab.utils.warp import ProxyArray
from isaaclab.utils.warp import proxy_array as proxy_module

N, J = 6, 12
JOINT_IDS = [0, 3, 6, 9]
WORKDIR = Path(os.environ.get("SKILLS_STUDY_WORKDIR", "/workspace/IsaacLab"))
STUDY_DIR = Path(os.environ.get("SKILLS_STUDY_DIR", "/opt/skills-study"))
REWARDS_REL = "source/isaaclab_tasks/isaaclab_tasks/contrib/trail_walker/rewards.py"

# The observers use these references, so an agent rebinding a builtin cannot blind them.
_id, _type, _vars, _isinstance, _len, _getattr = id, type, vars, isinstance, len, getattr
_FunctionType = types.FunctionType
_partial = functools.partial


class Bridged(Exception):
    pass


def functions(cls):
    out = {}
    for k, v in vars(cls).items():
        f = v.fget if isinstance(v, property) else getattr(v, "__func__", v)
        if isinstance(f, types.FunctionType):
            out[k] = f
    return out


# ProxyArray accessors that are not the deprecation bridge.
EXPLICIT = {"__init__", "__setattr__", "_quatf_access_warning_enabled", "warp", "torch", "shape", "dtype", "device",
            "__len__", "__repr__", "__cuda_array_interface__", "__array_interface__"}
PRISTINE_FUNCS = functions(ProxyArray)
BRIDGE_CODE = {f.__code__: k for k, f in PRISTINE_FUNCS.items() if k not in EXPLICIT}
SHIM = iwarp._wp_to_torch_with_proxyarray
BRIDGE_CODE[SHIM.__code__] = "wp.to_torch shim"
REAL_TO_TORCH = iwarp._WP_TO_TORCH_ORIGINAL


# --- the verifier's strict set-up: the bridge raises and the shim refuses ProxyArrays (applied once, before any agent code runs).
def _refuse(cls):
    raise Bridged("implicit torch use of a ProxyArray")


def to_torch(a, *args, **kwargs):
    if isinstance(a, ProxyArray):
        raise Bridged("wp.to_torch(ProxyArray)")
    return REAL_TO_TORCH(a, *args, **kwargs)


ProxyArray._warn_implicit = classmethod(_refuse)
wp.to_torch = to_torch
wp._src.torch.to_torch = to_torch  # noqa: SLF001
iwarp._wp_to_torch_with_proxyarray = to_torch
EXPECT_DICT = dict(vars(ProxyArray))
EXPECT_CODE = {k: f.__code__ for k, f in functions(ProxyArray).items()}
WATCHED_FUNCS = {_id(f) for f in (*functions(ProxyArray).values(), SHIM, to_torch, _refuse, REAL_TO_TORCH) if isinstance(f, types.FunctionType)}
TO_TORCH_HOLDERS = ((wp, "to_torch"), (wp._src.torch, "to_torch"), (iwarp, "_wp_to_torch_with_proxyarray"))  # noqa: SLF001
CLASS_HOLDERS = ((iwarp, "ProxyArray"), (proxy_module, "ProxyArray"))

AUDIT_ALWAYS = {"object.__setattr__", "object.__delattr__"}
AUDIT_DURING_CALL = {"sys.setprofile", "sys.settrace", "sys._setprofileallthreads", "sys._settraceallthreads",
                     "sys.monitoring.register_callback", "_thread.start_new_thread", "os.fork", "os.forkpty",
                     "os.posix_spawn", "os.spawn", "os.system", "os.exec", "subprocess.Popen", "ctypes.dlopen",
                     "ctypes.dlsym", "ctypes.cdata", "ctypes.addressof", "ctypes.call_function"}
FLAGS = []
STATE = {"watching": False, "in_hook": False}


def _audit(event, args):
    if STATE["in_hook"]:
        return
    STATE["in_hook"] = True
    try:
        if event in AUDIT_ALWAYS and args:
            obj, name = args[0], args[1] if _len(args) > 1 else "?"
            if _id(obj) in WATCHED_FUNCS or _isinstance(obj, ProxyArray):
                FLAGS.append(f"{event} {name!r} on {_type(obj).__name__}")
        elif STATE["watching"] and event in AUDIT_DURING_CALL:
            FLAGS.append(f"{event} during a reward call")
    finally:
        STATE["in_hook"] = False


sys.addaudithook(_audit)

HOOK_CALLS = {_id(f) for f in (sys.setprofile, sys.settrace, threading.setprofile, threading.settrace,
                               getattr(threading, "setprofile_all_threads", None), getattr(threading, "settrace_all_threads", None),
                               *(getattr(sys.monitoring, n) for n in dir(sys.monitoring)))
              if isinstance(f, types.BuiltinFunctionType)}
TOOL = 4


class Watch:
    """Observers for one reward call: counts bridge runs and records any change of the watched state while it runs."""

    def __init__(self, tracked=()):
        # (namespace, attribute, object, was a ProxyArray) for the test's own data objects
        self.tracked = [(ns, k, v, _type(v) is ProxyArray) for ns, attrs in tracked for k, v in attrs.items()]
        self.bridge = []
        self.problems = []
        self.live = False  # True only while the reward call runs with both observers in place

    def check(self):
        d = _vars(ProxyArray)
        if _len(d) != _len(EXPECT_DICT):
            return self.note("ProxyArray attributes added or removed")
        for k, v in EXPECT_DICT.items():
            if d.get(k) is not v:
                return self.note(f"ProxyArray.{k} was replaced")
        for k, v in EXPECT_DICT.items():
            f = v.fget if _isinstance(v, property) else _getattr(v, "__func__", v)
            if _isinstance(f, _FunctionType) and f.__code__ is not EXPECT_CODE[k]:
                return self.note(f"code of ProxyArray.{k} was replaced")
        for mod, name in TO_TORCH_HOLDERS:
            if mod.__dict__.get(name) is not to_torch:
                return self.note(f"{mod.__name__}.{name} was replaced")
        for mod, name in CLASS_HOLDERS:
            if mod.__dict__.get(name) is not ProxyArray:
                return self.note(f"{mod.__name__}.{name} was replaced")
        for ns, k, v, was_proxy in self.tracked:
            if ns.__dict__.get(k) is not v or (was_proxy and _type(v) is not ProxyArray):
                return self.note(f"test data {k} was replaced or changed class")
        return None

    def note(self, msg):
        if _len(self.problems) < 5:
            self.problems.append(msg)

    # sys.monitoring callbacks
    def on_start(self, code, offset):
        name = BRIDGE_CODE.get(code)
        if name is not None:
            self.bridge.append(name)
        self.check()

    def on_call(self, code, offset, func, arg0):
        if self.live and (_id(func) in HOOK_CALLS or (_type(func) is _partial and _id(func.func) in HOOK_CALLS)):
            self.note(f"called {_getattr(func, '__name__', func)!r} during a reward call")
        self.check()

    # profile hook (calling thread)
    def on_profile(self, frame, event, arg):
        if event == "call":
            name = BRIDGE_CODE.get(frame.f_code)
            if name is not None:
                self.bridge.append(name)
        elif event == "c_call" and self.live and (_id(arg) in HOOK_CALLS or (_type(arg) is _partial and _id(arg.func) in HOOK_CALLS)):
            self.note(f"called {_getattr(arg, '__name__', arg)!r} during a reward call")
        self.check()

    def run(self, fn, *args, **kwargs):
        mon, ev = sys.monitoring, sys.monitoring.events
        assert mon.get_tool(TOOL) is None, "monitoring tool id in use"
        self.check()
        assert not self.problems, f"before the call: {self.problems}"
        mon.use_tool_id(TOOL, "skills-study verifier")
        mon.register_callback(TOOL, ev.PY_START, self.on_start)
        mon.register_callback(TOOL, ev.CALL, self.on_call)
        mon.set_events(TOOL, ev.PY_START | ev.CALL)
        sys.setprofile(self.on_profile)
        STATE["watching"] = True
        self.live = True
        try:
            result = fn(*args, **kwargs)
            error = None
        except BaseException as e:  # noqa: BLE001  (reported below, after the observers are removed)
            result, error = None, e
        finally:
            self.live = False
            STATE["watching"] = False
            prof = sys.getprofile()
            events = mon.get_events(TOOL)
            sys.setprofile(None)
            mon.set_events(TOOL, 0)
            mon.register_callback(TOOL, ev.PY_START, None)
            mon.register_callback(TOOL, ev.CALL, None)
            mon.free_tool_id(TOOL)
        if prof != self.on_profile:
            self.note("the profile hook was removed during the call")
        if events != ev.PY_START | ev.CALL:
            self.note("sys.monitoring events were changed during the call")
        self.check()
        assert not self.bridge, f"the implicit-tensor bridge or the to_torch shim ran {len(self.bridge)} time(s): {sorted(set(self.bridge))}"
        assert not self.problems, f"ProxyArray, warp or the test data were patched during the call: {self.problems}"
        assert not FLAGS, f"audit: {FLAGS[:5]}"
        if error is not None:
            raise error
        return result


def state_problems():
    w = Watch()
    w.check()
    return w.problems + FLAGS[:5]


@pytest.fixture
def watch():
    assert not state_problems(), state_problems()
    yield Watch
    assert not state_problems(), f"after the test: {state_problems()}"


def call(watch_cls, env, name, **kwargs):
    """Import the terms (outside the observers), then run one term under them."""
    fn = getattr(rewards(), name)
    data = env.scene["robot"].data
    return watch_cls([(data, dict(vars(data))), (env, {"scene": env.scene})]).run(fn, env, **kwargs)


# --- static layer
BANNED_NAMES = ("_warn_implicit", "_deprecation_warned", "__torch_function__", "_binop", "_wp_to_torch", "_WP_TO_TORCH",
                "__dict__", "__code__", "__class__", "__setattr__", "__delattr__", "__getattribute__", "setprofile",
                "settrace", "monitoring", "_getframe", "addaudithook", "get_referents")
BANNED_EXACT = {"gc", "ctypes", "_src"}
BANNED_IN_STRINGS = BANNED_NAMES[:12]
REWARDS_BANNED_CALLS = {"setattr", "delattr", "exec", "eval", "compile", "__import__", "globals", "vars", "locals"}
REWARDS_BANNED_IMPORTS = {"importlib", "builtins", "sys", "gc", "ctypes", "threading", "inspect", "types"}
IMPORTABLE_BINARIES = (b".so", b".pyd")


def git(*args):
    env = dict(os.environ, GIT_NO_REPLACE_OBJECTS="1", GIT_CONFIG_NOSYSTEM="1", GIT_CONFIG_GLOBAL=os.devnull, GIT_ATTR_NOSYSTEM="1")
    r = subprocess.run(["git", "-c", "safe.directory=*", "-C", str(WORKDIR), *args], capture_output=True, env=env)
    assert r.returncode == 0, r.stderr.decode(errors="replace")
    return r.stdout


def base_tree():
    """{path: (mode, blob id)} of the workspace build commit (recorded root-only at image build)."""
    base = (STUDY_DIR / "base_commit").read_text().strip()
    out = {}
    for rec in git("ls-tree", "-r", "-z", "--full-tree", base).split(b"\0"):
        if rec:
            meta, path = rec.split(b"\t", 1)
            mode, kind, sha = meta.split()
            out[path] = (mode, sha.decode())
    return out


def changed_files():
    """(path, bytes) of every .py file on disk that is new or differs from the build commit, and other new importable files."""
    tree = base_tree()
    changed, odd = [], []
    root = os.fsencode(WORKDIR)
    for d, dirs, names in os.walk(root):
        if d == root:
            dirs[:] = [x for x in dirs if x != b".git"]
        for n in [*names, *[x for x in dirs if os.path.islink(os.path.join(d, x))]]:
            p = os.path.join(d, n)
            rel = os.path.relpath(p, root)
            if os.path.islink(p):
                if tree.get(rel, (b"",))[0] != b"120000":
                    odd.append(f"new symlink {os.fsdecode(rel)}")
                continue
            if n.endswith(IMPORTABLE_BINARIES) or (n.endswith(b".pyc") and os.path.basename(d) != b"__pycache__"):
                if rel not in tree:
                    odd.append(f"new importable file {os.fsdecode(rel)}")
                continue
            if not n.endswith(b".py"):
                continue
            data = open(p, "rb").read()
            sha = hashlib.sha1(b"blob %d\0" % len(data) + data).hexdigest()
            if tree.get(rel, (None, None))[1] != sha:
                changed.append((os.fsdecode(rel), data))
    return changed, odd


def static_problems(rel, data):
    try:
        tree = ast.parse(data, filename=rel)
    except (SyntaxError, ValueError):
        return  # cannot be imported either
    strict = rel == REWARDS_REL
    for node in ast.walk(tree):
        line = getattr(node, "lineno", "?")
        names = []
        if isinstance(node, ast.Attribute):
            names = [node.attr]
        elif isinstance(node, ast.Name):
            names = [node.id]
        elif isinstance(node, ast.alias):
            names = [node.name, node.asname or ""]
        elif isinstance(node, ast.ImportFrom):
            names = (node.module or "").split(".")
        elif isinstance(node, ast.Import):
            names = [part for a in node.names for part in a.name.split(".")]
        elif isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            names = [node.name]
        elif isinstance(node, ast.Constant) and isinstance(node.value, str):
            for b in BANNED_IN_STRINGS:
                if b in node.value:
                    yield f"{rel}:{line}: string names the bridge internals ({b})"
        for n in names:
            parts = n.split(".")
            if any(b in n for b in BANNED_NAMES) or BANNED_EXACT & set(parts):
                yield f"{rel}:{line}: names {n!r} (bridge internals or interpreter hooks)"
        if not strict:
            continue
        targets = []
        if isinstance(node, (ast.Assign, ast.Delete)):
            targets = node.targets
        elif isinstance(node, (ast.AugAssign, ast.AnnAssign)):
            targets = [node.target]
        for t in targets:
            if any(isinstance(sub, ast.Attribute) for sub in ast.walk(t)):
                yield f"{rel}:{line}: rewards.py assigns to an attribute"
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id in REWARDS_BANNED_CALLS:
            yield f"{rel}:{line}: rewards.py calls {node.func.id}()"
        if isinstance(node, (ast.ImportFrom, ast.Import)):
            mods = [node.module or ""] if isinstance(node, ast.ImportFrom) else [a.name for a in node.names]
            if any(m.split(".")[0] in REWARDS_BANNED_IMPORTS for m in mods):
                yield f"{rel}:{line}: rewards.py imports {mods}"


def test_no_bridge_patching():
    """The migration is in the terms: no changed file touches the bridge, the shim or the interpreter's hooks."""
    changed, odd = changed_files()
    assert not odd, odd[:5]
    assert REWARDS_REL in {rel for rel, _ in changed}, "rewards.py is unchanged"
    problems = [p for rel, data in changed for p in static_problems(rel, data)]
    assert not problems, problems[:5]


def rewards():
    import isaaclab_tasks  # noqa: F401

    return importlib.import_module("isaaclab_tasks.contrib.trail_walker.rewards")


def proxy(t, dtype=None):
    t = t.contiguous()
    return ProxyArray(wp.from_torch(t, dtype=dtype) if dtype is not None else wp.from_torch(t))


def make_env(seed=0, command=None):
    g = torch.Generator().manual_seed(seed)
    jp, djp = torch.randn(N, J, generator=g), torch.randn(N, J, generator=g)
    jv = torch.randn(N, J, generator=g) * 20
    root = torch.randn(N, 3, generator=g)
    data = SimpleNamespace(joint_pos=proxy(jp), default_joint_pos=proxy(djp), joint_vel=proxy(jv), root_pos_w=proxy(root, wp.vec3f))
    cmd = command if command is not None else torch.tensor([[0.0, 0.05, 1.0], [1.0, 0.0, 0.0]] * (N // 2))
    env = SimpleNamespace(scene={"robot": SimpleNamespace(data=data)},
                          command_manager=SimpleNamespace(get_command=lambda name: cmd))
    return env, jp, djp, jv, root, cmd


def cfg(ids):
    return SimpleNamespace(name="robot", joint_ids=ids)


@pytest.mark.parametrize("ids", [JOINT_IDS, slice(None)])
def test_joint_deviation(watch, ids):
    env, jp, djp, *_ = make_env(1)
    got = call(watch, env, "joint_deviation_l1", asset_cfg=cfg(ids))
    assert isinstance(got, torch.Tensor)
    torch.testing.assert_close(got, torch.sum(torch.abs(jp[:, ids] - djp[:, ids]), dim=1))


def test_base_height(watch):
    env, *_, root, _ = make_env(2)
    got = call(watch, env, "base_height_l2", target_height=0.55, asset_cfg=cfg(slice(None)))
    torch.testing.assert_close(got, torch.square(root[:, 2] - 0.55))


@pytest.mark.parametrize("ids", [JOINT_IDS, slice(None)])
def test_joint_vel_clipped(watch, ids):
    env, _, _, jv, _, _ = make_env(3)
    before = jv.clone()
    got = call(watch, env, "joint_vel_l2_clipped", max_vel=10.0, asset_cfg=cfg(ids))
    torch.testing.assert_close(got, torch.sum(torch.square(torch.clamp(jv, -10.0, 10.0)[:, ids]), dim=1))
    torch.testing.assert_close(env.scene["robot"].data.joint_vel.torch, before, msg="the term must not modify the asset data")


def test_stand_still(watch):
    env, jp, djp, _, _, cmd = make_env(4)
    before = jp.clone()
    got = call(watch, env, "stand_still_deviation", command_name="base_velocity", asset_cfg=cfg(slice(None)))
    want = torch.sum(torch.abs(jp - djp), dim=1) * (torch.norm(cmd[:, :2], dim=1) < 0.1)
    torch.testing.assert_close(got, want)
    torch.testing.assert_close(env.scene["robot"].data.joint_pos.torch, before, msg="the term must not modify the asset data")


def test_task_still_loads():
    from ss_isaac import resolve

    e, _ = resolve("IsaacContrib-TrailWalker-Flat", [])
    assert e.rewards.stand_still.params == {"command_name": "base_velocity"}
    assert not state_problems(), state_problems()
