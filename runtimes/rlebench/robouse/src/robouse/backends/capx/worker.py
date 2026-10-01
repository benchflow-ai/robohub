"""CaP-X simulator worker. Runs inside the CaP-X virtualenv (CaP-X's robosuite fork, MuJoCo 3.5.0, PyRoKi on JAX),
started by the robouse `capx` backend (backend.py) and spoken to over stdin/stdout (robouse.workers.wire). Only the
vendored CaP-X code (capx_upstream/, MIT) and its simulator stack are imported here, never the robouse package.

The worker builds CaP-X's own low-level environment for a task and CaP-X's own privileged API for it, and runs one API
call at a time in a greenlet. Every robosuite control step the API wants to take is held at a gate until the episode
server grants it (`advance`), so the server counts, records and budgets each step exactly as it does for any other
Robo Use skill, while the API code itself runs unmodified.
"""

from __future__ import annotations

import importlib
import inspect
import io
import json
import os
import random
import sys
import traceback

import greenlet
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
UPSTREAM = os.path.join(HERE, "capx_upstream")
sys.path.insert(0, UPSTREAM)  # the `capx` package (never shadows a simulator package)

# task -> CaP-X low-level environment and privileged API (the classes the *_privileged.yaml configs name)
TASKS = {
    "cube_lifting": (
        "capx.envs.simulators.robosuite_cube_lift:FrankaRobosuiteCubeLiftLowLevel",
        "capx.integrations.franka.control_privileged:FrankaControlPrivilegedApi",
    ),
    "cube_stack": (
        "capx.envs.simulators.robosuite_cubes:FrankaRobosuiteCubesLowLevel",
        "capx.integrations.franka.control_privileged:FrankaControlPrivilegedApi",
    ),
    "cube_restack": (
        "capx.envs.simulators.robosuite_cubes_restack:FrankaRobosuiteCubesRestackLowLevel",
        "capx.integrations.franka.control_privileged:FrankaControlPrivilegedApi",
    ),
    "spill_wipe": (
        "capx.envs.simulators.robosuite_spill_wipe:FrankaRobosuiteSpillWipeLowLevel",
        "capx.integrations.franka.spill_wipe_privileged:FrankaControlSpillWipePrivilegedApi",
    ),
    "nut_assembly": (
        "capx.envs.simulators.robosuite_nut_assembly:FrankaRobosuiteNutAssembly",
        "capx.integrations.franka.nut_assembly_privileged:FrankaControlNutAssemblyPrivilegedApi",
    ),
    "two_arm_lift": (
        "capx.envs.simulators.robosuite_two_arm_lift:RobosuiteTwoArmLiftEnv",
        "capx.integrations.franka.two_arm_lift_privileged:FrankaTwoArmLiftPrivilegedApi",
    ),
    "two_arm_handover": (
        "capx.envs.simulators.robosuite_handover:RobosuiteHandoverEnv",
        "capx.integrations.franka.handover_privileged:FrankaHandoverPrivilegedApi",
    ),
}


LIBERO_ENV = "capx.envs.simulators.libero:FrankaLiberoEnv"
LIBERO_API = "capx.integrations.franka.libero_privileged:FrankaLiberoPrivilegedApi"
# functions of an API that cannot be called across the robo boundary (they take a Python callable)
NOT_EXPOSED = {"goto_pose_interactive_cartesian"}


def _load(spec: str):
    mod, name = spec.split(":")
    return getattr(importlib.import_module(mod), name)


def jsonable(x):
    """API return values as JSON: arrays become lists, tuples lists, numpy scalars numbers."""
    if isinstance(x, dict):
        return {str(k): jsonable(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [jsonable(v) for v in x]
    if hasattr(x, "tolist"):
        return x.tolist()
    if isinstance(x, (bool, int, float, str)) or x is None:
        return x
    return repr(x)


class _Abort(BaseException):
    pass


class Worker:
    def __init__(self) -> None:
        os.chdir(UPSTREAM)  # CaP-X's controller configs are relative paths (capx/integrations/robosuite/...)
        self.env = None
        self.api = None
        self.task = ""
        self._api = None
        self._abort = False
        self.sim_steps = 0

    # ---- setup ---------------------------------------------------------------------------------------------------
    def make(self, task: str, seed: int) -> dict:
        libero = task.startswith("libero:")  # libero:<LIBERO-PRO suite>:<task index>
        if task not in TASKS and not libero:
            raise KeyError(f"unknown CaP-X task {task!r}")
        self.task = task
        random.seed(seed)
        np.random.seed(seed)
        env_cls, api_cls = (_load(s) for s in ((LIBERO_ENV, LIBERO_API) if libero else TASKS[task]))
        params = inspect.signature(env_cls.__init__).parameters
        # privileged, with rendering on, as CaP-X runs its privileged configs when it records video
        kw = {k: v for k, v in dict(privileged=True, enable_render=True, viser_debug=False).items() if k in params}
        if libero:
            _, suite, idx = task.split(":")
            kw.update(suite_name=suite, task_id=int(idx))
        self.env = env_cls(**kw)
        rs = self._rs()
        # CaP-X leaves robosuite's layout sampler unseeded (a new layout every trial); Robo Use fixes it per task
        rng = np.random.default_rng(seed)
        rs.rng = rng
        pi = getattr(rs, "placement_initializer", None)
        for s in [pi, *getattr(pi, "samplers", {}).values()] if pi is not None else []:
            if hasattr(s, "rng"):
                s.rng = rng
        self.env.reset(seed=seed)  # LIBERO: seed n is LIBERO's initial state n - 1, as in CaP-X's trials
        self.api = api_cls(self.env)
        self._warm_ik()
        self._wrap_step()
        return {
            "functions": self.functions(),
            "max_steps": int(getattr(self.env, "max_steps", 0)),
            "language": getattr(getattr(self.env, "handle", None), "task_language", None),
        }

    @staticmethod
    def _warm_ik() -> None:
        """Compile PyRoKi's two IK solves once before the episode starts (JAX compiles on first use, which can take a
        minute on a busy CPU and would otherwise land inside the agent's first goto_pose)."""
        try:
            import capx.integrations.motion.pyroki_snippets as pks
            from capx.integrations.motion.pyroki import _robot
            from capx.integrations.motion.pyroki_context import get_pyroki_context

            p, q = np.array([0.5, 0.0, 0.3]), np.array([0.0, 0.0, 1.0, 0.0])
            # the server-style robot (joint-limit margin) and the in-process context robot the two-arm and LIBERO APIs use
            for r in (_robot(), get_pyroki_context("panda_description", target_link_name="panda_hand").robot):
                cfg = pks.solve_ik(robot=r, target_link_name="panda_hand", target_position=p, target_wxyz=q)
                pks.solve_ik_vel_cost(
                    robot=r, target_link_name="panda_hand", target_position=p, target_wxyz=q, prev_cfg=cfg
                )
        except Exception as e:
            print(f"IK warm-up skipped: {type(e).__name__}: {e}", file=sys.stderr)

    def functions(self) -> list:
        return sorted(n for n in self.api.functions() if n not in NOT_EXPOSED)

    def _rs(self):
        """The robosuite environment: CaP-X's RoboSuite wrappers hold it as robosuite_env, its LIBERO wrapper as handle.env."""
        return self.env.robosuite_env if hasattr(self.env, "robosuite_env") else self.env.handle.env

    def _wrap_step(self) -> None:
        rs = self._rs()
        real = rs.step

        def gated(*a, **k):
            if self._api is not None and greenlet.getcurrent() is self._api:
                self._api.parent.switch(("step", None))  # back to the episode server until it grants this step
                if self._abort:
                    raise _Abort()
            self.sim_steps += 1
            return real(*a, **k)

        rs.step = gated

    def docs(self) -> dict:
        """Each API function's signature and docstring, as CaP-X writes them into the prompt (ApiBase.combined_doc)."""
        fns = self.api.functions()
        keep = {n: f for n, f in fns.items() if n not in NOT_EXPOSED}  # CaP-X's order
        self.api.functions = lambda: keep  # the documented functions are the exposed ones
        try:
            doc = self.api.combined_doc()
        finally:
            del self.api.functions
        return {"combined_doc": doc, "functions": self.functions()}

    # ---- API calls -----------------------------------------------------------------------------------------------
    # An API call runs in a greenlet on this same thread (simulation and rendering stay on one OS thread, as the
    # OpenGL contexts require); it switches back here at every robosuite step and resumes when the step is granted.
    def api_start(self, name: str, args: list, kwargs: dict) -> dict:
        if self._api is not None:
            raise RuntimeError("an API call is still running")
        fns = {n: f for n, f in self.api.functions().items() if n not in NOT_EXPOSED}
        if name not in fns:
            raise ValueError(f"unknown API function {name!r}; functions: {', '.join(sorted(fns))}")
        fn = fns[name]
        conv = [np.asarray(a, dtype=np.float64) if isinstance(a, list) else a for a in args]
        kconv = {k: (np.asarray(v, dtype=np.float64) if isinstance(v, list) else v) for k, v in kwargs.items()}
        self._abort = False
        buf = io.StringIO()

        def body():
            try:
                ret = fn(*conv, **kconv)
                return ("done", {"return": jsonable(ret)})
            except _Abort:
                return ("done", {"stopped": True})
            except BaseException as e:
                tb = traceback.format_exception(type(e), e, e.__traceback__)
                return ("done", {"error": f"{type(e).__name__}: {e}", "traceback": "".join(tb[-3:])[-1500:]})

        self._api = greenlet.greenlet(body)
        self._buf = buf
        return self._resume()

    def _resume(self) -> dict:
        real = sys.stdout
        sys.stdout = self._buf  # what the API prints: CaP-X shows it to the model as the console output
        try:
            kind, payload = self._api.switch()
        finally:
            sys.stdout = real
        if kind == "step":
            return {"state": "step"}
        self._api = None
        out = self._buf.getvalue()
        return {"state": "done", **payload, **({"stdout": out[-4000:]} if out else {})}

    def api_advance(self) -> dict:
        """Let the running API call take its pending control step; returns when it asks for the next one or ends."""
        if self._api is None:
            raise RuntimeError("no API call is running")
        return self._resume()

    def api_abort(self) -> dict:
        if self._api is None:
            return {"state": "idle"}
        self._abort = True
        return self._resume()

    def hold(self) -> dict:
        """One control step holding the current joint targets and gripper (CaP-X's `_step_once`)."""
        if self._api is not None:
            raise RuntimeError("an API call is running")
        self.env._step_once()
        return {}

    # ---- state ---------------------------------------------------------------------------------------------------
    def observe(self) -> dict:
        """The robot's own state: joint positions and each gripper's opening, from robosuite's observation."""
        rs = self._rs()
        # LIBERO: the observation of the last step, as CaP-X keeps it (computing a new one would render its cameras)
        o = rs._get_observations() if hasattr(rs, "_get_observations") else (self.env._current_obs or {})
        out = {}
        for k in sorted(o):
            if k.startswith("robot") and k.endswith(("_joint_pos", "_gripper_qpos")):
                out[k] = [round(float(v), 5) for v in np.asarray(o[k]).ravel()]
        out["sim_steps"] = self.sim_steps
        return out

    def success(self) -> dict:
        return {"success": bool(self.env.task_completed()), "reward": float(self.env.compute_reward())}

    def render(self, camera: str, width: int = 512, height: int = 512) -> np.ndarray:
        """A picture from `camera` with robosuite's own offscreen renderer (the one CaP-X's observations use; a second
        OpenGL context in this process would corrupt its depth images)."""
        rs = self._rs()
        img = rs.sim.render(camera_name=camera, width=int(width), height=int(height), depth=False)
        return np.ascontiguousarray(np.asarray(img)[::-1], dtype=np.uint8)

    def camera_names(self) -> list:
        return [n for n in self._rs().sim.model.camera_names if n]

    def close(self) -> None:
        if self._api is not None:
            try:
                self.api_abort()
            except Exception:
                pass
        try:
            self._rs().close()
        except Exception:
            pass


if __name__ == "__main__":  # smoke test: python worker.py TASK SEED
    w = Worker()
    print(json.dumps(w.make(sys.argv[1], int(sys.argv[2]))))
    print(w.docs()["combined_doc"][:400])
