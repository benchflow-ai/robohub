"""CaP-X backend: code-as-policy tasks from CaP-X (NVIDIA, UC Berkeley, Stanford, CMU; MIT;
https://github.com/capgym/cap-x), CaP-Bench's RoboSuite tasks with the privileged API.

CaP-X's agent writes Python that calls a task API (get_object_pose, sample_grasp_pose, goto_pose, open_gripper, ...).
Here each API function is a Robo Use skill: `robo skill NAME ARG ...`, one JSON value per argument (or `key=VALUE`),
and the task instruction gives the agent a small Python module that turns the skills back into Python functions with
CaP-X's names and signatures, so the agent writes CaP-X-style code. The API code is CaP-X's own
(capx_upstream/, vendored at 53e9966, MIT), running in a simulator worker (worker.py) on CaP-X's robosuite fork, with
PyRoKi inverse kinematics on the CPU; every robosuite control step an API call takes is one step of the episode budget
(CaP-X's horizon for the task).

Tier. These tasks use CaP-X's privileged API (the API of CaP-Bench's S1 tier: object poses from the simulator, no
perception). CaP-Bench's S1 is single-turn: one program, run once. Robo Use cannot hold a coding agent to one program,
so the agent may run code several times in the episode and see each call's result and printed output, and it may look
at the scene camera; that is multi-turn interaction on the S1 API, which is not one of CaP-Bench's eight tiers. The
perception tiers (S2-S4, M1-M4: SAM3, Molmo 2, OWL-ViT, SAM2, Contact-GraspNet on a GPU) are not included.

There is no low-level control: `robo act` takes one number and only holds the robot still for one step, as CaP-X's
agent has only the API. Success is CaP-X's `task_completed()` (robosuite's `_check_success()` in CaP-X's fork), judged
after `robo done` and the usual 10 still steps.
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from ... import config
from ...core.embodiment_spec import Skill, SkillArg
from ...workers.client import StdioWorker
from ..base import ActionSpec, Backend, G, StepInfo

WORKER = Path(__file__).with_name("worker.py")

# task -> CaP-X horizon (the low-level env's max_steps), camera
TASKS: dict[str, dict] = {
    "cube_lifting": dict(max_steps=1500, camera="robot0_robotview", robots=1),
    "cube_stack": dict(max_steps=1500, camera="robot0_robotview", robots=1),
    "cube_restack": dict(max_steps=1500, camera="robot0_robotview", robots=1),
    "spill_wipe": dict(max_steps=4000, camera="robot0_robotview", robots=1),
    "nut_assembly": dict(max_steps=1000, camera="birdview", robots=1),
    "two_arm_lift": dict(max_steps=5000, camera="agentview", robots=2),
    "two_arm_handover": dict(max_steps=5000, camera="agentview", robots=2),
}
API_STEP = "__capx_api_step__"  # what an API skill yields for each control step it takes
CHUNK = 40  # control steps per `robo skill` request; a longer API call returns running and goes on with capx_continue
CONTINUE = "capx_continue"


def parse_arg(s: str):
    """One skill argument: a JSON value (number, list, true/false, quoted string), else the text itself."""
    try:
        return json.loads(s)
    except (ValueError, TypeError):
        return s


class CapxBackend(Backend):
    name = "capx"
    venv = "capx"
    probe_success = False
    image_flipped = False

    def __init__(self, spec: dict):
        self.spec = spec
        self.kind = str(spec.get("env"))
        t = self.task_info(self.kind)
        self.max_steps = int(spec.get("max_steps", t["max_steps"]))
        self.camera = str(spec.get("camera", t["camera"]))
        self.render_size = (int(spec.get("image_size", 512)),) * 2
        self.robot_kind = "bimanual" if t["robots"] == 2 else "arm"
        self.robot_name = "panda+panda" if t["robots"] == 2 else "panda"
        self.action_spec = ActionSpec(
            names=["HOLD"],
            low=[0.0],
            high=[0.0],
            doc="No low-level control: CaP-X's agent has only the API. `robo act 0` holds the robot still for one step.",
        )
        self.worker = StdioWorker(
            self.venv.replace("-", "_"),
            config.sim_python(self.venv.replace("-", "_"), self.venv),
            WORKER,
            setup="docs/suites/capx.md",
            timeout=900,
        )
        self.functions: list[str] = []
        self._running = False  # an API call is part-way through (it took CHUNK steps and waits for capx_continue)
        self._state: dict | None = None
        self._success = False

    @staticmethod
    def task_info(kind: str) -> dict:
        if kind not in TASKS:
            raise KeyError(f"unknown CaP-X task {kind!r}")
        return TASKS[kind]

    def action_layout(self) -> list[G]:
        return [G("hold", 1, "other", doc="0: hold still for one step")]

    # ---- episode -------------------------------------------------------------------------------------------------
    def reset(self, seed: int) -> None:
        r = self.worker.call("make", task=self.kind, seed=int(self.spec.get("seed", seed)))
        self.functions = list(r["functions"])
        self._state = None

    def step(self, action) -> StepInfo:
        if isinstance(action, str) and action == API_STEP:
            r = self.worker.call("api_advance")
            self._pending = r
        else:
            if self._running:  # holding still ends an API call that is part-way through
                self.worker.call("api_abort")
                self._running = False
            self.worker.call("hold")
        self._state = None
        return StepInfo(success=False)

    def hold_action(self) -> list[float]:
        return [0.0]

    # ---- skills (the API) ----------------------------------------------------------------------------------------
    def skills(self) -> list[str]:
        return [*self.functions, CONTINUE]

    def skill_specs(self) -> list[Skill]:
        return [
            *(
                Skill(
                    n,
                    [SkillArg("args", "str", optional=True, doc="one JSON value per argument, or key=VALUE")],
                    doc=f"CaP-X API function {n}",
                )
                for n in self.functions
            ),
            Skill(
                CONTINUE, doc=f"go on with an API call that returned running (each request runs at most {CHUNK} steps)"
            ),
        ]

    def _steps(self, r: dict):
        """Run the pending API call for at most CHUNK control steps; returns its result, or running."""
        n = 0
        try:
            while r.get("state") == "step":
                if n >= CHUNK:
                    self._running = True
                    return {"running": True, "note": f"the call goes on: run `robo skill {CONTINUE}` until it returns"}
                self._pending = None
                yield API_STEP
                n += 1
                r = self._pending or {"state": "done", "error": "the API call stopped"}
        except GeneratorExit:  # budget used up or episode finished mid-call: stop the API call
            self.worker.call("api_abort")
            self._running = False
            raise
        self._running = False
        self._state = None
        return {k: v for k, v in r.items() if k != "state"}

    def start_skill(self, name: str, raw_args: list[str]):
        if name == CONTINUE:
            if not self._running:
                raise ValueError("no API call is running")
            self._running = False

            def cont():
                return (yield from self._steps({"state": "step"}))

            return cont()
        if self._running:
            raise ValueError(f"an API call is still running: run `robo skill {CONTINUE}` until it returns")
        if name not in self.functions:
            raise ValueError(f"unknown API function {name!r}; functions: {', '.join(self.functions)}")
        args, kwargs = [], {}
        for a in raw_args:
            k, sep, v = a.partition("=")
            if sep and k.isidentifier() and not a.lstrip().startswith(("[", "{", '"')):
                kwargs[k] = parse_arg(v)
            else:
                args.append(parse_arg(a))

        def gen():
            r = self.worker.call("api_start", name=name, args=args, kwargs=kwargs)
            return (yield from self._steps(r))

        return gen()

    # ---- observation and judging ---------------------------------------------------------------------------------
    def observe(self) -> dict:
        if self._state is None:
            self._state = self.worker.call("observe")
        return dict(self._state)

    def success(self) -> bool:
        return bool(self.worker.call("success")["success"])

    def judge(self, outcome: str, text: str = "") -> bool:
        if self._running:
            self.worker.call("api_abort")
            self._running = False
        r = self.worker.call("success")
        self.last_judge = {**r, "rule": "CaP-X task_completed() (robosuite _check_success in CaP-X's fork)"}
        return outcome == "done" and bool(r["success"])

    def render(self, width: int | None = None, height: int | None = None) -> np.ndarray:
        w, h = (width, height) if width and height else self.render_size
        return self.worker.call("render", camera=self.camera, width=int(w), height=int(h))

    def api_docs(self) -> str:
        return self.worker.call("docs")["combined_doc"]

    def close(self) -> None:
        self.worker.close()


class CapxLiberoBackend(CapxBackend):
    """CaP-X's LIBERO-PRO tasks with its privileged LIBERO API (FrankaLiberoPrivilegedApi), in the capx-libero runtime.
    `env` is `libero:<LIBERO-PRO suite>:<task index>`; the seed n selects LIBERO's initial state n - 1, as CaP-X's trial n
    does. CaP-X's `goto_pose_interactive_cartesian` takes a Python callable, which cannot cross the robo boundary, so it is
    not offered."""

    name = "capx_libero"
    venv = "capx-libero"

    @staticmethod
    def task_info(kind: str) -> dict:
        if not kind.startswith("libero:") or len(kind.split(":")) != 3:
            raise KeyError(f"unknown CaP-X LIBERO task {kind!r}; expected libero:<suite>:<index>")
        return dict(max_steps=4000, camera="agentview", robots=1)


def oracle_main(env: str) -> None:
    from .oracles import run

    run(env)
