"""BEHAVIOR-1K (Stanford; OmniGibson on NVIDIA Isaac Sim) household activities as a robouse backend.

Isaac Sim needs an NVIDIA RTX GPU, so the simulator runs remotely: one `worker.py` process per activity on
a GPU machine (see docs/suites/behavior.md), each holding its scene loaded. This backend is a thin HTTP client for
that worker; the episode server, budgets, trace, video, verifier and harnesses all run locally as for any other
backend. The agent never sees the worker's address: it is read from a private endpoints file (denied to model
harnesses by the local runner's sandbox), not from the environment.

Worker endpoints come from $ROBOUSE_BEHAVIOR_REMOTE (default: <config>/remote/behavior.json, see robouse.config):
  {"secret": "...", "headers": {...}, "workers": {"<activity>": "https://...", "<activity>@<instance>": "https://..."}}
A worker entry may also be {"url": "https://...", "headers": {...}} when workers run on several machines.
A task's worker key is its `worker` field, else the activity name for instance 0 and `<activity>@<instance>` otherwise.

Action interface
  robo skill NAME [OBJ]   one BEHAVIOR symbolic primitive (navigate_to, grasp, place_on_top, place_inside, open,
                          close, toggle_on, toggle_off, release); one skill = one step of the budget
  robo act 0 [--repeat N] no low-level control: holds the robot still for N simulator steps
Observation: robot pose/room/held object and every task-relevant object (BDDL instance name, category, room,
position, distance, in_reach, open/toggled_on, inside/on_top_of relations to other task objects), plus
`object_names`. Vision tasks (`obs_mode: vision`, `visible_fields: [robot, object_names]`) show only the robot's own
state and the object names; everything else comes from the cameras.
Success: BEHAVIOR's BDDL goal predicates, evaluated by the worker after `robo done`.
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from ... import config
from ...workers.client import decode_png_b64, load_endpoints, remote_worker
from ..base import ActionSpec, Backend, StepInfo

SETUP = "docs/suites/behavior.md"


SKILLS = [  # (name, takes an object, doc, preconditions)
    ("navigate_to", True, "Drive the base to a free spot next to OBJ, facing it; a held object comes along.", []),
    (
        "grasp",
        True,
        "Pick up OBJ; task objects resting in or on it come along.",
        ["hand empty", "OBJ within reach", "OBJ not fixed, not larger than 2.5 m", "OBJ not inside a closed container"],
    ),
    ("place_on_top", True, "Put the held object on top of OBJ.", ["holding an object", "OBJ within reach"]),
    (
        "place_inside",
        True,
        "Put the held object inside OBJ.",
        ["holding an object", "OBJ within reach", "OBJ open if it opens"],
    ),
    ("open", True, "Open a door, drawer or lid of OBJ.", ["hand empty", "OBJ within reach"]),
    ("close", True, "Close OBJ.", ["hand empty", "OBJ within reach"]),
    ("toggle_on", True, "Switch OBJ on.", ["hand empty", "OBJ within reach"]),
    ("toggle_off", True, "Switch OBJ off.", ["hand empty", "OBJ within reach"]),
    ("release", False, "Drop the held object where the hand is.", ["holding an object"]),
]


def r1pro_embodiment(max_steps: int, cameras: list[str], obs_mode: str = "state"):
    """The R1Pro as an embodiment spec (robouse.core.embodiment_spec): a skill-only
    robot. Its one action group only waits; every manipulation is a backend skill that costs one budget step."""
    from ...core.embodiment_spec import (
        ActionGroup,
        Budgets,
        Camera,
        Embodiment,
        Field,
        RewardSpec,
        Sensors,
        Skill,
        SkillArg,
    )

    obj = [SkillArg("obj", "object", doc="BDDL instance name as listed by robo observe, e.g. cabinet.n.01_1")]
    CAMERAS = {
        "follow": Camera(
            "follow",
            "body",
            640,
            480,
            calibrated=False,
            doc="third-person, 2.6 m behind and 2.0 m above the base, looking along its heading",
        ),
        "head": Camera("head", "head", 640, 480, calibrated=False, doc="the robot's head camera"),
    }
    return Embodiment(
        name="galaxea-r1pro",
        kind="skill_only",
        action_groups=[
            ActionGroup(
                "base.wait",
                ["wait"],
                [0.0],
                [1.0],
                "wait",
                units="simulator steps",
                doc="no low-level control: holds the robot still for one simulator step",
            )
        ],
        sensors=Sensors(
            cameras=[CAMERAS.get(c, Camera(c, "body", 640, 480, calibrated=False)) for c in cameras],
            proprioception=[Field("robot", doc="base xy (m), yaw_deg, room, holding")],
            state=[
                Field(
                    "objects",
                    privileged=True,
                    doc="per task object: category, room, position, distance_m, in_reach, open, toggled_on, "
                    "inside/on_top_of relations",
                ),
                Field("object_names", doc="the task objects' BDDL instance names (what skills take)"),
            ],
        ),
        skills=[
            Skill(
                n,
                obj if takes else [],
                doc=doc,
                preconditions=pre,
                impl="backend",
                aliases=[n.replace("_", "-")] if "_" in n else [],
            )
            for n, takes, doc, pre in SKILLS
        ],
        budgets=Budgets(max_steps=int(max_steps), max_skill_steps=1),
        reward=RewardSpec(dense="sparse", success_mode="final"),
        doc="Galaxea R1Pro wheeled two-arm mobile manipulator in BEHAVIOR-1K (OmniGibson on Isaac Sim), driven through "
        "symbolic skills; "
        + (
            "observations are the cameras plus object names"
            if obs_mode == "vision"
            else "observations include every task object's state"
        ),
    ).validate()


def worker_key(spec: dict) -> str:
    inst = int(spec.get("instance", 0))
    return spec.get("worker") or (spec["env"] if inst == 0 else f"{spec['env']}@{inst}")


class BehaviorBackend(Backend):
    name = "behavior"
    robot_name = "galaxea-r1pro"
    robot_kind = "skill_only"
    proprioception = ("robot",)
    probe_success = False  # success is evaluated by the remote worker, only when the episode is judged

    def embodiment(self):
        return r1pro_embodiment(
            self.max_steps, list(self.spec.get("cameras", ["follow", "head"])), str(self.spec.get("obs_mode", "state"))
        )

    def __init__(self, spec: dict):
        self.spec = spec
        self.activity = spec["env"]
        self.worker = remote_worker(
            load_endpoints("behavior", SETUP), worker_key(spec), "BEHAVIOR", float(spec.get("rpc_timeout_s", 300))
        )
        self.max_steps = int(spec.get("max_steps", 60))
        self.camera = "follow"
        self._grip = 0.0
        self.action_spec = ActionSpec(
            ["wait"],
            [0.0],
            [1.0],
            "No low-level control in this suite: `robo act 0` holds the robot still for one "
            "simulator step. Use `robo skill NAME OBJECT` for everything.",
        )
        self._pending: list[np.ndarray] = []
        self._last_frame: np.ndarray | None = None
        self._skills = {}

    def _call(self, method: str, **args):
        return self.worker.call(method, **args)

    def close(self) -> None:
        self.worker.close()

    # ---- Backend interface -----------------------------------------------------------------------------------------
    def reset(self, seed: int) -> None:
        info = self._call("info")
        self._skills = info.get("skills", {})
        self._call("reset")
        self._last_frame = None

    def step(self, action) -> StepInfo:
        self._call("wait", steps=1)
        self._last_frame = None
        return StepInfo(success=False)

    def run_skill(self, name: str, args: list[str]) -> dict:
        r = self._call("skill", name=name, args=args, frames=True)
        self._pending.extend(decode_png_b64(f) for f in r.pop("frames", []) or [])
        self._last_frame = None
        return {"ok": bool(r.get("ok")), "message": r.get("message", "")}

    def pop_frames(self) -> list[np.ndarray]:
        out, self._pending = self._pending, []
        return out

    def observe(self) -> dict:
        return self._call("observe")

    def render(self, width: int = 640, height: int = 480) -> np.ndarray:
        if self.camera == "follow" and self._last_frame is not None:
            return self._last_frame
        img = decode_png_b64(self._call("render", camera=self.camera)["jpeg"])
        if self.camera == "follow":
            self._last_frame = img
        return img

    def render_record(self, width: int, height: int) -> None:
        return None  # OmniGibson's worker renders fixed-size frames; the video keeps that native size

    def success(self) -> bool:
        return bool(self._call("success")["success"])

    def goal_status(self) -> dict:
        return self._call("success")

    def skills(self) -> list[str]:
        return list(self._skills.values()) if self._skills else []


def oracle_main(env: str) -> None:
    """Reference solution: the task's scripted primitive sequence (oracle/plan.json, path in $ROBOUSE_BEHAVIOR_PLAN),
    sent through the socket like an agent's `robo skill` calls."""
    from ...agent_cli import _send

    plan_path = Path(config.env("ROBOUSE_BEHAVIOR_PLAN"))  # set by the task's oracle/solve.sh
    plan = json.loads(plan_path.read_text())
    for step in plan:
        r = _send({"op": "skill", "name": step[0], "args": step[1:]})
        res = r.get("result") or {}
        print(step, r.get("ok"), res.get("ok"), res.get("message") or r.get("error"), flush=True)
        if not r.get("ok") or "episode" in res:
            break
    _send({"op": "done", "text": "oracle plan finished"})
