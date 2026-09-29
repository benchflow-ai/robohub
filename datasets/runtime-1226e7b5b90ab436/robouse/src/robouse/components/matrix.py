"""The compatibility matrix: every remixable embodiment x scene x task, whether it fits and why not.

A cell is compatible when the static resolver accepts it (resolve.check) and the task can be laid out in the scene
for that embodiment (remix.instantiate with seed 0). Modifier rules are per task (which perturbations and safety
overlays apply), so a static page can show compatibility for any modifier choice without re-running anything.
"""
from __future__ import annotations

from .registry import Registry
from .remix import RemixError, instantiate
from .resolve import MOD_DEFAULTS, check


def build(reg: Registry) -> dict:
    E, S, T = reg.remixable("embodiments"), reg.remixable("scenes"), reg.remixable("tasks")
    cells = {}
    for e in E:
        for s in S:
            for t in T:
                rep = check(reg, e, s, t)
                if rep.ok:
                    try:
                        instantiate(e, s, t, dict(MOD_DEFAULTS), 0)
                    except RemixError as err:
                        rep.ok, rep.reasons = False, [str(err)]
                cells[f"{e}|{s}|{t}"] = {"ok": rep.ok, **({"why": "\n".join(rep.reasons)} if rep.reasons else {})}
    tasks = {t: {"perturbable": reg.tasks[t].get("perturbable", True), "safety_levels": reg.tasks[t].get("safety_levels", [])}
             for t in T}
    from . import catalog as C

    return {"version": C.VERSION, "tracks": C.TRACKS, "scenarios": C.SCENARIOS, "capabilities": C.CAPABILITIES,
            "harm_levels": C.HARM_LEVELS, "suites": C.SUITES,
            "embodiments": E, "scenes": S, "tasks": T, "task_rules": tasks,
            "modifiers": {k: list(v["values"]) for k, v in reg.modifiers.items() if k != "seed"},
            "cells": cells, "compatible": sum(c["ok"] for c in cells.values()), "total": len(cells)}
