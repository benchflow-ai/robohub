"""Batches of trials on a real robot (`run-many`): a safety stop halts the batch.

A trial on a real robot (or its hardware-in-the-loop mock) that ends in a safety stop (e-stop, over-temperature,
watchdog, lost telemetry, contact, a refused gate, a hardware fault), that has a stop after a normal end (a park that
was stopped, an object still in the gripper, no fresh reading of the arm), or a live trial that crashed before recording
a result, holds the batch: the next trial does not start until the operator has inspected the robot and answered the
`inspect` gate on the stopped trial's operator channel. The answer is recorded in that trial's session log
(episode/session/events.jsonl). A refused or unanswered inspection, or a stop without an attended operator, ends the
batch: the remaining trials are not run, and `halt.json` in the run folder says which trial stopped and why. Simulated
trials never hold a batch.
"""

from __future__ import annotations

import json
import time
from pathlib import Path


def stop_of(trial_dir: Path, spec: dict | None = None) -> dict | None:
    """The safety stop a real-robot trial ended with, or None. `spec` (the task's) names the robot mode of a trial that
    left no episode result: a live trial that crashed holds the batch too, since nobody knows how it left the arm."""
    try:
        res = json.loads((trial_dir / "result.json").read_text())
    except (OSError, ValueError):
        res = {}
    ep = res.get("episode") or {}
    sess = ep.get("session") or {}
    mode = ep.get("embodiment_mode")
    after = sess.get("stop_after_end") or {}  # a stop after a normal end: the arm holds where it is
    if mode in ("real", "hil-mock") and (ep.get("outcome") == "safety_stop" or sess.get("safety_stop") or after):
        return {
            "trial": trial_dir.name,
            "task": res.get("task_name") or ep.get("task"),
            "mode": mode,
            "kind": sess.get("stop_kind") or after.get("kind") or "safety_stop",
            "reason": sess.get("safety_stop") or after.get("reason") or ep.get("agent_text") or "safety stop",
            "attended": bool(sess.get("attended", True)),
            "channel": sess.get("operator_channel") or ("script" if mode == "hil-mock" else "tty"),
            "secured": sess.get("secured"),
        }
    if not ep and spec is not None:
        from ..backends.real.backend import real_config, real_mode

        if real_mode(spec) == "real":
            cfg = real_config(spec)
            info = res.get("exception_info") or {}
            return {
                "trial": trial_dir.name,
                "task": res.get("task_name") or spec.get("id"),
                "mode": "real",
                "kind": "no_result",
                "reason": "the trial left no episode result"
                + (f" ({info.get('exception_message')})" if info.get("exception_message") else ""),
                "attended": bool(cfg.get("attended", True)),
                "channel": cfg.get("operator", "tty"),
            }
    return None


def _record(events: Path, kind: str, **fields) -> None:
    """Append to the stopped trial's session log (the runner writes after the episode server has closed it)."""
    events.parent.mkdir(parents=True, exist_ok=True)
    rec = {"t": None, "event": kind, "at": time.strftime("%Y-%m-%d %H:%M:%S"), **fields}
    with open(events, "a") as f:
        f.write(json.dumps(rec, default=str) + "\n")


def inspect(stop: dict, job_dir: Path) -> bool:
    """After a safety stop: True once an attended operator has inspected the robot and confirmed the next trial may
    start; False (the batch ends) otherwise. Both are recorded in the stopped trial's session log and in halt.json."""
    from .operator import Operator
    from .safety import SafetyStop, estop_engaged

    events = job_dir / stop["trial"] / "episode" / "session" / "events.jsonl"
    _record(events, "batch_hold", cause=stop["kind"], reason=stop["reason"])
    ok, answer, note = False, "none", "no attended operator: the batch stops"
    if stop["attended"]:
        latch = estop_engaged()
        prompt = (
            f"Trial {stop['trial']} ended in a safety stop ({stop['kind']}: {stop['reason']})."
            + (f" The arm is {stop['secured']}." if stop.get("secured") else "")
            + " Inspect the robot and its workspace before anything else moves."
            + (f" The e-stop latch is set ({latch}): clear it with `robouse estop --clear` when safe." if latch else "")
            + " Start the next trial?"
        )
        try:
            op = Operator(
                stop["channel"],
                mock=stop["mode"] == "hil-mock",
                log=lambda kind, **kw: _record(events, kind, **kw),
            )
            ok, answer, note = op.confirm("inspect", prompt)
        except (SafetyStop, ValueError) as e:  # no operator terminal, a channel the runner cannot use
            ok, answer, note = False, "none", str(e)
    _record(events, "batch_resume" if ok else "batch_halt", answer=answer, note=note)
    if not ok:
        (job_dir / "halt.json").write_text(
            json.dumps({**stop, "answer": answer, "note": note, "at": time.strftime("%Y-%m-%d %H:%M:%S")}, indent=2)
        )
    return ok


def fail_closed(job_dir: Path, task: str, trial: str | None, mode: str | None, error: BaseException) -> dict:
    """A batch hold that could not be checked or recorded (an exception in the trial, in reading its result, in the
    inspection or in writing the logs) ends the batch: nobody knows how the robot was left. Writes halt.json if it can
    and says why on stderr; run-many then exits 3."""
    import sys

    stop = {
        "trial": trial or "(unknown)",
        "task": task,
        "mode": mode or "real",
        "kind": "batch_error",
        "reason": f"the batch hold could not be checked or recorded ({type(error).__name__}: {error})",
        "attended": False,
        "at": time.strftime("%Y-%m-%d %H:%M:%S"),
    }
    try:
        (job_dir / "halt.json").write_text(json.dumps(stop, indent=2))
    except OSError as e:
        stop["halt_json"] = f"not written ({type(e).__name__}: {e})"
    print(f"robouse: {stop['reason']}: the batch stops, no further trial starts", file=sys.stderr, flush=True)
    return stop


def halt_message(stop: dict, not_run: int) -> str:
    return (
        f"robouse: batch halted: trial {stop['trial']} ({stop['task']}, {stop['mode']}) ended in a safety stop "
        f"({stop['kind']}: {stop['reason']}); {not_run} trial(s) not run"
    )
