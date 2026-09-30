"""Human-attended mode: the operator channel between the trusted episode server and the person at the robot.

Real-robot episodes stop at gates the operator must confirm, and real tasks without a measurable success condition ask
the operator for the verdict. Channels (task `real.operator`, or ROBOUSE_OPERATOR):

  tty     prompts on the controlling terminal (/dev/tty) of the process that started the run (default for live runs)
  file    the server writes the request to ~/.config/robouse-operator/pending.json and waits for an answer given with
          `robouse operator answer yes|no [--note TEXT]` from any terminal (headless rigs, remote operators)
  script  answers from ROBOUSE_OPERATOR_SCRIPT, e.g. "arm=yes,reset=yes,verdict=no"; only allowed with the
          hardware-in-the-loop mock (never with a live robot)

Gates: `arm` (before torque is enabled), `reset` (the scene is set up for this episode), `release` (before torque is
released at the end). A gate that is refused or times out stops the episode (`safety_stop`) before anything moves.
Verdicts: yes / no / partial / skip, plus a note; skip and timeouts score 0 and are recorded as unjudged.

Live feedback: `robouse operator say "text"` appends a remark that the next `robo observe` returns under
`operator_notes` (typed, feedback-only: it cannot end the episode or change the verdict).

Nothing here is reachable from the agent: the operator folder is outside the agent's workspace and is denied by the
runner's sandbox profile.
"""

from __future__ import annotations

import json
import time
import uuid
from pathlib import Path

from .. import config
from .safety import OPERATOR_DIR, SafetyStop

VERDICTS = ("yes", "no", "partial", "skip")


class Operator:
    def __init__(self, channel: str, *, mock: bool, timeout_s: float = 600.0, log=None):
        channel = config.env("ROBOUSE_OPERATOR", channel)
        if channel == "script" and not mock:
            raise SafetyStop("the scripted operator is only allowed with the hardware-in-the-loop mock driver")
        if channel not in ("tty", "file", "script"):
            raise ValueError(f"unknown operator channel {channel!r} (tty, file or script)")
        self.channel, self.timeout_s = channel, float(timeout_s)
        self.log = log or (lambda _kind, **kw: None)
        self._script = dict(p.split("=", 1) for p in config.env("ROBOUSE_OPERATOR_SCRIPT").split(",") if "=" in p)
        self._feedback_seen = 0
        self.feedback_file = OPERATOR_DIR / "feedback.jsonl"
        if self.feedback_file.exists():  # remarks written before this episode are not delivered
            self._feedback_seen = len(self.feedback_file.read_text().splitlines())

    # ---- prompts ---------------------------------------------------------------------------------------------------
    def _ask(self, key: str, prompt: str, choices: tuple[str, ...]) -> tuple[str, str]:
        if self.channel == "script":  # mock only; gates default to yes unless the script says otherwise
            ans = self._script.get(key, self._script.get("*", "yes" if key in ("arm", "reset", "release") else ""))
            return (ans if ans in choices else "", "scripted operator (hardware-in-the-loop mock)")
        if self.channel == "tty":
            try:
                with open("/dev/tty", "r+") as tty:
                    tty.write(f"\n[robouse operator] {prompt} [{'/'.join(choices)}] > ")
                    tty.flush()
                    line = tty.readline().strip()
            except OSError as e:
                raise SafetyStop(
                    f"no operator terminal ({e}); use ROBOUSE_OPERATOR=file and `robouse operator answer`"
                ) from e
            ans, _, note = line.partition(" ")
            return (ans.lower() if ans.lower() in choices else "", note.strip())
        # file channel
        OPERATOR_DIR.mkdir(parents=True, exist_ok=True)
        rid = uuid.uuid4().hex[:8]
        pending = OPERATOR_DIR / "pending.json"
        answer = OPERATOR_DIR / f"answer-{rid}.json"
        pending.write_text(
            json.dumps(
                {
                    "id": rid,
                    "key": key,
                    "prompt": prompt,
                    "choices": list(choices),
                    "asked_at": time.strftime("%Y-%m-%d %H:%M:%S"),
                },
                indent=2,
            )
        )
        deadline = time.time() + self.timeout_s
        try:
            while time.time() < deadline:
                if answer.exists():
                    a = json.loads(answer.read_text())
                    answer.unlink()
                    ans = str(a.get("answer", "")).lower()
                    return (ans if ans in choices else "", str(a.get("note", "")))
                time.sleep(0.25)
        finally:
            if pending.exists():
                try:
                    if json.loads(pending.read_text()).get("id") == rid:
                        pending.unlink()
                except (OSError, ValueError):
                    pass
        return ("", "timed out waiting for the operator")

    def gate(self, key: str, prompt: str) -> None:
        ans, note = self._ask(key, prompt, ("yes", "no"))
        self.log("operator_gate", gate=key, answer=ans or "none", note=note, channel=self.channel)
        if ans != "yes":
            raise SafetyStop(f"operator did not confirm the {key} gate ({note or ans or 'no answer'})")

    def verdict(self, prompt: str, choices: tuple[str, ...] = VERDICTS) -> tuple[str, str]:
        ans, note = self._ask("verdict", prompt, choices)
        ans = ans or "skip"
        self.log("operator_verdict", verdict=ans, note=note, channel=self.channel)
        return ans, note

    # ---- live feedback -----------------------------------------------------------------------------------------------
    def new_feedback(self) -> list[str]:
        if not self.feedback_file.exists():
            return []
        lines = self.feedback_file.read_text().splitlines()
        new = lines[self._feedback_seen :]
        self._feedback_seen = len(lines)
        out = []
        for l in new:
            try:
                out.append(str(json.loads(l).get("text", "")))
            except ValueError:
                continue
        if out:
            self.log("operator_feedback", texts=out)
        return [t for t in out if t]


def answer(ans: str, note: str = "") -> str:
    """`robouse operator answer`: answer the pending request."""
    pending = OPERATOR_DIR / "pending.json"
    if not pending.exists():
        raise SystemExit("no pending operator request")
    req = json.loads(pending.read_text())
    if ans not in req["choices"]:
        raise SystemExit(f"answer must be one of {req['choices']}")
    (OPERATOR_DIR / f"answer-{req['id']}.json").write_text(json.dumps({"answer": ans, "note": note}))
    return f"answered {req['key']}: {ans}"


def say(text: str) -> Path:
    OPERATOR_DIR.mkdir(parents=True, exist_ok=True)
    f = OPERATOR_DIR / "feedback.jsonl"
    with open(f, "a") as fh:
        fh.write(json.dumps({"t": time.time(), "text": text}) + "\n")
    return f
