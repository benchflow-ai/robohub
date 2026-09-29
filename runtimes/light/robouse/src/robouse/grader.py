"""VLM grader: a vision-language model judges an episode from its camera frames and a rubric.

Used two ways:
- `robouse grade TRIAL... [--model M] [--rubric FILE]`: a post-hoc second opinion on finished trials. It sends the first,
  middle and last frames of the episode video with the task's goal and success rule, and writes
  `<trial>/grader/vlm.json` (verdict, optional rubric stage, reason, tokens, cost, agreement with the trusted verdict).
  `robouse report` then shows `vlm_agree` (agreement rate) per group.
- `real.verdict: vlm` on a real-robot task: the episode server asks the model at the end of the episode, from the
  final camera frames (for rigs without an operator; the operator verdict stays the default).

Any OpenAI-compatible chat endpoint with image input works; the default is Kimi K3 on Baseten (BASETEN_API_KEY).
"""
from __future__ import annotations

import base64
import io
import json
import os
import re
import time
import urllib.request
from pathlib import Path

import numpy as np

DEFAULT_MODEL = "moonshotai/Kimi-K3"
DEFAULT_URL = "https://inference.baseten.co/v1"

PROMPT = """You are grading one episode of a robot task from camera frames taken during the episode (in time order; the last frame is the final state).

Task and success rule:
{task}

{rubric}Judge only what the frames show. Answer with one JSON object and nothing else:
{{"verdict": "yes" or "no" (did the robot meet the success rule by the end?), {stage}"reason": "one or two sentences"}}"""


def _png_b64(img: np.ndarray, max_side: int = 512) -> str:
    from PIL import Image

    im = Image.fromarray(np.asarray(img, dtype=np.uint8))
    s = max_side / max(im.size)
    if s < 1:
        im = im.resize((int(im.width * s), int(im.height * s)))
    b = io.BytesIO()
    im.save(b, "PNG")
    return base64.b64encode(b.getvalue()).decode()


def _key() -> str:
    k = os.environ.get("ROBOUSE_GRADER_API_KEY") or os.environ.get("BASETEN_API_KEY")
    if not k:
        from .harnesses import BASETEN_ENV, _read_env

        k = _read_env(BASETEN_ENV).get("BASETEN_API_KEY", "")
    if not k:
        raise RuntimeError("no grader key: set BASETEN_API_KEY (or ROBOUSE_GRADER_API_KEY with ROBOUSE_GRADER_URL)")
    return k


def vlm_verdict(frames: list[np.ndarray], task: str, rubric: list[str] | None = None, model: str = DEFAULT_MODEL) -> dict:
    rub = ("Rubric (report the highest stage reached):\n" + "\n".join(f"{i}: {d}" for i, d in enumerate(rubric)) + "\n\n") if rubric else ""
    text = PROMPT.format(task=task.strip(), rubric=rub, stage='"stage": integer, ' if rubric else "")
    content = [{"type": "text", "text": text}] + [{"type": "image_url", "image_url": {"url": "data:image/png;base64," + _png_b64(f)}}
                                                  for f in frames]
    body = {"model": model, "max_tokens": 8000, "messages": [{"role": "user", "content": content}]}
    url = os.environ.get("ROBOUSE_GRADER_URL", DEFAULT_URL).rstrip("/") + "/chat/completions"
    last = None
    for attempt in range(3):
        try:
            req = urllib.request.Request(url, data=json.dumps(body).encode(), headers={"Content-Type": "application/json",
                                                                                         "Authorization": f"Api-Key {_key()}"})
            with urllib.request.urlopen(req, timeout=300) as r:
                resp = json.loads(r.read())
            break
        except Exception as e:  # noqa: BLE001
            last = e
            time.sleep(3 * (attempt + 1))
    else:
        raise RuntimeError(f"grader request failed: {last}")
    msg = resp["choices"][0]["message"].get("content") or ""
    m = re.search(r"\{.*\}", msg, re.S)
    try:
        out = json.loads(m.group(0)) if m else {}
    except ValueError:
        out = {}
    verdict = str(out.get("verdict", "")).lower()
    usage = resp.get("usage") or {}
    res = {"model": model, "verdict": verdict if verdict in ("yes", "no") else "unparsed", "reason": out.get("reason", msg[:300]),
           "prompt_tokens": usage.get("prompt_tokens"), "completion_tokens": usage.get("completion_tokens")}
    if rubric:
        res["stage"] = out.get("stage")
    try:
        price = json.loads(Path(__file__).with_name("pricing.json").read_text()).get(model)
        if price and usage:
            res["cost_usd"] = round((usage.get("prompt_tokens", 0) * price["input"] + usage.get("completion_tokens", 0) * price["output"]) / 1e6, 5)
    except (OSError, ValueError):
        pass
    return res


def _frames(trial: Path, n: int = 3) -> list[np.ndarray]:
    import imageio.v2 as imageio

    vid = trial / "artifacts" / "recording.mp4"
    if not vid.exists():
        vid = trial / "episode" / "recording.mp4"
    r = imageio.get_reader(vid)
    try:
        total = r.count_frames()
        idx = sorted({0, total // 2, total - 1}) if n >= 3 else [total - 1]
        return [r.get_data(i) for i in idx]
    finally:
        r.close()


def _task_text(trial: Path) -> tuple[str, list[str] | None]:
    from .tasks import load_task

    cfg = json.loads((trial / "config.json").read_text())
    t = load_task(cfg["task_path"])
    body = t.body
    m = re.search(r"## Task\n\n(.*?)(?:\n\*\*Controls|\n## )", body, re.S)
    return (m.group(1).strip() if m else body[:1500]), (t.spec.get("real") or {}).get("rubric")


def grade_trial(trial: Path, model: str = DEFAULT_MODEL, rubric_file: str | None = None) -> dict:
    task, rubric = _task_text(trial)
    if rubric_file:
        task += "\n\n" + Path(rubric_file).read_text()
    res = vlm_verdict(_frames(trial), task, rubric, model)
    r = json.loads((trial / "result.json").read_text())
    trusted = float((r.get("verifier_result") or {}).get("rewards", {}).get("reward", 0)) >= 1
    res["trusted_success"] = trusted
    res["agrees"] = res["verdict"] in ("yes", "no") and (res["verdict"] == "yes") == trusted
    (trial / "grader").mkdir(exist_ok=True)
    (trial / "grader" / "vlm.json").write_text(json.dumps(res, indent=2))
    return res


def main(trials: list[str], model: str, rubric: str | None) -> int:
    paths = []
    for t in trials:
        p = Path(t)
        paths += [d for d in sorted(p.iterdir()) if (d / "result.json").exists() and (d / "config.json").exists()] if not (p / "config.json").exists() else [p]
    agree = n = 0
    for p in paths:
        try:
            r = grade_trial(p, model, rubric)
        except Exception as e:  # noqa: BLE001
            print(f"{p.name}: grader error {type(e).__name__}: {str(e)[:200]}")
            continue
        n += 1
        agree += int(r["agrees"])
        print(f"{p.name}: vlm={r['verdict']} trusted={'yes' if r['trusted_success'] else 'no'} {'agree' if r['agrees'] else 'DISAGREE'}"
              f" ({r.get('reason', '')[:120]})")
    if n:
        print(f"agreement {agree}/{n}")
    return 0
