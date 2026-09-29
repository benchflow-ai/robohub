"""Convert raw harness output into an ATIF trajectory (agent/trajectory.json) and a video index
(artifacts/recording_index.json). See docs/viewer.md.

Inputs (all optional; missing or malformed files degrade to fewer steps, never to an exception):
  agent/stdout.jsonl       Claude Code stream-json, `codex exec --json` events, or plain text
  agent/mini_traj.json     mini-swe-agent trajectory (mini-swe-agent-glm)
  episode/trace.jsonl      every robo request the episode server handled: {t, steps, req, resp}
  episode/frames.jsonl     one line per video frame: {i, t, step}; the video plays them at FPS
  episode/result.json      outcome, agent_text, wall_time_s

Outputs
  agent/trajectory.json    ATIF-v1.7 (the agent trajectory format the BenchFlow viewer and trainer read). One user step with the
                           prompt, then one step per assistant message (Claude Code), per item (Codex), per
                           assistant turn (mini-swe-agent) or per group of robo requests (oracle, noop).
  artifacts/recording_index.json
                           the BenchFlow viewer's recording index: for every trajectory step, the video
                           time span of the robo requests it made ({index, action, t_video_start_s,
                           t_video_end_s, ...}).

Aligning LLM tool calls with the trace. Neither Claude Code nor Codex timestamps its events, so tool calls
are matched to trace entries by order: each shell command consumes as many trace entries as it made robo
requests. That count is the larger of the `robo <subcommand>` invocations written in the command and the
result blocks in its output (loops print one block per call); then the state printed last in the output
(hand_pos and steps_used) is looked up in the trace to correct the end point. Requests the runner made
itself (status, the closing give_up) are not the agent's and become a final system step.

  python -m robouse.atif TRIAL_DIR [TRIAL_DIR ...]     (re)build both files for existing trials
"""
from __future__ import annotations

import json
import os
import re
import sys
from bisect import bisect_right
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

FPS = 30
OBS_LIMIT = 4000  # characters kept of one tool result (head and tail)
MSG_LIMIT = 20000
ORACLE_ACTS_PER_STEP = 10
# what the harnesses run when config.json leaves the model empty (see harnesses.py)
DEFAULT_MODELS = {"claude-code": "claude-opus-5-5", "codex": "gpt-6-astra", "claude-code-glm": "zai-org/GLM-5.3",
                  "codex-glm": "zai-org/GLM-5.3", "mini-swe-agent-glm": "zai-org/GLM-5.3", "oracle": "scripted", "noop": "none"}
ROBO_SUBCOMMANDS = ("info", "observe", "act", "move-to", "move_to", "grip", "done", "give-up", "give_up")
ROBO_RE = re.compile(r"(?:^|[^\w.-])robo\s+(" + "|".join(re.escape(s) for s in ROBO_SUBCOMMANDS) + r")(?![\w-])", re.M)
SCRIPT_RE = re.compile(r"(?:^|[\s;&|(])(?:python\d?(?:\.\d+)?|bash|sh|zsh|node|\./)", re.M)
LOOP_RE = re.compile(r"(?:^|[\s;&|(])(?:for|while|until)\s|\bxargs\b|\bseq\b", re.M)
RESULT_MARK_RE = re.compile(r'^(?:steps_used: |outcome: |error: |\{"ok": )', re.M)
HAND_RE = re.compile(r'"?hand_pos"?:\s*(\[[^\]]*\])')
STEPS_RE = re.compile(r'"?steps_used"?:\s*(\d+)')


# ---------------------------------------------------------------------------------------------
# small helpers


def _read_json(p: Path) -> Any:
    try:
        return json.loads(p.read_text(errors="replace"))
    except (OSError, ValueError):
        return None


def _read_jsonl(p: Path) -> list[dict]:
    out: list[dict] = []
    try:
        lines = p.read_text(errors="replace").splitlines()
    except OSError:
        return out
    for line in lines:
        line = line.strip()
        if not line.startswith("{"):
            continue
        try:
            d = json.loads(line)
        except ValueError:
            continue
        if isinstance(d, dict):
            out.append(d)
    return out


def _clip(text: str, limit: int = OBS_LIMIT) -> str:
    if len(text) <= limit:
        return text
    head = int(limit * 0.7)
    tail = limit - head
    return f"{text[:head]}\n[... {len(text) - limit} characters omitted ...]\n{text[-tail:]}"


def _iso(epoch: float | None) -> str | None:
    if epoch is None:
        return None
    return datetime.fromtimestamp(epoch, tz=timezone.utc).isoformat().replace("+00:00", "Z")


def _parse_iso(s: Any) -> float | None:
    if not isinstance(s, str):
        return None
    try:
        return datetime.fromisoformat(s.replace("Z", "+00:00")).timestamp()
    except ValueError:
        return None


def _text_of(content: Any) -> str:
    """Plain text of a tool result / message content (str, list of blocks, or anything else)."""
    if content is None:
        return ""
    if isinstance(content, str):
        return content
    if isinstance(content, list):
        parts = []
        for b in content:
            if isinstance(b, dict):
                if b.get("type") == "text":
                    parts.append(str(b.get("text", "")))
                elif b.get("type") == "image":
                    parts.append("[image]")
                elif "text" in b:
                    parts.append(str(b["text"]))
            elif isinstance(b, str):
                parts.append(b)
        return "\n".join(parts)
    if isinstance(content, dict):
        return str(content.get("text") or json.dumps(content)[:OBS_LIMIT])
    return str(content)


def _unwrap_shell(cmd: str) -> str:
    """`/bin/zsh -lc 'robo info'` -> `robo info` (Codex wraps every command)."""
    m = re.match(r"^\S*/(?:ba|z)?sh\s+-l?c\s+(['\"])(.*)\1\s*$", cmd.strip(), re.S)
    if m:
        inner = m.group(2)
        return inner.replace("'\\''", "'") if m.group(1) == "'" else inner
    return cmd


def _label(cmd: str, limit: int = 90) -> str:
    one = " ".join(cmd.split())
    return one if len(one) <= limit else one[: limit - 1] + "…"


# ---------------------------------------------------------------------------------------------
# intermediate representation: a list of steps, each a dict
#   {source, message, reasoning, calls: [{id, name, args, command, output, is_error, images}],
#    model, metrics, epoch, extra}


def _step(source: str, message: str = "", **kw) -> dict:
    return {"source": source, "message": message, "reasoning": kw.get("reasoning"), "calls": kw.get("calls") or [],
            "model": kw.get("model"), "metrics": kw.get("metrics"), "epoch": kw.get("epoch"), "extra": kw.get("extra") or {},
            "notes": kw.get("notes") or []}


def _call(cid: str, name: str, args: dict, command: str | None = None) -> dict:
    return {"id": cid, "name": name, "args": args if isinstance(args, dict) else {"input": args}, "command": command,
            "output": None, "is_error": None, "images": [], "trace": [], "epoch_end": None}


# ---------------------------------------------------------------------------------------------
# Claude Code stream-json


def _parse_claude(events: list[dict], trial: Path) -> tuple[list[dict], dict]:
    steps: list[dict] = []
    by_msg: dict[str, dict] = {}
    by_call: dict[str, dict] = {}
    meta: dict[str, Any] = {"events": len(events)}
    for ev in events:
        et = ev.get("type")
        if et == "system" and ev.get("subtype") == "init":
            meta.update({k: ev.get(k) for k in ("session_id", "model", "cwd", "claude_code_version", "permissionMode") if ev.get(k)})
            meta["tools"] = ev.get("tools")
        elif et == "system":
            # api retries, compaction and similar notices: keep them visible
            sub = ev.get("subtype") or "system"
            txt = ev.get("message") or ev.get("error") or ev.get("content") or ""
            if sub not in ("init",) and (txt or sub in ("api_retry", "compact_boundary")):
                steps.append(_step("system", _clip(f"[{sub}] {_text_of(txt) if not isinstance(txt, str) else txt}".strip())))
        elif et == "assistant":
            msg = ev.get("message") or {}
            mid = msg.get("id") or f"anon-{len(steps)}"
            st = by_msg.get(mid)
            if st is None:
                st = _step("agent", model=msg.get("model"))
                by_msg[mid] = st
                steps.append(st)
                u = msg.get("usage") or {}
                if u:
                    cached = int(u.get("cache_read_input_tokens") or 0)
                    prompt = int(u.get("input_tokens") or 0) + cached + int(u.get("cache_creation_input_tokens") or 0)
                    st["metrics"] = {"prompt_tokens": prompt, "completion_tokens": int(u.get("output_tokens") or 0),
                                     "cached_tokens": cached}
            texts, thinks = [], []
            for b in msg.get("content") or []:
                if not isinstance(b, dict):
                    continue
                bt = b.get("type")
                if bt == "text":
                    texts.append(str(b.get("text", "")))
                elif bt == "thinking":
                    if b.get("thinking"):
                        thinks.append(str(b["thinking"]))
                elif bt == "redacted_thinking":
                    thinks.append("[redacted thinking]")
                elif bt in ("tool_use", "server_tool_use"):
                    inp = b.get("input") if isinstance(b.get("input"), dict) else {"input": b.get("input")}
                    name = str(b.get("name") or "tool")
                    cmd = inp.get("command") if name == "Bash" and isinstance(inp.get("command"), str) else None
                    c = _call(str(b.get("id") or f"call-{len(by_call)}"), name, inp, cmd)
                    st["calls"].append(c)
                    by_call[c["id"]] = c
            if texts:
                st["message"] = "\n\n".join(x for x in [st["message"], *texts] if x)
            if thinks:
                st["reasoning"] = "\n\n".join(x for x in [st["reasoning"] or "", *thinks] if x)
            if ev.get("error"):
                st["extra"]["error"] = ev.get("error")
        elif et == "user":
            msg = ev.get("message") or {}
            content = msg.get("content")
            if isinstance(content, str):
                if content.strip():
                    steps.append(_step("user", _clip(content, MSG_LIMIT)))
                continue
            for b in content or []:
                if not isinstance(b, dict):
                    continue
                if b.get("type") == "tool_result":
                    c = by_call.get(str(b.get("tool_use_id")))
                    out = b.get("content")
                    text = _text_of(out)
                    if c is None:
                        steps.append(_step("system", _clip(f"[tool result without a matching call] {text}")))
                        continue
                    c["output"] = text
                    c["is_error"] = bool(b.get("is_error"))
                    if isinstance(out, list):
                        c["images"] += [x for x in out if isinstance(x, dict) and x.get("type") == "image"]
                elif b.get("type") == "text" and str(b.get("text", "")).strip():
                    steps.append(_step("user", _clip(str(b["text"]), MSG_LIMIT)))
        elif et == "result":
            meta["result"] = {k: ev.get(k) for k in ("subtype", "is_error", "num_turns", "duration_ms", "duration_api_ms",
                                                     "total_cost_usd", "stop_reason", "usage", "result", "api_error_status")
                              if ev.get(k) is not None}
        elif et == "rate_limit_event":
            info = ev.get("rate_limit_info") or {}
            if info.get("status") not in (None, "allowed", "allowed_warning"):
                steps.append(_step("system", f"[rate limit] {json.dumps(info)[:500]}"))
    # Read of an observation image: point the image at the saved copy instead of embedding base64
    for c in by_call.values():
        if c["images"]:
            fp = str(c["args"].get("file_path") or "")
            name = os.path.basename(fp)
            if name and (trial / "artifacts" / "observations" / name).is_file():
                c["image_paths"] = [f"../artifacts/observations/{name}"]
    return steps, meta


def _claude_final(meta: dict) -> dict | None:
    r = meta.get("result") or {}
    u = r.get("usage") or {}
    if not r:
        return None
    cached = int(u.get("cache_read_input_tokens") or 0)
    prompt = int(u.get("input_tokens") or 0) + cached + int(u.get("cache_creation_input_tokens") or 0)
    fm: dict[str, Any] = {"total_prompt_tokens": prompt or None, "total_completion_tokens": u.get("output_tokens"),
                          "total_cached_tokens": cached or None, "total_cost_usd": r.get("total_cost_usd")}
    fm["extra"] = {k: r[k] for k in ("subtype", "num_turns", "duration_ms", "stop_reason", "is_error") if k in r}
    return {k: v for k, v in fm.items() if v is not None}


# ---------------------------------------------------------------------------------------------
# Codex exec --json


def _parse_codex(events: list[dict], trial: Path) -> tuple[list[dict], dict]:
    steps: list[dict] = []
    meta: dict[str, Any] = {"events": len(events), "usage": []}
    pending_reasoning: list[str] = []

    def add(st: dict) -> None:
        if pending_reasoning and st["source"] == "agent":
            st["reasoning"] = "\n\n".join(pending_reasoning)
            pending_reasoning.clear()
        steps.append(st)

    items: dict[str, dict] = {}
    seq: list[Any] = []  # item ids (first-seen order) and system steps, in stream order
    for ev in events:
        et = ev.get("type")
        if et == "thread.started":
            meta["session_id"] = ev.get("thread_id")
        elif et in ("item.started", "item.updated", "item.completed"):
            it = ev.get("item") or {}
            iid = str(it.get("id") or f"item-{len(seq)}")
            if iid not in items:
                seq.append(iid)
            prev = items.get(iid, {})
            items[iid] = {**prev, **it, "_done": et == "item.completed" or prev.get("_done", False)}
        elif et == "turn.completed":
            meta["usage"].append(ev.get("usage") or {})
        elif et in ("turn.failed", "error"):
            err = ev.get("error") or ev.get("message") or ev
            seq.append(_step("system", _clip(f"[{et}] {err if isinstance(err, str) else json.dumps(err)}")))
    for s in seq:
        if isinstance(s, dict):
            add(s)
            continue
        it = items[s]
        typ = it.get("type")
        if typ == "reasoning":
            if it.get("text"):
                pending_reasoning.append(str(it["text"]))
        elif typ == "agent_message":
            add(_step("agent", _clip(str(it.get("text", "")), MSG_LIMIT)))
        elif typ == "command_execution":
            raw = str(it.get("command", ""))
            cmd = _unwrap_shell(raw)
            c = _call(str(it.get("id")), "shell", {"command": cmd}, cmd)
            c["output"] = it.get("aggregated_output") if it.get("_done") else (it.get("aggregated_output") or "[no result: command did not finish]")
            c["is_error"] = it.get("exit_code") not in (0, None) or it.get("status") == "failed"
            c["exit_code"] = it.get("exit_code")
            add(_step("agent", calls=[c]))
        elif typ == "file_change":
            c = _call(str(it.get("id")), "apply_patch", {"changes": it.get("changes")})
            c["output"] = it.get("status")
            add(_step("agent", calls=[c]))
        elif typ == "mcp_tool_call":
            c = _call(str(it.get("id")), f"{it.get('server')}.{it.get('tool')}", it.get("arguments") or {})
            c["output"] = _text_of((it.get("result") or {}).get("content") if isinstance(it.get("result"), dict) else it.get("result")) or it.get("error")
            add(_step("agent", calls=[c]))
        elif typ == "web_search":
            c = _call(str(it.get("id")), "web_search", {"query": it.get("query")})
            add(_step("agent", calls=[c]))
        elif typ == "todo_list":
            items_txt = "\n".join(f"[{'x' if t.get('completed') else ' '}] {t.get('text')}" for t in it.get("items") or [] if isinstance(t, dict))
            add(_step("agent", f"(plan)\n{items_txt}"))
        elif typ == "error":
            add(_step("system", _clip(f"[codex] {it.get('message', '')}")))
        else:
            add(_step("agent", _clip(f"[{typ}] {json.dumps({k: v for k, v in it.items() if not k.startswith('_')})}")))
    if pending_reasoning:
        steps.append(_step("agent", "", reasoning="\n\n".join(pending_reasoning)))
    return steps, meta


def _codex_final(meta: dict) -> dict | None:
    us = meta.get("usage") or []
    if not us:
        return None
    tot = lambda k: sum(int(u.get(k) or 0) for u in us)  # noqa: E731
    fm = {"total_prompt_tokens": tot("input_tokens"), "total_completion_tokens": tot("output_tokens"),
          "total_cached_tokens": tot("cached_input_tokens")}
    fm["extra"] = {"reasoning_output_tokens": tot("reasoning_output_tokens"), "turns": len(us)}
    return fm


# ---------------------------------------------------------------------------------------------
# mini-swe-agent (mini_traj.json, trajectory_format mini-swe-agent-1.x)


def _parse_mini(data: dict) -> tuple[list[dict], dict]:
    steps: list[dict] = []
    meta: dict[str, Any] = {"info": {k: v for k, v in (data.get("info") or {}).items() if k != "config"}}
    by_call: dict[str, dict] = {}
    last_agent: dict | None = None
    msgs = data.get("messages") or []
    for i, m in enumerate(msgs):
        if not isinstance(m, dict):
            continue
        role = m.get("role")
        ex = m.get("extra") or {}
        epoch = ex.get("timestamp") if isinstance(ex.get("timestamp"), (int, float)) else None
        content = _text_of(m.get("content"))
        if role == "system":
            continue  # the harness's own system prompt; the task is the first user message
        if role == "assistant":
            st = _step("agent", _clip(content, MSG_LIMIT), epoch=epoch, model=None)
            st["reasoning"] = m.get("reasoning_content") or None
            tcs = m.get("tool_calls") or []
            for tc in tcs:
                if not isinstance(tc, dict):
                    continue
                fn = tc.get("function") or {}
                try:
                    args = json.loads(fn.get("arguments") or "{}")
                except ValueError:
                    args = {"raw": fn.get("arguments")}
                cmd = args.get("command") if isinstance(args, dict) else None
                c = _call(str(tc.get("id") or f"call-{i}"), str(fn.get("name") or "bash"), args, cmd if isinstance(cmd, str) else None)
                st["calls"].append(c)
                by_call[c["id"]] = c
            if not tcs:  # text-based format: actions parsed from ```bash blocks
                for k, a in enumerate(ex.get("actions") or []):
                    if isinstance(a, dict) and isinstance(a.get("command"), str):
                        c = _call(f"call-{i}-{k}", "bash", {"command": a["command"]}, a["command"])
                        st["calls"].append(c)
            u = (ex.get("response") or {}).get("usage") if isinstance(ex.get("response"), dict) else None
            if isinstance(u, dict):
                st["metrics"] = {"prompt_tokens": u.get("prompt_tokens"), "completion_tokens": u.get("completion_tokens"),
                                 "cost_usd": ex.get("cost")}
            steps.append(st)
            last_agent = st
        elif role == "tool" or (role == "user" and last_agent is not None and "raw_output" in ex):
            c = by_call.get(str(m.get("tool_call_id")))
            if c is None and last_agent is not None:
                c = next((x for x in last_agent["calls"] if x["output"] is None), None)
            if c is None:
                steps.append(_step("system", _clip(content), epoch=epoch))
                continue
            c["output"] = ex.get("raw_output") if isinstance(ex.get("raw_output"), str) else content
            c["is_error"] = ex.get("returncode") not in (0, None)
            c["epoch_end"] = epoch
        elif role == "user":
            steps.append(_step("user", _clip(content, MSG_LIMIT), epoch=epoch))
        elif role == "exit":
            steps.append(_step("system", f"[exit] {content} {json.dumps(ex)[:500] if ex else ''}".strip(), epoch=epoch))
    return steps, meta


# ---------------------------------------------------------------------------------------------
# no-LLM harnesses: steps straight from the trace


def _req_label(req: dict) -> str:
    op = req.get("op")
    if op == "act":
        a = req.get("action") or []
        r = req.get("repeat", 1)
        return f"robo act {' '.join(f'{x:.2f}' if isinstance(x, (int, float)) else str(x) for x in a)}" + (f" --repeat {r}" if r and r != 1 else "")
    if op == "move_to":
        p = req.get("pos") or []
        return f"robo move-to {' '.join(str(x) for x in p)}" + (f" --grip {req['grip']}" if req.get("grip") is not None else "")
    if op == "grip":
        return f"robo grip {req.get('value')}"
    if op == "observe":
        return "robo observe" + (" --image" if req.get("image") else "")
    if op in ("done", "give_up"):
        return f"robo {op.replace('_', '-')} {json.dumps(req.get('text', ''))}" if req.get("text") else f"robo {op.replace('_', '-')}"
    return f"robo {op}"


def _resp_text(resp: dict) -> str:
    if not isinstance(resp, dict):
        return str(resp)
    if not resp.get("ok"):
        return f"error: {resp.get('error')}"
    r = resp.get("result")
    if not isinstance(r, dict):
        return str(r)
    lines = []
    for k, v in r.items():
        if k == "state" and isinstance(v, dict):
            lines.append("state: " + ", ".join(f"{a}={b}" for a, b in v.items()))
        else:
            lines.append(f"{k}: {v}")
    return "\n".join(lines)


def _steps_from_trace(entries: list[int], trace: list[dict], harness: str) -> list[dict]:
    """Group the agent's trace entries: up to ORACLE_ACTS_PER_STEP motion requests (and the observes around
    them) per step; done/give_up get their own step."""
    steps: list[dict] = []
    cur: list[int] = []
    acts = 0

    def flush() -> None:
        nonlocal cur, acts
        if not cur:
            return
        st = _step("agent", "")
        st["llm_call_count"] = 0
        for j in cur:
            e = trace[j]
            c = _call(f"trace-{j}", "robo", e.get("req") or {}, _req_label(e.get("req") or {}))
            c["output"] = _resp_text(e.get("resp") or {})
            c["is_error"] = not (e.get("resp") or {}).get("ok", False)
            c["trace"] = [j]
            st["calls"].append(c)
        ops = [trace[j].get("req", {}).get("op") for j in cur]
        motion = [o for o in ops if o in ("act", "move_to", "grip")]
        st["message"] = (f"{harness}: {len(motion)} motion request(s)" if motion else f"{harness}: {', '.join(str(o) for o in ops)}")
        if any(o in ("done", "give_up") for o in ops):
            txt = next((trace[j]["req"].get("text") for j in cur if trace[j].get("req", {}).get("op") in ("done", "give_up")), "")
            st["message"] = f"{harness}: {ops[-1]}" + (f" ({txt})" if txt else "")
        steps.append(st)
        cur, acts = [], 0

    for j in entries:
        op = (trace[j].get("req") or {}).get("op")
        if op in ("done", "give_up"):
            flush()
            cur = [j]
            flush()
            continue
        if op in ("act", "move_to", "grip"):
            if acts >= ORACLE_ACTS_PER_STEP:
                flush()
            acts += 1
        cur.append(j)
    flush()
    return steps


# ---------------------------------------------------------------------------------------------
# aligning tool calls with trace entries


def _is_runner_entry(e: dict) -> bool:
    req = e.get("req") or {}
    return req.get("op") == "status" or (req.get("op") == "give_up" and str(req.get("text", "")).startswith("[runner]"))


def _entry_sig(e: dict) -> tuple | None:
    """What `robo` prints for this request that is specific enough to find it again: the hand position
    (and steps_used) of a state, or the error text."""
    resp = e.get("resp") or {}
    if not resp.get("ok", True):
        return ("error", str(resp.get("error", "")).strip())
    r = resp.get("result")
    if not isinstance(r, dict):
        return None
    st = r.get("state")
    hp = st.get("hand_pos") if isinstance(st, dict) else None
    if hp is None:
        return None
    return ("state", json.dumps(hp).replace(" ", ""), int(r.get("steps_used") or 0))


ERROR_LINE_RE = re.compile(r'^error: (.+)$|"error": "((?:[^"\\]|\\.)*)"', re.M)


def _output_sig(out: str) -> tuple[tuple, int] | None:
    """Signature of the last recognisable robo result printed in a tool result (a state's hand_pos with
    steps_used when shown, or an error line), and how many result blocks follow it. Agents often filter
    robo output with grep, so steps_used may be missing; hand_pos alone (4 decimals, changes whenever the
    arm moves) still matches."""
    hs = list(HAND_RE.finditer(out))
    es = list(ERROR_LINE_RE.finditer(out))
    last_h = hs[-1] if hs else None
    last_e = es[-1] if es else None
    if last_h is None and last_e is None:
        return None
    if last_e is not None and (last_h is None or last_e.start() > last_h.start()):
        sig: tuple = ("error", (last_e.group(1) or last_e.group(2) or "").strip())
        anchor = last_e.end()
    else:
        ss = [m for m in STEPS_RE.finditer(out) if m.start() > last_h.end()]
        steps = int(ss[0].group(1)) if ss else None
        sig = ("state", last_h.group(1).replace(" ", ""), steps)
        anchor = ss[0].end() if ss else last_h.end()
    after = sum(1 for m in RESULT_MARK_RE.finditer(out) if m.start() > anchor)
    return sig, after


def _sig_match(entry_sig: tuple | None, osig: tuple) -> bool:
    if entry_sig is None or entry_sig[0] != osig[0]:
        return False
    if osig[0] == "error":
        return entry_sig[1] == osig[1]
    return entry_sig[1] == osig[1] and (osig[2] is None or entry_sig[2] == osig[2])


def _align(steps: list[dict], agent_entries: list[int], trace: list[dict]) -> list[int]:
    """Assign agent trace entries to tool calls (call['trace']), in order. Returns the entries left over.

    Each shell call is expected to own k consecutive entries (k = robo invocations in the command, or the
    result blocks in its output when that is larger). The last result it printed is then looked up in the
    trace to fix the end point. A script (python, bash, ...) whose output shows no robo results owns the
    entries between the previous matched call and the next one."""
    sigs = [_entry_sig(trace[j]) for j in agent_entries]
    n = len(agent_entries)
    calls = [c for st in steps for c in st["calls"] if isinstance(c.get("command"), str)]
    k_static = [len(ROBO_RE.findall(c["command"])) for c in calls]
    need_after = [sum(k_static[i + 1:]) for i in range(len(calls))]
    pos = 0
    pending: dict | None = None  # an earlier call that may have made more requests than we could see
    blind = False  # True when that call showed no robo results at all (a script)
    for i, c in enumerate(calls):
        if pos >= n:
            break
        cmd, out = c["command"], c.get("output") or ""
        scripty = bool(SCRIPT_RE.search(cmd))
        ks = k_static[i]
        k_out = max(len(RESULT_MARK_RE.findall(out)), len(HAND_RE.findall(out))) if (ks or scripty) else 0
        k = max(ks, k_out)
        end = pos + k - 1 if k else None
        found = _output_sig(out) if (ks or scripty) else None
        if found is not None:
            osig, after = found
            hi = min(n, pos + max(k, 1) + 400)
            cands = [q for q in range(pos, hi) if _sig_match(sigs[q], osig) and q + after <= n - 1 - need_after[i]] or \
                    [q for q in range(pos, hi) if _sig_match(sigs[q], osig)]
            if cands:
                if pending is not None and blind:
                    best = max(cands)  # a script made requests we could not see: take the latest fit
                else:
                    target = (end - after) if end is not None else pos
                    best = min(cands, key=lambda q: (abs(q - target), q))
                end = best + after
        if end is None:
            if scripty and not ks:
                pending, blind = c, True
            continue
        end = min(max(end, pos), n - 1)
        start = pos
        own_start = max(pos, end - max(k, 1) + 1)
        if pending is not None and own_start > pos:
            # requests before this command's own ones: the earlier script or loop made them without printing
            pending["trace"] = (pending.get("trace") or []) + agent_entries[pos:own_start]
            start = own_start
        c["trace"] = agent_entries[start:end + 1]
        pos = end + 1
        pending, blind = (c, False) if (scripty or LOOP_RE.search(cmd)) else (None, False)
    rest = agent_entries[pos:]
    if rest and pending is not None:
        pending["trace"] = (pending.get("trace") or []) + rest
        rest = []
    return rest


# ---------------------------------------------------------------------------------------------
# video time


class Video:
    def __init__(self, frames: list[dict], fps: int = FPS):
        fr = [f for f in frames if isinstance(f.get("t"), (int, float))]
        self.frames = fr
        self.ts = [float(f["t"]) for f in fr]
        self.fps = fps
        self.n = len(fr)

    def idx_at(self, t: float) -> int:
        """Index of the last frame recorded at or before wall time t (0 if none)."""
        return max(0, bisect_right(self.ts, t + 1e-6) - 1)

    def span(self, t0: float, t1: float | None) -> tuple[float, float, float | None]:
        """Video seconds for the frames recorded in [t0, t1): start shows the state just before t0."""
        if not self.n:
            return 0.0, 0.0, None
        a = self.idx_at(t0)
        if t1 is None:
            b = self.n - 1
        else:
            b = max(a, bisect_right(self.ts, t1 - 1e-6) - 1)
        real = (self.ts[b] - t0) if b > a else None
        return round(a / self.fps, 3), round((b + 1) / self.fps, 3), (round(real, 3) if real is not None and real >= 0 else None)

    @property
    def duration(self) -> float:
        return round(self.n / self.fps, 3)


# ---------------------------------------------------------------------------------------------
# main entry


def _episode_epoch(trial: Path, episode: dict) -> float | None:
    """Wall-clock epoch of the episode's t = 0: episode/result.json is written at t = wall_time_s."""
    p = trial / "episode" / "result.json"
    try:
        mt = p.stat().st_mtime
    except OSError:
        return None
    w = episode.get("wall_time_s")
    if not isinstance(w, (int, float)):
        return None
    return mt - float(w)


# harnesses without an LLM transcript: their steps come from the episode server's trace
TRACE_HARNESSES = ("oracle", "noop", "molmoact2", "vla")


def _prompt(trial: Path, cfg: dict, harness: str) -> str:
    instr = ""
    tp = cfg.get("task_path")
    if tp:
        try:
            from .tasks import load_task

            instr = load_task(tp).instruction
        except Exception:
            instr = ""
    if harness in TRACE_HARNESSES:
        return instr or f"task {cfg.get('task')}"
    try:
        from .harnesses import PROMPT_PREFIX
    except Exception:  # pragma: no cover
        PROMPT_PREFIX = ""
    return PROMPT_PREFIX + instr if instr else f"(prompt not recorded) task {cfg.get('task')}"


def _to_atif_step(sid: int, st: dict, default_model: str | None) -> dict:
    out: dict[str, Any] = {"step_id": sid, "source": st["source"]}
    if st.get("epoch"):
        out["timestamp"] = _iso(st["epoch"])
    msg = st.get("message") or ""
    out["message"] = msg
    if st["source"] == "agent":
        if st.get("model") and st["model"] != default_model:
            out["model_name"] = st["model"]
        if st.get("reasoning") and st.get("llm_call_count") != 0:
            out["reasoning_content"] = st["reasoning"]
        if st["calls"]:
            out["tool_calls"] = [{"tool_call_id": c["id"], "function_name": c["name"], "arguments": c["args"]}
                                 for c in st["calls"]]
            results = []
            for c in st["calls"]:
                content: Any = _clip(c["output"]) if isinstance(c.get("output"), str) else ("" if c.get("output") is None else _clip(json.dumps(c["output"])))
                if c.get("image_paths"):
                    content = ([{"type": "text", "text": content}] if content else []) + \
                        [{"type": "image", "source": {"media_type": "image/png", "path": p}} for p in c["image_paths"]]
                r: dict[str, Any] = {"source_call_id": c["id"], "content": content}
                ex = {}
                if c.get("is_error"):
                    ex["is_error"] = True
                if c.get("exit_code") is not None:
                    ex["exit_code"] = c["exit_code"]
                if c.get("trace"):
                    ex["robo_requests"] = len(c["trace"])
                if ex:
                    r["extra"] = ex
                results.append(r)
            out["observation"] = {"results": results}
        if st.get("metrics") and st.get("llm_call_count") != 0:
            out["metrics"] = {k: v for k, v in st["metrics"].items() if v is not None}
        if st.get("llm_call_count") is not None:
            out["llm_call_count"] = st["llm_call_count"]
    extra = dict(st.get("extra") or {})
    if st.get("video"):
        extra["video"] = st["video"]
    if extra:
        out["extra"] = extra
    return out


def build(trial: Path, harness: str | None = None) -> tuple[dict, dict]:
    """Return (trajectory, recording_index) for one trial directory without writing anything."""
    trial = Path(trial)
    cfg = _read_json(trial / "config.json") or {}
    res = _read_json(trial / "result.json") or {}
    harness = harness or cfg.get("harness") or ((res.get("agent_info") or {}).get("name")) or "unknown"
    episode = _read_json(trial / "episode" / "result.json") or res.get("episode") or {}
    trace = _read_jsonl(trial / "episode" / "trace.jsonl")
    frames = _read_jsonl(trial / "episode" / "frames.jsonl")
    video = Video(frames)
    epoch0 = _episode_epoch(trial, episode)
    model = ((res.get("agent_info") or {}).get("model_info") or {}).get("name") or cfg.get("model") or DEFAULT_MODELS.get(harness)
    notes: list[str] = []

    runner_entries = [j for j, e in enumerate(trace) if _is_runner_entry(e)]
    agent_entries = [j for j, e in enumerate(trace) if not _is_runner_entry(e)]

    meta: dict[str, Any] = {}
    fm: dict | None = None
    raw = trial / "agent" / "stdout.jsonl"
    if harness in TRACE_HARNESSES:
        steps = _steps_from_trace(agent_entries, trace, harness)
        leftover: list[int] = []
        notes.append(f"{harness} harness: no LLM; steps are the episode server's trace of robo requests "
                     f"(up to {ORACLE_ACTS_PER_STEP} motion requests per step).")
    else:
        if harness.startswith("mini-swe-agent") and (trial / "agent" / "mini_traj.json").is_file():
            data = _read_json(trial / "agent" / "mini_traj.json")
            steps, meta = _parse_mini(data if isinstance(data, dict) else {})
            if not isinstance(data, dict):
                notes.append("agent/mini_traj.json is not valid JSON")
            # the first user message is mini's rendering of the task; keep it as the prompt
        else:
            events = _read_jsonl(raw)
            if harness.startswith("codex"):
                steps, meta = _parse_codex(events, trial)
                fm = _codex_final(meta)
            elif harness.startswith("claude-code"):
                steps, meta = _parse_claude(events, trial)
                fm = _claude_final(meta)
            else:
                steps, meta = [], {}
            if not events:
                try:
                    txt = raw.read_text(errors="replace")
                except OSError:
                    txt = ""
                if txt.strip():
                    steps.append(_step("agent", _clip(txt, MSG_LIMIT)))
                    notes.append("agent/stdout.jsonl has no JSON events; kept as one text step")
                else:
                    notes.append("agent/stdout.jsonl is empty or missing")
            n_bad = 0
            try:
                n_bad = sum(1 for line in raw.read_text(errors="replace").splitlines() if line.strip().startswith("{")) - len(events)
            except OSError:
                pass
            if n_bad > 0:
                notes.append(f"{n_bad} malformed JSON line(s) in agent/stdout.jsonl skipped")
        leftover = _align(steps, agent_entries, trace)
        if leftover:
            notes.append(f"{len(leftover)} robo request(s) in the trace could not be matched to a tool call; "
                         "listed in the last system step")

    # prompt step first (unless the harness recorded its own user message first)
    if not steps or steps[0]["source"] != "user":
        steps.insert(0, _step("user", _prompt(trial, cfg, harness), epoch=epoch0))

    # the final system step: runner requests, unmatched requests, the episode's verdict
    tail_lines = []
    if leftover:
        tail_lines.append("robo requests not matched to a tool call:\n" + "\n".join(
            f"  t={trace[j].get('t')}s {_req_label(trace[j].get('req') or {})} -> {_resp_text(trace[j].get('resp') or {})[:200]}" for j in leftover))
    for j in runner_entries:
        req = trace[j].get("req") or {}
        if req.get("op") == "give_up":
            tail_lines.append(f"runner closed the episode: {req.get('text')}")
    if episode:
        tail_lines.append(f"episode: outcome={episode.get('outcome')} success={episode.get('success')} "
                          f"steps_used={episode.get('steps_used')}/{episode.get('max_steps')}")
    rw = ((res.get("verifier_result") or {}).get("rewards") or {}).get("reward")
    if rw is None:
        try:
            rw = float((trial / "verifier" / "reward.txt").read_text().strip())
        except (OSError, ValueError):
            rw = None
    if rw is not None:
        tail_lines.append(f"verifier reward: {rw}")
    sys_step = _step("system", "\n".join(tail_lines)) if tail_lines else None
    if sys_step is not None:
        sys_step["_trace"] = leftover + runner_entries
        steps.append(sys_step)

    # timing: a step with robo requests spans the video frames recorded while they ran; a step without any
    # sits at the previous step's end. Video time is monotone over steps.
    last_end = 0.0
    index_rows = []
    for sid, st in enumerate(steps, start=1):
        ents = sorted({j for c in st["calls"] for j in c.get("trace") or []} | set(st.get("_trace") or []))
        row: dict[str, Any] = {"index": sid, "action": _step_label(st)}
        if ents and trace:
            t_first = float(trace[ents[0]].get("t") or 0.0)
            last = ents[-1]
            nxt = next((float(trace[q].get("t") or 0.0) for q in range(last + 1, len(trace))), None)
            a, b, real = video.span(t_first, nxt)
            a = max(a, last_end)
            b = max(a, b)
            if epoch0 is not None and not st.get("epoch"):
                st["epoch"] = epoch0 + t_first
            s0 = max(0, int(trace[ents[0]].get("steps") or 0) - _env_steps_of(trace[ents[0]]))
            s1 = int(trace[last].get("steps") or 0)
            st["video"] = {"t_video_start_s": a, "t_video_end_s": b, "episode_t_s": t_first, "env_steps": [s0, s1],
                           "robo_requests": len(ents), "trace_lines": [ents[0] + 1, last + 1]}
            row.update({"t_video_start_s": a, "t_video_end_s": b, "real_duration_s": real, "episode_t_s": t_first,
                        "env_steps": [s0, s1], "robo_requests": len(ents)})
            last_end = b
        else:
            row.update({"t_video_start_s": round(last_end, 3), "t_video_end_s": round(last_end, 3), "real_duration_s": None})
        index_rows.append(row)

    atif_steps = [_to_atif_step(i, st, model) for i, st in enumerate(steps, start=1)]
    agent = {"name": harness, "version": str(meta.get("claude_code_version") or "unknown"), "model_name": model}
    agent_extra = {k: meta[k] for k in ("cwd", "session_id") if meta.get(k)}
    if meta.get("tools"):
        agent_extra["tools"] = meta["tools"]
    if agent_extra:
        agent["extra"] = agent_extra
    traj: dict[str, Any] = {
        "schema_version": "ATIF-v1.7",
        "session_id": str(meta.get("session_id") or cfg.get("trial_name") or trial.name),
        "agent": {k: v for k, v in agent.items() if v is not None},
        "steps": atif_steps,
        "notes": " ".join(["Converted by robouse.atif from " + _source_name(trial, harness) + "."] + notes),
        "extra": {"robouse": {"task": cfg.get("task") or res.get("task_name"), "harness": harness,
                              "episode_outcome": episode.get("outcome"), "episode_success": episode.get("success"),
                              "robo_requests": len(agent_entries), "runner_requests": len(runner_entries),
                              "recording_index": "artifacts/recording_index.json"}},
    }
    fm = dict(fm or {})
    fm["total_steps"] = len(atif_steps)
    if harness.startswith("mini-swe-agent"):
        ms = (meta.get("info") or {}).get("model_stats") or {}
        if ms.get("instance_cost") is not None:
            fm["total_cost_usd"] = ms["instance_cost"]
    traj["final_metrics"] = {k: v for k, v in fm.items() if v is not None}
    if meta.get("result", {}).get("result") and not any(s["source"] == "agent" and s.get("message") for s in atif_steps[-3:]):
        traj["extra"]["final_result_text"] = _clip(str(meta["result"]["result"]), MSG_LIMIT)

    size = _video_size(trial / "artifacts" / "recording.mp4")
    index = {
        "synthesised": False,
        "source": "robouse episode server: MuJoCo offscreen render of the simulator, one frame every few env steps "
                  "while the robot moves (episode/frames.jsonl)",
        "time_axis": "video time runs only while the simulator steps; the agent's thinking time between robo "
                     "requests is not in the video",
        "fps": FPS,
        "size": size,
        "n_frames": video.n,
        "duration_s": video.duration,
        "recording_start_epoch": round(epoch0, 3) if epoch0 is not None else None,
        "harness": harness,
        "steps": index_rows,
    }
    return traj, index


def _video_size(mp4: Path) -> list[int] | None:
    import shutil
    import subprocess

    probe = shutil.which("ffprobe") or ("/opt/homebrew/bin/ffprobe" if Path("/opt/homebrew/bin/ffprobe").exists() else None)
    if not probe or not mp4.is_file():
        return None
    try:
        r = subprocess.run([probe, "-v", "error", "-select_streams", "v:0", "-show_entries", "stream=width,height",
                            "-of", "csv=p=0", str(mp4)], capture_output=True, text=True, timeout=20)
        w, h = (int(x) for x in r.stdout.strip().split(",")[:2])
        return [w, h]
    except (OSError, ValueError, subprocess.SubprocessError):
        return None


def _env_steps_of(e: dict) -> int:
    r = (e.get("resp") or {}).get("result")
    if isinstance(r, dict) and isinstance(r.get("executed_steps"), int):
        return int(r["executed_steps"])
    return 0


def _step_label(st: dict) -> str:
    if st["source"] == "user":
        return "task"
    if st["source"] == "system":
        return "system: " + _label((st.get("message") or "").split("\n", 1)[0], 70)
    if st["calls"]:
        c = st["calls"][0]
        more = f" (+{len(st['calls']) - 1})" if len(st["calls"]) > 1 else ""
        if c.get("command"):
            return _label(c["command"]) + more
        arg = next((v for v in c["args"].values() if isinstance(v, str)), "")
        return _label(f"{c['name']} {arg}".strip()) + more
    if st.get("message"):
        return "message: " + _label(st["message"], 70)
    if st.get("reasoning"):
        return "thinking"
    return "step"


def _source_name(trial: Path, harness: str) -> str:
    if harness in TRACE_HARNESSES:
        return "episode/trace.jsonl"
    if harness.startswith("mini-swe-agent") and (trial / "agent" / "mini_traj.json").is_file():
        return "agent/mini_traj.json"
    return "agent/stdout.jsonl (" + ("Codex exec --json" if harness.startswith("codex") else "Claude Code stream-json"
                                     if harness.startswith("claude-code") else "raw output") + ")"


def write_trajectory(trial_dir: Path, harness: str) -> None:
    trial = Path(trial_dir)
    traj, index = build(trial, harness)
    (trial / "agent").mkdir(parents=True, exist_ok=True)
    (trial / "artifacts").mkdir(parents=True, exist_ok=True)
    (trial / "agent" / "trajectory.json").write_text(json.dumps(traj, indent=1, ensure_ascii=False))
    (trial / "artifacts" / "recording_index.json").write_text(json.dumps(index, indent=1))
    err = trial / "agent" / "trajectory_error.txt"
    if err.exists():
        err.unlink()


def main(argv: list[str] | None = None) -> int:
    args = list(sys.argv[1:] if argv is None else argv)
    if not args or args[0] in ("-h", "--help"):
        print(__doc__)
        return 0
    rc = 0
    for a in args:
        p = Path(a)
        cfg = _read_json(p / "config.json") or {}
        try:
            write_trajectory(p, cfg.get("harness") or "unknown")
            t = _read_json(p / "agent" / "trajectory.json") or {}
            print(f"{p}: {len(t.get('steps', []))} steps")
        except Exception as e:  # keep going over many trials
            rc = 1
            print(f"{p}: {type(e).__name__}: {e}", file=sys.stderr)
    return rc


if __name__ == "__main__":
    sys.exit(main())
