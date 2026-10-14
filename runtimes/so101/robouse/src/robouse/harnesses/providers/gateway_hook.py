"""LiteLLM proxy hook for Robo Use's model gateway. Copied next to the proxy's config.yaml and loaded by LiteLLM, so it
imports nothing from robouse.

Before each request (any protocol: chat completions, Responses, Anthropic Messages) it applies the model's image
capability, read from ROBOUSE_GATEWAY_MAX_IMAGES:
  0   text-only model: every image part is replaced by a text note
  N   at most N images per request: the oldest are replaced, in batches of 5 so the prompt prefix (and the provider's
      prompt cache) stays the same for several turns (the rule of the earlier Baseten proxy)
  -1  no limit
In a Responses request (Codex) it then moves every image left in a tool result (function_call_output) into a user
message right after it: Baseten's Responses endpoint rejects an image inside a tool result and takes it in a user
message. Nothing else in the request changes (reasoning effort and earlier reasoning items pass through as sent).
Right before the upstream call it counts the image parts that actually leave the gateway, and appends one line per
request to $ROBOUSE_GATEWAY_LOG (JSON: call type, images in, removed, sent upstream). No request content is logged.

Spend cap: after each response it adds the response's cost (its token usage at $ROBOUSE_GATEWAY_PRICE, USD per 1M
tokens; cached input at the input price unless `cached_input` is given) to the trial's total. Once the total reaches
$ROBOUSE_GATEWAY_SPEND_CAP (USD; 0: no cap) it writes $ROBOUSE_GATEWAY_STOP_FILE (cap and spend, JSON) and refuses
every further request with HTTP 402. The runner watches that file and stops the agent. The last response before the
cap can take the total a little past it.
"""

from __future__ import annotations

import json
import os
import time

from litellm.integrations.custom_logger import CustomLogger

IMAGE_TYPES = {"image": "text", "image_url": "text", "input_image": "input_text"}
NOTE_TEXT_ONLY = "[image removed: no image input in this run]"
NOTE_CAPPED = "[older image removed: the model endpoint takes at most {n} images per request]"
BATCH = 5


def _image_refs(node, out: list, seen: set | None = None) -> list:
    """(container, index) of every image part, in document order (= conversation order). A request body can hold the
    same list object twice (LiteLLM keeps a reference to the original body); each part is counted once."""
    seen = set() if seen is None else seen
    if isinstance(node, list):
        if id(node) in seen:
            return out
        seen.add(id(node))
        for i, v in enumerate(node):
            if isinstance(v, dict) and v.get("type") in IMAGE_TYPES:
                out.append((node, i))
            else:
                _image_refs(v, out, seen)
    elif isinstance(node, dict):
        if id(node) in seen:
            return out
        seen.add(id(node))
        for v in node.values():
            _image_refs(v, out, seen)
    return out


def _data_uris(node) -> int:
    """Base64 images anywhere in the payload, whatever part type carries them (a second, type-blind count)."""
    s = json.dumps(node, default=str)
    return s.count("data:image/") + s.count('"media_type": "image/')


def removed_count(n: int, cap: int) -> int:
    if cap < 0 or n == 0:
        return 0
    if cap == 0:
        return n
    if n <= cap:
        return 0
    return min(n, BATCH * ((n - (cap - BATCH + 1)) // BATCH)) if cap >= BATCH else n - cap


def apply_cap(body: dict, cap: int) -> tuple[int, int]:
    refs = _image_refs(body, [])
    k = removed_count(len(refs), cap)
    note = NOTE_TEXT_ONLY if cap == 0 else NOTE_CAPPED.format(n=cap)
    for lst, i in refs[:k]:
        lst[i] = {"type": IMAGE_TYPES[lst[i]["type"]], "text": note}
    return len(refs), k


NOTE_MOVED = "[the image this tool returned follows in the next message]"


def move_tool_images(body: dict) -> int:
    """Responses API, native route: Baseten's Responses endpoint rejects an image inside a tool result
    (function_call_output) but takes one in a user message. Each tool result's images move into a user message right
    after it, and the tool result says so. The input list is changed in place (LiteLLM may hold it twice)."""
    items = body.get("input")
    if not isinstance(items, list):
        return 0
    out, moved = [], 0
    for it in items:
        out.append(it)
        if not (
            isinstance(it, dict) and it.get("type") == "function_call_output" and isinstance(it.get("output"), list)
        ):
            continue
        imgs = [p for p in it["output"] if isinstance(p, dict) and p.get("type") == "input_image"]
        if not imgs:
            continue
        it["output"] = [p for p in it["output"] if p not in imgs] + [{"type": "input_text", "text": NOTE_MOVED}]
        intro = {"type": "input_text", "text": f"The image returned by tool call {it.get('call_id', '')}:"}
        out.append({"type": "message", "role": "user", "content": [intro, *imgs]})
        moved += len(imgs)
    if moved:
        items[:] = out
    return moved


def _dump(name: str, obj) -> None:
    """Debugging only (ROBOUSE_GATEWAY_DUMP=<folder>): the request bodies, which hold the trial's content. Never on
    for published runs. Headers and keys are left out."""
    folder = os.environ.get("ROBOUSE_GATEWAY_DUMP")
    if not folder or not isinstance(obj, dict):
        return
    drop = ("metadata", "proxy_server_request", "secret_fields", "headers", "extra_headers", "api_key", "user_api_key")
    keep = {k: v for k, v in obj.items() if not k.startswith(("litellm_", "user_api_key")) and k not in drop}
    with open(os.path.join(folder, f"{name}.jsonl"), "a") as f:
        f.write(json.dumps({"t": round(time.time(), 3), **keep}, default=str) + "\n")


def _log(rec: dict) -> None:
    path = os.environ.get("ROBOUSE_GATEWAY_LOG")
    if path:
        with open(path, "a") as f:
            f.write(json.dumps({"t": round(time.time(), 3), **rec}) + "\n")


def usage_cost(usage: dict, price: dict) -> float:
    """USD for one response's token usage (OpenAI-style usage, as LiteLLM reports it for every protocol; prompt_tokens
    include cached tokens) at `price` (USD per 1M tokens)."""
    prompt = int(usage.get("prompt_tokens") or usage.get("input_tokens") or 0)
    out = int(usage.get("completion_tokens") or usage.get("output_tokens") or 0)
    details = usage.get("prompt_tokens_details") or usage.get("input_tokens_details") or {}
    cached = int((details or {}).get("cached_tokens") or usage.get("cache_read_input_tokens") or 0)
    cached = min(cached, prompt)
    inp = float(price.get("input", 0))
    return (
        (prompt - cached) * inp + cached * float(price.get("cached_input", inp)) + out * float(price.get("output", 0))
    ) / 1e6


class RobouseGateway(CustomLogger):
    def __init__(self) -> None:
        super().__init__()
        self.cap = int(os.environ.get("ROBOUSE_GATEWAY_MAX_IMAGES", "-1"))
        self.spend_cap = float(os.environ.get("ROBOUSE_GATEWAY_SPEND_CAP", "0") or 0)
        self.price = json.loads(os.environ.get("ROBOUSE_GATEWAY_PRICE") or "{}")
        self.stop_file = os.environ.get("ROBOUSE_GATEWAY_STOP_FILE", "")
        self.spent = 0.0

    def over_cap(self) -> bool:
        return self.spend_cap > 0 and self.spent >= self.spend_cap

    def add_spend(self, usage: dict) -> None:
        self.spent += usage_cost(usage, self.price) if self.price else 0.0
        if self.over_cap() and self.stop_file and not os.path.exists(self.stop_file):
            with open(self.stop_file, "w") as f:
                json.dump({"cap_usd": self.spend_cap, "spent_usd": round(self.spent, 5), "t": round(time.time(), 3)}, f)

    async def async_pre_call_hook(self, user_api_key_dict, cache, data, call_type):
        if self.over_cap():
            from fastapi import HTTPException

            _log({"event": "refused", "call_type": str(call_type), "spent_usd": round(self.spent, 5)})
            raise HTTPException(
                status_code=402,
                detail=f"spend cap reached: this trial's model spend is ${self.spent:.2f} of its ${self.spend_cap:g} cap",
            )
        _dump("in", data)
        n, k = apply_cap(data, self.cap)
        moved = move_tool_images(data)  # Responses bodies only: images out of tool results into the next user message
        bearer = os.environ.get("ROBOUSE_GATEWAY_BEARER_ENV")
        if bearer and os.environ.get(bearer):  # providers whose Anthropic endpoint wants a Bearer token, not x-api-key
            data["extra_headers"] = {
                **(data.get("extra_headers") or {}),
                "Authorization": f"Bearer {os.environ[bearer]}",
            }
        tools = data.get("tools")
        _log(
            {
                "event": "request",
                "call_type": str(call_type),
                "images_in": n,
                "images_removed": k,
                "images_moved": moved,
                "tool_types": sorted({str(t.get("type")) for t in tools if isinstance(t, dict)})
                if isinstance(tools, list)
                else [],
                # names and keys only (no descriptions or schemas) of tools that are not plain functions
                "tool_shapes": [
                    {
                        "type": t.get("type"),
                        "name": t.get("name"),
                        "keys": sorted(t),
                        "inner": [(i.get("type"), i.get("name")) for i in t.get("tools", []) if isinstance(i, dict)],
                    }
                    for t in tools
                    if isinstance(t, dict) and t.get("type") != "function"
                ]
                if isinstance(tools, list)
                else [],
            }
        )
        return data

    def log_pre_api_call(self, model, messages, kwargs):
        payload = {"messages": messages, "input": (kwargs.get("optional_params") or {}).get("input")}
        extra = (kwargs.get("additional_args") or {}).get("complete_input_dict")
        _dump("upstream", extra if isinstance(extra, dict) else {"messages": messages})
        _log(
            {
                "event": "upstream",
                "model": str(model),
                # the same request as LiteLLM's messages and as the raw provider body: the larger count of the two
                "image_parts": max(len(_image_refs(payload, [])), len(_image_refs(extra, [])) if extra else 0),
                "data_uris": max(_data_uris(payload), _data_uris(extra) if extra else 0),
            }
        )

    async def async_log_success_event(self, kwargs, response_obj, start_time, end_time):
        usage = getattr(response_obj, "usage", None)
        if usage is None and isinstance(response_obj, dict):
            usage = response_obj.get("usage")
        u = usage.model_dump() if hasattr(usage, "model_dump") else usage if isinstance(usage, dict) else {}
        self.add_spend(u)
        _log({"event": "success", "usage": u or str(usage), "spent_usd": round(self.spent, 5)})

    async def async_log_failure_event(self, kwargs, response_obj, start_time, end_time):
        _log({"event": "failure", "error": str(kwargs.get("exception", ""))[:400]})


proxy_handler_instance = RobouseGateway()
