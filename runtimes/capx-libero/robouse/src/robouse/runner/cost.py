"""Tokens and USD per trial."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from .. import config

PRICING = Path(__file__).with_name("pricing.json")


def price_of(model: str) -> dict | None:
    """USD per 1M tokens for a model id as the provider knows it (e.g. zai-org/GLM-5.3), from pricing.json."""
    price = json.loads(config.path("ROBOUSE_PRICING", PRICING).read_text()).get(model)
    return price if isinstance(price, dict) else None


def trial_cost(tdir: Path, model: str, provider: str | None) -> dict:
    """Tokens and USD for one trial, from the ATIF trajectory's final metrics; USD from the harness when it reports one,
    else from pricing.json (per 1M tokens). Scripted harnesses (oracle, noop, VLAs) cost 0 model tokens."""
    out: dict[str, Any] = {
        "prompt_tokens": None,
        "completion_tokens": None,
        "cached_tokens": None,
        "cost_usd": None,
        "cost_source": None,
    }
    try:
        fm = json.loads((tdir / "agent" / "trajectory.json").read_text()).get("final_metrics") or {}
    except (OSError, ValueError):  # no trajectory (the harness wrote none) or an unreadable one
        fm = {}
    out.update(
        prompt_tokens=fm.get("total_prompt_tokens"),
        completion_tokens=fm.get("total_completion_tokens"),
        cached_tokens=fm.get("total_cached_tokens"),
    )
    price = price_of(model)
    if price and out["prompt_tokens"] is not None:
        cached = out["cached_tokens"] or 0
        inp = (out["prompt_tokens"] - cached) * price["input"] + cached * price.get("cached_input", price["input"])
        out["cost_usd"] = round((inp + (out["completion_tokens"] or 0) * price["output"]) / 1e6, 5)
        out["cost_source"] = f"pricing.json ({price.get('source', '')})"
    elif fm.get("total_cost_usd") is not None and provider is None:
        out["cost_usd"], out["cost_source"] = round(float(fm["total_cost_usd"]), 5), "harness"
    elif out["prompt_tokens"] is None and (model in ("scripted", "none", "") or provider == "vla"):
        out.update(prompt_tokens=0, completion_tokens=0, cost_usd=0.0, cost_source="no model")
    return out
