"""Model ids, providers and model capabilities, in BenchFlow's conventions.

A model id is provider-prefixed, as in BenchFlow: `glm/glm-5.3` (GLM-5.3 on the Z.ai Coding Plan, through BenchFlow's
`glm` provider), `openrouter/<org>/<model>`, `anthropic/claude-opus-5-5`, `openai/gpt-6-astra`. The provider part
names an entry in BenchFlow's provider registry (`benchflow.agents.providers.PROVIDERS`); Robo Use keeps no registry of
its own. `PENDING_PROVIDERS` holds the one entry BenchFlow does not ship, `baseten`, in BenchFlow's own
`ProviderConfig` shape: it served the earlier GLM-5.3 and Kimi K3 runs (and robouse 0.4.0's defaults), and their
trial folders still name it. A new run refuses a `baseten/` id (`history_refusal()`).

A bare id keeps working for a harness's own vendor: `claude-*` is Anthropic's, `gpt-*`/`o<n>` OpenAI's. The legacy
`-glm` harness names of the September runs read as Baseten's (`legacy()`, `canonical()`); a new run refuses them
(`legacy_refusal()`).

Capabilities come from the registry's model metadata (`models[].input`, BenchFlow's field: ["text"] or
["text", "image"]). `accepts_images()` answers True, False or None (unknown).
"""

from __future__ import annotations

import sys
from dataclasses import dataclass

# Legacy harness names of the September 2026 runs: (harness, provider). Their trial folders and URLs keep these names,
# so they are read everywhere (legacy(), canonical()); a new run refuses them (legacy_refusal()).
LEGACY_HARNESSES = {
    "claude-code-glm": ("claude-code", "baseten"),
    "codex-glm": ("codex", "baseten"),
    "mini-swe-agent-glm": ("mini-swe-agent", "baseten"),
}
LEGACY_DEFAULT_MODEL = "zai-org/GLM-5.3"

# First-party providers every harness knows natively (their own subscription or API key), and whose models take images.
NATIVE_VENDORS = {"claude-code": "anthropic", "codex": "openai"}
FIRST_PARTY_IMAGE_PROVIDERS = {"anthropic", "openai"}

# GLM-5.3 on the Z.ai Coding Plan, through BenchFlow's `glm` provider: the key in GLM_API_KEY and the plan's
# OpenAI-compatible endpoint in GLM_BASE_URL (BenchFlow has no default for it; ZAI_CODING_PLAN_URL is the endpoint of a
# plan bought on bigmodel.cn, the one tested). The plan is a flat subscription, so pricing.json has no per-token price
# for glm-5.3, and a trial needs ROBOUSE_SPEND_CAP_USD=0.
ZAI_CODING_PLAN_MODEL = "glm/glm-5.3"
ZAI_CODING_PLAN_URL = "https://open.bigmodel.cn/api/coding/paas/v4"
# what an error about a provider's missing settings adds, per provider: how the default setup sets them
SETUP_HINTS = {
    "glm": f"the Z.ai Coding Plan's settings, as for the default {ZAI_CODING_PLAN_MODEL}: GLM_API_KEY=<the plan's API "
    f"key>, GLM_BASE_URL={ZAI_CODING_PLAN_URL} and ROBOUSE_SPEND_CAP_USD=0, since the plan has no per-token price"
}

# the model a harness runs when none is given; show-harness has none, since its model must read the camera images
DEFAULT_MODELS = {
    "claude-code": "claude-sonnet-5",
    "codex": "gpt-6-astra",
    "mini-swe-agent": ZAI_CODING_PLAN_MODEL,
    "dimcode": ZAI_CODING_PLAN_MODEL,
}
# History: the defaults when the trial folders that leave config.json's `model` empty were recorded (robouse 0.4.0 and
# earlier, served by Baseten). canonical() reads an empty model with these, never with DEFAULT_MODELS, so those
# folders keep their pair whatever the defaults become.
RECORDED_DEFAULT_MODELS = {
    "claude-code": "claude-sonnet-5",
    "codex": "gpt-6-astra",
    "mini-swe-agent": "baseten/zai-org/GLM-5.3",
    "dimcode": "baseten/zai-org/GLM-5.3",
    "show-harness": "baseten/zai-org/GLM-5.3",
}
# harnesses that never pass an image to their model (mini-swe-agent sends text only; its agent can still read image
# files with code)
NO_IMAGE_HARNESSES = {"mini-swe-agent"}

# Baseten served the earlier GLM-5.3 and Kimi K3 runs (the legacy -glm names, then baseten/ ids). The entry stays so
# those trial folders resolve (split(), the image metadata below); a new run refuses a baseten/ id (history_refusal()),
# and no default, help text or key file uses it. Verified on 2026-09-30 against inference.baseten.co: all three
# protocols answer, and both models below read an image on each of them
# (scratch/provider-spike/evidence/image-probe2.jsonl). maxImages: the Messages endpoint rejects a request with more
# than 8 images. Baseten's Responses endpoint rejects an image inside a tool result (function_call_output); see
# `gateway.py` for how the gateway routes Codex around it.
BASETEN_URL = "https://inference.baseten.co/v1"
PENDING_PROVIDERS: dict[str, dict] = {
    "baseten": {
        "name": "baseten",
        "base_url": BASETEN_URL,
        "api_protocol": "openai-completions",
        "auth_type": "api_key",
        "auth_env": "BASETEN_API_KEY",
        "endpoints": {
            "openai-completions": BASETEN_URL,
            "openai-responses": BASETEN_URL,
            "anthropic-messages": "https://inference.baseten.co",
        },
        "models": [
            {"id": "zai-org/GLM-5.3", "name": "GLM-5.3", "reasoning": True, "input": ["text", "image"], "maxImages": 8},
            # probed 2026-10-01 and 10-02: it named the blocks in a rendered tabletop frame; on both chat completions
            # and Anthropic Messages it took 50 images in one request and refused 80 ("Too many images: 80 (max 64)"),
            # but miscounted them beyond 16 (32 read as 29 or 31, 50 as 55 or 60). Capped at 8 per request, like
            # GLM-5.3 and Kimi K3.
            {
                "id": "zai-org/GLM-5.3-Flash",
                "name": "GLM-5.3 Flash",
                "reasoning": True,
                "input": ["text", "image"],
                "maxImages": 8,
            },
            {
                "id": "moonshotai/Kimi-K3",
                "name": "Kimi K3",
                "reasoning": True,
                "input": ["text", "image"],
                "maxImages": 8,
            },
        ],
    }
}


def _python() -> tuple[int, int]:
    return sys.version_info[0], sys.version_info[1]


def provider_registry() -> dict:
    """BenchFlow's provider registry plus the entries pending upstream. Needs the `benchflow` extra."""
    try:
        from benchflow.agents.providers import PROVIDERS, ProviderConfig
    except ImportError as e:
        if _python() < (3, 12):  # the llm extra installs nothing there: its packages need 3.12
            raise RuntimeError(
                "provider-prefixed models need Python 3.12 or newer: the llm extra (BenchFlow's provider registry and "
                f"LiteLLM) does not install on Python {_python()[0]}.{_python()[1]}"
            ) from e
        raise RuntimeError(
            "provider-prefixed models need BenchFlow's provider registry: pip install 'robouse[llm]'"
        ) from e
    for name, fields in PENDING_PROVIDERS.items():
        PROVIDERS.setdefault(name, ProviderConfig(**fields))
    return PROVIDERS


@dataclass(frozen=True)
class ModelRef:
    """One resolved model: `provider/bare` (the id recorded in config.json), and its registry metadata."""

    provider: str  # registry name, or "anthropic"/"openai" for first-party ids
    bare: str  # the id the provider knows, e.g. zai-org/GLM-5.3
    meta: dict

    @property
    def id(self) -> str:
        return f"{self.provider}/{self.bare}"


def split(model: str) -> tuple[str | None, str]:
    """('glm', 'glm-5.3') for 'glm/glm-5.3'; (None, model) when there is no known provider."""
    head, _, rest = model.partition("/")
    if rest and (head in FIRST_PARTY_IMAGE_PROVIDERS or head in PENDING_PROVIDERS or _registered(head)):
        return head, rest
    return None, model


def _registered(name: str) -> bool:
    try:
        return name in provider_registry()
    except RuntimeError:
        return False


def resolve(harness: str, model: str) -> ModelRef:
    """The provider-prefixed model for a (canonical harness, model) pair. A bare id goes to the harness's own vendor."""
    provider, bare = split(model)
    if provider is None:
        if "/" in model:  # a provider prefix only the registry knows: without BenchFlow, say what to install
            provider_registry()
        if harness in NATIVE_VENDORS:
            provider = NATIVE_VENDORS[harness]
        elif "/" in model:  # a prefix, but not a provider: a typo or a guessed name
            raise ValueError(
                f"model {model!r}: {model.partition('/')[0]!r} is not a provider in BenchFlow's registry; a model "
                f"id starts with one, e.g. {ZAI_CODING_PLAN_MODEL}"
            )
        else:
            raise ValueError(f"model {model!r} needs a provider prefix, e.g. {ZAI_CODING_PLAN_MODEL}")
    meta: dict = {}
    if provider not in FIRST_PARTY_IMAGE_PROVIDERS:
        cfg = provider_registry()[provider]
        meta = next((m for m in cfg.models if str(m.get("id", "")).lower() == bare.lower()), {})
    return ModelRef(provider, bare, meta)


def legacy(harness: str, model: str) -> tuple[str, str]:
    """(canonical harness, provider-prefixed model) for a legacy harness name, as its trial folders mean it; other
    names pass through. For reading recorded trials: a new run refuses the legacy names (legacy_refusal())."""
    if harness not in LEGACY_HARNESSES:
        return harness, model
    canon, provider = LEGACY_HARNESSES[harness]
    m = model or LEGACY_DEFAULT_MODEL
    return canon, m if split(m)[0] else f"{provider}/{m}"


def legacy_refusal(harness: str) -> str | None:
    """Why a new run cannot use this harness name (a legacy `-glm` name meant a Baseten model), or None."""
    if harness not in LEGACY_HARNESSES:
        return None
    canon = LEGACY_HARNESSES[harness][0]
    if canon == "codex":  # Codex needs a Responses API, which the Z.ai Coding Plan does not serve
        instead = f"Run --harness codex with its own model ({DEFAULT_MODELS['codex']}, the default)"
    else:
        instead = f"Run --harness {canon} with --model (a provider-prefixed id, e.g. {ZAI_CODING_PLAN_MODEL})"
    return (
        f"{harness} is the name the September 2026 runs used for {canon} with a Baseten model; it is kept only to "
        f"read those trials. {instead}"
    )


def history_refusal(model: str) -> str | None:
    """Why a new run cannot use this model (a `baseten/` id: Baseten served the earlier runs, and Robo Use sends it no
    new request), or None. Those trials still read with their baseten/ pair (split(), canonical())."""
    if split(model)[0] != "baseten":
        return None
    return (
        f"{model} is a Baseten model of the earlier runs; it is kept only to read those trials, and a new run sends "
        f"no request to Baseten. Run a provider-prefixed model instead, e.g. {ZAI_CODING_PLAN_MODEL}, the default of "
        "mini-swe-agent and dimcode"
    )


def required_env(cfg) -> list[str]:
    """The environment variables a registry provider needs: its key (`auth_env`) and every base-URL parameter
    (`url_params`) without a default."""
    defaults = getattr(cfg, "url_param_defaults", None) or {}
    names = [str(cfg.auth_env)] if getattr(cfg, "auth_env", None) else []
    return names + [str(v) for k, v in (getattr(cfg, "url_params", None) or {}).items() if not defaults.get(k)]


def accepts_images(ref: ModelRef) -> bool | None:
    """True/False from the registry's `input` metadata; first-party Anthropic and OpenAI models take images; None if
    the registry does not say."""
    if ref.provider in FIRST_PARTY_IMAGE_PROVIDERS:
        return True
    inputs = ref.meta.get("input")
    return None if inputs is None else "image" in inputs


def max_images(ref: ModelRef) -> int:
    """Images per request the endpoint takes (0: no limit declared)."""
    return int(ref.meta.get("maxImages") or 0)


def secret_env_names() -> set[str]:
    """Every provider key variable the registry names (`auth_env`): the runner keeps them out of agent environments,
    as BenchFlow's `_provider_secret_env_names` does. Only the provider prefixes when BenchFlow is not installed."""
    names = {str(p["auth_env"]) for p in PENDING_PROVIDERS.values() if p.get("auth_env")}
    try:
        names |= {str(c.auth_env) for c in provider_registry().values() if getattr(c, "auth_env", None)}
    except RuntimeError:
        pass
    return names


def canonical(harness: str, model: str) -> tuple[str, str]:
    """The (harness, model) pair a trial belongs to, for old and new trial folders alike: legacy harness names become
    the agent program with a `baseten/` model, an empty model becomes the default the folder was recorded with
    (RECORDED_DEFAULT_MODELS: a new trial always records its model), and a first-party model keeps its bare id
    (`codex`, `gpt-6-astra`), as published."""
    h, m = legacy(harness, model)
    m = m or RECORDED_DEFAULT_MODELS.get(h, "")
    provider, bare = split(m)
    if provider is not None and NATIVE_VENDORS.get(h) == provider:
        m = bare
    return h, m


# What the model could see in trials recorded before `image_input` existed: Codex on Baseten ran with images turned
# off (Baseten's Responses endpoint rejected them), and mini-swe-agent never passes images.
LEGACY_IMAGE_INPUT = {"codex-glm": False, "mini-swe-agent-glm": False, "mini-swe-agent": False}


def recorded_image_input(harness: str, result: dict | None, cfg: dict | None = None) -> bool | None:
    """A trial's `image_input`: as recorded (result.json, then config.json), else inferred from its harness name. None
    for harnesses without a language model."""
    for rec in (result or {}, cfg or {}):
        if "image_input" in rec:
            return rec["image_input"]
    if harness in LEGACY_IMAGE_INPUT:
        return LEGACY_IMAGE_INPUT[harness]
    return True if legacy(harness, "")[0] in ("claude-code", "codex") else None
