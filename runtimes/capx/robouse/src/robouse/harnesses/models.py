"""Model ids, providers and model capabilities, in BenchFlow's conventions.

A model id is provider-prefixed, as in BenchFlow: `baseten/zai-org/GLM-5.3`, `openrouter/<org>/<model>`,
`anthropic/claude-opus-5-5`, `openai/gpt-6-astra`. The provider part names an entry in BenchFlow's provider registry
(`benchflow.agents.providers.PROVIDERS`); Robo Use keeps no registry of its own. Until BenchFlow ships the `baseten`
entry, `PENDING_PROVIDERS` holds it here in BenchFlow's own `ProviderConfig` shape, and `provider_registry()` adds it.

A bare id keeps working: `claude-*` is Anthropic's, `gpt-*`/`o<n>` OpenAI's, and a bare `org/model` id given with a
legacy `-glm` harness is Baseten's (`legacy()`).

Capabilities come from the registry's model metadata (`models[].input`, BenchFlow's field: ["text"] or
["text", "image"]). `accepts_images()` answers True, False or None (unknown).
"""

from __future__ import annotations

import sys
from dataclasses import dataclass

# Legacy harness names: hidden aliases for (harness, provider). Old trial folders and URLs keep these names.
LEGACY_HARNESSES = {
    "claude-code-glm": ("claude-code", "baseten"),
    "codex-glm": ("codex", "baseten"),
    "mini-swe-agent-glm": ("mini-swe-agent", "baseten"),
}
LEGACY_DEFAULT_MODEL = "zai-org/GLM-5.3"

# First-party providers every harness knows natively (their own subscription or API key), and whose models take images.
NATIVE_VENDORS = {"claude-code": "anthropic", "codex": "openai"}
FIRST_PARTY_IMAGE_PROVIDERS = {"anthropic", "openai"}
# the model a harness runs when none is given
DEFAULT_MODELS = {
    "claude-code": "claude-sonnet-5",
    "codex": "gpt-6-astra",
    "mini-swe-agent": "baseten/zai-org/GLM-5.3",
    "dimcode": "baseten/zai-org/GLM-5.3",
}
# harnesses that never pass an image to their model (mini-swe-agent sends text only; its agent can still read image
# files with code)
NO_IMAGE_HARNESSES = {"mini-swe-agent"}

# Proposed for BenchFlow's registry (src/benchflow/agents/providers.py). Verified on 2026-09-30 against
# inference.baseten.co: all three protocols answer, and both models below read an image on each of them
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
    """('baseten', 'zai-org/GLM-5.3') for 'baseten/zai-org/GLM-5.3'; (None, model) when there is no known provider."""
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
        if harness in NATIVE_VENDORS:
            provider = NATIVE_VENDORS[harness]
        else:
            raise ValueError(f"model {model!r} needs a provider prefix, e.g. baseten/zai-org/GLM-5.3")
    meta: dict = {}
    if provider not in FIRST_PARTY_IMAGE_PROVIDERS:
        cfg = provider_registry()[provider]
        meta = next((m for m in cfg.models if str(m.get("id", "")).lower() == bare.lower()), {})
    return ModelRef(provider, bare, meta)


def legacy(harness: str, model: str) -> tuple[str, str]:
    """(canonical harness, provider-prefixed model) for a legacy harness name; other names pass through."""
    if harness not in LEGACY_HARNESSES:
        return harness, model
    canon, provider = LEGACY_HARNESSES[harness]
    m = model or LEGACY_DEFAULT_MODEL
    return canon, m if split(m)[0] else f"{provider}/{m}"


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
    the agent program with a `baseten/` model, an empty model becomes the harness's default, and a first-party model
    keeps its bare id (`codex`, `gpt-6-astra`), as published."""
    h, m = legacy(harness, model)
    m = m or DEFAULT_MODELS.get(h, "")
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
