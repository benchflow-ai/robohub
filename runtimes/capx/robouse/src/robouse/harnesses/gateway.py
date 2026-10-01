"""The model gateway: one LiteLLM proxy per trial, between the agent and the model provider.

Routing is BenchFlow's: `benchflow.providers.litellm_config.resolve_litellm_route` turns the provider-prefixed model id
into a LiteLLM route from BenchFlow's provider registry, and `litellm_proxy_config` writes the proxy config. Each
harness reaches the provider over its own protocol: Claude Code over Anthropic Messages, Codex over the provider's own
Responses API (the request passes through unchanged, reasoning effort and earlier reasoning included; images in tool
results move into the next user message, which every Responses provider takes), mini-swe-agent over chat completions.
The gateway holds the provider key (the agent gets only a per-trial master key), applies the trial's image input to
every request, and enforces the trial's spend cap (`providers/gateway_hook.py`): once the model's cost reaches the
cap, every further request is refused and the gateway writes gateway/budget_stop.json, which the runner watches to
stop the agent. No Responses-to-chat-completions bridge: it dropped Codex's reasoning effort and every earlier
reasoning item, and Codex on GLM-5.3 reasoned about a quarter as much and solved fewer tasks.

Files, in the trial folder (outside the agent's sandbox): gateway/config.yaml, gateway/requests.jsonl (image counts
per request, no content), gateway/proxy.log. The provider key is referenced as os.environ/<VAR> in the config and
lives only in the proxy's environment.
"""

from __future__ import annotations

import json
import os
import secrets
import shutil
import socket
import subprocess
import sys
import time
import urllib.request
from dataclasses import dataclass, field
from pathlib import Path

from . import models

HOOK = Path(__file__).parent / "providers" / "gateway_hook.py"
STOP_FILE = "budget_stop.json"  # the hook's name for it too (gateway_hook.py imports nothing from robouse)


@dataclass
class Gateway:
    url: str  # http://127.0.0.1:<port>
    key: str  # per-trial master key: the only credential the agent sees
    model: str  # the model name the agent sends (a proxy alias)
    proc: subprocess.Popen
    dir: Path
    stats: dict = field(default_factory=dict)

    @property
    def stop_file(self) -> Path:
        return self.dir / STOP_FILE

    def stop(self) -> None:
        if self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.proc.kill()
        self.stats.update(summarize(self.dir / "requests.jsonl"))


def summarize(log: Path) -> dict:
    out = {
        "requests": 0,
        "images_in": 0,
        "images_removed": 0,
        "upstream_calls": 0,
        "upstream_image_parts": 0,
        "upstream_data_uris": 0,
        "failures": 0,
        "refused_over_cap": 0,
        "spent_usd": 0.0,
        "images_moved": 0,
    }
    if not log.exists():
        return out
    for line in log.read_text().splitlines():
        r = json.loads(line)
        if r["event"] == "request":
            out["requests"] += 1
            out["images_in"] += r["images_in"]
            out["images_removed"] += r["images_removed"]
            out["images_moved"] += r.get("images_moved", 0)
        elif r["event"] == "upstream":
            out["upstream_calls"] += 1
            out["upstream_image_parts"] += r["image_parts"]
            out["upstream_data_uris"] += r["data_uris"]
        elif r["event"] == "failure":
            out["failures"] += 1
        elif r["event"] == "success":
            out["spent_usd"] = r.get("spent_usd", out["spent_usd"])
        elif r["event"] == "refused":
            out["refused_over_cap"] += 1
    out["spent_usd"] = round(out["spent_usd"], 5)
    return out


def _free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def image_cap(ref: models.ModelRef, images: bool) -> int:
    """ROBOUSE_GATEWAY_MAX_IMAGES for this run: 0 when the run sends no images, else the endpoint's limit (-1: none)."""
    if not images:
        return 0
    return models.max_images(ref) or -1


def _same_protocol_route(route, cfg, protocol: str | None, env: dict):
    """Reach the provider over the agent's own protocol when the provider serves it (Codex: the Responses endpoint, set
    as the alias's api_base; Claude Code: Anthropic Messages, below).

    BenchFlow's route always uses a provider's chat-completions endpoint when it has one. For Claude Code that loses
    images: LiteLLM turns an image inside an Anthropic tool_result into text on the way to chat completions (spike
    evidence: gateway-probe-images.jsonl, messages_tool_result_image). A provider that serves Anthropic Messages is
    therefore reached over Messages, unchanged. Proposed upstream as a `protocol` argument to resolve_litellm_route."""
    import dataclasses

    from benchflow.agents.providers import resolve_base_url

    if protocol == "openai-responses":
        params = dict(route.litellm_params)
        params["api_base"] = resolve_base_url(cfg, env, protocol=protocol)
        return dataclasses.replace(route, litellm_params=params)
    if protocol != "anthropic-messages" or protocol not in cfg.all_endpoints:
        return route
    params = dict(route.litellm_params)
    params["model"] = f"anthropic/{models.split(route.requested_model)[1]}"
    params["api_base"] = resolve_base_url(cfg, env, protocol=protocol)
    return dataclasses.replace(route, upstream_model=params["model"], litellm_params=params)


def prices(ref: models.ModelRef) -> dict | None:
    """USD per 1M tokens for this model from runner/pricing.json ({input, output, cached_input}), or None."""
    from ..runner.cost import price_of

    return price_of(ref.bare)


def start(
    ref: models.ModelRef,
    run_dir: Path,
    images: bool,
    protocol: str | None = None,
    spend_cap_usd: float = 0.0,
    timeout: float = 90,
) -> Gateway:
    import yaml
    from benchflow.providers.litellm_config import litellm_proxy_config, resolve_litellm_route

    cfg = models.provider_registry()[ref.provider]
    env = dict(os.environ)
    if protocol == "openai-responses" and protocol not in cfg.all_endpoints:
        raise RuntimeError(f"Codex needs a provider that serves the Responses API; {ref.provider} does not")
    if cfg.auth_env and not env.get(cfg.auth_env):
        raise RuntimeError(f"model {ref.id} needs {cfg.auth_env} in the environment")
    route = _same_protocol_route(resolve_litellm_route(ref.id, env), cfg, protocol, env)
    key = "sk-robouse-" + secrets.token_hex(16)
    conf = litellm_proxy_config(route, master_key=key, callback_module="gateway_hook")
    # a provider's rate limit (HTTP 429) is shared by everyone on the account: the gateway waits and retries (LiteLLM's
    # backoff, honouring Retry-After) instead of passing it to the harness, whose own few retries end the trial
    conf.setdefault("router_settings", {})["retry_policy"] = {
        "RateLimitErrorRetries": 8,
        "InternalServerErrorRetries": 2,
    }
    d = Path(run_dir).resolve() / "gateway"  # absolute: the proxy runs in this folder and is given paths inside it
    d.mkdir(parents=True, exist_ok=True)
    shutil.copy(HOOK, d / "gateway_hook.py")
    (d / "config.yaml").write_text(yaml.safe_dump(conf, sort_keys=False))
    port = _free_port()
    exe = shutil.which("litellm", path=str(Path(sys.executable).parent)) or shutil.which("litellm")
    if not exe:
        raise RuntimeError("the model gateway needs LiteLLM: pip install 'robouse[llm]'")
    price = prices(ref)
    if spend_cap_usd > 0 and price is None:
        raise RuntimeError(
            f"no price for {ref.bare} in pricing.json, so the ${spend_cap_usd:g} spend cap cannot be enforced; add its "
            "price (ROBOUSE_PRICING) or set ROBOUSE_SPEND_CAP_USD=0"
        )
    penv = {
        **env,
        "ROBOUSE_GATEWAY_MAX_IMAGES": str(image_cap(ref, images)),
        "ROBOUSE_GATEWAY_LOG": str(d / "requests.jsonl"),
        "ROBOUSE_GATEWAY_SPEND_CAP": str(spend_cap_usd),
        "ROBOUSE_GATEWAY_STOP_FILE": str(d / STOP_FILE),
        "ROBOUSE_GATEWAY_PRICE": json.dumps(price or {}),
        "PYTHONPATH": str(d),
        **({"ROBOUSE_GATEWAY_BEARER_ENV": cfg.auth_env} if route.upstream_model.startswith("anthropic/") else {}),
        "DOCS_URL": "",
        "NO_DOCS": "true",
    }
    log = open(d / "proxy.log", "w")  # noqa: SIM115 - the child keeps its own handle
    proc = subprocess.Popen(
        [exe, "--config", str(d / "config.yaml"), "--host", "127.0.0.1", "--port", str(port)],
        env=penv,
        stdout=log,
        stderr=subprocess.STDOUT,
        cwd=d,
        start_new_session=True,
    )
    url = f"http://127.0.0.1:{port}"
    deadline = time.time() + timeout
    while time.time() < deadline:
        if proc.poll() is not None:
            raise RuntimeError(f"model gateway exited ({proc.returncode}); see {d / 'proxy.log'}")
        try:
            with urllib.request.urlopen(url + "/health/liveliness", timeout=2):
                break
        except OSError:
            time.sleep(0.5)
    else:
        proc.kill()
        raise TimeoutError("model gateway did not start")
    return Gateway(
        url,
        key,
        route.model_alias,
        proc,
        d,
        {
            "image_cap": image_cap(ref, images),
            "route": route.upstream_model,
            "spend_cap_usd": spend_cap_usd or None,
        },
    )
