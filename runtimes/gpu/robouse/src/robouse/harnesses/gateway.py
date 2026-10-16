"""The model gateway: one LiteLLM proxy per trial, between the agent and the model provider.

Routing is BenchFlow's: `benchflow.providers.litellm_config.resolve_litellm_route` turns the provider-prefixed model id
into a LiteLLM route from BenchFlow's provider registry, and `litellm_proxy_config` writes the proxy config. Each
harness reaches the provider over its own protocol: Claude Code over Anthropic Messages, Codex over the provider's own
Responses API (the request passes through unchanged, reasoning effort and earlier reasoning included; images in tool
results move into the next user message, which every Responses provider takes), mini-swe-agent over chat completions.
A provider that serves only chat completions, such as BenchFlow's `glm` (GLM-5.3 on the Z.ai Coding Plan), is reached
over chat completions by every harness but Codex (LiteLLM translates Claude Code's and dimcode's Messages requests, and
the gateway removes their `thinking`, for which LiteLLM would call a Responses API); Codex needs a Responses endpoint.
The gateway holds the provider key (the agent gets only a per-trial master key), applies the trial's image input to
every request, and enforces the trial's spend cap (`providers/gateway_hook.py`): once the model's cost reaches the
cap, every further request is refused and the gateway writes gateway/budget_stop.json, which the runner watches to
stop the agent. No Responses-to-chat-completions bridge: it dropped Codex's reasoning effort and every earlier
reasoning item, and Codex on GLM-5.3 reasoned about a quarter as much and solved fewer tasks.

Files, in the trial folder (outside the agent's sandbox): gateway/config.yaml, gateway/requests.jsonl (image counts
per request, no content), gateway/proxy.log. No key is written to any of them: the provider key and the per-trial
master key are referenced in the config as os.environ/<VAR> and live only in the proxy's environment.
"""

from __future__ import annotations

import json
import os
import re
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
MASTER_KEY_ENV = "LITELLM_MASTER_KEY"  # LiteLLM's own variable; config.yaml names it, the key stays out of the file
# a failed request the provider answered with HTTP 429 (its rate or usage limit), as LiteLLM reports it
RATE_LIMITED = re.compile(r"RateLimitError|\b429\b")
# A provider's rate limit (HTTP 429) is shared by everyone on the account: the gateway waits and retries (LiteLLM's
# backoff, about 40 s for eight retries) instead of passing it to the harness, whose own few retries end the trial
RETRY_POLICY = {"RateLimitErrorRetries": 8, "InternalServerErrorRetries": 2}


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
        # LiteLLM's failure events: one per failed request over chat completions (after the retries), two per failed
        # call when it translates Anthropic Messages, so a count of failures, not of requests
        "failures": 0,
        "rate_limited": 0,  # failures whose error was HTTP 429: the provider's rate or usage limit
        "successes": 0,  # requests the provider answered without an error
        "refused_over_cap": 0,
        "spent_usd": 0.0,
        "images_moved": 0,
        "thinking_dropped": 0,
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
            out["thinking_dropped"] += bool(r.get("thinking_dropped"))
        elif r["event"] == "upstream":
            out["upstream_calls"] += 1
            out["upstream_image_parts"] += r["image_parts"]
            out["upstream_data_uris"] += r["data_uris"]
        elif r["event"] == "failure":
            out["failures"] += 1
            out["rate_limited"] += bool(RATE_LIMITED.search(str(r.get("error", ""))))
        elif r["event"] == "success":
            out["successes"] += 1
            out["spent_usd"] = r.get("spent_usd", out["spent_usd"])
        elif r["event"] == "refused":
            out["refused_over_cap"] += 1
    out["spent_usd"] = round(out["spent_usd"], 5)
    return out


def litellm_exe() -> str | None:
    """LiteLLM's `litellm` command: next to this Python (the llm extra, in an environment that is not activated), else
    on PATH. None when it is not installed. `robouse doctor` reports the same."""
    return shutil.which("litellm", path=str(Path(sys.executable).parent)) or shutil.which("litellm")


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


def drops_thinking(route, cfg, protocol: str | None) -> bool:
    """True when the gateway must remove `thinking` from the agent's Anthropic Messages requests: the provider is
    reached over chat completions and serves no Responses API. LiteLLM sends a Messages request with thinking enabled
    to an OpenAI-compatible provider's Responses API instead (to return reasoning text), and such a provider answers
    404: dimcode on glm/glm-5.3 (the Z.ai Coding Plan) stopped at its first request."""
    return (
        protocol == "anthropic-messages"
        and not route.upstream_model.startswith("anthropic/")
        and "openai-responses" not in cfg.all_endpoints
    )


def prices(ref: models.ModelRef) -> dict | None:
    """USD per 1M tokens for this model from runner/pricing.json ({input, output, cached_input}), or None."""
    from ..runner.cost import price_of

    return price_of(ref.bare)


def provider_environ() -> dict:
    """The environment the model gateway reads a provider's settings from. start() and the run command's check before a
    trial (`runner.trial.check_model`) both use this one function, so they agree on what is set: a variable robouse
    derives from another belongs here, not in either caller."""
    return dict(os.environ)


def check_settings(ref: models.ModelRef, protocol: str | None, spend_cap_usd: float, env: dict | None = None) -> None:
    """Raise one RuntimeError that names everything this model is missing before the gateway can serve it: the
    provider's variables (its key and any base URL without a default), and a price when the spend cap is on. `env`
    defaults to provider_environ(). No request is sent."""
    env = provider_environ() if env is None else env
    registry = models.provider_registry()
    if ref.provider not in registry:  # e.g. anthropic/...: BenchFlow's registry has no first-party Anthropic entry
        own = next((h for h, v in models.NATIVE_VENDORS.items() if v == ref.provider), None)
        raise RuntimeError(
            f"the model gateway cannot reach {ref.id}: {ref.provider} is not in BenchFlow's provider registry"
            + (f" (run it on --harness {own} with the bare id, {ref.bare})" if own else "")
        )
    cfg = registry[ref.provider]
    if protocol == "openai-responses" and protocol not in cfg.all_endpoints:
        raise RuntimeError(
            f"Codex needs a provider that serves the Responses API; {ref.provider} does not ({ref.id} runs on "
            f"claude-code, mini-swe-agent or dimcode; Codex runs on its own model, {models.DEFAULT_MODELS['codex']})"
        )
    problems = []
    if need := [v for v in models.required_env(cfg) if not env.get(v)]:
        problems.append(f"model {ref.id} needs {' and '.join(need)} in the environment")
    if spend_cap_usd > 0 and prices(ref) is None:
        problems.append(
            f"no price for {ref.bare} in pricing.json, so the ${spend_cap_usd:g} spend cap cannot be enforced; add its "
            "price (ROBOUSE_PRICING) or set ROBOUSE_SPEND_CAP_USD=0"
        )
    if problems:
        hint = models.SETUP_HINTS.get(ref.provider)
        raise RuntimeError("; ".join(problems) + (f" ({hint})" if hint else ""))


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

    env = provider_environ()
    check_settings(ref, protocol, spend_cap_usd, env)
    cfg = models.provider_registry()[ref.provider]
    route = _same_protocol_route(resolve_litellm_route(ref.id, env), cfg, protocol, env)
    key = "sk-robouse-" + secrets.token_hex(16)
    # the trial folder keeps config.yaml: it names the master key's variable, and the key itself goes only into the
    # proxy's environment (penv), as the provider key does
    conf = litellm_proxy_config(route, master_key=f"os.environ/{MASTER_KEY_ENV}", callback_module="gateway_hook")
    conf.setdefault("router_settings", {})["retry_policy"] = dict(RETRY_POLICY)
    d = Path(run_dir).resolve() / "gateway"  # absolute: the proxy runs in this folder and is given paths inside it
    d.mkdir(parents=True, exist_ok=True)
    shutil.copy(HOOK, d / "gateway_hook.py")
    (d / "config.yaml").write_text(yaml.safe_dump(conf, sort_keys=False))
    port = _free_port()
    exe = litellm_exe()
    if not exe:
        raise RuntimeError("the model gateway needs LiteLLM: pip install 'robouse[llm]'")
    price = prices(ref)
    drop_thinking = drops_thinking(route, cfg, protocol)
    penv = {
        **env,
        MASTER_KEY_ENV: key,
        "ROBOUSE_GATEWAY_MAX_IMAGES": str(image_cap(ref, images)),
        "ROBOUSE_GATEWAY_LOG": str(d / "requests.jsonl"),
        "ROBOUSE_GATEWAY_SPEND_CAP": str(spend_cap_usd),
        "ROBOUSE_GATEWAY_STOP_FILE": str(d / STOP_FILE),
        "ROBOUSE_GATEWAY_PRICE": json.dumps(price or {}),
        "PYTHONPATH": str(d),
        **({"ROBOUSE_GATEWAY_BEARER_ENV": cfg.auth_env} if route.upstream_model.startswith("anthropic/") else {}),
        **({"ROBOUSE_GATEWAY_DROP_THINKING": "1"} if drop_thinking else {}),
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
            # True: the gateway removes the agent's extended-thinking setting from its Messages requests
            # (drops_thinking); summarize() counts the requests it was removed from as thinking_dropped
            "drops_thinking": drop_thinking,
        },
    )
