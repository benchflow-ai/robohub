"""The `show-harness` harness: NUS Show Lab's Show-Harness (arXiv 2609.10522; github.com/showlab/Show-Harness at
403f068, Apache-2.0), the VLM-as-policy loop with parameter-free action units grounded by an interpreter. The client
(client.py, standard library only) runs inside the agent sandbox and drives the episode socket directly, like the VLA
clients; see its docstring for the loop.

Models (--model is required: the policy is the model's reading of the camera images, so there is no default):
  provider-prefixed, of a model the provider registry  zero-shot mode, through the trial's model gateway
  lists as reading images (others are refused)         (chat completions; image input and spend cap apply)
  any other name, with ROBOUSE_SHOW_HARNESS_URL        a self-hosted OpenAI-compatible server (vLLM serving a
                                                      Show-Harness LoRA, e.g. Qwen3.5-2B): fine-tuned mode, no gateway
  ROBOUSE_SHOW_HARNESS_TOKEN   token for that server (optional): a bearer token, or under the header the overrides name
  ROBOUSE_SHOW_HARNESS_CONFIG  a JSON file of overrides: {"mode": "zs"|"ft", "fine_step_m": ..., "plugins": {...},
                               "token_header": "x-daytona-preview-token", ...}

The zero-shot mode gets the task.md body (the same task text as every harness); the fine-tuned mode gets the task's
one-sentence language instruction, as the VLA clients do, because the adapters were trained on such sentences.
"""

from __future__ import annotations

import json
from pathlib import Path

from ... import config

CLIENT = Path(__file__).with_name("client.py")
MODES = ("zs", "ft")
NO_MODEL = (
    "show-harness needs --model: a provider-prefixed id of a model that reads images (through the model gateway), or "
    "the name of a model you serve at ROBOUSE_SHOW_HARNESS_URL"
)


def overrides() -> dict:
    """The user's overrides file ($ROBOUSE_SHOW_HARNESS_CONFIG), or {}."""
    f = config.env("ROBOUSE_SHOW_HARNESS_CONFIG")
    if not f:
        return {}
    data = json.loads(Path(f).expanduser().read_text())
    if not isinstance(data, dict):
        raise ValueError(f"{f}: show-harness overrides must be a JSON object")
    return data


def build(model, prompt, workspace: Path, task, env: dict):
    from .. import GATEWAY_PROTOCOLS, Launch, _gateway, _gateway_launch, _task_language, models

    if not model:
        raise ValueError(NO_MODEL)
    over = overrides()
    served = config.env("ROBOUSE_SHOW_HARNESS_URL")
    m = model
    via_gateway = models.split(m)[0] is not None
    mode = str(over.pop("mode", "") or ("zs" if via_gateway or not served else "ft"))
    if mode not in MODES:
        raise ValueError(f"unknown show-harness mode {mode!r}; modes: {', '.join(MODES)}")
    client = workspace / ".robouse" / "show_harness_client.py"
    client.write_text(CLIENT.read_text())
    settings = {"mode": mode, "task_sentence": _task_language(task) if mode == "ft" else "", "overrides": over}
    cmd = ["python3", str(client), prompt]
    if via_gateway:
        ref = models.resolve("show-harness", m)
        gw = _gateway(ref, env, GATEWAY_PROTOCOLS["show-harness"])
        env.update(
            {
                "ROBOUSE_GATEWAY_URL": gw.url,  # also lets the macOS sandbox allow this local port
                "ROBOUSE_GATEWAY_KEY": gw.key,
                "ROBOUSE_SHOW_HARNESS_MODEL": gw.model,
                "ROBOUSE_SHOW_HARNESS_SETTINGS": json.dumps(settings),
            }
        )
        launch = _gateway_launch(gw, ref, cmd, env)
        launch.notes["mode"] = mode
        return launch
    if not served:
        raise RuntimeError(
            f"show-harness model {m!r} has no provider prefix, so it needs ROBOUSE_SHOW_HARNESS_URL (an OpenAI-"
            "compatible server, e.g. vLLM serving a Show-Harness adapter); for a hosted model use a provider-prefixed "
            "id of a model that reads images"
        )
    env.update(
        {
            "ROBOUSE_SHOW_HARNESS_URL": served,
            "ROBOUSE_SHOW_HARNESS_MODEL": m,
            "ROBOUSE_SHOW_HARNESS_SETTINGS": json.dumps(settings),
        }
    )
    if config.env("ROBOUSE_SHOW_HARNESS_TOKEN"):
        env["ROBOUSE_SHOW_HARNESS_TOKEN"] = config.env("ROBOUSE_SHOW_HARNESS_TOKEN")
    return Launch(cmd, env, notes={"model": m, "provider": "self-hosted", "server": served, "mode": mode})
