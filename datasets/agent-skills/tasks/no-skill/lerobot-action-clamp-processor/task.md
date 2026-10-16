---
schema_version: '1.3'
task:
  name: agent-skills/lerobot-action-clamp-processor
  description: Add a registered ProcessorStep that clamps policy actions to per-dimension bounds, with the bounds stored as pipeline state, so it round-trips through DataProcessorPipeline save_pretrained/from_pretrained.
metadata:
  author_name: benchflow
  source_repo: https://github.com/huggingface/lerobot
  source_commit: 297774fca4e3ad0aa34e7425fbf937111637e77b
  source_licence: Apache-2.0
  skill: AGENTS.md#Architecture (processor/)
  scenario: 'AGENTS.md Architecture: "processor/ - ProcessorStep base with registry. DataProcessorPipeline / PolicyProcessorPipeline chain steps": a new step registered in ProcessorStepRegistry, exported from lerobot.processor, with tensor state saved to safetensors and config to JSON'
  variant: no-skill
  suite: agent-skills
  category: coding
  difficulty: medium
  tags:
  - agent-skills
  - lerobot
  - no-skill
  - processor
  - registry
  - serialization
  - library
  gpu: none
agent:
  timeout_sec: 1800
verifier:
  timeout_sec: 600
sandbox:
  cpus: 2
  memory_mb: 4096
  build_timeout_sec: 3600
---

You are working in a checkout of LeRobot (https://github.com/huggingface/lerobot, commit 297774fca4e3) at `/workspace/lerobot`. It is a git repository with one commit, so `git diff` shows your changes.

The Python environment is already set up: run `python` (Python 3.12 with CPU-only PyTorch, LeRobot installed from this checkout in editable mode). There is no GPU and no robot hardware. Do not install or upgrade packages. Leave your changes in the working tree; you do not need to commit them.

Do not stop to ask questions: there is nobody to answer. Make reasonable assumptions, do the work, and check it as far as this machine allows.

## Request

Before policy actions go to my robot I want them clamped to safe per-joint limits, as a step of the policy postprocessor pipeline that is saved with the checkpoint. Please add it to LeRobot:

- Class `ActionClampProcessorStep` in `src/lerobot/processor/action_clamp_processor.py`, importable as `from lerobot.processor import ActionClampProcessorStep`, and registered in LeRobot's processor step registry under the name `action_clamp_processor`.
- Constructor: `ActionClampProcessorStep(min_action=None, max_action=None, margin=0.0)`. `min_action`/`max_action` are 1-D tensors (one value per action dimension); `margin` is a float that widens the limits on both sides.
- It processes policy actions (tensors of shape `(D,)` or `(B, D)`): every element is clamped to `[min_action - margin, max_action + margin]` for its dimension. The output keeps the input's shape, dtype and device. It does not change the pipeline's feature descriptions.
- `min_action`/`max_action` are state: they must be saved in the pipeline's safetensors state file, not in the JSON config. `margin` is plain config.
- A `PolicyProcessorPipeline` containing the step, saved with `save_pretrained`, must load back with `PolicyProcessorPipeline.from_pretrained` in a fresh Python process that only imports `lerobot.processor`, and behave the same. Overriding the margin at load time with `overrides={"action_clamp_processor": {"margin": ...}}` must work.
