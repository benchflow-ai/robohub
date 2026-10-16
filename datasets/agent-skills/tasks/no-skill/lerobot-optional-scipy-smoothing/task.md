---
schema_version: '1.3'
task:
  name: agent-skills/lerobot-optional-scipy-smoothing
  description: Add a Savitzky-Golay action-smoothing utility to LeRobot that depends on the optional scipy package, guarded the way LeRobot guards optional dependencies.
metadata:
  author_name: benchflow
  source_repo: https://github.com/huggingface/lerobot
  source_commit: 297774fca4e3ad0aa34e7425fbf937111637e77b
  source_licence: Apache-2.0
  skill: AGENTS.md#Notes (Optional dependencies)
  scenario: 'AGENTS.md "Optional dependencies": guard the optional import with TYPE_CHECKING or the _scipy_available flag from utils/import_utils.py at module top, call require_package(...) at use time, and do not call is_package_available in the new module'
  variant: no-skill
  suite: agent-skills
  category: coding
  difficulty: medium
  tags:
  - agent-skills
  - lerobot
  - no-skill
  - optional-dependencies
  - library
  gpu: none
agent:
  timeout_sec: 1500
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

I replay recorded SO-101 action trajectories and they are jittery, so I'd like a smoothing helper in LeRobot itself. Please add `src/lerobot/utils/action_smoothing.py` with:

`smooth_actions(actions, window_length: int = 9, polyorder: int = 2)`

- `actions` is a NumPy array or a torch tensor of shape `(T, D)` (time first). It returns the same kind of object (a tensor keeps its dtype and device) with each dimension smoothed over time by a Savitzky-Golay filter, computed with `scipy.signal.savgol_filter(..., window_length, polyorder, axis=0, mode="interp")`.

scipy is not a core LeRobot dependency: it comes with the optional `scipy-dep` extra, and it is not installed in this environment. So:

- `import lerobot.utils.action_smoothing` must work when scipy is not installed.
- Calling `smooth_actions` without scipy must raise an `ImportError` that tells the user to install the `scipy-dep` extra of lerobot.
- Follow the conventions LeRobot already uses for optional dependencies in its own modules.
