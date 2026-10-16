# Checks

All checks ran on 2026-10-16 in Daytona x86_64 CPU sandboxes (`python:3.12-slim-bookworm` plus `env/<source>/requirements.txt`, the upstream repository at the pinned commit). No model was called.

## Per-task checks (`tools/check.py`, records in `checks/<source>.jsonl`)

For every task and both variants, the harness resets the repository, replays the image's own steps (setup, variant, integrity manifest), applies one mode and runs the verifier exactly as BenchFlow does. A check is OK when the reference solution gives reward 1 and the no-op and each trivial edit give 0. Verifier time: 3 to 40 s for most tasks; `lerobot-mlp-policy-factory` about 78 s on 4 vCPU (it trains 200 steps); Genesis 10 to 30 s with a warm kernel cache, plus 70 to 100 s for the first scene build (the tests use their own cache folder, so an agent cannot pre-warm it).

| Source | Tasks | Reference passes | No-op fails | Trivial edits fail | Result |
|---|---|---|---|---|---|
| Isaac Lab | 13 | 26/26 | 26/26 | 52/52 | 104/104 OK |
| LeRobot | 8 | 16/16 | 16/16 | 32/32 | 64/64 OK |
| Genesis | 8 | 16/16 | 16/16 | 32/32 | 64/64 OK |
| All | 29 | 58/58 | 58/58 | 116/116 | 232/232 OK |

The trivial edits are the two most tempting shortcuts per task (for example: run the repository's quaternion finder with `--fix` and accept everything; silence the ProxyArray deprecation warnings instead of migrating; point the RSL-RL entry point at the maintained Cartpole config; switch the task to depth outright instead of adding a preset; register the env without an agent config). Each is in `src/<id>/trivial/`.

## End-to-end runs through bench on Daytona (`--agent oracle`, real task images)

| Tasks | with-skill | no-skill | Time per run (two tasks, image builds included) |
|---|---|---|---|
| `isaaclab-presets-newton`, `isaaclab-migrate-proxyarray` | 2/2 pass | 2/2 pass | 2.0 and 1.8 min |
| `lerobot-reach-env-config`, `lerobot-smolvla-short-run-config` | 2/2 pass | 2/2 pass | 1.8 and 1.8 min |
| `genesis-box-size-validation`, `genesis-linear-momentum-getter` | 2/2 pass | 2/2 pass | 3.3 and 3.2 min |

## Variant isolation probe (real images on Daytona)

A probe run (the `isaaclab-presets-newton` image of each variant, with a solve script that only inspects the workspace) found:

| | with-skill | no-skill |
|---|---|---|
| `skills/`, `.agents/skills`, `.claude/skills`, `AGENTS.md`, `CLAUDE.md` | present (26 guide files) | absent (0 guide files) |
| `docs/source/migration/migrating_to_isaaclab_3-0.rst` | present | present |
| git history | 1 commit; 24 `SKILL.md` paths in it | 1 commit; no `SKILL.md` path in it |
| workspace owner, agent can write it | `agent`, yes | `agent`, yes |
| agent can read the integrity manifest, write site-packages | no, no | no, no |
| torch | 2.11.0+cpu | 2.11.0+cpu |

## What was not checked

- No agent trial was run (no model keys are used for this item); `study/` holds the job files for the owner.
- The trivial edits are a sample of shortcuts, not a proof that no shortcut exists. The integrity manifest blocks edits to the library under test, but an agent with network access could still read the deleted guides on GitHub; `tools/summarize.py` lists every no-skill trial whose trajectory mentions a guide path or fetches the upstream guides.
