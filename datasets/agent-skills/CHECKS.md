# Checks

All checks ran on 2026-10-16 (the full set again after each of the two review fix rounds) in Daytona x86_64 CPU sandboxes (`python:3.12-slim-bookworm` plus `env/<source>/requirements.txt`, the upstream repository at the pinned commit). No model was called.

## Per-task checks (`tools/check.py`, records in `checks/<source>.jsonl`)

For every task and both variants, the harness resets the repository (including its local git settings), replays the image's own steps (setup, variant, integrity manifest, the workspace commit the verifier compares with), applies one mode and runs the verifier exactly as BenchFlow does. A check is OK when the reference solution and each valid alternative give reward 1 and the no-op and each trivial edit give 0. Verifier time: 3 to 40 s for most tasks; `lerobot-mlp-policy-factory` about 78 s on 4 vCPU (it trains 200 steps); Genesis 10 to 30 s with a warm kernel cache, plus 70 to 100 s for the first scene build (the tests use their own cache folder, so an agent cannot pre-warm it).

| Source | Tasks | Reference passes | Valid alternatives pass | No-op fails | Trivial edits fail | Result |
|---|---|---|---|---|---|---|
| Isaac Lab | 13 | 26/26 | 14/14 | 26/26 | 72/72 | 138/138 OK |
| LeRobot | 8 | 16/16 | 2/2 | 16/16 | 32/32 | 66/66 OK |
| Genesis | 8 | 16/16 | 4/4 | 16/16 | 42/42 | 78/78 OK |
| All | 29 | 58/58 | 20/20 | 58/58 | 146/146 | 282/282 OK |

The trivial edits are the two most tempting shortcuts per task (for example: run the repository's quaternion finder with `--fix` and accept everything; silence the ProxyArray deprecation warnings instead of migrating; point the RSL-RL entry point at the maintained Cartpole config; switch the task to depth outright instead of adding a preset; register the env without an agent config). Each is in `src/<id>/trivial/`. A review found three more, now trivial edits that fail: switching the ProxyArray deprecation bridge off inside `rewards.py` or from the package `__init__.py` (`isaaclab-migrate-proxyarray`), and committing non-ASCII Genesis code so that a diff against `HEAD` misses it (`genesis-box-size-validation`; the Genesis ASCII checks now compare with the workspace build commit, which the image records root-only).

A second review found that a patch undone before each term returns still passed `isaaclab-migrate-proxyarray`, and that a `.gitattributes` `-diff` line hid non-ASCII Genesis code. The proxyarray verifier now watches every reward call with `sys.monitoring` and a profile hook (any run of the bridge or shim code fails, and ProxyArray, warp's `to_torch` and the test data are checked at every function start and call), keeps an audit hook for `__class__` and `__code__` changes and for threads or hook changes during a call, and scans every .py file that differs from the build commit by content. The Genesis ASCII checks read the files from disk and compare them byte for byte with the build commit's blobs, so git attributes, textconv and clean filters, excludes and commits cannot hide a line. Every verifier now runs pytest with a fresh bytecode cache, so a stale `.pyc` cannot stand in for its source. Twelve more shortcuts are trivial edits now (eight for proxyarray: the two patch-and-undo bypasses, a one-line C no-op patch, a `__code__` swap, a worker thread, a `__class__` swap, the shim's second binding `warp._src.torch.to_torch`, a forged stale `.pyc`; four for Genesis: `-diff`, a textconv driver, a clean filter, `.git/info/exclude`); seven of the eight and all four got reward 1 from the verifiers before this round.

Valid alternatives (`src/<id>/valid/`) are correct solutions that differ from the reference and must pass: the link-frame pose and CoM-frame velocity writers in `isaaclab-migrate-write-index`, the minimal per-use `.torch` migration, reads through a small helper module, and the reference committed by the agent in `isaaclab-migrate-proxyarray`, and a SmolVLA config that keeps the default 30000 decay steps and relies on LeRobot's auto-scaling in `lerobot-smolvla-short-run-config`.

A third review ran honest solutions and found four that got reward 0; all four are valid alternatives now. Genesis's upstream `tests/` folder was fully protected, so the regression test that Genesis's `CLAUDE.md` asks a bug fix to add (the with-skill arm's skill) failed every Genesis task; it is now add-only (`+tests` in `sources.yaml`): new test files and new lines are accepted, every original line must stay byte for byte, and the verifier runs only its own hidden tests, never the workspace's (`genesis-box-size-validation` valid-1 appends a test to `tests/core/test_misc.py`, valid-2 adds a new test file). In `isaaclab-migrate-proxyarray` the static scan no longer reads prose (docstrings and bare string statements, so a docstring may name the `__torch_function__` bridge, as the migration guide does: valid-5), the agent's own test files may re-arm ProxyArray's warn-once flag (valid-4; the run-time layer still sees any change of it during a reward call), and a term may use `torch.compile` (valid-6): the verifier compiles in-process and lets through only the processes inductor's own code starts (the system C++ compiler without plugin or wrapper options, its ISA probe, `openssl sha512`); any other process during a reward call still fails. That verifier run takes about 2 minutes, because the compile runs under the observers.

## End-to-end runs through bench on Daytona (`--agent oracle`, real task images)

| Tasks | with-skill | no-skill | Time per run (image builds included) |
|---|---|---|---|
| `isaaclab-presets-newton`, `isaaclab-migrate-proxyarray` | 2/2 pass | 2/2 pass | 2.0 and 1.8 min |
| `lerobot-reach-env-config`, `lerobot-smolvla-short-run-config` | 2/2 pass | 2/2 pass | 1.8 and 1.8 min |
| `genesis-box-size-validation`, `genesis-linear-momentum-getter` | 2/2 pass | 2/2 pass | 3.3 and 3.2 min |
| After the review fixes: `genesis-box-size-validation`, `isaaclab-migrate-proxyarray`, `isaaclab-migrate-write-index`, `lerobot-smolvla-short-run-config` | 4/4 pass | 4/4 pass | 3.1 and 2.3 min |
| After the second fix round: the same four tasks | 4/4 pass | 4/4 pass | 3.2 and 1.2 min |
| After the third fix round: the same four tasks | 4/4 pass | 4/4 pass | 4.7 and 4.3 min |

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
- The trivial edits are a sample of shortcuts, not a proof that no shortcut exists. The proxyarray verifier runs in the same Python process as the agent's code, so its observers are strong evidence, not a proof: defeating them needs the hidden verifier's design, switching `sys.monitoring` off without a call they see, and patching ProxyArray only with C callables that do the bridge's work. The integrity manifest blocks edits to the library under test, but an agent with network access could still read the deleted guides on GitHub; `tools/summarize.py` lists every no-skill trial whose trajectory mentions a guide path or fetches the upstream guides.
