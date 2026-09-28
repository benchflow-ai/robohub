# robohub

A temporary BenchFlow hub for Robo Use robotics tasks. Each dataset version is a set of native BenchFlow task packages pinned to a commit of this repository in [`registry.json`](registry.json), so the released `bench` runs it by name:

```sh
bench eval run -d robouse-core@0.1 --registry https://robouse.ai/hub/registry.json --agent oracle
```

This repository is private for now, and it is temporary: the embodied layer is being ported into BenchFlow itself, after which these datasets will be re-exported from Robo Use. Browse the datasets at [robouse.ai/hub](https://robouse.ai/hub/).

## Datasets

| Dataset | Tasks | Suites | Simulator runtime |
|---|---|---|---|
| `robouse-core@0.1` | 100 | a sample of all 14 suites | all five |
| `robouse-metaworld@0.1` | 50 | metaworld | base |
| `robouse-gymrobotics@0.1` | 12 | gymrobotics | base |
| `robouse-tabletop@0.1` | 119 | arc-style, libero-style, robo-use-families, safety, hard, vision | base |
| `robouse-libero@0.1` | 20 | libero | libero |
| `robouse-robosuite@0.1` | 21 | robosuite | robosuite |
| `robouse-menagerie@0.1` | 15 | menagerie | menagerie |
| `robouse-dexjoco@0.1` | 14 | dexjoco | dexjoco |
| `robouse-drone@0.1` | 10 | drone | menagerie |
| `robouse-roboharm@0.1` | 80 | roboharm | menagerie |
| `robouse-noop-control@0.1` | 100 | robouse-core's tasks with no-op reference solutions (negative control) | all five |

The nine suite datasets together hold all 341 tasks of the Robo Use 0.1.1 package; RoboCasa is not included (it runs only from a Robo Use checkout). `robouse-core` lists its tasks in [`core.txt`](core.txt). A task in `robouse-core` is byte-identical to the same task in its suite dataset (same digest).

## Registry

- Raw: `https://raw.githubusercontent.com/benchflow-ai/robohub/main/registry.json`. While the repository is private, raw GitHub URLs need a token, which `bench` cannot send; download the file first and pass the path: `gh api repos/benchflow-ai/robohub/contents/registry.json -H 'Accept: application/vnd.github.raw' > registry.json`, then `--registry registry.json`.
- Mirror: `https://robouse.ai/hub/registry.json` (a public copy of the same file).

Either way, `bench` then clones this repository at the pinned commit, so the machine needs read access to it (for example `gh auth login`, which sets up git's credential helper). Every task's content digest is recomputed and must match the registry before anything runs.

The schema is BenchFlow's dataset registry (the same as `benchflow-ai/skillsbench`'s `registry.json`): a list of `{name, version, description, git_tag, bench_version, tasks: [{name, git_url, git_commit_id, path, digest}]}`. `bench_version` is `>=0.7.4,<0.8`. Each version is also tagged `<name>-v<version>` (for example `robouse-core-v0.1`).

## Running

Requirements: Docker with Compose v2.24.4 or newer (the task compose files use `!override`), and `bench` 0.7.4. On a Mac, Colima works (the dogfood below used 6 CPUs and 12 GiB). Run `bench` from a directory under your home folder: it keeps the cloned snapshot in `.cache/datasets` of the current project, and Colima shares only the home folder with its VM.

```sh
bench eval run -d robouse-core@0.1 --registry https://robouse.ai/hub/registry.json --agent oracle --concurrency 4
bench eval run -d robouse-core@0.1 --registry https://robouse.ai/hub/registry.json --agent oracle --include arc-gravity --include metaworld-reach
bench eval run -d robouse-noop-control@0.1 --registry https://robouse.ai/hub/registry.json --agent oracle   # must score 0
```

A task runs as two containers. `main` is BenchFlow's agent container with only the `robo` client; `simulator` is trusted, has no network, runs the Robo Use episode server (MuJoCo) and the verifier, and shares only a Unix socket with `main`. The reward is 1 if the episode server judged the task solved from the physical state, else 0. Robo Use's documentation describes the task format and the trust model in detail.

## Simulator images

The simulator images are built from this repository, not pulled from a registry. Each task's `environment/docker-compose.yaml` builds its `simulator` service from one of the shared folders in `runtimes/` (`../../../../runtimes/<name>`) and passes that folder's content digest as a build argument; the build fails if the folder does not match. So a task's registry digest also pins the simulator it runs on.

| Runtime | Backends | What the build downloads | Image size |
|---|---|---|---|
| `base` | metaworld, gymrobotics, tabletop | pinned pip packages | 1.1 GB |
| `menagerie` | menagerie, roboharm, drone | pinned pip packages; 87 MuJoCo Menagerie files (43 MB) from Menagerie commit `8161bba`, each checked against its SHA-256 (`runtimes/menagerie/assets/*.json`) | 1.2 GB |
| `robosuite` | robosuite | pinned pip packages (robosuite 1.5.2) | 2.5 GB |
| `libero` | libero | pinned pip packages, CPU PyTorch 2.14.0, LIBERO assets (about 400 MB) from the Hugging Face dataset `lerobot/libero-assets` at revision `0b3ea86` | 4.1 GB |
| `dexjoco` | dexjoco | pinned pip packages, a Python 3.11 environment, DexJoCo at commit `8d23b0f` (about 300 MB) | 3.1 GB |

Image sizes are for linux/arm64. The first run of a dataset builds each image it needs once (minutes for `base`, longer for `libero` and `dexjoco`); later trials are served from the Docker build cache. BenchFlow removes each trial's images when the trial ends, so keep the build cache. Rendering is software OpenGL (OSMesa) on the CPU, so heavy scenes are slow: a DexJoCo reference solution took up to 8 minutes for 200 steps in the dogfood below.

## Layout

```
registry.json               the dataset registry (generated)
hub.json                    the index behind robouse.ai/hub (generated)
hub.yaml                    datasets, runtimes, and per-suite facts for the hub page
core.txt                    the task list of robouse-core
export.json                 what was exported: Robo Use commit, runtime digests, task -> suite/backend/runtime
dogfood.json                reduced results of the dogfood runs (generated by scripts/summarize_jobs.py)
datasets/<dataset>/<task>/  native BenchFlow task packages (task.md, environment/, verifier/, oracle/)
runtimes/<name>/            simulator image build contexts (Dockerfile, pinned requirements, Robo Use source)
templates/                  the templates export.py writes packages and runtimes from
scripts/export.py           Robo Use checkout -> runtimes/ and datasets/
scripts/build_registry.py   datasets at a commit -> registry.json and hub.json
scripts/summarize_jobs.py   bench job folders -> dogfood.json
```

## Regenerating

```sh
# 1. export from a Robo Use checkout (its dev venv has PyYAML and Meta-World, whose scripted experts the Meta-World oracles vendor)
~/benchflow/robouse/.venv/bin/python scripts/export.py --robouse ~/benchflow/robouse
git add -A && git commit -m "Export Robo Use <commit> as datasets <...>"

# 2. pin new dataset versions to that commit, with BenchFlow's own task_digest (run with bench's Python)
~/.local/share/uv/tools/benchflow/bin/python scripts/build_registry.py --tag
git add registry.json hub.json && git commit -m "Registry: pin <...>" && git push origin main --tags
```

Published versions are immutable: `build_registry.py` keeps every entry already in `registry.json` and pins only versions that are new in `hub.yaml`. To change a dataset, bump its version in `hub.yaml`, export, commit, and pin; the old version keeps pointing at its old commit. The exporter keeps each task's oracle token, so an unchanged task keeps its digest across exports.

## Dogfood

Run on 2026-09-28 with the released `bench` 0.7.4 (PyPI), Docker 29.5 in Colima (linux/arm64, 6 CPUs, 12 GiB) on an M2 Max, from `~/benchflow/robohub-dogfood`. Every run printed `digests verified` for the pinned commit `9e672aa` before starting. Reduced results per task: [`dogfood.json`](dogfood.json).

| Run | Command | Result |
|---|---|---|
| Oracle, all of `robouse-core` | `bench eval run -d robouse-core@0.1 --registry registry.json --agent oracle --concurrency 4` (registry fetched with `gh api`) | **100/100**, mean reward 1.00, 0 errors, 97.8 min |
| No-op control, 10 tasks (one or two per runtime) | `bench eval run -d robouse-noop-control@0.1 --registry https://robouse.ai/hub/registry.json --agent oracle --include ...` | **0/10**, every episode closed by the verifier (`agent_exited`, 0 steps). One trial (`robosuite-lift-panda`) first failed with a Docker error (`No such container`) while another job shared the Docker VM; its rerun scored 0 |
| Codex + gpt-6-astra, 3 tasks | `CODEX_AUTH_JSON=... bench eval run -d robouse-core@0.1 ... --agent codex --model gpt-6-astra` | **not run**: bench 0.7.4 pins `codex-acp` 0.0.45, which rejects the model id before any request (`Unsupported format of modelId`; with `gpt-6-astra[medium]`, `Unknown model`). A BenchFlow 0.7.6 development build (`codex-acp` 1.6.0) failed the same way |
| Claude Code + claude-sonnet-5, 1 task | `CLAUDE_CODE_OAUTH_TOKEN=... bench eval run ... --agent claude --model claude-sonnet-5` | **not run**: both maintainer OAuth tokens returned `You've hit your weekly limit` (resets Oct 2) |

So the datasets, the registry, digest verification, the simulator images for all five runtimes and the verifier are checked end to end on released `bench`; a model-driven run on this hub is still to do.
