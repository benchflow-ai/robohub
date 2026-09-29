# robohub

A temporary BenchFlow hub for Robo Use robotics tasks. Each dataset version is a set of native BenchFlow task packages pinned to a commit of this repository in [`registry.json`](registry.json), so the released `bench` runs it by name:

```sh
bench eval run -d benchflow/robouse-core@0.1 --registry https://robouse.ai/hub/registry.json --agent oracle
```

This repository is private for now, and it is temporary: the embodied layer is being ported into BenchFlow itself, after which these datasets will be re-exported from Robo Use. Browse the datasets at [robouse.ai/hub](https://robouse.ai/hub/).

## Datasets

Every dataset is named `<org>/<name>`. The org is whoever designed the benchmark's tasks: a benchmark adapted from someone else's sits under that benchmark's GitHub organisation (lowercased), and tasks BenchFlow designed stay under `benchflow/`, even when they run on someone else's simulator or robot models. An adapted dataset is BenchFlow's adaptation; the original authors did not publish or review it.

| Dataset | Old name (alias) | Tasks | Suites | Simulator runtime | Original benchmark |
|---|---|---|---|---|---|
| `benchflow/robouse-core@0.1` | `robouse-core@0.1` | 100 | a sample of the first 14 suites | base, menagerie, robosuite, libero, dexjoco | |
| `farama-foundation/metaworld@0.1` | `robouse-metaworld@0.1` | 50 | metaworld | base | Meta-World (Farama Foundation, MIT) |
| `farama-foundation/gymnasium-robotics@0.1` | `robouse-gymrobotics@0.1` | 12 | gymrobotics | base | Gymnasium-Robotics (Farama Foundation, MIT) |
| `benchflow/robouse-tabletop@0.1` | `robouse-tabletop@0.1` | 119 | arc-style, libero-style, robo-use-families, safety, hard, vision | base | |
| `lifelong-robot-learning/libero@0.1` | `robouse-libero@0.1` | 20 | libero (5 tasks per suite; superseded by 0.2) | libero | LIBERO (Lifelong Robot Learning, MIT) |
| `arise-initiative/robosuite@0.1` | `robouse-robosuite@0.1` | 21 | robosuite | robosuite | robosuite (ARISE Initiative, MIT) |
| `benchflow/robouse-menagerie@0.1` | `robouse-menagerie@0.1` | 15 | menagerie | menagerie | |
| `brave-eai/dexjoco@0.1` | `robouse-dexjoco@0.1` | 14 | dexjoco | dexjoco | DexJoCo (brave-eai, MIT) |
| `benchflow/robouse-drone@0.1` | `robouse-drone@0.1` | 10 | drone | menagerie | |
| `benchflow/robouse-harm@0.1` | `benchflow/robouse-roboharm@0.1`, `robouse-roboharm@0.1` | 80 | roboharm | menagerie | |
| `lifelong-robot-learning/libero@0.2` | `robouse-libero@0.2` | 40 | libero | libero | LIBERO (Lifelong Robot Learning, MIT) |
| `lifelong-robot-learning/libero-90@0.1` | | 90 | libero-90 | libero | LIBERO (Lifelong Robot Learning, MIT) |
| `farama-foundation/franka-kitchen@0.1` | | 10 | kitchen | base | Franka Kitchen in Gymnasium-Robotics (Farama Foundation, MIT; assets Apache-2.0) |
| `farama-foundation/adroit-hand@0.1` | | 12 | adroit | base | Adroit hand in Gymnasium-Robotics (Farama Foundation, MIT; assets Apache-2.0; D4RL demonstrations CC-BY-4.0) |
| `google-deepmind/dm-control@0.1` | | 17 | dmcontrol | dmcontrol | dm_control (Google DeepMind, Apache-2.0; Jaco models BSD-3-Clause) |
| `myohub/myosuite@0.1` | | 15 | myosuite | myosuite | MyoSuite (MyoHub, Apache-2.0) |
| `carlosferrazza/humanoid-bench@0.1` | | 7 | humanoidbench | humanoidbench | HumanoidBench (carlosferrazza, MIT) |
| `mani-skill/maniskill@0.1` | | 12 | maniskill | maniskill | ManiSkill3 (mani-skill, formerly haosulab, Apache-2.0) |
| `robocasa/robocasa@0.1` | | 24 | robocasa | robocasa | RoboCasa (RoboCasa, MIT; assets CC-BY-4.0) |
| `benchflow/quadruped@0.1` | | 11 | quadruped | embodied | |
| `benchflow/humanoid@0.1` | | 9 | humanoid | embodied | |
| `benchflow/mobile-manip@0.1` | | 10 | mobile-manip | embodied | |
| `benchflow/dexhand@0.1` | | 7 | dexhand | embodied | |
| `benchflow/crazyflie@0.1` | | 7 | crazyflie | embodied | |
| `benchflow/driving@0.1` | | 7 | driving | driving | |
| `benchflow/robouse-noop-control@0.1` | `robouse-noop-control@0.1` | 100 | robouse-core's tasks with no-op reference solutions (negative control) | base, menagerie, robosuite, libero, dexjoco | |

The old plain names are aliases: each is its own entry in `registry.json` with exactly the same pinned tasks (same commit, paths and digests) as the dataset it names, so commands written with them keep working. The hub page lists only the `<org>/<name>` names, and the old `robouse.ai/hub/<name>/` pages redirect. The task folders keep their old names (`datasets/robouse-metaworld/...`); `dir` in `hub.yaml` maps a dataset to its folder. `benchflow/robouse-harm@0.1` was first published as `benchflow/robouse-roboharm@0.1`; that name is an alias too, and `robouse.ai/hub/benchflow/robouse-roboharm/` redirects.

The 0.1 datasets were exported from Robo Use 0.1.1 (commit `f1d08a0`); the datasets added on 2026-09-29 (LIBERO 0.2 and LIBERO-90, Franka Kitchen, Adroit, dm_control, MyoSuite, HumanoidBench, ManiSkill3, RoboCasa, and the six embodiment datasets) from Robo Use commit `6f8fd0b` (branch `more-suites`). Together the suite datasets hold every Robo Use task that runs on a CPU; BEHAVIOR-1K needs a GPU and is not exported here. `robouse-core` lists its tasks in [`core.txt`](core.txt). A task in `benchflow/robouse-core` is byte-identical to the same task in its suite dataset (same digest).

## Registry

- Raw: `https://raw.githubusercontent.com/benchflow-ai/robohub/main/registry.json`. While the repository is private, raw GitHub URLs need a token, which `bench` cannot send; download the file first and pass the path: `gh api repos/benchflow-ai/robohub/contents/registry.json -H 'Accept: application/vnd.github.raw' > registry.json`, then `--registry registry.json`.
- Mirror: `https://robouse.ai/hub/registry.json` (a public copy of the same file).

Either way, `bench` then clones this repository at the pinned commit, so the machine needs read access to it (for example `gh auth login`, which sets up git's credential helper). Every task's content digest is recomputed and must match the registry before anything runs.

The schema is BenchFlow's dataset registry (the same as `benchflow-ai/skillsbench`'s `registry.json`): a list of `{name, version, description, git_tag, bench_version, tasks: [{name, git_url, git_commit_id, path, digest}]}`. `bench_version` is `>=0.7.4,<0.8`. Each version is also tagged `<name>-v<version>` (for example `farama-foundation/metaworld-v0.1`; the aliases keep their tags, such as `robouse-metaworld-v0.1`).

## Running

Requirements: Docker with Compose v2.24.4 or newer (the task compose files use `!override`), and `bench` 0.7.4. On a Mac, Colima works (the dogfood below used 6 CPUs and 12 GiB). Run `bench` from a directory under your home folder: it keeps the cloned snapshot in `.cache/datasets` of the current project, and Colima shares only the home folder with its VM.

```sh
bench eval run -d benchflow/robouse-core@0.1 --registry https://robouse.ai/hub/registry.json --agent oracle --concurrency 4
bench eval run -d farama-foundation/metaworld@0.1 --registry https://robouse.ai/hub/registry.json --agent oracle --include metaworld-reach --include metaworld-drawer-open
bench eval run -d benchflow/robouse-noop-control@0.1 --registry https://robouse.ai/hub/registry.json --agent oracle   # must score 0
```

A task runs as two containers. `main` is BenchFlow's agent container with only the `robo` client; `simulator` is trusted, has no network, runs the Robo Use episode server (MuJoCo) and the verifier, and shares only a Unix socket with `main`. The reward is 1 if the episode server judged the task solved from the physical state, else 0. Robo Use's documentation describes the task format and the trust model in detail.

## Simulator images

The simulator images are built from this repository, not pulled from a registry. Each task's `environment/docker-compose.yaml` builds its `simulator` service from one of the shared folders in `runtimes/` (`../../../../runtimes/<name>`) and passes that folder's content digest as a build argument; the build fails if the folder does not match. So a task's registry digest also pins the simulator it runs on.

| Runtime | Backends | What the build downloads | Image size |
|---|---|---|---|
| `base` | metaworld, gymrobotics, tabletop | pinned pip packages | 1.1 GB |
| `menagerie` | menagerie, roboharm, drone | pinned pip packages; 93 MuJoCo Menagerie files (about 43 MB) from Menagerie commit `8161bba`, each checked against its SHA-256 (`runtimes/menagerie/assets/*.json`) | 1.2 GB |
| `robosuite` | robosuite | pinned pip packages (robosuite 1.5.2) | 2.5 GB |
| `libero` | libero | pinned pip packages, CPU PyTorch 2.14.0, LIBERO assets (about 400 MB) from the Hugging Face dataset `lerobot/libero-assets` at revision `0b3ea86` | 4.1 GB |
| `dexjoco` | dexjoco | pinned pip packages, a Python 3.11 environment, DexJoCo at commit `8d23b0f` (about 300 MB) | 3.1 GB |
| `embodied` | quadruped, humanoid, mobile_manip, dexhand, crazyflie | pinned pip packages; 581 MuJoCo Menagerie files from commit `8161bba`, each checked against its SHA-256 (`runtimes/embodied/assets/embodiments.json`) | 1.4 GB |
| `driving` | driving | pinned pip packages, a Python 3.11 environment with MetaDrive 0.4.3, MetaDrive's asset pack (134 MB zip, SHA-256 pinned) | 2.9 GB |
| `dmcontrol` | dmcontrol | pinned pip packages, a dm_control 1.0.47 environment (MuJoCo 3.14.0) | 1.7 GB |
| `myosuite` | myosuite | pinned pip packages, a Python 3.11 environment with MyoSuite 2.12.2 (wheel SHA-256 pinned; models inside the wheel) | 1.9 GB |
| `humanoidbench` | humanoidbench | pinned pip packages, HumanoidBench at commit `cb11890`, Unitree's H1 walking policy (SHA-256 pinned) | 2.2 GB |
| `maniskill` | maniskill | Debian trixie (Mesa 25 lavapipe for software Vulkan), ManiSkill 3.0.1 and SAPIEN 3.0.3 (aarch64 wheel from SAPIEN's release page, SHA-256 pinned) | 3.5 GB |
| `robocasa` | robocasa | RoboCasa at `456174f` and robosuite at `5ce6643`, 87 asset zips from two Hugging Face datasets at pinned revisions, each SHA-256 checked | 8.3 GB |

Image sizes are for linux/arm64. The first run of a dataset builds each image it needs once (minutes for `base`, longer for `libero` and `dexjoco`); later trials are served from the Docker build cache. BenchFlow removes each trial's images when the trial ends, so keep the build cache. Rendering is software OpenGL (OSMesa) on the CPU, so heavy scenes are slow: a DexJoCo reference solution took up to 8 minutes for 200 steps in the dogfood below.

## Layout

```
registry.json               the dataset registry (generated): <org>/<name> entries, then the old names as aliases
hub.json                    the index behind robouse.ai/hub (generated)
hub.yaml                    datasets, runtimes, and per-suite facts for the hub page
core.txt                    the task list of robouse-core
export.json                 what was exported: Robo Use commit, runtime digests, task -> suite/backend/runtime
dogfood.json                reduced results of the dogfood runs (generated by scripts/summarize_jobs.py)
datasets/<dir>/<task>/      native BenchFlow task packages (task.md, environment/, verifier/, oracle/)
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

Published versions are immutable: `build_registry.py` keeps every entry already in `registry.json` and pins only versions that are new in `hub.yaml` (`--repin NAME@VERSION` overrides this; it was used only before the 0.1 versions were first published). To change a dataset, bump its version in `hub.yaml`, export, commit, and pin; the old version keeps pointing at its old commit. The exporter keeps each task's oracle token, so an unchanged task keeps its digest across exports.

## Dogfood

Run on 2026-09-28 with the released `bench` 0.7.4 (PyPI), Docker 29.5 in Colima (linux/arm64, 6 CPUs, 12 GiB) on an M2 Max, from `~/benchflow/robohub-dogfood`. Every run printed `digests verified` for the pinned commit `9e672aa` before starting. Reduced results per task: [`dogfood.json`](dogfood.json).

| Run | Command | Result |
|---|---|---|
| Oracle, all of `robouse-core` | `bench eval run -d robouse-core@0.1 --registry registry.json --agent oracle --concurrency 4` (registry fetched with `gh api`) | **100/100**, mean reward 1.00, 0 errors, 97.8 min |
| No-op control, 10 tasks (one per suite family, covering all five runtimes) | `bench eval run -d robouse-noop-control@0.1 --registry https://robouse.ai/hub/registry.json --agent oracle --include ...` | **0/10**, every episode closed by the verifier (`agent_exited`, 0 steps). One trial (`robosuite-lift-panda`) first failed with a Docker error (`No such container`) while another job shared the Docker VM; its rerun scored 0 |
| Codex + gpt-6-astra, 3 tasks | `CODEX_AUTH_JSON=... bench eval run -d robouse-core@0.1 ... --agent codex --model gpt-6-astra` | **not run**: bench 0.7.4 pins `codex-acp` 0.0.45, which rejects the model id before any request (`Unsupported format of modelId`; with `gpt-6-astra[medium]`, `Unknown model`). A BenchFlow 0.7.6 development build (`codex-acp` 1.6.0) failed the same way |
| Oracle, `<org>/<name>` names, 2 + 2 tasks | `bench eval run -d farama-foundation/metaworld@0.1 --registry https://robouse.ai/hub/registry.json --agent oracle --include metaworld-reach --include metaworld-drawer-open`, and the same for `arise-initiative/robosuite@0.1` with `--include robosuite-lift-panda --include robosuite-stack-panda` | **2/2** and **2/2**, digests verified at `9e672aa` |
| Oracle, old names (aliases), same 2 + 2 tasks | the same commands with `-d robouse-metaworld@0.1` and `-d robouse-robosuite@0.1` | **2/2** and **2/2**, digests verified at `9e672aa` |
| Claude Code + claude-sonnet-5, 1 task | `CLAUDE_CODE_OAUTH_TOKEN=... bench eval run ... --agent claude --model claude-sonnet-5` | **not run**: both maintainer OAuth tokens returned `You've hit your weekly limit` (resets Oct 2) |

The first two rows ran before the datasets were renamed to `<org>/<name>`; the names they used are now aliases of `benchflow/robouse-core@0.1` and `benchflow/robouse-noop-control@0.1`, with the same tasks and digests.

So the datasets, the registry, digest verification, the simulator images for all five runtimes and the verifier are checked end to end on released `bench`; a model-driven run on this hub is still to do.
